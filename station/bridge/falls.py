"""Black-box fall recorder (issue #50).

Keeps the last few seconds of telemetry / range readings / log lines / commands in RAM. When the robot was
balancing (armed + upright, mode != IDLE) and then is not (it tipped past bal.cutoff_deg, the firmware cut the
motors, or the serial link died), it freezes PRE_S seconds before and POST_S seconds after into one small file
and tags the likely cause. A user stop (E-stop, SET_MODE IDLE, remote switch) is not a fall: nothing is saved.

  rec = FallRecorder(out_dir, params=lambda: {"bal.cutoff_deg": 45.0, ...}, session=lambda: "sbr3-....jsonl")
  rec.on_tlm(decoded_telemetry_dict)  rec.on_range(d)  rec.on_event("log", "text")  rec.user_stop("pc")
  rec.tick()                          -> [meta, ...] of records finished since the last call (writer thread)

Cost: one deque append per telemetry frame; a freeze copies a list of references (~2000 at 400 Hz); analysis and the
file write run in a worker thread (a few tens of ms).

File "fall-YYYYmmdd-HHMMSS.jsonl", one JSON array per line, t = seconds relative to the trigger (robot clock):
  ["fall", {meta: id, time, session, session_t, trigger, tags, cause, tipped, params, cols, spark, ...}]
  ["tlm", t, v1, ...] in meta["cols"] order      ["rng", t, v1, ...] in meta["rng_cols"] order
  ["log", t, "text"]  ["cmd", t, {...}]  ["bridge", t, "text"]
Analysis (analyze()) is pure and also runs on trimmed session logs: see station/tools/falls_scan.py.
"""
import collections
import json
import math
import os
import re
import threading
import time

from . import link_protocol as lp

PRE_S = 5.0              # seconds kept before the trigger
POST_S = 2.0             # ... and after it
POST_MAX_S = 6.0         # a second fall inside the window extends it, up to this long after the first trigger
RING_MAX = 8000          # frames (5 s at 400 Hz = 2000; this guards a runaway rate)
EVENTS_MAX = 400
MAX_FILES = 200
MAX_MB = 100
STOP_WINDOW_S = 1.0      # a user stop this long before the trigger explains it
SPARK_N = 120
NAME_RE = re.compile(r"^fall-\d{8}-\d{6}(-\d+)?\.jsonl$")

COLS = ["t_us", "mode", "status", "radio_ok", "pitch_deg", "pitch_rate_dps", "balance_target_deg", "motor_left",
        "motor_right", "bal_p", "bal_i", "bal_d", "steer_out", "vel_out_deg", "speed_mms", "cmd_speed_mms",
        "cmd_turn_dps", "wheel_left_mms", "wheel_right_mms", "battery_v", "loop_hz", "remote_flags", "nav_state",
        "step"]
RNG_COLS = ["t_us", "range_mm", "sensor_id", "sensor_angle_deg", "valid"]

# tag -> (label, short help). Order = priority when several match: the first one is the "cause".
TAGS = collections.OrderedDict([
    ("link_lost", ("Link lost", "the serial link died while balancing: Teensy reset or brown-out?")),
    ("radio", ("Radio dropout", "the RC link dropped out just before the motors were cut")),
    ("mode_drop", ("Motors cut", "mode left BALANCE (upright or not) with nobody asking for it")),
    ("battery", ("Battery sag", "supply voltage dropped or was low")),
    ("loop", ("IMU / loop dip", "control loop slowed down or telemetry skipped")),
    ("obstacle", ("Hit obstacle", "range sensor saw something close in the direction of travel")),
    ("slip", ("Wheel slip", "wheel speed did not match the motor output")),
    ("oscillation", ("Oscillation", "pitch swings were growing")),
    ("runaway", ("Speed loop ran away", "the lean target wandered far from upright")),
    ("pushed", ("Pushed / bump", "pitch-rate spike while the motors were not saturated")),
    ("saturated", ("Saturated", "motors at full power: ran out of authority")),
])


# ------------------------------------------------------------------ analysis (pure)
def _median(xs):
    s = sorted(xs)
    n = len(s)
    return 0.0 if not n else (s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2]))


def _runs(flags, ts, min_s):
    """[(t_start, t_end)] of consecutive True flags lasting >= min_s."""
    out, i, n = [], 0, len(flags)
    while i < n:
        if flags[i]:
            j = i
            while j + 1 < n and flags[j + 1]:
                j += 1
            if ts[j] - ts[i] >= min_s - 1e-9:
                out.append((ts[i], ts[j]))
            i = j + 1
        else:
            i += 1
    return out


def rows_from_frames(frames):
    """[(pi_time, decoded telemetry dict)] -> rows with "t" (s, robot clock, 0 = the last frame)."""
    if not frames:
        return []
    ref = frames[-1][1]["t_us"]
    out = []
    for _pi, d in frames:
        r = {k: d.get(k, 0) for k in COLS}
        dt = ((d["t_us"] - ref + 0x80000000) & 0xFFFFFFFF) - 0x80000000     # 32-bit wrap safe
        r["t"] = dt / 1e6
        out.append(r)
    return out


def analyze(rows, params=None, events=None, ranges=None, trigger=None):
    """Tag the likely cause of a fall.

    rows     telemetry dicts (COLS keys) with "t" relative to the trigger (negative = before), time ordered
    params   {name: value} active at the time
    events   [(t, kind, payload)] log / cmd lines (same t base)
    ranges   [{"t", "range_mm", "sensor_id", "valid"}]
    trigger  {"kind": "tip" | "cut" | "link", "t": 0.0}
    Returns {"tags": [{"tag", "label", "detail", "score"}], "cause", "tipped", "metrics": {...}}; tags are in
    priority order (the first is the cause), score 0..1."""
    params = params or {}
    events = events or []
    ranges = ranges or []
    trigger = trigger or {"kind": "tip"}
    cutoff = float(params.get("bal.cutoff_deg", 45.0))
    offset = float(params.get("bal.offset_deg", 3.15))
    out = {"tags": [], "cause": None, "tipped": False, "metrics": {}}
    if len(rows) < 3:
        if trigger.get("kind") == "link":
            out["tags"].append(_tag("link_lost", "no telemetry after the last frame", 0.9))
            out["cause"] = "link_lost"
        return out
    ts = [r["t"] for r in rows]
    pre = [r for r in rows if r["t"] <= 0.0]
    post = [r for r in rows if r["t"] > 0.0]
    out["tipped"] = any(abs(r["pitch_deg"]) >= 0.9 * cutoff for r in rows if r["t"] >= -0.3)
    m = out["metrics"]
    tags = {}

    def win(a, b):
        return [r for r in rows if a <= r["t"] <= b]

    # --- cut with nobody asking: radio dropout / unexplained mode drop
    kind = trigger.get("kind")
    at0 = win(-0.3, 0.05)
    pitch0 = max([abs(r["pitch_deg"]) for r in at0] or [0.0])
    if kind == "cut" or (kind == "tip" and pitch0 < cutoff - 8.0):
        lead = win(-4.0, 0.3)
        radio = [r["radio_ok"] for r in lead]
        if radio and min(radio) == 0 and max(radio) == 1:       # went away (and maybe came back)
            drops = sum(1 for a, b in zip(radio, radio[1:]) if a and not b)
            lost_s = _longest([r["radio_ok"] == 0 for r in lead], [r["t"] for r in lead])
            tags["radio"] = _tag("radio", "RC link dropped out %d time(s) in the last 4 s, longest %.2f s, then the "
                                 "mode fell to IDLE with the robot still upright (%.1f deg)" % (drops, lost_s, pitch0),
                                 0.95)
        else:
            tags["mode_drop"] = _tag("mode_drop", "mode left BALANCE with the robot at %.1f deg, no command or radio "
                                     "dropout seen" % pitch0, 0.6)
    if kind == "link":
        tags["link_lost"] = _tag("link_lost", "the serial link died while balancing", 0.9)

    lead = win(-3.0, 0.1)

    # --- battery: >0.6 V drop in the last 2 s, or the low-battery flag
    v2 = win(-2.2, 0.1)
    if len(v2) > 5:
        n0 = max(1, len(v2) // 12)
        start = sum(r["battery_v"] for r in v2[:n0]) / n0
        lowest = min(_smooth([r["battery_v"] for r in v2], max(1, len(v2) // 40)))
        m["battery_drop_v"] = round(start - lowest, 2)
        low_flag = any(int(r["status"]) & lp.TLM_LOW_BATT for r in v2)
        if start - lowest > 0.6 or low_flag:
            tags["battery"] = _tag("battery", "supply %s %.2f V -> %.2f V in the last 2 s" % (
                "dropped" if start - lowest > 0.6 else "was low:", start, lowest),
                min(1.0, 0.5 + (start - lowest)) if start - lowest > 0.6 else 0.7)

    # --- loop / IMU: loop rate dip, telemetry gaps, impossible pitch jumps, frozen pitch
    l2 = win(-2.0, -0.002)           # (before the trigger frame: after it the robot is lying down)
    if len(l2) > 10:
        hz = _median([r["loop_hz"] for r in win(-5.0, -2.0)] or [r["loop_hz"] for r in l2])
        nominal = hz if hz > 20 else float(params.get("ctl.rate_hz", 400.0))
        lows = [r["loop_hz"] for r in l2 if r["loop_hz"] > 0]
        dip = min(lows) if lows else nominal
        dts = [b["t"] - a["t"] for a, b in zip(l2, l2[1:])]
        med_dt = _median(dts) or 0.0025
        gap = max(dts) if dts else 0.0
        jump = 0.0
        for a, b in zip(l2, l2[1:]):
            d = b["t"] - a["t"]
            if 0 < d < 0.05:
                jump = max(jump, abs(b["pitch_deg"] - a["pitch_deg"]) / d)
        frozen = _longest([abs(b["pitch_deg"] - a["pitch_deg"]) < 1e-6 for a, b in zip(l2, l2[1:])],
                          [r["t"] for r in l2[1:]])
        m["loop_hz_min"] = round(dip, 0)
        m["gap_ms"] = round(gap * 1000, 1)
        why = []
        if nominal > 0 and dip < 0.85 * nominal:
            why.append("loop rate fell to %.0f Hz (nominal %.0f)" % (dip, nominal))
        if gap > max(0.03, 8 * med_dt) and med_dt < 0.02:
            why.append("telemetry stalled for %.0f ms" % (gap * 1000))
        if jump > 2500:
            why.append("pitch jumped %.0f deg/s in one frame" % jump)
        if frozen > 0.12:
            why.append("pitch stayed frozen for %.0f ms" % (frozen * 1000))
        if why:
            tags["loop"] = _tag("loop", ", ".join(why), 0.8)

    # --- obstacle: something close in the direction of travel, or the firmware's obstacle flag
    obst_mm = float(params.get("rng.obstacle_mm", 300.0))
    sp = _median([r["speed_mms"] for r in win(-1.0, 0.0)] or [0.0])
    hits = [g for g in ranges if -1.5 <= g["t"] <= 0.3 and g.get("valid") and 0 < g["range_mm"] < max(obst_mm, 150.0)]
    ahead = [g for g in hits if (g.get("sensor_id", 0) == 0) == (sp >= 0) and abs(sp) > 40]
    flagged = any(int(r["status"]) & lp.TLM_OBSTACLE for r in win(-1.0, 0.1))
    if ahead or (flagged and abs(sp) > 40):
        near = min([g["range_mm"] for g in ahead] or [0])
        tags["obstacle"] = _tag("obstacle", ("range sensor saw %d mm ahead while moving at %.0f mm/s" % (near, sp))
                                if ahead else "firmware obstacle flag set while moving at %.0f mm/s" % sp, 0.75)

    # --- wheel slip: L/R wheels disagree while the motors do not, or wheel speed jumps with no motor change
    w2 = win(-2.0, 0.1)
    if len(w2) > 10:
        wl = [r["wheel_left_mms"] for r in w2]
        wr = [r["wheel_right_mms"] for r in w2]
        ml = [r["motor_left"] for r in w2]
        mr = [r["motor_right"] for r in w2]
        wts = [r["t"] for r in w2]
        mism = [abs(a - b) > 300 and abs(c - d) < 0.12 and abs(r["steer_out"]) < 300
                for a, b, c, d, r in zip(wl, wr, ml, mr, w2)]
        runs = _runs(mism, wts, 0.08)
        # jump: mean wheel speed moves > 350 mm/s within 60 ms while the mean motor output moves < 0.2
        wm = [(a + b) / 2 for a, b in zip(wl, wr)]
        mm = [(a + b) / 2 for a, b in zip(ml, mr)]
        mmax = [max(abs(a), abs(b)) for a, b in zip(ml, mr)]
        jumps, j, k = 0, 0, 0
        for i in range(len(w2)):
            while wts[i] - wts[j] > 0.06:
                j += 1
            while wts[i] - wts[k] > 0.20:
                k += 1
            # wheels move >350 mm/s in 60 ms although the motor output was steady over the last 200 ms
            # (a wheel spinning up on a saturated motor, or following a motor ramp, is not slip)
            if abs(wm[i] - wm[j]) > 350 and max(mm[k:i + 1]) - min(mm[k:i + 1]) < 0.15 and max(mmax[k:i + 1]) < 0.8:
                jumps += 1
        if runs or jumps > 3:
            tags["slip"] = _tag("slip", ("left/right wheel speeds differed by >300 mm/s for %.0f ms with equal motor "
                                         "output" % (1000 * max(b - a for a, b in runs))) if runs else
                                "wheel speed jumped >350 mm/s with no matching motor change", 0.7)

    # --- oscillation: error (pitch - target) zero-crossing rate >= 2.5 Hz and a growing envelope
    o = win(-3.0, -0.1)
    if len(o) > 40:
        err = [r["pitch_deg"] - r["balance_target_deg"] for r in o]
        mean = sum(err) / len(err)
        e0 = [e - mean for e in err]
        zc, state = 0, 0
        for e in e0:
            if state <= 0 and e > 0.4:
                zc += 1 if state != 0 else 0
                state = 1
            elif state >= 0 and e < -0.4:
                zc += 1 if state != 0 else 0
                state = -1
        span = o[-1]["t"] - o[0]["t"]
        f = zc / 2.0 / span if span > 0 else 0.0
        q = len(o) // 4
        amps = [max(e0[i:i + q]) - min(e0[i:i + q]) for i in range(0, len(o) - q + 1, q)][:4]
        grow = amps[-1] / amps[0] if amps and amps[0] > 1e-6 else 0.0
        m["osc_hz"] = round(f, 2)
        m["osc_growth"] = round(grow, 2)
        if f >= 2.5 and grow >= 1.5 and amps[-1] >= 3.0:
            tags["oscillation"] = _tag("oscillation", "pitch swung at ~%.1f Hz, amplitude grew x%.1f (%.1f deg "
                                       "peak-to-peak)" % (f, grow, amps[-1]), min(1.0, 0.5 + 0.1 * grow))

    # --- runaway: lean target far from upright for a while
    r1 = win(-1.5, 0.0)
    if r1:
        dev = max(abs(r["balance_target_deg"] + offset) for r in r1)
        m["target_dev_deg"] = round(dev, 1)
        if dev > 12.0:
            tags["runaway"] = _tag("runaway", "lean target reached %.1f deg from upright (speed loop wound up)" % dev,
                                   min(1.0, dev / 20.0))

    # --- saturation: motor at full power for >= 150 ms, ending within 0.6 s before / at the trigger
    s2 = win(-3.0, 0.1)
    sat_flags = [max(abs(r["motor_left"]), abs(r["motor_right"])) >= 0.97 for r in s2]
    sat_runs = _runs(sat_flags, [r["t"] for r in s2], 0.15)
    sat_total = sum(1 for f in sat_flags if f) / max(1, len(sat_flags)) * (s2[-1]["t"] - s2[0]["t"]) if s2 else 0.0
    m["saturated_s"] = round(sat_total, 2)
    near = [(a, b) for a, b in sat_runs if b >= -0.6]
    if near:
        a, b = max(near, key=lambda x: x[1] - x[0])
        tags["saturated"] = _tag("saturated", "motors at full power for %.2f s before the fall" % (b - a),
                                 min(1.0, 0.5 + (b - a)))

    # --- pushed: a pitch-rate spike while the motors were not saturated just before it
    p1 = win(-2.0, 0.0)
    calm = [r["pitch_rate_dps"] for r in win(-5.0, -2.0)] or [r["pitch_rate_dps"] for r in p1]
    rms = math.sqrt(sum(x * x for x in calm) / len(calm)) if calm else 0.0
    thr = max(100.0, 3.5 * rms)
    for r in p1:
        if abs(r["pitch_rate_dps"]) >= thr and int(r["status"]) & lp.TLM_UPRIGHT:     # (not while being stood up)
            before = [b for b in rows if r["t"] - 0.15 <= b["t"] < r["t"] - 0.005]    # (the controller reacts in the same frame)
            # (balancing for the whole 150 ms before: not the lift-off when the robot is stood up again)
            if len(before) > 3 and all(int(b["status"]) & lp.TLM_UPRIGHT for b in before) and \
                    max(max(abs(b["motor_left"]), abs(b["motor_right"])) for b in before) < 0.9:
                m["spike_dps"] = round(r["pitch_rate_dps"], 0)
                tags["pushed"] = _tag("pushed", "pitch rate spiked to %.0f deg/s at t=%.2f s with the motors below "
                                      "full power" % (r["pitch_rate_dps"], r["t"]), 0.6)
                break

    ordered = [tags[k] for k in TAGS if k in tags]
    out["tags"] = ordered
    out["cause"] = ordered[0]["tag"] if ordered else None
    if not ordered:
        out["tags"] = [_tag("unknown", "no rule matched: look at the plot", 0.0)]
        out["cause"] = "unknown"
    return out


def _tag(tag, detail, score):
    return {"tag": tag, "label": TAGS.get(tag, ("Unknown", ""))[0], "detail": detail, "score": round(score, 2)}


def _longest(flags, ts):
    runs = _runs(flags, ts, 0.0)
    return max([b - a for a, b in runs] or [0.0])


def _smooth(xs, k):
    if k <= 1:
        return xs
    out, s = [], 0.0
    for i, x in enumerate(xs):
        s += x
        if i >= k:
            s -= xs[i - k]
        out.append(s / min(i + 1, k))
    return out


def tag_label(tag):
    return TAGS.get(tag, ("Unknown", ""))[0]


# ------------------------------------------------------------------ recorder
def _safe(fn):
    """A recorder bug must never reach the robot link: remember it, carry on."""
    def wrapper(self, *a, **kw):
        try:
            return fn(self, *a, **kw)
        except Exception as e:
            self.error = "%s: %s" % (type(e).__name__, e)
            return None
    wrapper.__name__, wrapper.__doc__ = fn.__name__, fn.__doc__
    return wrapper


class FallRecorder:
    def __init__(self, out_dir, params=None, session=None, say=None, max_files=MAX_FILES, max_mb=MAX_MB,
                 threaded=True, clock=time.monotonic, wall=time.time, pre_s=PRE_S, post_s=POST_S):
        self.dir = os.path.abspath(os.path.expanduser(out_dir)) if out_dir else None   # None: no robot profile yet (#97)
        self.params_fn = params or (lambda: {})
        self.session_fn = session or (lambda: None)
        self.say = say or (lambda text: None)
        self.max_files, self.max_bytes = max_files, int(max_mb * 1024 * 1024)
        self.threaded = threaded
        self.clock, self.wall = clock, wall
        self.pre_s, self.post_s = pre_s, post_s
        self.ring = collections.deque(maxlen=RING_MAX)          # (pi_time, telemetry dict)
        self.rng_ring = collections.deque(maxlen=2000)          # (pi_time, range dict)
        self.events = collections.deque(maxlen=EVENTS_MAX)      # (pi_time, kind, payload)
        self.stops = collections.deque(maxlen=20)               # pi_times of user stop commands
        self.live = False
        self.armed = False
        self.tip_latched = False
        self.last_armed_drop = None
        self.pending = None
        self.ignored = collections.deque(maxlen=20)             # {"time", "why"}
        self.ignored_total = 0
        self.saved_total = 0
        self.last_tlm = None
        self._done = []
        self._lock = threading.Lock()
        self._threads = []
        self.error = None
        self._cutoff_v, self._cutoff_at = 45.0, -1e9
        self._meta_cache = {}                                    # name -> (mtime, size, meta)
        self._msgs = []                                          # lines for say(), spoken from tick()

    # ---------------------------------------------------------------- feeding
    def _now(self):
        return self.clock()

    @_safe
    def on_range(self, d, now=None):
        now = self._now() if now is None else now
        self.rng_ring.append((now, d))
        while self.rng_ring and now - self.rng_ring[0][0] > self.pre_s + 1.0:
            self.rng_ring.popleft()
        if self.pending is not None:
            self.pending["rng"].append((now, d))

    @_safe
    def on_event(self, kind, payload, now=None):
        now = self._now() if now is None else now
        self.events.append((now, kind, payload))
        while self.events and now - self.events[0][0] > self.pre_s + 1.0:
            self.events.popleft()
        if self.pending is not None:
            self.pending["events"].append((now, kind, payload))

    @_safe
    def user_stop(self, who="?", now=None, event=True):
        """E-stop / SET_MODE IDLE: the robot stopping next is not a fall."""
        now = self._now() if now is None else now
        self.stops.append(now)
        if event:
            self.on_event("cmd", {"t": "stop", "who": who}, now)

    @_safe
    def on_tlm(self, d, now=None):
        now = self._now() if now is None else now
        self.last_tlm = (now, d)
        ring = self.ring
        ring.append((now, d))
        while ring and now - ring[0][0] > self.pre_s + 1.0:
            ring.popleft()
        if self.pending is not None:
            self.pending["post"].append((now, d))
        st = int(d["status"])
        armed, upright, mode = bool(st & lp.TLM_ARMED), bool(st & lp.TLM_UPRIGHT), d["mode"]
        live = armed and upright and mode != lp.MODE_IDLE
        was_live, was_armed = self.live, self.armed
        self.live, self.armed = live, armed
        if was_armed and not armed:
            self.last_armed_drop = now
        cutoff = self._cutoff()
        trig = None
        if was_live and not live:
            tipped = abs(d["pitch_deg"]) >= 0.9 * cutoff
            trig = "tip" if tipped else "cut"
        elif live and abs(d["pitch_deg"]) > cutoff + 3.0 and not self.tip_latched:
            trig = "tip"            # still "upright" past the cutoff (old firmware / odd params)
        if live and abs(d["pitch_deg"]) > cutoff + 3.0:
            self.tip_latched = True
        elif abs(d["pitch_deg"]) < cutoff - 5.0:
            self.tip_latched = False
        if trig:
            self._trigger(now, d, trig)

    @_safe
    def on_link_lost(self, now=None):
        """The serial link closed. A fall if the robot was balancing a moment ago."""
        now = self._now() if now is None else now
        if self.last_tlm and self.live and now - self.last_tlm[0] < 1.0:
            self._trigger(self.last_tlm[0], self.last_tlm[1], "link")
        self.live = self.armed = False

    # ---------------------------------------------------------------- trigger
    def _cutoff(self):
        """bal.cutoff_deg from the live params, looked up at most every 2 s (called for every frame)."""
        now = self.clock()
        if now - self._cutoff_at > 2.0 or now < self._cutoff_at:
            self._cutoff_at = now
            try:
                self._cutoff_v = float(self.params_fn().get("bal.cutoff_deg", 45.0))
            except (TypeError, ValueError, AttributeError):
                self._cutoff_v = 45.0
        return self._cutoff_v

    def _why_not_a_fall(self, now, d, kind):
        if kind == "link":
            return None
        if any(-0.2 <= now - s <= STOP_WINDOW_S for s in self.stops):
            return "stopped from the station (E-stop / IDLE)"
        if int(d.get("remote_flags", 0)) & lp.LINK_RF_ESTOP or any(
                int(f.get("remote_flags", 0)) & lp.LINK_RF_ESTOP for t, f in (self.ring[i] for i in range(-min(60, len(self.ring)), 0)) if now - t < 1.0):
            return "E-stop from the remote"
        armed = bool(int(d["status"]) & lp.TLM_ARMED)
        recent = [f for t, f in self.ring if now - t < 3.0]
        radio_flap = recent and min(f["radio_ok"] for f in recent) == 0 and max(f["radio_ok"] for f in recent) == 1
        if not armed and not radio_flap and d["radio_ok"] and abs(d["pitch_deg"]) < 0.9 * self._cutoff():
            return "disarmed from the remote"
        return None

    def _trigger(self, now, d, kind):
        p = self.pending
        if p is not None:
            if now - p["now"] < POST_MAX_S:
                p["triggers"].append((now - p["now"], kind))
                p["deadline"] = min(now + self.post_s, p["now"] + POST_MAX_S)
            return
        why = self._why_not_a_fall(now, d, kind)
        if why:
            self.ignored_total += 1
            self.ignored.append({"time": time.strftime("%H:%M:%S", time.localtime(self.wall())), "why": why})
            return
        frames = [(t, f) for t, f in self.ring if now - t <= self.pre_s]
        self.pending = {
            "now": now, "kind": kind, "d": d, "wall": self.wall(), "frames": frames,
            "post": [], "triggers": [(0.0, kind)], "deadline": now + self.post_s,
            "rng": [(t, g) for t, g in self.rng_ring if now - t <= self.pre_s],
            "events": [(t, k, p_) for t, k, p_ in self.events if now - t <= self.pre_s],
            "params": dict(self.params_fn() or {}), "session": self.session_fn(),
        }

    # ---------------------------------------------------------------- finishing
    def tick(self, now=None):
        """Call a few times a second. Returns the meta dicts of records saved since the last call."""
        now = self._now() if now is None else now
        p = self.pending
        if p is not None and now >= p["deadline"]:
            self.pending = None
            if self.threaded:
                th = threading.Thread(target=self._finish, args=(p,), daemon=True, name="sbr3-fall-writer")
                th.start()
                self._threads.append(th)
            else:
                self._finish(p)
        self._threads = [t for t in self._threads if t.is_alive()]
        with self._lock:
            done, self._done = self._done, []
            msgs, self._msgs = self._msgs, []
        for m in msgs:
            self.say(m)
        return done

    def flush(self, timeout=5.0):
        """Wait for writer threads (tests, shutdown)."""
        for t in list(self._threads):
            t.join(timeout)

    def _finish(self, p):
        try:
            meta = self._build_and_save(p)
        except Exception as e:     # never let a recorder bug touch the robot link
            self.error = "%s: %s" % (type(e).__name__, e)
            with self._lock:
                self._msgs.append("fall recorder error: %s" % self.error)
            return
        if meta:
            with self._lock:
                self._done.append(meta)
                self._msgs.append("fall recorded: %s (%s)" % (meta["cause_label"], meta["id"]))

    def set_dir(self, out_dir):
        """Another robot profile (#97): records go to its folder from now on (writer threads finish first)."""
        self.flush(2.0)
        self.dir = os.path.abspath(os.path.expanduser(out_dir)) if out_dir else None
        self._meta_cache.clear()

    def _build_and_save(self, p):
        if not self.dir:
            raise RuntimeError("fall not saved: no robot identified yet")
        t0 = p["now"]
        frames = p["frames"] + [fr for fr in p["post"] if fr[0] > t0 or fr[1] is not p["d"]]
        seen, uniq = set(), []
        for t, f in frames:         # (the trigger frame is in both lists)
            if id(f) not in seen:
                seen.add(id(f))
                uniq.append((t, f))
        rows = rows_from_frames(uniq)
        # rows_from_frames puts t=0 on the LAST frame; shift so t=0 is the trigger frame
        ref = next((r["t"] for (t, f), r in zip(uniq, rows) if f is p["d"]), 0.0)
        for r in rows:
            r["t"] = round(r["t"] - ref, 4)
        events = [(round(t - t0, 3), k, pl) for t, k, pl in p["events"]]
        ranges = []
        for t, g in p["rng"]:
            ranges.append({"t": round(t - t0, 3), "range_mm": g["range_mm"], "sensor_id": g["sensor_id"],
                           "sensor_angle_deg": g["sensor_angle_deg"], "valid": g["valid"], "t_us": g["t_us"]})
        trig = {"kind": p["kind"], "t": 0.0}
        res = analyze(rows, p["params"], events, ranges, trig)
        # a cut nobody asked for, with no radio trouble, that did not end in a tip-over: the robot was caught
        # or the remote's mode switch was flipped. Not a fall.
        if p["kind"] == "cut" and not res["tipped"] and res["cause"] == "mode_drop":
            self.ignored_total += 1
            self.ignored.append({"time": time.strftime("%H:%M:%S", time.localtime(p["wall"])),
                                 "why": "motors cut but the robot did not tip over"})
            return None
        meta = self._meta(p, rows, res, t0)
        meta["id"] = self._new_name(meta["wall"])
        text = self._serialize(meta, rows, ranges, events)
        meta["bytes"] = len(text)
        self._write(meta["id"], text)
        self.saved_total += 1
        return public_meta(meta)

    def _meta(self, p, rows, res, t0):
        wall = p["wall"]
        d = p["d"]
        tl = [r for r in rows if r["t"] <= 0.0]
        hz = 0.0
        if len(rows) > 2 and rows[-1]["t"] > rows[0]["t"]:
            hz = (len(rows) - 1) / (rows[-1]["t"] - rows[0]["t"])
        meta = {
            "v": 1, "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(wall)), "wall": round(wall, 2),
            "session": p["session"], "trigger": {"kind": p["kind"], "extra": [[round(a, 2), k] for a, k in p["triggers"][1:]]},
            "mode": lp.MODE_NAMES[d["mode"]] if 0 <= d["mode"] < len(lp.MODE_NAMES) else str(d["mode"]),
            "pitch_deg": round(d["pitch_deg"], 1), "battery_v": round(d["battery_v"], 2),
            "tags": res["tags"], "cause": res["cause"], "cause_label": tag_label(res["cause"]),
            "tipped": res["tipped"], "metrics": res["metrics"], "tlm_hz": round(hz, 0),
            "pre_s": round(-rows[0]["t"], 2) if rows else 0, "post_s": round(rows[-1]["t"], 2) if rows else 0,
            "params": {k: round(v, 6) if isinstance(v, float) else v for k, v in p["params"].items()},
            "cols": COLS, "rng_cols": RNG_COLS, "spark": spark(rows),
        }
        return meta

    def _serialize(self, meta, rows, ranges, events):
        lines = [json.dumps(["fall", meta], separators=(",", ":"))]
        for r in rows:
            lines.append(json.dumps(["tlm", r["t"]] + [_rc(c, r[c]) for c in COLS], separators=(",", ":")))
        for g in ranges:
            lines.append(json.dumps(["rng", g["t"]] + [_r(g.get(c, 0)) for c in RNG_COLS], separators=(",", ":")))
        for t, k, pl in events:
            lines.append(json.dumps([k, t, pl], separators=(",", ":")))
        body = lines[1:]
        body.sort(key=lambda s: float(s.split(",", 2)[1]))
        return "\n".join([lines[0]] + body) + "\n"

    def _new_name(self, wall):
        os.makedirs(self.dir, exist_ok=True)
        base = time.strftime("fall-%Y%m%d-%H%M%S", time.localtime(wall))
        name, n = base + ".jsonl", 1
        while os.path.exists(os.path.join(self.dir, name)):
            name = "%s-%d.jsonl" % (base, n)
            n += 1
        return name

    def _write(self, name, text):
        tmp = os.path.join(self.dir, name + ".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(tmp, os.path.join(self.dir, name))
        self.prune()

    # ---------------------------------------------------------------- store
    def _files(self):
        out = []
        if not self.dir:
            return out
        try:
            names = os.listdir(self.dir)
        except OSError:
            return out
        for n in names:
            if NAME_RE.match(n):
                try:
                    st = os.stat(os.path.join(self.dir, n))
                except OSError:
                    continue
                out.append((n, st.st_size, st.st_mtime))
        out.sort(key=lambda x: x[0])
        return out

    def prune(self):
        files = self._files()
        total = sum(f[1] for f in files)
        while files and (len(files) > self.max_files or total > self.max_bytes):
            n, size, _m = files.pop(0)
            try:
                os.remove(os.path.join(self.dir, n))
            except OSError:
                pass
            total -= size

    def list(self, limit=100):
        out = []
        for n, size, mtime in reversed(self._files()[-limit:]):
            c = self._meta_cache.get(n)
            if c is None or c[0] != mtime or c[1] != size:
                m = read_meta(os.path.join(self.dir, n))
                c = self._meta_cache[n] = (mtime, size, public_meta(dict(m, id=n, bytes=size)) if m else None)
            if c[2]:
                out.append(c[2])
        for n in [n for n in self._meta_cache if not os.path.exists(os.path.join(self.dir, n))]:
            del self._meta_cache[n]
        return out

    def path(self, name):
        if not self.dir or not NAME_RE.match(name or ""):
            return None
        p = os.path.join(self.dir, name)
        return p if os.path.isfile(p) else None

    def delete(self, name):
        p = self.path(name)
        if p:
            os.remove(p)
        return bool(p)

    def stats(self):
        return {"saved": self.saved_total, "ignored": self.ignored_total, "count": len(self._files()),
                "pending": self.pending is not None, "error": self.error,
                "recent_ignored": list(self.ignored)[-5:]}

    def load(self, name, max_points=1200):
        """One record for the browser: meta + decimated telemetry columns + events."""
        p = self.path(name)
        return load_record(p, max_points) if p else None


# ------------------------------------------------------------------ files
def _r(v):
    return round(v, 3) if isinstance(v, float) else v


_DEC = {"pitch_deg": 2, "pitch_rate_dps": 1, "balance_target_deg": 2, "motor_left": 3, "motor_right": 3, "bal_p": 1,
        "bal_i": 1, "bal_d": 1, "steer_out": 1, "vel_out_deg": 2, "speed_mms": 0, "cmd_speed_mms": 0,
        "cmd_turn_dps": 0, "wheel_left_mms": 0, "wheel_right_mms": 0, "battery_v": 2, "loop_hz": 0}


def _rc(col, v):
    """Round a telemetry value for the file (decimals per column; ints stay ints)."""
    n = _DEC.get(col)
    if n is None or not isinstance(v, float):
        return v
    v = round(v, n)
    return int(v) if n == 0 else v


def public_meta(m):
    """What lists carry: no params (they come with the record)."""
    out = {k: v for k, v in m.items() if k not in ("params", "cols", "rng_cols")}
    out["n_params"] = len(m.get("params", {}))
    return out


def read_meta(path):
    try:
        with open(path, encoding="utf-8") as f:
            row = json.loads(f.readline())
        return row[1] if row and row[0] == "fall" else None
    except (OSError, ValueError, IndexError):
        return None


def load_record(path, max_points=1200):
    meta, rows, events, rngs = None, [], [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                a = json.loads(line)
            except ValueError:
                continue
            if a[0] == "fall":
                meta = a[1]
            elif a[0] == "tlm":
                rows.append(a[1:])
            elif a[0] == "rng":
                rngs.append(a[1:])
            else:
                events.append([a[0], a[1], a[2]])
    if meta is None:
        return None
    if len(rows) > max_points:           # decimate (keep the first and last rows)
        step = len(rows) / float(max_points)
        rows = [rows[int(i * step)] for i in range(max_points)] + [rows[-1]]
    return {"meta": meta, "cols": ["t"] + meta["cols"], "rows": rows, "events": events,
            "rng_cols": ["t"] + meta["rng_cols"], "rng": rngs[-400:]}


def spark(rows, n=SPARK_N):
    """Small decimated series for the list card: t, pitch, target, motor_left, motor_right."""
    if not rows:
        return {}
    step = max(1.0, len(rows) / float(n))
    pick = [rows[min(len(rows) - 1, int(i * step))] for i in range(int(len(rows) / step))]
    return {"t": [round(r["t"], 2) for r in pick], "pitch": [round(r["pitch_deg"], 1) for r in pick],
            "target": [round(r["balance_target_deg"], 1) for r in pick],
            "ml": [round(r["motor_left"], 2) for r in pick], "mr": [round(r["motor_right"], 2) for r in pick]}


def record_rows(path):
    """(meta, rows, events, ranges) of a saved record in the analyze() shapes (tests, tools)."""
    rec = load_record(path, 10 ** 9)
    cols = rec["cols"]
    rows = [dict(zip(cols, r)) for r in rec["rows"]]
    events = [(e[1], e[0], e[2]) for e in rec["events"]]
    rcols = rec["rng_cols"]
    ranges = [dict(zip(rcols, r)) for r in rec["rng"]]
    return rec["meta"], rows, events, ranges


# ------------------------------------------------------------------ HTTP
def handle_http(rec, req):
    """GET /api/falls -> list + stats; GET /api/falls/<id> -> record (JSON, decimated), ?raw=1 -> the file;
    DELETE /api/falls/<id>."""
    from .httpws import Response
    name = req.path[len("/api/falls/"):] if req.path.startswith("/api/falls/") else ""
    if not name:
        return Response.json({"falls": rec.list(), "stats": rec.stats(), "dir": rec.dir, "tags": {
            k: {"label": v[0], "help": v[1]} for k, v in TAGS.items()}})
    path = rec.path(name)
    if not path:
        return Response.text("no such fall", 404)
    if req.method == "DELETE":
        rec.delete(name)
        return Response.json({"t": "ok"})
    if req.query.get("raw"):
        return Response(200, ctype="application/x-ndjson", file=path,
                        headers={"Content-Disposition": 'attachment; filename="%s"' % name})
    try:
        n = int(req.query.get("n", "1200"))
    except ValueError:
        n = 1200
    return Response.json(load_record(path, max(50, min(n, 20000))))