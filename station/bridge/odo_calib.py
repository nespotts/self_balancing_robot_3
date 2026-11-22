"""Calibrate odometry (issue #14, part of the SLAM plan #71): wheel diameter, left/right ratio, track width
and the gyro drift at rest, from guided runs with a tape measure.

The odometry pose uses the IMU heading for direction and the wheels for distance, so what is wrong shows up as:
  * distance scale (wheel diameter, odo.wheel_d_mm): a straight run, robot vs tape measure
  * left/right ratio (odo.lr_ratio) and track width (odo.track_mm): the wheels' travel difference against the
    gyro heading, over the straight run and the two squares (least squares over all runs, no tape needed)
  * the squares (UMBmark, Borenstein 1996) also give the closing error and the textbook E_b / E_d numbers,
    shown as a cross-check (they assume the heading comes from the wheels; here it comes from the gyro)
  * gyro drift: heading change at rest minus what the wheels turned, per minute

The bridge never arms and never drives by itself. Every run needs the robot armed and upright and the owner's
tap on Start; it then sends GOTO waypoints (slow: nav.max_speed is lowered to <= 150 mm/s for the run and put back
after; the obstacle stop stays on). STOP, Disarm, E-stop, a push, a fall, a lost link, a blocked path or anyone
driving ends the run at once (waypoints cleared, back to BALANCE unless the robot was disarmed / stopped).

Browser commands (WebSocket or POST /api/cmd); the state is pushed to everyone as {"t":"calib_state", ...}:
  {"t":"calib","action":"start","test":"straight"|"cw"|"ccw"|"gyro"[,"length_mm":2000,"secs":60]}
  {"t":"calib","action":"measured","test":"straight","forward_cm":198.5[,"side_cm":-2]}   side: left +
  {"t":"calib","action":"measured","test":"cw"|"ccw","x_cm":-3,"y_cm":5}   end vs start mark: x ahead of the start
        heading, y to its left
  {"t":"calib","action":"stop"|"apply"|"revert"|"reset"|"status"}
Saving is the existing {"t":"cfg","action":"save"} (EEPROM write, only with the motors off).
"""
import json
import math
import time

from . import link_protocol as lp

WHEEL_D, TRACK, LR = "odo.wheel_d_mm", "odo.track_mm", "odo.lr_ratio"
NAV_SPEED = "nav.max_speed"
RUN_SPEED_MMS = 120.0         # nav.max_speed during a run (the limit is 150)
SPEED_LIMIT_MMS = 150.0
DEFAULTS = {"straight": 2000.0, "square": 1000.0}
LENGTH_LIMITS = {"straight": (500.0, 4000.0), "square": (500.0, 2000.0)}
GYRO_SECS = (20.0, 300.0)
GYRO_DEFAULT_S = 60.0
START_SPEED_MMS = 80.0        # "standing still" to start
ABORT_SPEED_MMS = 260.0       # much faster than the run speed: something is wrong
GOTO_GRACE_S = 3.0            # the mode takes a moment to follow the request
BLOCKED_ABORT_S = 6.0
ARRIVE_SPEED_MMS = 30.0
ARRIVE_HOLD_S = 1.0
TLM_STALE_S = 1.0
PUSH_HZ = 4.0
MAX_DT = 0.2
TESTS = ("straight", "cw", "ccw", "gyro")
NAMES = {"straight": "Straight run", "cw": "Square, clockwise", "ccw": "Square, counter-clockwise",
         "gyro": "Gyro drift at rest"}
ACTIVE = ("running", "returning")


# ---------------------------------------------------------------------------------- the maths (pure)
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def route(test, length_mm, x0, y0, h0_deg):
    """GOTO waypoints (odom frame) of a run started at (x0, y0) facing h0_deg. Local frame: x ahead, y left."""
    L = float(length_mm)
    local = {"straight": [(L, 0.0)],
             "cw": [(L, 0.0), (L, -L), (0.0, -L), (0.0, 0.0)],
             "ccw": [(L, 0.0), (L, L), (0.0, L), (0.0, 0.0)]}[test]
    c, s = math.cos(math.radians(h0_deg)), math.sin(math.radians(h0_deg))
    return [(round(x0 + c * px - s * py, 1), round(y0 + s * px + c * py, 1)) for px, py in local]


def to_local(dx, dy, h0_deg):
    """World offset -> offset in the start frame (x ahead of the start heading, y to its left)."""
    c, s = math.cos(math.radians(h0_deg)), math.sin(math.radians(h0_deg))
    return c * dx + s * dy, -s * dx + c * dy


def distance_scale(reported_mm, true_mm):
    """The wheel diameter must be multiplied by this: true distance / reported distance."""
    if reported_mm <= 1.0 or true_mm <= 1.0:
        raise ValueError("need a distance")
    return true_mm / reported_mm


def fit_track_lr(runs):
    """runs: [{"X": wheel travel difference right - left (mm), "S": mean wheel travel (mm), "psi": gyro heading
    change (rad)}]. Model per run: X = T * psi + eps * S, T = effective track (reported wheel mm per rad),
    eps = relative right-vs-left over-reading. Returns (T or None, eps or None); T is None unless the runs turned."""
    runs = [r for r in runs if abs(r["S"]) > 100.0]
    if not runs:
        return None, None
    spp = sum(r["psi"] ** 2 for r in runs)
    sps = sum(r["psi"] * r["S"] for r in runs)
    sss = sum(r["S"] ** 2 for r in runs)
    sxp = sum(r["X"] * r["psi"] for r in runs)
    sxs = sum(r["X"] * r["S"] for r in runs)
    det = spp * sss - sps * sps
    turning = spp > math.radians(120.0) ** 2 and det > 0.05 * spp * sss
    if turning:
        return (sxp * sss - sxs * sps) / det, (sxs * spp - sxp * sps) / det
    straight = [r for r in runs if abs(r["psi"]) < math.radians(8.0)]
    if not straight:
        return None, None
    return None, sum(r["X"] * r["S"] for r in straight) / sum(r["S"] ** 2 for r in straight)


def lr_from_eps(r_param, eps):
    """New odo.lr_ratio from the measured right-vs-left over-reading eps (see fit_track_lr)."""
    return r_param * (1.0 - eps / 2.0) / (1.0 + eps / 2.0)


def umbmark(cw, ccw, side_mm, track_mm):
    """Borenstein's UMBmark (1996). cw / ccw: end offset (x, y) mm of the clockwise / counter-clockwise square
    from its start, x along the first leg, y to the left. Returns type A (alpha, turn error per corner) and type B
    (beta, curvature) in degrees, the track factor E_b = b_actual / b_nominal, the wheel diameter ratio
    E_d = D_right / D_left, and the radius R of the curvature (None when straight)."""
    L = float(side_mm)
    alpha = (cw[0] + ccw[0]) / (-4.0 * L)
    beta = (cw[0] - ccw[0]) / (-4.0 * L)
    a_deg, b_deg = math.degrees(alpha), math.degrees(beta)
    out = {"alpha_deg": a_deg, "beta_deg": b_deg, "E_b": 90.0 / (90.0 - a_deg), "E_d": 1.0, "R_mm": None}
    if abs(beta) > 1e-9:
        R = (L / 2.0) / math.sin(beta / 2.0)
        out["R_mm"] = R
        out["E_d"] = (R + track_mm / 2.0) / (R - track_mm / 2.0)
    out["track_mm"] = out["E_b"] * track_mm
    return out


def drift_per_min(ts, gyro_deg, wheel_deg):
    """deg/min the gyro heading moves relative to the wheels' heading: slope of (gyro - wheels) over time."""
    n = len(ts)
    if n < 10:
        return 0.0, 0.0, 0.0

    def slope(ys):
        mt, my = sum(ts) / n, sum(ys) / n
        den = sum((t - mt) ** 2 for t in ts)
        return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / den if den > 0 else 0.0
    g, w = slope(gyro_deg), slope(wheel_deg)
    return (g - w) * 60.0, g * 60.0, w * 60.0


def verdict(dpm):
    a = abs(dpm)
    return "excellent" if a < 0.5 else "good" if a < 1.5 else "so-so" if a < 4.0 else "poor"


# ---------------------------------------------------------------------------------- the tool
class OdoCalib:
    def __init__(self, bridge):
        self.b = bridge
        self.state = "idle"          # idle | running | returning (driving back is part of the run) | measure | aborted
        self.test = None
        self.text = ""
        self.warn = []
        self.cfg = {}
        self.data = {}               # test -> run result (+ "measured")
        self.orig = None             # {param name: value} when the first Apply happened (Undo)
        self.applied = None          # {param name: value} set by Apply
        self.nav_saved = None        # (value we set, value it had)
        self.t0 = self.t_last = self.t_push = 0.0
        self.t_still = None
        self.t_blocked = None
        self.start = None            # (x, y, heading) at the start of a run
        self.acc = None              # integrals of the run in progress
        self.who = "?"
        self.last_d = None           # the newest telemetry frame seen during a run

    # ------------------------------------------------------------------ helpers
    def _param(self, name):
        return next((p for p in self.b.params.values() if p["name"] == name), None)

    def _val(self, name):
        p = self._param(name)
        return p["value"] if p else None

    def _say(self, text):
        self.b.say("calibrate: " + text)

    def active(self):
        return self.state in ACTIVE

    def _fresh(self, now):
        return bool(self.b.tlm) and now - self.b.tlm_time < TLM_STALE_S and self.b.link_state == "connected"

    def why_not(self, test, now):
        """None when `test` may start now, else a short reason (for the user)."""
        if self._param(WHEEL_D) is None or self._param(TRACK) is None or self._param(LR) is None:
            return "The robot firmware has no odo.* params yet: flash this version"
        if not self._fresh(now):
            return "Robot not responding"
        d = self.b.tlm
        if getattr(self.b, "estop", False):
            return "E-stop is active"
        if not d["status"] & lp.TLM_ARMED:
            return "Arm the robot first (two taps)"
        if d["mode"] not in (lp.MODE_BALANCE, lp.MODE_HOLD, lp.MODE_STATION) or \
                (test == "gyro" and d["mode"] == lp.MODE_STATION):
            return "Needs BALANCE or HOLD mode (it is %s)" % lp.MODE_NAMES[d["mode"]]
        if not d["status"] & lp.TLM_UPRIGHT:
            return "The robot is not upright"
        if getattr(self.b, "cmd_active", False):
            return "Someone is driving the robot"
        if abs(d["speed_mms"]) > START_SPEED_MMS:
            return "The robot is moving: let it stand still"
        if self.state in ACTIVE:
            return "A run is going"
        return None

    def _live_problem(self, d, now):
        """Checked every telemetry frame during a run: a reason to abort, or None."""
        if not d["status"] & lp.TLM_ARMED:
            return "The robot was disarmed"
        if not d["status"] & lp.TLM_UPRIGHT:
            return "The robot fell over"
        if getattr(self.b, "estop", False):
            return "E-stop"
        if abs(d["speed_mms"]) > ABORT_SPEED_MMS:
            return "The robot was pushed or went too fast"
        want = (lp.MODE_BALANCE, lp.MODE_HOLD) if self.test == "gyro" else (lp.MODE_GOTO,)
        if now - self.t0 > GOTO_GRACE_S and d["mode"] not in want:
            return "The mode changed to %s" % lp.MODE_NAMES[d["mode"]]
        return None

    # ------------------------------------------------------------------ messages
    def _tests(self, now):
        out = {}
        for t in TESTS:
            r = self.data.get(t)
            out[t] = {"name": NAMES[t], "ran": bool(r), "measured": bool(r and r.get("measured")),
                      "result": self._brief(t), "ready": self.why_not(t, now) if self.b.tlm else "Not connected"}
        return out

    def _brief(self, t):
        r = self.data.get(t)
        if not r:
            return None
        if t == "gyro":
            return {"drift_dpm": round(r["drift_dpm"], 2), "verdict": verdict(r["drift_dpm"]), "secs": round(r["secs"])}
        out = {"reported_mm": round(r["reported_mm"], 1), "end_mm": [round(r["end_local"][0]), round(r["end_local"][1])],
               "turn_deg": round(math.degrees(r["psi"]), 1)}
        m = r.get("measured")
        if m:
            out["measured"] = {k: round(v, 1) for k, v in m.items()}
        return out

    def msg(self, now=None):
        now = now if now is not None else time.monotonic()
        a = self.acc or {}
        running = self.state in ACTIVE
        cur = {n: self._val(n) for n in (WHEEL_D, TRACK, LR)}
        prop = self.proposal()
        changed = bool(self.orig) and any(abs((self._val(n) or 0) - v) > 1e-6 for n, v in self.orig.items())
        return {"t": "calib_state", "state": self.state, "test": self.test, "text": self.text, "warn": self.warn,
                "tests": self._tests(now), "params": cur, "proposal": prop, "orig": self.orig, "changed": changed,
                "applied": self.applied,
                "run": {"elapsed": round(now - self.t0, 1), "dist_mm": round(a.get("path", 0.0)),
                        "turn_deg": round(math.degrees(a.get("psi", 0.0)), 1), "target": self.cfg.get("secs"),
                        "length_mm": self.cfg.get("length_mm"), "live_drift_dpm": a.get("live")} if running else None,
                "speed_limit": SPEED_LIMIT_MMS}

    def push(self, now=None):
        self.t_push = now if now is not None else time.monotonic()
        self.b.broadcast(json.dumps(self.msg(now), separators=(",", ":")))

    # ------------------------------------------------------------------ proposal
    def proposal(self):
        """Proposed odo.* values from the finished runs, with the numbers behind them. None until there is any."""
        d0, t0, r0 = self._val(WHEEL_D), self._val(TRACK), self._val(LR)
        if d0 is None:
            return None
        notes, rows = [], {}
        st = self.data.get("straight")
        d_new = d0
        if st and st.get("measured"):
            m = st["measured"]
            true = math.hypot(m["forward"], m.get("side", 0.0))
            f = distance_scale(st["reported_mm"], true)
            d_new = d0 * f
            rows["wheel_d"] = {"name": WHEEL_D, "from": d0, "to": d_new,
                               "why": "robot said %.0f mm, tape says %.0f mm" % (st["reported_mm"], true)}
            if abs(m.get("side", 0.0)) > 0.03 * true:
                notes.append("It ended %.0f cm to the %s: the path curved (wheel slip, or a drifting gyro)" % (
                    abs(m["side"]) / 10.0, "left" if m["side"] > 0 else "right"))
        elif st:
            notes.append("Measure the straight run with the tape to fix the wheel diameter")
        fit_runs = [{"X": r["Sr"] - r["Sl"], "S": 0.5 * (r["Sr"] + r["Sl"]), "psi": r["psi"]}
                    for k, r in self.data.items() if k in ("straight", "cw", "ccw")]
        T_eff, eps = fit_track_lr(fit_runs)
        if eps is not None and abs(eps) > 1e-5:
            r_new = lr_from_eps(r0, eps)
            rows["lr"] = {"name": LR, "from": r0, "to": r_new,
                          "why": "right wheel reads %+.2f %% vs the left over the same gyro heading" % (eps * 100.0)}
        if T_eff is not None and T_eff > 20.0:
            t_new = T_eff * d_new / d0
            rows["track"] = {"name": TRACK, "from": t0, "to": t_new,
                             "why": "wheels turned %.1f mm per rad for the gyro's heading" % T_eff}
        elif fit_runs and not any(k in self.data for k in ("cw", "ccw")):
            notes.append("Run both squares to get the track width")
        cw, ccw = self.data.get("cw"), self.data.get("ccw")
        um = None
        if cw and ccw and cw.get("measured") and ccw.get("measured"):
            L = cw["length_mm"]
            um = umbmark((cw["measured"]["x"], cw["measured"]["y"]), (ccw["measured"]["x"], ccw["measured"]["y"]), L, t0)
            um = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in um.items()}
            if "track" not in rows:
                rows["track"] = {"name": TRACK, "from": t0, "to": um["track_mm"], "why": "UMBmark E_b %.4f" % um["E_b"]}
            if "lr" not in rows:
                rows["lr"] = {"name": LR, "from": r0, "to": r0 * um["E_d"], "why": "UMBmark E_d %.4f" % um["E_d"]}
        for r in rows.values():
            lo = self._param(r["name"])
            r["to"] = round(max(lo["min"], min(lo["max"], r["to"])), 4)
            r["from"] = round(r["from"], 4)
        close = {}
        for k in ("cw", "ccw"):
            r = self.data.get(k)
            if r and r.get("measured"):
                close[k] = round(math.hypot(r["measured"]["x"], r["measured"]["y"]) / 10.0, 1)
        if not rows and not notes and not close:
            return None
        return {"rows": list(rows.values()), "notes": notes, "umbmark": um, "closing_cm": close}

    # ------------------------------------------------------------------ commands
    def command(self, m, who, now=None):
        now = now if now is not None else time.monotonic()
        action = m.get("action")
        self.who = who
        if action == "status":
            return self.msg(now)
        if action == "start":
            return self._start(m, who, now)
        if action == "stop":
            if self.active():
                self._abort("Stopped", now)
            return {"t": "ok"}
        if action == "measured":
            return self._measured(m, now)
        if action == "apply":
            return self._apply(now)
        if action == "revert":
            return self._revert(now)
        if action == "reset":
            if self.active():
                raise ValueError("a run is going: stop it first")
            self.data, self.warn, self.state, self.text, self.test = {}, [], "idle", "", None
            self.push(now)
            return {"t": "ok"}
        raise ValueError("bad calib action %r" % action)

    def _num(self, m, key, lo, hi):
        try:
            v = float(m.get(key))
        except (TypeError, ValueError):
            raise ValueError("%s: need a number" % key)
        if not math.isfinite(v) or not lo <= v <= hi:
            raise ValueError("%s must be between %g and %g" % (key, lo, hi))
        return v

    def _start(self, m, who, now):
        test = m.get("test")
        if test not in TESTS:
            raise ValueError("bad test %r" % test)
        why = self.why_not(test, now)
        if why:
            raise ValueError(why)
        d = self.b.tlm
        self.test, self.warn = test, []
        self.cfg = {}
        if test == "gyro":
            secs = GYRO_SECS
            self.cfg["secs"] = self._num(m, "secs", *secs) if m.get("secs") is not None else GYRO_DEFAULT_S
        else:
            kind = "straight" if test == "straight" else "square"
            lo, hi = LENGTH_LIMITS[kind]
            self.cfg["length_mm"] = self._num(m, "length_mm", lo, hi) if m.get("length_mm") is not None else DEFAULTS[kind]
        self.t0 = self.t_last = now
        self.t_still = self.t_blocked = None
        self.start = (d["x_mm"], d["y_mm"], d["heading_deg"])
        self.acc = {"Sl": 0.0, "Sr": 0.0, "psi": 0.0, "path": 0.0, "h_prev": d["heading_deg"],
                    "ts": [], "g": [], "w": [], "wheel_h": 0.0, "live": None}
        if test == "gyro":
            self.state = "running"
            self.text = "Standing still: measuring the gyro against the wheels"
            self._say("%s: gyro drift at rest, %g s" % (who, self.cfg["secs"]))
            self.push(now)
            return {"t": "ok"}
        pts = route(test, self.cfg["length_mm"], *self.start)
        self._slow_down()
        self.state = "running"
        self.text = "Driving %s" % ("straight %.1f m" % (self.cfg["length_mm"] / 1000.0) if test == "straight"
                                    else "the square")
        self._say("%s: %s, %.1f m, %d waypoint(s), max %.0f mm/s" % (
            who, NAMES[test], self.cfg["length_mm"] / 1000.0, len(pts), RUN_SPEED_MMS))
        err = self.b.handle_command({"t": "goto", "points": pts, "replace": True, "go": True}, "calib")
        if err and err.get("t") == "error":
            self._restore()
            self.state, self.text = "idle", ""
            raise ValueError(err.get("text") or "could not send the route")
        self.push(now)
        return {"t": "ok"}

    def _slow_down(self):
        p = self._param(NAV_SPEED)
        if p is None:
            return
        want = min(RUN_SPEED_MMS, p["value"])
        if self.nav_saved is None:
            self.nav_saved = (want, p["value"])
        if abs(p["value"] - want) > 1e-6:
            self.b.handle_command({"t": "set_param", "id": p["id"], "value": want}, "calib")

    def _restore(self):
        if self.nav_saved is None:
            return
        want, was = self.nav_saved
        self.nav_saved = None
        p = self._param(NAV_SPEED)
        if p is not None and abs(p["value"] - want) < 1e-6 and abs(was - want) > 1e-6:
            self.b.handle_command({"t": "set_param", "id": p["id"], "value": was}, "calib")

    # ------------------------------------------------------------------ runs
    def _track_rest(self, d):
        """After a drive run, until the owner has measured: the end pose follows the robot while it stands still
        (it keeps creeping a little after the run), so the tape and the odometry compare the same spot."""
        r = self.data.get(self.test)
        if not r or r.get("measured") or not r.get("start") or getattr(self.b, "cmd_active", False):
            return
        if not d["status"] & lp.TLM_ARMED or abs(d["speed_mms"]) > 60.0:
            return
        x0, y0, h0 = r["start"]
        dx, dy = d["x_mm"] - x0, d["y_mm"] - y0
        r["reported_mm"], r["end_local"] = math.hypot(dx, dy), to_local(dx, dy, h0)

    def on_tlm(self, d, now):
        if self.state == "measure":
            return self._track_rest(d)
        if self.state not in ACTIVE or self.acc is None:
            return
        self.last_d = d
        bad = self._live_problem(d, now)
        if bad:
            return self._abort(bad, now)
        a = self.acc
        dt = max(0.0, min(MAX_DT, now - self.t_last))
        self.t_last = now
        a["Sl"] += d["wheel_left_mms"] * dt
        a["Sr"] += d["wheel_right_mms"] * dt
        a["path"] += abs(0.5 * (d["wheel_left_mms"] + d["wheel_right_mms"])) * dt
        a["psi"] += math.radians(wrap180(d["heading_deg"] - a["h_prev"]))
        a["h_prev"] = d["heading_deg"]
        a["end"] = (d["x_mm"], d["y_mm"], d["heading_deg"])
        if self.test == "gyro":
            a["wheel_h"] += d["turn_rate_dps"] * dt
            a["ts"].append(now - self.t0)
            a["g"].append(math.degrees(a["psi"]))
            a["w"].append(a["wheel_h"])
            if abs(d["speed_mms"]) > START_SPEED_MMS * 1.5:
                return self._abort("The robot moved: keep it still", now)
            if len(a["ts"]) > 60 and now - self.t0 > 15:
                a["live"] = round(drift_per_min(a["ts"], a["g"], a["w"])[0], 2)
            return
        if now - self.t0 <= GOTO_GRACE_S:
            return
        arrived = d["nav_state"] == lp.NAV_ARRIVED or bool(d["status"] & lp.TLM_NAV_DONE)
        if arrived and abs(d["speed_mms"]) < ARRIVE_SPEED_MMS:
            self.t_still = self.t_still if self.t_still is not None else now
            if now - self.t_still >= ARRIVE_HOLD_S:
                return self._finish_drive(now)
        else:
            self.t_still = None
        if d["nav_state"] == lp.NAV_BLOCKED:
            self.t_blocked = self.t_blocked if self.t_blocked is not None else now
            if now - self.t_blocked > BLOCKED_ABORT_S:
                return self._abort("The path is blocked: clear it and run again", now)
        else:
            self.t_blocked = None

    def tick(self, now):
        if not self.active():
            return
        if not self._fresh(now):
            return self._abort("Lost the robot", now)
        if getattr(self.b, "cmd_active", False):
            return self._abort("Someone started driving the robot", now)
        if self.test == "gyro":
            if now - self.t0 >= self.cfg["secs"]:
                self._finish_gyro(now)
        else:
            path = self.cfg["length_mm"] * (1 if self.test == "straight" else 4)
            if now - self.t0 > 2.5 * path / RUN_SPEED_MMS + 90.0:
                self._abort("Took too long", now)
        if self.active() and now - self.t_push >= 1.0 / PUSH_HZ:
            self.push(now)

    def on_lost(self):
        if self.active():
            self._abort("Lost the robot", time.monotonic())

    def _finish_gyro(self, now):
        a = self.acc
        dpm, gpm, wpm = drift_per_min(a["ts"], a["g"], a["w"])
        self.data["gyro"] = {"drift_dpm": dpm, "gyro_dpm": gpm, "wheel_dpm": wpm, "secs": now - self.t0,
                             "psi": a["psi"], "reported_mm": 0.0, "end_local": (0.0, 0.0), "Sl": 0.0, "Sr": 0.0}
        self.state, self.text = "idle", "Gyro drift %+.2f deg/min (%s)" % (dpm, verdict(dpm))
        self.warn = []
        if abs(wpm) > 1.0:
            self.warn.append("The wheels turned %.1f deg/min: the robot was not still" % wpm)
        self._say(self.text + " (gyro %+.2f, wheels %+.2f deg/min, %.0f s)" % (gpm, wpm, now - self.t0))
        self.push(now)

    def _finish_drive(self, now):
        a, (x0, y0, h0) = self.acc, self.start
        x1, y1, _h1 = a["end"]
        dx, dy = x1 - x0, y1 - y0
        loc = to_local(dx, dy, h0)
        self.data[self.test] = {"reported_mm": math.hypot(dx, dy), "end_local": loc, "psi": a["psi"], "Sl": a["Sl"],
                                "Sr": a["Sr"], "path": a["path"], "length_mm": self.cfg["length_mm"], "start": self.start,
                                "params": [self._val(WHEEL_D), self._val(TRACK), self._val(LR)]}
        self._stop_driving()
        self._restore()
        self.state = "measure"
        if self.test == "straight":
            self.text = "Done. The robot says it rolled %.0f mm. Measure the real distance with the tape" % math.hypot(dx, dy)
        else:
            self.text = "Done. Measure where it ended compared with the start mark"
        self._say("%s: robot says (%.0f, %.0f) mm from the start, turned %.1f deg, wheel travel L %.0f R %.0f mm" % (
            NAMES[self.test], loc[0], loc[1], math.degrees(a["psi"]), a["Sl"], a["Sr"]))
        self.push(now)

    def _stop_driving(self):
        """Waypoints cleared; GOTO -> BALANCE (never arms, never from a mode the user already left)."""
        t = self.last_d or self.b.tlm or {}
        self.b.handle_command({"t": "event", "event": lp.EVT_CLEAR_WAYPOINTS}, "calib")
        if t.get("mode") == lp.MODE_GOTO and (t.get("status", 0) & (lp.TLM_ARMED | lp.TLM_UPRIGHT)) == (lp.TLM_ARMED | lp.TLM_UPRIGHT) \
                and not getattr(self.b, "estop", False):
            self.b.handle_command({"t": "set_mode", "mode": lp.MODE_BALANCE}, "calib")

    def _abort(self, reason, now):
        if self.test != "gyro":
            self._stop_driving()
        self._restore()
        self.state, self.text = "aborted", reason
        self._say("aborted: " + reason)
        self.push(now)

    # ------------------------------------------------------------------ results
    def _measured(self, m, now):
        test = m.get("test")
        r = self.data.get(test)
        if test not in ("straight", "cw", "ccw") or not r:
            raise ValueError("run that test first")
        if self.active():
            raise ValueError("a run is going")
        if test == "straight":
            fwd = self._num(m, "forward_cm", 10.0, 1000.0) * 10.0
            side = self._num(m, "side_cm", -200.0, 200.0) * 10.0 if m.get("side_cm") not in (None, "") else 0.0
            r["measured"] = {"forward": fwd, "side": side}
        else:
            r["measured"] = {"x": self._num(m, "x_cm", -300.0, 300.0) * 10.0,
                             "y": self._num(m, "y_cm", -300.0, 300.0) * 10.0}
        if self.state in ("measure", "aborted"):
            self.state, self.text = "idle", ""
        self._say("%s measured: %s" % (NAMES[test], r["measured"]))
        self.push(now)
        return {"t": "ok"}

    def _apply(self, now):
        if self.active():
            raise ValueError("a run is going")
        prop = self.proposal()
        rows = [r for r in (prop or {}).get("rows", []) if abs(r["to"] - r["from"]) > 1e-6]
        if not rows:
            raise ValueError("nothing to apply")
        if self.orig is None:
            self.orig = {n: self._val(n) for n in (WHEEL_D, TRACK, LR)}
        self.applied = {}
        for r in rows:
            p = self._param(r["name"])
            self.b.handle_command({"t": "set_param", "id": p["id"], "value": r["to"]}, "calib")
            self.applied[r["name"]] = r["to"]
        self._say("%s: applied %s" % (self.who, ", ".join("%s %.4g -> %.4g" % (r["name"], r["from"], r["to"]) for r in rows)))
        # the runs were made with the old values: keep them for the record, but ask for new ones
        self.data = {k: v for k, v in self.data.items() if k == "gyro"}
        self.state, self.text = "idle", "Applied. Run the square again to check, then Save to robot"
        self.push(now)
        return {"t": "ok"}

    def _revert(self, now):
        if not self.orig:
            raise ValueError("nothing to undo")
        if self.active():
            raise ValueError("a run is going")
        for n, v in self.orig.items():
            p = self._param(n)
            if p is not None:
                self.b.handle_command({"t": "set_param", "id": p["id"], "value": v}, "calib")
        self._say("%s: back to %s" % (self.who, ", ".join("%s %.4g" % kv for kv in self.orig.items())))
        self.orig = self.applied = None
        self.data = {k: v for k, v in self.data.items() if k == "gyro"}
        self.state, self.text = "idle", "Back to the old values"
        self.push(now)
        return {"t": "ok"}
