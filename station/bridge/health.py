"""Health page data (#51): one green / amber / red answer to "can I drive?", plus battery minutes left.

Read-only: nothing here arms or commands the robot. The bridge feeds it telemetry and the firmware's `radio:` log
line, and once a second asks for a `health` message (tiles + preflight checklist + battery + Pi stats).
Everything the rules need arrives in one plain dict (`evaluate(ctx)`), so the rules are unit-testable.
The Pi is read through /proc and /sys only (a few tiny files per second); on other systems those fields are None.
"""
import glob
import math
import os
import re
import shutil
from collections import deque

from . import link_protocol as lp

OK, WARN, BAD, NA = "ok", "warn", "bad", "na"
_RANK = {NA: 0, OK: 1, WARN: 2, BAD: 3}

# firmware: "radio: 50.0 pkt/s, gaps>100ms 0 (max 32 ms), bad 0, multi 0, ack fail 0, poll max 61 us"
RADIO_RE = re.compile(r"^radio: ([\d.]+) pkt/s, gaps>100ms (\d+) \(max (\d+) ms\), bad (\d+), multi (\d+), ack fail (\d+)")
RADIO_NOMINAL_PKT_S = 50.0
RADIO_FRESH_S = 30.0          # the firmware prints every 10 s while packets arrive

CELLS = 3
BENCH_BELOW_V = 3.0           # below any single cell: no balance lead / bench supply, the taps read ~0
LOW_CELL_DEFAULT = 3.4        # batt.low_cell
# resting LiPo cell voltage -> percent (rough: the minutes estimate is the more useful number)
OCV = [(3.30, 0), (3.50, 10), (3.70, 40), (3.80, 60), (3.90, 75), (4.00, 88), (4.20, 100)]

TEMP_WARN_C, TEMP_BAD_C = 70.0, 80.0
CPU_WARN_PCT = 90.0
DISK_WARN_GB, DISK_BAD_GB = 2.0, 0.3
LOG_USAGE_EVERY_S = 30.0


def worst(levels):
    return max(levels, key=lambda l: _RANK[l], default=NA)


def parse_radio(text):
    """The firmware's periodic radio line -> dict, or None."""
    m = RADIO_RE.match(text)
    if not m:
        return None
    return {"pkt_s": float(m.group(1)), "gaps": int(m.group(2)), "max_gap_ms": int(m.group(3)),
            "bad": int(m.group(4)), "multi": int(m.group(5)), "ack_fail": int(m.group(6))}


def cell_pct(cell_v):
    if cell_v <= OCV[0][0]:
        return 0
    for (v0, p0), (v1, p1) in zip(OCV, OCV[1:]):
        if cell_v <= v1:
            return int(round(p0 + (p1 - p0) * (cell_v - v0) / (v1 - v0)))
    return 100


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


class PiStats:
    """CPU % (delta of /proc/stat), temperature (/sys/class/thermal), memory, free disk. Cheap: read once a second."""

    def __init__(self, disk_path=".", proc="/proc", thermal="/sys/class/thermal"):
        self.disk_path, self.proc, self.thermal = disk_path, proc, thermal
        self._cpu = None
        self.last = {"cpu_pct": None, "temp_c": None, "mem_pct": None, "disk_free_gb": None, "disk_pct": None}

    def read(self):
        out = dict(self.last)
        s = _read(os.path.join(self.proc, "stat"))
        if s:
            try:
                f = [float(x) for x in s.split("\n", 1)[0].split()[1:]]
                total, idle = sum(f[:8]), f[3] + (f[4] if len(f) > 4 else 0.0)
                if self._cpu and total > self._cpu[0]:
                    out["cpu_pct"] = round(max(0.0, min(100.0, 100.0 * (1 - (idle - self._cpu[1]) / (total - self._cpu[0])))), 1)
                self._cpu = (total, idle)
            except (ValueError, IndexError):
                pass
        temps = []
        for p in glob.glob(os.path.join(self.thermal, "thermal_zone*", "temp")):
            v = _read(p)
            try:
                t = float(v)
                temps.append(t / 1000.0 if t > 200 else t)
            except (TypeError, ValueError):
                pass
        out["temp_c"] = round(max(temps), 1) if temps else None
        m = _read(os.path.join(self.proc, "meminfo"))
        if m:
            mm = dict(re.findall(r"(\w+):\s+(\d+)", m))
            if "MemTotal" in mm and "MemAvailable" in mm and int(mm["MemTotal"]) > 0:
                out["mem_pct"] = round(100.0 * (1 - int(mm["MemAvailable"]) / int(mm["MemTotal"])), 1)
        try:
            du = shutil.disk_usage(self.disk_path)
            out["disk_free_gb"] = round(du.free / 1e9, 1)
            out["disk_pct"] = round(100.0 * du.used / du.total, 1) if du.total else None
        except OSError:
            pass
        self.last = out
        return out


def dir_bytes(path):
    """Total size of the files directly in `path` and below (logs, falls, maps)."""
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for n in files:
                try:
                    total += os.path.getsize(os.path.join(root, n))
                except OSError:
                    pass
    except OSError:
        pass
    return total


class BatteryModel:
    """Minutes left from the voltage trend under load.

    Samples (time, volts, load = mean |motor duty|) are averaged into BIN_S bins over the last WINDOW_S. A least-squares
    fit  v = a + b*t + c*load  separates the discharge trend b (V/s, negative) from the sag under load c (V per full
    duty, negative). Time left = (v at the current typical load - 3 x batt.low_cell) / -b. A LiPo falls off faster at the
    end, so this runs optimistic near empty; the percent comes from the sag-corrected resting voltage."""
    BIN_S, WINDOW_S, MIN_SPAN_S, GAP_RESET_S = 5.0, 600.0, 120.0, 60.0
    MIN_SLOPE = 2e-5            # V/s (~1.2 mV/min): flatter than this is noise or a charger

    def __init__(self):
        self.reset()

    def reset(self):
        self.bins = deque()                 # [t_mid, v_mean, load_mean]
        self.cur = None                     # [t_start, sum_t, sum_v, sum_load, n]
        self.last_t = None
        self.v = None                       # latest smoothed volts
        self.fit = None                     # (a, b, c, t0, load_ref)
        self.bench = False                  # the last sample read ~0 V: no balance lead / bench supply

    def add(self, t, volts, load):
        if self.last_t is not None and t - self.last_t > self.GAP_RESET_S:
            self.reset()                    # the link / robot was away: rest voltage recovery would fake a trend
        self.last_t = t
        if volts < BENCH_BELOW_V:           # bench supply: nothing to learn
            self.reset()
            self.last_t, self.bench = t, True
            return
        self.bench = False
        self.v = volts if self.v is None else self.v + 0.3 * (volts - self.v)
        c = self.cur
        if c is None:
            c = self.cur = [t, 0.0, 0.0, 0.0, 0]
        c[1] += t
        c[2] += volts
        c[3] += load
        c[4] += 1
        if t - c[0] >= self.BIN_S:
            self.bins.append([c[1] / c[4], c[2] / c[4], c[3] / c[4]])
            self.cur = None
            while self.bins and t - self.bins[0][0] > self.WINDOW_S:
                self.bins.popleft()
            self.fit = self._fit()

    def _fit(self):
        bins = self.bins
        if len(bins) < 6 or bins[-1][0] - bins[0][0] < self.MIN_SPAN_S:
            return None
        t0 = bins[0][0]
        load_ref = sum(b[2] for b in bins) / len(bins)
        sol = None
        loads = [b[2] for b in bins]
        if max(loads) - min(loads) > 0.05:      # enough load variation to separate sag from the trend
            sol = _lstsq([[1.0, b[0] - t0, b[2]] for b in bins], [b[1] for b in bins])
            if sol and sol[2] > 0:              # voltage cannot rise with load: refit without it
                sol = None
        if sol is None:
            two = _lstsq([[1.0, b[0] - t0] for b in bins], [b[1] for b in bins])
            if two is None:
                return None
            sol = [two[0], two[1], 0.0]
        return (sol[0], sol[1], sol[2], t0, load_ref)

    def estimate(self, now, cells=CELLS, low_cell=LOW_CELL_DEFAULT):
        """-> {"v","cell_v","pct","minutes","state","load"}; state: bench | learning | ok | steady | low."""
        if self.v is None:
            return {"v": None, "cell_v": None, "pct": None, "minutes": None,
                    "state": "bench" if self.bench else "none", "load": None}
        out = {"v": round(self.v, 2), "cell_v": round(self.v / cells, 2), "pct": None, "minutes": None,
               "state": "learning", "load": None}
        fit = self.fit
        thr = cells * low_cell
        if fit is None:
            out["pct"] = cell_pct(self.v / cells)
            return out
        a, b, c, t0, load_ref = fit
        v_now = a + b * (now - t0) + c * load_ref        # trend value at the typical load
        v_rest = a + b * (now - t0)                      # ... and with no load
        out["load"] = round(load_ref, 2)
        out["pct"] = cell_pct(v_rest / cells)
        if v_now <= thr:
            out["minutes"], out["state"] = 0, "low"
        elif b > -self.MIN_SLOPE:
            out["state"] = "steady"
        else:
            out["minutes"] = int(min(999, round((v_now - thr) / -b / 60.0)))
            out["state"] = "ok"
        return out


def _lstsq(rows, y):
    """Least squares via the normal equations (3x3 at most). None if singular."""
    n = len(rows[0])
    A = [[sum(r[i] * r[j] for r in rows) for j in range(n)] + [sum(r[i] * yy for r, yy in zip(rows, y))] for i in range(n)]
    for i in range(n):
        p = max(range(i, n), key=lambda k: abs(A[k][i]))
        if abs(A[p][i]) < 1e-12:
            return None
        A[i], A[p] = A[p], A[i]
        for k in range(i + 1, n):
            f = A[k][i] / A[i][i]
            for j in range(i, n + 1):
                A[k][j] -= f * A[i][j]
    x = [0.0] * n
    for i in reversed(range(n)):
        x[i] = (A[i][n] - sum(A[i][j] * x[j] for j in range(i + 1, n))) / A[i][i]
    return x


def _tile(id, label, level, value, why=""):
    return {"id": id, "label": label, "level": level, "value": value, "why": why}


def evaluate(c):
    """Rules: context dict -> {"tiles": [...], "preflight": {...}}. See Health.context() for the keys."""
    t = c.get("tlm")
    p = c.get("params") or {}
    link = c.get("link")
    alive = bool(t) and link == "connected" and c.get("robot_alive")
    tiles = {}
    pre = []

    def add_pre(id, label, level, why):
        pre.append({"id": id, "label": label, "level": level, "why": why})

    # ---- link
    if link == "released":
        lk = _tile("link", "Link", WARN, "released", "serial port released (flashing)")
    elif link != "connected":
        lk = _tile("link", "Link", BAD, "no robot", "the bridge has no USB link to the Teensy")
    elif not c.get("robot_alive"):
        lk = _tile("link", "Link", BAD, "silent", "no telemetry for 1 s")
    else:
        want = float(p.get("tlm.hz", 100.0))
        hz = c.get("tlm_hz", 0.0)
        crc = c.get("crc_errors", 0)
        lvl = OK if hz >= 0.7 * want else WARN
        lk = _tile("link", "Link", lvl, "%d Hz" % round(hz), ("telemetry %d/s of %d" % (round(hz), round(want)) if lvl != OK else
                   "telemetry on time") + (", %d bad frames" % crc if crc else ""))
    tiles["link"] = lk

    def na(id, label, why="no robot data"):
        return _tile(id, label, NA, "-", why)

    if alive:
        err = int(t.get("error_code", 0))
        # ---- loop
        want = float(p.get("ctl.rate_hz", 400.0))
        lh = float(t.get("loop_hz", 0.0))
        lvl = OK if lh >= 0.9 * want else WARN if lh >= 0.7 * want else BAD
        tiles["loop"] = _tile("loop", "Loop", lvl, "%d Hz" % round(lh),
                              "balance loop on time" if lvl == OK else "balance loop %d of %d Hz" % (round(lh), round(want)))
        # ---- IMU, encoders (the firmware's own fault code)
        tiles["imu"] = _tile("imu", "IMU", BAD if err in (1, 2) else OK, "FAULT" if err in (1, 2) else "OK",
                             {1: "IMU did not initialise", 2: "IMU samples stale"}.get(err, "samples arriving"))
        for i, (id, lab) in enumerate((("encl", "Enc L"), ("encr", "Enc R"))):
            bad = err == 3 + i
            tiles[id] = _tile(id, lab, BAD if bad else OK, "FAULT" if bad else "OK",
                              "encoder fault: velocity loop off" if bad else "no fault flagged")
        # ---- range sensors
        if float(p.get("rng.enable", 0.0)) < 0.5:
            for id, lab in (("rngf", "Range F"), ("rngr", "Range R")):
                tiles[id] = _tile(id, lab, NA, "off", "rng.enable is 0")
        else:
            hz = c.get("rng_hz") or [0.0, 0.0]
            for i, (id, lab) in enumerate((("rngf", "Range F"), ("rngr", "Range R"))):
                h = hz[i] if i < len(hz) else 0.0
                lvl = OK if h >= 4 else WARN if h > 0 else BAD
                tiles[id] = _tile(id, lab, lvl, "%d/s" % round(h),
                                  "readings arriving" if lvl == OK else "slow" if lvl == WARN else "no readings")
        # ---- radio
        rd = c.get("radio")
        if not c.get("remote_seen"):
            tiles["radio"] = _tile("radio", "Radio", NA, "no remote", "no RC remote in use: the station arms the robot")
        elif not t.get("radio_ok"):
            tiles["radio"] = _tile("radio", "Radio", BAD, "LOST", "remote silent: failsafe (%d so far)" % c.get("failsafes", 0))
        else:
            parts = []
            lvl, val = OK, "ok"
            if rd:
                q = max(0, min(100, int(round(100.0 * rd["pkt_s"] / RADIO_NOMINAL_PKT_S))))
                val = "%d%%" % q
                parts.append("%.0f pkt/s" % rd["pkt_s"])
                if rd["gaps"]:
                    lvl = WARN
                    parts.append("%d gaps >100 ms (max %d ms)" % (rd["gaps"], rd["max_gap_ms"]))
                if q < 90:
                    lvl = WARN
            if c.get("failsafes"):
                parts.append("%d failsafes this session" % c["failsafes"])
            tiles["radio"] = _tile("radio", "Radio", lvl, val, ", ".join(parts) or "remote present")
        # ---- firmware
        hello = c.get("hello") or {}
        if hello:
            lv = hello.get("link_version")
            tiles["fw"] = _tile("fw", "Firmware", OK if lv == lp.LINK_PROTOCOL_VERSION else WARN,
                                str(hello.get("firmware", "?"))[:14],
                                "link v%s" % lv if lv == lp.LINK_PROTOCOL_VERSION else
                                "link v%s but the bridge speaks v%d" % (lv, lp.LINK_PROTOCOL_VERSION))
        else:
            tiles["fw"] = na("fw", "Firmware", "no HELLO yet")
    else:
        for id, lab in (("loop", "Loop"), ("imu", "IMU"), ("encl", "Enc L"), ("encr", "Enc R"), ("rngf", "Range F"),
                        ("rngr", "Range R"), ("radio", "Radio"), ("fw", "Firmware")):
            tiles[id] = na(id, lab)

    # ---- E-stop
    latched = bool(c.get("estop") or c.get("robot_estop"))
    tiles["estop"] = _tile("estop", "E-stop", BAD if latched else OK, "LATCHED" if latched else "released",
                           "Release E-stop first (motors are locked off)" if latched else "motors free to arm")

    # ---- battery
    bt = c.get("battery") or {}
    low_flag = bool(alive and t.get("status", 0) & lp.TLM_LOW_BATT)
    if not alive:
        tiles["batt"] = na("batt", "Battery")
    elif t.get("battery_v", 0.0) < BENCH_BELOW_V:
        tiles["batt"] = _tile("batt", "Battery", OK, "bench supply", "no balance lead / ~0 V on the cell taps")
    else:
        low_cell = float(p.get("batt.low_cell", LOW_CELL_DEFAULT))
        v = t["battery_v"]
        cell = v / CELLS
        mins = bt.get("minutes")
        txt = "%.1f V" % v
        if bt.get("pct") is not None:
            txt += "  %d%%" % bt["pct"]
        if mins is not None:
            txt += "  ~%d min" % mins
        why = "%.2f V per cell (pack / %d)" % (cell, CELLS)
        if low_flag or cell < low_cell:
            lvl, why = BAD, "LOW: %.2f V per cell, limit %.2f" % (cell, low_cell)
        elif cell < low_cell + 0.15 or (mins is not None and mins < 5):
            lvl, why = WARN, why + ", nearly empty"
        else:
            lvl = OK
            if bt.get("state") == "learning":
                why += ", minutes left: learning (needs ~2 min of data)"
            elif bt.get("state") == "steady":
                why += ", voltage steady (no minutes estimate)"
        tiles["batt"] = _tile("batt", "Battery", lvl, txt, why)

    # ---- Pi
    pi = c.get("pi") or {}
    if pi.get("temp_c") is None and pi.get("cpu_pct") is None:
        tiles["pi"] = _tile("pi", "Pi", NA, "n/a", "no /proc or /sys here (not the Pi)")
    else:
        lv, why = OK, []
        tc, cpu = pi.get("temp_c"), pi.get("cpu_pct")
        if tc is not None:
            if tc >= TEMP_BAD_C:
                lv = worst([lv, BAD])
            elif tc >= TEMP_WARN_C:
                lv = worst([lv, WARN])
        if cpu is not None and cpu >= CPU_WARN_PCT:
            lv = worst([lv, WARN])
        if tc is not None and tc >= TEMP_WARN_C:
            why.append("hot")
        if cpu is not None and cpu >= CPU_WARN_PCT:
            why.append("CPU busy")
        val = ("%dC" % round(tc) if tc is not None else "") + ("  CPU %d%%" % round(cpu) if cpu is not None else "")
        if pi.get("mem_pct") is not None:
            why.append("memory %d%%" % round(pi["mem_pct"]))
        tiles["pi"] = _tile("pi", "Pi", lv, val.strip(), ", ".join(why) or "temperature and load fine")

    # ---- disk / logs
    free = pi.get("disk_free_gb")
    lb = pi.get("log_mb")
    if c.get("log_error"):
        tiles["disk"] = _tile("disk", "Logs", BAD, "log OFF", "session log is off: " + str(c["log_error"]))
    elif free is None:
        tiles["disk"] = _tile("disk", "Logs", NA, "n/a", "disk size unknown")
    else:
        lvl = BAD if free < DISK_BAD_GB else WARN if free < DISK_WARN_GB else OK
        tiles["disk"] = _tile("disk", "Logs", lvl, ("%d MB" % lb if lb is not None else "-") + ", %.0f GB free" % free,
                              "disk almost full: recordings may stop" if lvl != OK else "plenty of space")

    # ================== preflight checklist ==================
    if not alive:
        add_pre("link", "Robot link", tiles["link"]["level"] if tiles["link"]["level"] != OK else BAD, tiles["link"]["why"])
    else:
        add_pre("imu", "IMU ok", tiles["imu"]["level"], tiles["imu"]["why"])
        lp_t = tiles["loop"]
        add_pre("loop", "Loop ~%d Hz" % round(float(p.get("ctl.rate_hz", 400.0))), lp_t["level"], lp_t["why"])
        add_pre("enc", "Encoders ok", worst([tiles["encl"]["level"], tiles["encr"]["level"]]),
                tiles["encl"]["why"] if tiles["encl"]["level"] == BAD else tiles["encr"]["why"])
        add_pre("batt", "Battery ok", tiles["batt"]["level"], tiles["batt"]["why"])
        if c.get("remote_seen"):
            add_pre("ctl", "Remote present", OK if t.get("radio_ok") else BAD,
                    "RC remote answering" if t.get("radio_ok") else "RC remote lost: failsafe")
        else:
            n = c.get("clients", 0)
            add_pre("ctl", "Remote or station", OK if n else WARN,
                    "the station (this page) is in charge" if n else "no remote and no station client")
        if float(p.get("rng.enable", 0.0)) >= 0.5:
            add_pre("rng", "Range sensors", worst([tiles["rngf"]["level"], tiles["rngr"]["level"]]),
                    "both sensors reading" if worst([tiles["rngf"]["level"], tiles["rngr"]["level"]]) == OK else
                    "front: %s, rear: %s" % (tiles["rngf"]["why"], tiles["rngr"]["why"]))
    add_pre("estop", "E-stop released", tiles["estop"]["level"], tiles["estop"]["why"])
    lv = worst([i["level"] for i in pre])
    nbad = sum(1 for i in pre if i["level"] == BAD)
    nwarn = sum(1 for i in pre if i["level"] == WARN)
    if not alive:
        lv = BAD                    # nothing to check without the robot, so never "ready"
        head = "Not ready: " + ("serial port released" if link == "released" else "no robot link")
    elif lv == BAD:
        head = "Not ready: %d problem%s" % (nbad, "" if nbad == 1 else "s")
    elif lv == WARN:
        head = "Ready, %d to check" % nwarn
    else:
        head = "Ready to arm"
    order = ["link", "loop", "imu", "encl", "encr", "rngf", "rngr", "radio", "pi", "disk", "batt", "fw", "estop"]
    return {"tiles": [tiles[k] for k in order],
            "preflight": {"level": {OK: "green", WARN: "amber", BAD: "red", NA: "red"}[lv], "text": head, "items": pre}}


class Health:
    """Collects what the rules need between ticks (cheap per telemetry frame) and builds the 1 Hz message."""

    def __init__(self, log_dir=".", proc="/proc", thermal="/sys/class/thermal"):
        self.log_dir = log_dir
        self.pi = PiStats(log_dir, proc, thermal)
        self.battery = BatteryModel()
        self.radio = None
        self.radio_t = -1e9
        self.failsafes = 0
        self._prev_radio_ok = None
        self._sum_v = self._sum_load = 0.0
        self._n = 0
        self._log_mb = None
        self._log_mb_t = -1e9

    def on_tlm(self, d, now):
        ok = bool(d["radio_ok"])
        if self._prev_radio_ok and not ok:
            self.failsafes += 1                 # the remote was answering and went quiet
        self._prev_radio_ok = ok
        self._sum_v += d["battery_v"]
        self._sum_load += (abs(d["motor_left"]) + abs(d["motor_right"])) / 2.0
        self._n += 1

    def on_lost(self):
        self._prev_radio_ok = None              # the first frame after a reconnect is not a "remote went quiet"
        self._sum_v = self._sum_load = 0.0
        self._n = 0

    def on_log(self, text, now):
        r = parse_radio(text)
        if r:
            self.radio, self.radio_t = r, now

    def tick(self, now, ctx):
        """Once a second. ctx: link, robot_alive, tlm, tlm_hz, rng_hz, params {name: value}, hello, estop, robot_estop,
        remote_seen, clients, crc_errors, log_error. Returns the `health` message dict."""
        if self._n:
            if ctx.get("robot_alive"):
                self.battery.add(now, self._sum_v / self._n, self._sum_load / self._n)
            self._sum_v = self._sum_load = 0.0
            self._n = 0
        pi = dict(self.pi.read())
        if now - self._log_mb_t >= LOG_USAGE_EVERY_S:
            self._log_mb_t = now
            self._log_mb = round(dir_bytes(self.log_dir) / 1e6)
        pi["log_mb"] = self._log_mb
        low = float((ctx.get("params") or {}).get("batt.low_cell", LOW_CELL_DEFAULT))
        bat = self.battery.estimate(now, CELLS, low)
        c = dict(ctx)
        c.update(pi=pi, battery=bat, failsafes=self.failsafes,
                 radio=self.radio if now - self.radio_t < RADIO_FRESH_S else None)
        out = evaluate(c)
        out.update({"t": "health", "battery": bat, "pi": pi, "failsafes": self.failsafes,
                    "armed": bool(ctx.get("tlm") and ctx["tlm"].get("status", 0) & lp.TLM_ARMED)})
        return out
