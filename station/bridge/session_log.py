"""Session logs: one JSON-lines file per robot connection, everything at full rate.

Line format (one JSON array per line, compact):
  ["schema", {"tlm": [...field names], "rng": [...field names]}]   (first line)
  ["hello", t, {...}]          ["tlm", t, v1, v2, ...]   (values in schema order)
  ["rng", t, v1, ...]          ["param", t, id, name, value, min, max]
  ["nav", t, {...}]            ["log", t, "text"]       ["cmd", t, {...client command}]
  ["bridge", t, "text"]
t = seconds since the session started (Pi clock, 3 decimals).
CSV export of tlm / rng rows: see to_csv().
"""
import json
import os
import re
import time

from . import link_protocol as lp

TLM_COLS = list(lp.TELEMETRY_FIELDS)
RNG_COLS = list(lp.RANGE_FIELDS)
NAME_RE = re.compile(r"^sbr3-\d{8}-\d{6}(-\d+)?\.jsonl$")


def _r(v):
    return round(v, 4) if isinstance(v, float) else v


class SessionLog:
    def __init__(self, log_dir, max_total_mb=2048):
        self.dir = os.path.abspath(os.path.expanduser(log_dir))
        os.makedirs(self.dir, exist_ok=True)
        self.max_bytes = int(max_total_mb * 1024 * 1024)
        self.f = None
        self.name = None
        self.t0 = 0.0
        self.last_flush = 0.0
        self.error = ""       # why logging is off (full / read-only / missing dir), "" when fine

    # -- lifecycle --
    def start(self):
        """Open a new session file. False (and .error set) when the disk will not let us: the bridge then
        runs without logging instead of dying, and tries again later."""
        self.stop()
        try:
            os.makedirs(self.dir, exist_ok=True)
            self._prune()
            base = time.strftime("sbr3-%Y%m%d-%H%M%S")
            name, n = base + ".jsonl", 1
            while os.path.exists(os.path.join(self.dir, name)):
                name = "%s-%d.jsonl" % (base, n)
                n += 1
            f = open(os.path.join(self.dir, name), "w", buffering=64 * 1024, encoding="utf-8")
        except OSError as e:
            self.f, self.name, self.error = None, None, "%s" % (e.strerror or e)
            return False
        self.name, self.f, self.error = name, f, ""
        self.t0 = time.monotonic()
        self._w(["schema", {"tlm": TLM_COLS, "rng": RNG_COLS, "started": time.strftime("%Y-%m-%dT%H:%M:%S")}])
        return self.f is not None

    def stop(self):
        if self.f:
            try:
                self.f.close()
            except OSError:
                pass
        self.f = None

    def _w(self, row):
        if not self.f:
            return
        try:
            self.f.write(json.dumps(row, separators=(",", ":")) + "\n")
            now = time.monotonic()
            if now - self.last_flush > 2.0:
                self.f.flush()
                self.last_flush = now
        except (OSError, ValueError) as e:
            self.f = None  # disk full / removed: stop logging rather than crash
            self.error = "%s" % (getattr(e, "strerror", None) or e)

    def _t(self):
        return round(time.monotonic() - self.t0, 3)

    # -- records --
    def tlm(self, d):
        if self.f:
            self._w(["tlm", self._t()] + [_r(d[c]) for c in TLM_COLS])

    def rng(self, d):
        if self.f:
            self._w(["rng", self._t()] + [_r(d[c]) for c in RNG_COLS])

    def record(self, kind, *vals):
        if self.f:
            self._w([kind, self._t()] + list(vals))

    # -- listing / export --
    def list(self):
        out = []
        try:
            names = os.listdir(self.dir)
        except OSError:
            return out
        for n in names:
            if NAME_RE.match(n):
                try:
                    st = os.stat(os.path.join(self.dir, n))
                except OSError:       # deleted by a prune in another thread
                    continue
                out.append({"name": n, "size": st.st_size, "mtime": int(st.st_mtime),
                            "active": n == self.name and self.f is not None})
        out.sort(key=lambda d: d["name"], reverse=True)
        return out

    def path(self, name):
        if not NAME_RE.match(name or ""):
            return None
        p = os.path.join(self.dir, name)
        return p if os.path.isfile(p) else None

    def prune(self):
        """Delete the oldest sessions while the total is above the cap (the active one is kept).
        Touches the disk a lot: the bridge runs it in a worker thread."""
        files = sorted(self.list(), key=lambda d: d["name"])
        total = sum(f["size"] for f in files)
        for f in files:
            if total <= self.max_bytes:
                break
            if f["active"]:
                continue
            try:
                os.remove(os.path.join(self.dir, f["name"]))
            except OSError:
                pass
            total -= f["size"]

    _prune = prune

    def flush(self):
        if self.f:
            try:
                self.f.flush()
            except OSError:
                pass


def to_csv(path, kind="tlm"):
    """Yield CSV lines (str) for one record kind ("tlm" or "rng") of a session file."""
    cols = TLM_COLS if kind == "tlm" else RNG_COLS
    yield "t_s," + ",".join(cols) + "\n"
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.startswith('["%s"' % kind):
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue  # last line of a live file may be partial
            yield ",".join(str(v) for v in row[1:]) + "\n"
