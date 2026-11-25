"""Cheap per-component timing of the bridge's event-loop work (issue #102), always on.

Every piece of loop work runs inside `with prof.part("name"):`; the time is exclusive (a part nested in another is
subtracted from its parent, so the numbers add up to the loop's busy time). Two windows:
- `since`: since the last tick (Bridge.tick_task). A tick that comes more than STALL_LOG_S late logs one line naming
  the top 3 parts of that gap (`stall_line`), so a stall on the robot is diagnosable without a profiler;
- WINDOW_S (10 s): busy %, the top parts (ms per second) and the longest stall, for `status["loop"]` (Health, Pi tile).
Work in worker threads (planner jobs, autosave) is counted with `thread()`: it is not loop time, but it competes for the
GIL, so the stall line names it when it was running.
The garbage collector's pauses count as the part "gc" (install_gc(); a full collection of a big heap is a stall too).
Cost: two perf_counter() calls and a few dict operations per part (~1 us on a PC, ~10 us on the Pi Zero 2W).
"""
import gc
import threading
import time

WINDOW_S = 10.0
STALL_LOG_S = 0.15         # a tick this late logs the top offenders (Bridge.tick_task)
TOP_N = 3


class _Part:
    __slots__ = ("p", "name")

    def __init__(self, p, name):
        self.p, self.name = p, name

    def __enter__(self):
        self.p._stack.append([self.name, self.p.clock(), 0.0])
        return self

    def __exit__(self, *exc):
        p = self.p
        name, t0, child = p._stack.pop()
        dt = p.clock() - t0
        p._add(name, dt - child)
        if p._stack:
            p._stack[-1][2] += dt
        elif dt > p.win_chunk:              # a top-level part: one piece of loop work nothing could interrupt
            p.win_chunk = dt
            if dt > p.chunk_peak:
                p.chunk_peak = dt
            if dt > p.chunk_peaks.get(name, 0.0):
                p.chunk_peaks[name] = dt
        return False


class _Null:
    __slots__ = ()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


_NULL_PART = _Null()


class NullProf:
    """For modules used without a bridge (tests, tools): parts cost nothing."""

    def part(self, name):
        return _NULL_PART

    def thread(self, name):
        return _NULL_PART


NULL = NullProf()


class _ThreadPart:
    __slots__ = ("p", "name", "t0")

    def __init__(self, p, name):
        self.p, self.name = p, name

    def __enter__(self):
        self.t0 = self.p.clock()
        with self.p._lock:
            self.p._threads[self.name] = self.p._threads.get(self.name, 0) + 1
        return self

    def __exit__(self, *exc):
        dt = self.p.clock() - self.t0
        with self.p._lock:
            self.p._threads[self.name] -= 1
            self.p.thread_s[self.name] = self.p.thread_s.get(self.name, 0.0) + dt
            self.p.thread_ran.add(self.name)
        return False


class LoopProf:
    def __init__(self, clock=time.perf_counter, window_s=WINDOW_S):
        self.clock = clock
        self.window_s = window_s
        self._stack = []
        self._parts = {}
        self.since = {}                 # name -> s since the last tick
        self.win = {}                   # name -> s in the current window
        self.win_t0 = None
        self.win_stall = 0.0            # longest stall (s) in the current window
        self.win_chunk = 0.0            # longest top-level part (s) in the current window
        self.chunk_peak = 0.0           # ... since the start
        self.chunk_peaks = {}           # top-level part -> its longest run (s) since the start (when it was a window's longest)
        self.last = {"busy_pct": 0.0, "stall_max_ms": 0, "chunk_max_ms": 0.0, "top": [], "threads": {}}   # last window
        self.stall_total = 0            # ticks later than STALL_LOG_S since the start
        self._lock = threading.Lock()
        self._threads = {}              # name -> jobs running now (worker threads)
        self.thread_s = {}              # name -> s of worker-thread time in the current window
        self.thread_ran = set()         # names that ran (or still run) since the last tick

    def install_gc(self):
        """Time the garbage collector's pauses on the loop's thread as the part "gc" (taken out of the part it hit)."""
        if getattr(self, "_gc_cb", None):
            return
        main = threading.get_ident()
        t = [0.0]

        def cb(phase, info):
            if threading.get_ident() != main:
                return
            if phase == "start":
                t[0] = self.clock()
            else:
                dt = self.clock() - t[0]
                self._add("gc", dt)
                if self._stack:
                    self._stack[-1][2] += dt
        self._gc_cb = cb
        gc.callbacks.append(cb)

    def uninstall_gc(self):
        cb = getattr(self, "_gc_cb", None)
        if cb in gc.callbacks:
            gc.callbacks.remove(cb)
        self._gc_cb = None

    def part(self, name):
        p = self._parts.get(name)
        if p is None:
            p = self._parts[name] = _Part(self, name)
        return p

    def thread(self, name):
        """Worker-thread work (not loop time): `with prof.thread("planner.job"):` inside the job."""
        return _ThreadPart(self, name)

    def _add(self, name, dt):
        s = self.since
        s[name] = s.get(name, 0.0) + dt
        w = self.win
        w[name] = w.get(name, 0.0) + dt

    # ------------------------------------------------------------ ticks
    @staticmethod
    def top(d, n=TOP_N):
        return sorted(d.items(), key=lambda kv: -kv[1])[:n]

    def tick(self, now, late):
        """Once per Bridge tick, before its work: late = how much later than planned it came (s). Returns the stall
        line to log (late > STALL_LOG_S) or None; starts a new `since`."""
        line = None
        if late > STALL_LOG_S:
            self.stall_total += 1
            parts = ", ".join("%s %d ms" % (k, round(v * 1000)) for k, v in self.top(self.since) if v >= 0.0005)
            with self._lock:
                th = sorted(n for n in self.thread_ran if n)
            line = "WARNING: event loop stalled for %d ms (top: %s%s)" % (
                round(late * 1000), parts or "no timed part: untimed work or the machine was busy",
                "; worker thread running: " + ", ".join(th) if th else "")
        if late > self.win_stall:
            self.win_stall = late
        self.since = {}
        with self._lock:
            self.thread_ran = {n for n, k in self._threads.items() if k > 0}
        if self.win_t0 is None:
            self.win_t0 = now
        elif now - self.win_t0 >= self.window_s:
            dt = now - self.win_t0
            with self._lock:
                th = {k: round(100.0 * v / dt, 1) for k, v in self.thread_s.items() if v > 0}
                self.thread_s = {}
            self.last = {"busy_pct": round(100.0 * sum(self.win.values()) / dt, 1),
                         "stall_max_ms": int(round(self.win_stall * 1000)),
                         "chunk_max_ms": round(self.win_chunk * 1000, 1),
                         "top": [[k, round(1000.0 * v / dt, 1)] for k, v in self.top(self.win, 5)],
                         "threads": th}
            self.win, self.win_t0, self.win_stall, self.win_chunk = {}, now, 0.0, 0.0
        return line

    def status(self):
        """For status["loop"]: the last 10 s window (stall_max_ms also covers the window running now).
        busy_pct = event-loop work in % of one core; chunk_max_ms = the longest single piece of timed loop work;
        top = [[part, ms per s], ...]; threads = worker-thread % of a core; stalls = ticks later than STALL_LOG_S so far."""
        out = dict(self.last)
        out["stall_max_ms"] = max(out["stall_max_ms"], int(round(self.win_stall * 1000)))
        out["stalls"] = self.stall_total
        return out
