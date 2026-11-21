"""Tune page tools in the bridge (issue #16): presets, scored runs, wheels-up motor test.

Browser commands (WebSocket or POST /api/cmd), replies go to the sender, state changes to everyone:
  {"t":"preset","action":"list"}                                 -> {"t":"presets","list":[...]}
  {"t":"preset","action":"save","name":"...","note":"..."}       current tuning values -> ~/sbr3-presets
  {"t":"preset","action":"delete","name":"..."}
  {"t":"preset","action":"plan","name":"..."}                    -> {"t":"preset_plan","changes":[...]} (dry run)
  {"t":"preset","action":"apply","name":"...","confirm":true}    set_param per changed value (confirm needed while armed)
  {"t":"preset","action":"diff","a":"...","b":"..."}             -> {"t":"preset_diff","rows":[...]} ("" = live values)
  {"t":"score","action":"start"|"stop"|"cancel"}                 mark a window; stop scores it
  {"t":"score","action":"last","seconds":20}                     score the last N s now
  {"t":"score","action":"clear"}                                 empty the run history
  {"t":"motor","action":"start","test":1-4,"confirm":true}       mot.test = N (the robot's own interlocks decide)
  {"t":"motor","action":"cancel"}                                mot.test = 0
  {"t":"motor","action":"fit"[, "session":"sbr3-....jsonl"]}     -> {"t":"motor_fit", ...} (this session's lines by default)
  {"t":"motor","action":"apply","params":{"mot.dz_l_fwd":5.7,...}}   only mot.* proposal params (never test/drive_mode)
  {"t":"balpoint","action":"start"|"apply"|"revert"|"stop"|"status"}  Find balance point (#52): see balance_point.py
Pushed: score_state, score_runs, motor_state, presets, balpoint_state.
"""
import asyncio
import collections
import json
import os
import time

from . import link_protocol as lp
from . import motor_fit, presets, scoring
from .balance_point import BalancePoint

RUNS_MAX = 60
MT_LINES_MAX = 3000
PARAM_HIST_MAX = 4000
MOTOR_APPLY_OK = frozenset(("mot.model", "mot.vcomp", "mot.v_ref", "mot.dz_l_fwd", "mot.dz_l_rev", "mot.dz_r_fwd",
                            "mot.dz_r_rev", "mot.dz_band", "mot.left_scale", "mot.right_scale", "mot.k_l", "mot.k_r"))
LAST_S_MAX = 300


class TuningTools:
    def __init__(self, bridge, preset_dir="~/sbr3-presets"):
        self.b = bridge
        self.store = presets.PresetStore(preset_dir)
        self.runs = collections.deque(maxlen=RUNS_MAX)
        self.run_next = 1
        self.mark = None               # {"path", "session", "t0", "params"} while a window is open
        self.busy = False
        self.mt_items = []             # ("log", t, text) of this session
        self.mt = motor_fit.LiveState()
        self.param_hist = collections.deque(maxlen=PARAM_HIST_MAX)   # (session, t, name, old, new)
        self.balpoint = BalancePoint(bridge)                           # Find balance point (#52)

    # ------------------------------------------------------------------ hooks from the bridge
    def on_session(self):
        """A new session log (robot (re)connected)."""
        self.mt_items = []
        self.mt.reset()
        self._push_motor()

    def on_log(self, text):
        if not text.startswith("mt,"):
            return
        t = self._t()
        self.mt_items.append(("log", t, text))
        if len(self.mt_items) > MT_LINES_MAX:
            del self.mt_items[:len(self.mt_items) - MT_LINES_MAX]
        if self.mt.feed(text, t):
            self._push_motor()

    def param_changed(self, name, old, new):
        if old is None or presets._same(old, new):
            return
        self.param_hist.append((self.b.log.name, self._t(), name, old, new))

    def on_tlm(self, d, now):
        self.balpoint.on_tlm(d, now)

    def tick(self, now):
        self.balpoint.tick(now)

    def on_lost(self):
        self.balpoint.on_lost()

    def hello_msgs(self):
        return [self._state_msg(), self._runs_msg(), self._motor_msg(), self._presets_msg(), self.balpoint.msg()]

    # ------------------------------------------------------------------ helpers
    def _t(self):
        return round(time.monotonic() - self.b.log.t0, 3) if self.b.log.f else None

    def _params_by_name(self):
        return {p["name"]: p for p in self.b.params.values()}

    def _live_values(self):
        return presets.snapshot(self.b.params)

    def _armed(self):
        t = self.b.tlm
        return bool(t and t["status"] & lp.TLM_ARMED and self.b.tlm_time > time.monotonic() - 2.0)

    def _set(self, name, value, who):
        p = self._params_by_name().get(name)
        if p is None:
            return False
        v = max(p["min"], min(p["max"], float(value)))
        self.b.send(lp.encode_set_param(p["id"], v))
        self.param_changed(name, p["value"], v)
        p["value"] = v
        self.b.params_dirty = True
        return True

    def _broadcast(self, d):
        self.b.broadcast(json.dumps(d, separators=(",", ":")))

    def _spawn(self, coro):
        try:
            asyncio.get_running_loop().create_task(coro)
        except RuntimeError:      # no loop (unit tests calling handle directly)
            asyncio.run(coro)

    # ------------------------------------------------------------------ commands
    def handle(self, m, who):
        t, action = m.get("t"), m.get("action")
        try:
            if t == "preset":
                return self._preset(m, action, who)
            if t == "score":
                return self._score(m, action, who)
            if t == "motor":
                return self._motor(m, action, who)
            if t == "balpoint":
                return self.balpoint.command(m, who)
        except ValueError as e:
            return {"t": "error", "text": str(e)}
        return {"t": "error", "text": "unknown command %r" % t}

    # ---- presets
    def _presets_msg(self):
        return {"t": "presets", "list": self.store.list()}

    def _preset(self, m, action, who):
        name = str(m.get("name") or "")
        if action == "list":
            return self._presets_msg()
        if action == "save":
            d = self.store.save(name, self._live_values(), m.get("note", ""))
            self.b.say("%s: preset '%s' saved (%d values)" % (who, d["name"], len(d["values"])))
            self._broadcast(self._presets_msg())
            return {"t": "ok", "text": "saved '%s'" % d["name"]}
        if action == "delete":
            self.store.delete(name)
            self.b.say("%s: preset '%s' deleted" % (who, name))
            self._broadcast(self._presets_msg())
            return {"t": "ok"}
        if action == "diff":
            vals = []
            for key in ("a", "b"):
                n = m.get(key) or ""
                if not n:
                    vals.append(self._live_values())
                else:
                    p = self.store.get(n)
                    if not p:
                        raise ValueError("no preset '%s'" % n)
                    vals.append(p["values"])
            order = [p["name"] for p in sorted(self.b.params.values(), key=lambda p: p["id"])]
            return {"t": "preset_diff", "a": m.get("a") or "", "b": m.get("b") or "",
                    "rows": presets.diff(vals[0], vals[1], order)}
        if action in ("plan", "apply"):
            p = self.store.get(name)
            if not p:
                raise ValueError("no preset '%s'" % name)
            if not self.b.params:
                raise ValueError("no parameters from the robot yet")
            plan = presets.plan_apply(p["values"], self.b.params)
            if action == "plan":
                return dict(plan, t="preset_plan", name=name, armed=self._armed())
            if self._armed() and not m.get("confirm"):
                return {"t": "error", "text": "the robot is armed: confirm to apply '%s'" % name, "need_confirm": True}
            for c in plan["changes"]:
                self._set(c["name"], c["to"], who)
            self.b.say("%s: preset '%s' applied (%d change%s)" % (who, name, len(plan["changes"]),
                                                                   "" if len(plan["changes"]) == 1 else "s"))
            return dict(plan, t="preset_applied", name=name)
        raise ValueError("bad preset action %r" % action)

    # ---- scored runs
    def _state_msg(self):
        mk = self.mark
        return {"t": "score_state", "running": mk is not None, "t0": mk["t0"] if mk else None,
                "now": self._t(), "busy": self.busy, "session": self.b.log.name if self.b.log.f else None}

    def _runs_msg(self):
        return {"t": "score_runs", "runs": list(self.runs), "metrics": scoring.METRICS}

    def _params_at(self, session, t):
        """Tuning values at session time t: live values with later changes undone."""
        vals = self._live_values()
        for s, tt, name, old, new in reversed(self.param_hist):
            if s == session and tt is not None and tt > t and name in vals:
                vals[name] = old
        return vals

    def _changes_in(self, session, t0, t1):
        return [{"t": round(tt - t0, 2), "name": n, "from": o, "to": v} for s, tt, n, o, v in self.param_hist
                if s == session and tt is not None and t0 <= tt <= (t1 if t1 is not None else 1e18)
                and presets.tunable(n)]

    def _score(self, m, action, who):
        log = self.b.log
        if action == "clear":
            self.runs.clear()
            self._broadcast(self._runs_msg())
            return {"t": "ok"}
        if action == "cancel":
            self.mark = None
            self._broadcast(self._state_msg())
            return {"t": "ok"}
        if not log.f:
            raise ValueError("no session log yet (robot not connected)")
        path = os.path.join(log.dir, log.name)
        if action == "start":
            self.mark = {"path": path, "session": log.name, "t0": self._t(), "params": self._live_values()}
            self._broadcast(self._state_msg())
            return {"t": "ok"}
        if self.busy:
            raise ValueError("still scoring the previous run")
        if action == "stop":
            mk = self.mark
            if not mk:
                raise ValueError("no run started")
            self.mark = None
            t1 = self._t() if mk["session"] == log.name else None
            params, label = mk["params"], "marked"
            job = (mk["path"], mk["session"], mk["t0"], t1, None)
        elif action == "last":
            secs = max(2.0, min(LAST_S_MAX, float(m.get("seconds") or 20)))
            t1 = self._t()
            params, label = self._params_at(log.name, t1 - secs), "last %g s" % secs
            job = (path, log.name, None, None, secs)
        else:
            raise ValueError("bad score action %r" % action)
        self.busy = True
        self._broadcast(self._state_msg())
        self._spawn(self._score_job(job, params, label, who))
        return {"t": "ok"}

    async def _score_job(self, job, params, label, who):
        path, session, t0, t1, last_s = job
        self.b.log.flush()
        loop = asyncio.get_running_loop()

        def work():
            rows, _ = scoring.read_window(path, t0=t0, t1=t1, last_s=last_s)
            return rows
        try:
            rows = await loop.run_in_executor(None, work)
            ta = rows[0][0] if rows else (t0 or 0.0)
            tb = rows[-1][0] if rows else (t1 or ta)
            s = scoring.score(rows, round(tb - ta, 1))
            run = {"id": self.run_next, "label": label, "session": session, "time": time.strftime("%H:%M:%S"),
                   "t0": ta, "t1": tb, "seconds": round(tb - ta, 1), "score": s, "params": params,
                   "changed": self._changes_in(session, ta, tb)}
            self.run_next += 1
            self.runs.append(run)
            self.b.log.record("score", run)
            if s["ok"]:
                self.b.say("%s: run %d scored: pitch err rms %.2f deg, osc %.1f Hz, motor rms %.2f, %d push(es)" % (
                    who, run["id"], s["err_rms"], s["osc_hz"], s["motor_rms"], s["push_n"]))
            else:
                self.b.say("%s: run %d: %s" % (who, run["id"], s.get("reason")))
        except Exception as e:   # bad file etc.: tell the UI, keep the bridge running
            self.b.say("score failed: %s" % e)
        finally:
            self.busy = False
            self._broadcast(self._state_msg())
            self._broadcast(self._runs_msg())

    # ---- motor test
    def _motor_msg(self):
        return dict(self.mt.to_dict(), t="motor_state", now=self._t(), last=self.mt.last)

    def _push_motor(self):
        self._broadcast(self._motor_msg())

    def _motor(self, m, action, who):
        if action == "start":
            n = int(m.get("test") or 0)
            if n not in (1, 2, 3, 4):
                raise ValueError("test 1-4")
            if not m.get("confirm"):
                return {"t": "error", "text": "confirm the motor test (wheels off the ground)", "need_confirm": True}
            if not self._set("mot.test", n, who):
                raise ValueError("robot has no mot.test param (firmware too old?)")
            self.b.say("%s: motor test %d requested (wheels up)" % (who, n))
            return {"t": "ok"}
        if action == "cancel":
            self._set("mot.test", 0, who)
            self.b.say("%s: motor test cancelled" % who)
            return {"t": "ok"}
        if action == "fit":
            self._spawn(self._fit_job(m.get("session")))
            return {"t": "ok"}
        if action == "apply":
            vals = m.get("params") or {}
            bad = [k for k in vals if k not in MOTOR_APPLY_OK]
            if bad:
                raise ValueError("not a motor-fit param: %s" % ", ".join(bad))
            if self._armed() and not m.get("confirm"):
                return {"t": "error", "text": "the robot is armed: confirm to apply", "need_confirm": True}
            plan = presets.plan_apply(vals, self.b.params)
            for c in plan["changes"]:
                self._set(c["name"], c["to"], who)
            self.b.say("%s: motor fit applied: %s" % (who, ", ".join("%s=%g" % (c["name"], c["to"]) for c in plan["changes"]) or "no changes"))
            return dict(plan, t="motor_applied")
        raise ValueError("bad motor action %r" % action)

    async def _fit_job(self, session):
        loop = asyncio.get_running_loop()
        live = {p["name"]: p["value"] for p in self.b.params.values()}
        try:
            if session:
                path = self.b.log.path(session)
                if not path:
                    raise ValueError("no such session")
                items = await loop.run_in_executor(None, motor_fit.read_file, path)
                src = session
            else:
                items, src = list(self.mt_items), (self.b.log.name or "") + " (live)"
            out = await loop.run_in_executor(None, lambda: motor_fit.fit_items(items, params=live))
            out.update(t="motor_fit", source=src, session=session or None)
        except Exception as e:
            out = {"t": "motor_fit", "error": str(e), "source": session, "runs": [], "results": []}
        self._broadcast(out)
