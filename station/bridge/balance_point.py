"""Find balance point (issue #52): measure how far bal.offset_deg is from the true balance point.

While the robot balances still (BALANCE or HOLD, nobody touching it) the velocity loop has to lean it by a
steady angle to stop it creeping. That lean is the error of the balance point:

    robot target pitch = vel_out - offset          (src/controller.cpp)
    true balance point = pitch of a robot that stands still  =>  vel_out_mean = offset - true_offset
    new offset         = offset - mean(vel_out_deg)

The mean of vel_out_deg over a window (after a settle time: the velocity integrator needs a few seconds) is the
measurement. The bridge only reads telemetry and sets ONE param (bal.offset_deg); it never arms, never changes
the mode and never drives. Anything that moves the robot (push, cmd_vel, mode change, disarm, e-stop, link lost,
someone else changing the offset) aborts the run at once, leaving the offset at whatever was last set.

Browser commands (WebSocket or POST /api/cmd); the state is pushed to everyone as {"t":"balpoint_state", ...}:
  {"t":"balpoint","action":"start"[,"auto":true,"settle_s":6,"window_s":10,"rounds":3,"tol_deg":0.1]}
        measure (settle + window). Default: stop at a PROPOSAL and wait for "apply". auto: apply each correction
        itself and measure again, up to `rounds` rounds.
  {"t":"balpoint","action":"apply"}      set the proposed offset (slewed, ~4 deg/s), then measure again to check it
  {"t":"balpoint","action":"revert"}     back to the offset the run started with
  {"t":"balpoint","action":"stop"}       end the run (offset stays where it is)
  {"t":"balpoint","action":"status"}     -> the state message
Saving is the existing {"t":"cfg","action":"save"} (EEPROM write, only with the motors off).
"""
import json
import math
import time

from . import link_protocol as lp

OFFSET = "bal.offset_deg"
DEFAULTS = {"settle_s": 6.0, "window_s": 10.0, "rounds": 3, "tol_deg": 0.1, "max_step_deg": 2.0}
LIMITS = {"settle_s": (2.0, 30.0), "window_s": (4.0, 60.0), "rounds": (1, 5), "tol_deg": (0.02, 1.0),
          "max_step_deg": (0.2, 4.0)}
MIN_RATE_HZ = 8.0            # samples per second needed in a window (telemetry may be as slow as 25 Hz)
START_SPEED_MMS = 80.0       # "standing still" to start
ABORT_SPEED_MMS = 250.0      # driven / pushed while running
DRIFT_MMS = 60.0             # window mean speed above this: not standing still (slope, hand holding, ...)
ABORT_LEAN_ERR_DEG = 8.0     # |pitch - target| above this: pushed
START_LEAN_ERR_DEG = 6.0
BAL_I_WARN = 600.0           # |mean bal_i| (motor counts): the balance integrator is carrying a load
UNSETTLED_DEG = 0.4          # first half vs second half of the window differ more: still settling
MAX_EXTRA_WINDOWS = 2
SLEW_DEG_PER_S = 4.0         # how fast the offset is moved (a step would shove the robot)
PROPOSAL_TIMEOUT_S = 180.0
TLM_STALE_S = 1.0
PUSH_HZ = 4.0
HOLD_CMD_MMS = 60.0

ACTIVE = ("settle", "measure", "slew")


class Estimator:
    """Samples of the velocity loop over a window -> the lean the robot needs to stand still."""

    def __init__(self):
        self.v, self.speed, self.bal_i, self.trim = [], [], [], []
        self.t0 = self.t1 = None

    def add(self, t, vel_out, speed, bal_i, trim=0.0):
        if self.t0 is None:
            self.t0 = t
        self.t1 = t
        self.v.append(vel_out)
        self.speed.append(speed)
        self.bal_i.append(bal_i)
        self.trim.append(trim)

    @property
    def n(self):
        return len(self.v)

    def mean(self):
        return sum(self.v) / len(self.v) if self.v else 0.0

    def stats(self):
        n = len(self.v)
        if not n:
            return {"n": 0, "dur": 0.0, "mean": 0.0, "std": 0.0, "first": 0.0, "second": 0.0, "speed": 0.0,
                    "speed_max": 0.0, "bal_i": 0.0, "trim": 0.0}
        m = sum(self.v) / n
        h = n // 2 or 1
        return {"n": n, "dur": (self.t1 - self.t0) if self.t0 is not None else 0.0, "mean": m,
                "std": math.sqrt(sum((x - m) ** 2 for x in self.v) / n),
                "first": sum(self.v[:h]) / h, "second": sum(self.v[h:]) / max(1, n - h),
                "speed": sum(self.speed) / n, "speed_max": max(abs(x) for x in self.speed),
                "bal_i": sum(self.bal_i) / n, "trim": sum(self.trim) / n}


def propose(offset, mean_vel_out, max_step=2.0, lo=-15.0, hi=15.0, gain=1.0):
    """The offset that cancels the lean: offset - mean(vel_out), at most max_step away, inside the param range."""
    delta = max(-max_step, min(max_step, -gain * mean_vel_out))
    return round(max(lo, min(hi, offset + delta)), 3)


class BalancePoint:
    def __init__(self, bridge):
        self.b = bridge
        self.state = "idle"        # idle | settle | measure | slew | proposal | done | aborted
        self.text = ""
        self.auto = False
        self.cfg = dict(DEFAULTS)
        self.round = 0
        self.history = []          # [{"round", "offset", "mean", "std"}]
        self.orig = None           # offset when the run started
        self.expect = None         # offset we last set (someone else changing it aborts the run)
        self.target = None         # slew target
        self.after_slew = "settle"
        self.proposal = None
        self.extra = 0
        self.est = Estimator()
        self.t_phase = 0.0
        self.t_push = 0.0
        self.t_slew = None
        self.last = None           # stats of the last finished window
        self.warn = []
        self.who = "?"

    # ------------------------------------------------------------------ helpers
    def _param(self, name):
        return next((p for p in self.b.params.values() if p["name"] == name), None)

    def _offset(self):
        p = self._param(OFFSET)
        return p["value"] if p else None

    def _say(self, text):
        self.b.say("balance point: " + text)

    def active(self):
        return self.state in ACTIVE or self.state == "proposal"

    def _fresh(self, now):
        return bool(self.b.tlm) and now - self.b.tlm_time < TLM_STALE_S and self.b.link_state == "connected"

    def why_not(self, now, d=None):
        """None when the robot is in a state to measure, else a short reason (for the user)."""
        if self._offset() is None:
            return "The robot has no bal.offset_deg parameter yet"
        if not self._fresh(now):
            return "Robot not responding"
        d = d or self.b.tlm
        if getattr(self.b, "estop", False):
            return "E-stop is active"
        if not d["status"] & lp.TLM_ARMED:
            return "Arm and balance the robot first"
        if d["mode"] not in (lp.MODE_BALANCE, lp.MODE_HOLD):
            return "Needs BALANCE or HOLD mode (it is %s)" % lp.MODE_NAMES[d["mode"]]
        if not d["status"] & lp.TLM_UPRIGHT:
            return "The robot is not upright"
        loops = self._param("ctl.loops")
        if loops and not int(round(loops["value"])) & 2:
            return "The velocity loop is off (ctl.loops): it is what shows the lean"
        if getattr(self.b, "cmd_active", False):
            return "Someone is driving the robot"
        lim = HOLD_CMD_MMS if d["mode"] == lp.MODE_HOLD else 5.0
        if abs(d["cmd_speed_mms"]) > lim or abs(d["cmd_turn_dps"]) > 5.0:
            return "The robot is being driven"
        if abs(d["speed_mms"]) > START_SPEED_MMS:
            return "The robot is moving: let it stand still, hands off"
        if abs(d["pitch_deg"] - d["balance_target_deg"]) > START_LEAN_ERR_DEG:
            return "The robot is leaning hard: let it settle first"
        return None

    def _live_problem(self, d, soft=False):
        """Checks every telemetry frame while a run is going: a reason to abort, or None.
        soft (proposal waiting / offset moving): only 'still armed, in BALANCE or HOLD, upright'."""
        if not d["status"] & lp.TLM_ARMED or d["mode"] not in (lp.MODE_BALANCE, lp.MODE_HOLD):
            return "The robot was disarmed or changed mode"
        if not d["status"] & lp.TLM_UPRIGHT:
            return "The robot fell over"
        if soft:
            return None
        if abs(d["speed_mms"]) > ABORT_SPEED_MMS:
            return "The robot was pushed or driven"
        if abs(d["pitch_deg"] - d["balance_target_deg"]) > ABORT_LEAN_ERR_DEG:
            return "The robot was pushed"
        return None

    # ------------------------------------------------------------------ messages
    def msg(self, now=None):
        now = now if now is not None else time.monotonic()
        running = self.state in ("settle", "measure")
        live = self.est.mean() if self.state == "measure" and self.est.n else None
        cur = self._offset()
        total = self.cfg["window_s"] if self.state == "measure" else self.cfg["settle_s"]
        return {"t": "balpoint_state", "state": self.state, "text": self.text, "auto": self.auto,
                "round": self.round, "rounds": self.cfg["rounds"], "offset": cur, "orig": self.orig,
                "proposal": self.proposal, "history": self.history, "live": live,
                "phase": "measure" if self.state == "measure" else ("settle" if self.state == "settle" else None),
                "elapsed": round(now - self.t_phase, 1) if running else None,
                "length": total if running else None, "tol": self.cfg["tol_deg"],
                "last": ({k: round(v, 3) if isinstance(v, float) else v for k, v in self.last.items()}
                         if self.last else None),
                "warn": self.warn,
                "ready": self.why_not(now) if self.state in ("idle", "done", "aborted") and self.b.tlm else None}

    def push(self, now=None):
        self.t_push = now if now is not None else time.monotonic()
        self.b.broadcast(json.dumps(self.msg(now), separators=(",", ":")))

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
        if action == "apply":
            if self.state not in ("proposal", "done") or self.proposal is None:
                raise ValueError("nothing to apply")
            why = self.why_not(now)
            if why:
                raise ValueError(why)
            self._begin_slew(self.proposal, "settle" if self.round < self.cfg["rounds"] else "done", now)
            return {"t": "ok"}
        if action == "revert":
            if self.orig is None or self.state in ("settle", "measure", "slew"):
                raise ValueError("nothing to undo")
            if self._offset() is not None and abs(self._offset() - self.orig) > 1e-6:
                self._begin_slew(self.orig, "reverted", now)
            else:
                self._finish_idle("Nothing to undo", now)
            return {"t": "ok"}
        raise ValueError("bad balpoint action %r" % action)

    def _start(self, m, who, now):
        if self.state in ("settle", "measure", "slew", "proposal"):
            raise ValueError("already running")
        why = self.why_not(now)
        if why:
            raise ValueError(why)
        cfg = dict(DEFAULTS)
        for k in DEFAULTS:
            if m.get(k) is not None:
                lo, hi = LIMITS[k]
                v = float(m[k])
                if not math.isfinite(v):
                    raise ValueError("bad %s" % k)
                cfg[k] = max(lo, min(hi, v))
        cfg["rounds"] = int(round(cfg["rounds"]))
        self.cfg, self.auto = cfg, bool(m.get("auto"))
        self.round, self.history, self.proposal, self.last = 0, [], None, None
        self.orig = self.expect = self._offset()
        self.warn = []
        self._say("%s: finding the balance point from %.2f deg (%s, %g s settle + %g s window)" % (
            who, self.orig, "automatic" if self.auto else "you confirm each change", cfg["settle_s"], cfg["window_s"]))
        self._begin_round(now)
        return {"t": "ok"}

    # ------------------------------------------------------------------ state machine
    def _begin_round(self, now):
        self.round += 1
        self.extra = 0
        self._begin_settle(now)

    def _begin_settle(self, now):
        self.state, self.t_phase = "settle", now
        self.est = Estimator()
        self.text = "Standing still, letting the velocity loop settle"
        self.push(now)

    def _begin_measure(self, now):
        self.state, self.t_phase = "measure", now
        self.est = Estimator()
        self.text = "Measuring the lean"
        self.push(now)

    def _begin_slew(self, target, after, now):
        self.target, self.after_slew = target, after
        self.proposal = None
        self.state, self.t_slew = "slew", now
        self.text = "Moving the balance point to %.2f deg" % target
        self._say("%s: offset %.2f -> %.2f deg" % (self.who, self._offset(), target))
        self.push(now)

    def _finish_idle(self, text, now):
        self.state, self.text, self.proposal = "idle", text, None
        self.push(now)

    def _abort(self, reason, now):
        cur = self._offset()
        changed = self.orig is not None and cur is not None and abs(cur - self.orig) > 0.004
        self.state, self.proposal = "aborted", None
        self.text = reason + (" - offset left at %.2f deg" % cur if changed else " - offset not changed")
        self._say(self.text)
        self.push(now)

    def _finish(self, text, now):
        self.state, self.text = "done", text
        self._say(text)
        self.push(now)

    def on_tlm(self, d, now):
        if self.state not in ACTIVE and self.state != "proposal":
            return
        bad = self._live_problem(d, soft=self.state in ("slew", "proposal"))
        if bad:
            return self._abort(bad, now)
        if self.state != "measure":
            return
        off = self._offset()
        # the knob trim adds to the offset in the firmware: balance_target = vel_out - (offset + trim)
        trim = (d["vel_out_deg"] - d["balance_target_deg"]) - off if off is not None else 0.0
        self.est.add(now, d["vel_out_deg"], d["speed_mms"], d["bal_i"], trim)

    def on_lost(self):
        if self.active():
            self._abort("Lost the robot", time.monotonic())

    def tick(self, now):
        st = self.state
        if st in ("idle", "done", "aborted"):
            return
        if not self._fresh(now):
            return self._abort("Lost the robot", now)
        cur = self._offset()
        if st != "slew" and self.expect is not None and cur is not None and abs(cur - self.expect) > 0.01:
            return self._abort("The balance point was changed by someone else", now)
        why = self._driven()
        if why:
            return self._abort(why, now)
        if st == "slew":
            return self._tick_slew(now, cur)
        if st == "proposal":
            if now - self.t_phase > PROPOSAL_TIMEOUT_S:
                self._abort("Proposal expired", now)
            return
        if st == "settle":
            if now - self.t_phase >= self.cfg["settle_s"]:
                self._begin_measure(now)
        elif st == "measure":
            if now - self.t_phase >= self.cfg["window_s"]:
                self._evaluate(now)
        if now - self.t_push >= 1.0 / PUSH_HZ and self.state in ("settle", "measure"):
            self.push(now)

    def _driven(self):
        if getattr(self.b, "cmd_active", False):
            return "Someone started driving the robot"
        if getattr(self.b, "estop", False):
            return "E-stop"
        return None

    def _tick_slew(self, now, cur):
        dt = max(0.0, min(0.2, now - (self.t_slew or now)))
        self.t_slew = now
        if cur is None:
            return self._abort("Lost the robot", now)
        step = SLEW_DEG_PER_S * max(dt, 0.02)
        nxt = cur + max(-step, min(step, self.target - cur))
        if abs(self.target - nxt) < 1e-4:
            nxt = self.target
        self._set(nxt)
        self.expect = nxt
        if nxt != self.target:
            return
        if self.after_slew == "settle":
            self._begin_round(now)
        elif self.after_slew == "reverted":
            self.orig = None
            self._finish_idle("Back to %.2f deg" % self.target, now)
        else:   # "done": last allowed change, not measured again
            self._finish("Balance point set to %.2f deg (not re-checked)" % self.target, now)

    def _set(self, value):
        p = self._param(OFFSET)
        self.b.handle_command({"t": "set_param", "id": p["id"], "value": round(value, 3)}, "balpoint")

    def _evaluate(self, now):
        s = self.est.stats()
        off = self._offset()
        if s["n"] < MIN_RATE_HZ * self.cfg["window_s"]:
            return self._abort("Not enough telemetry (%d samples): raise tlm.hz" % s["n"], now)
        if abs(s["first"] - s["second"]) > UNSETTLED_DEG and self.extra < MAX_EXTRA_WINDOWS:
            self.extra += 1                           # still settling: measure another window
            self._say("lean still moving (%+.2f -> %+.2f deg): measuring again" % (s["first"], s["second"]))
            return self._begin_measure(now)
        if abs(s["first"] - s["second"]) > UNSETTLED_DEG:
            s["mean"] = s["second"]                    # out of patience: the later half is the better guess
        if abs(s["speed"]) > DRIFT_MMS:
            return self._abort("The robot is drifting (%.0f mm/s): put it on flat ground" % s["speed"], now)
        lim = self._param("vel.limit_deg")
        if lim and abs(s["mean"]) > 0.9 * lim["value"]:
            return self._abort("Lean %+.1f deg hits the velocity limit: the offset is far off" % s["mean"], now)
        warn = []
        if abs(s["trim"]) > 0.05:
            warn.append("Knob trim is %+.2f deg: centre the left knob for a clean result" % s["trim"])
        if abs(s["bal_i"]) > BAL_I_WARN:
            warn.append("Balance integrator carries %.0f counts: uneven motors or a slope?" % s["bal_i"])
        self.warn, self.last = warn, s
        self.history.append({"round": self.round, "offset": round(off, 3), "mean": round(s["mean"], 3),
                             "std": round(s["std"], 3)})
        mean = s["mean"]
        po = self._param(OFFSET)
        new = propose(off, mean, self.cfg["max_step_deg"], po["min"], po["max"])
        self._say("round %d: offset %.2f, lean %+.2f deg (+/- %.2f), speed %+.0f mm/s -> %.2f" % (
            self.round, off, mean, s["std"], s["speed"], new))
        if abs(mean) <= self.cfg["tol_deg"] or abs(new - off) < 0.02:
            self.proposal = None
            return self._finish(self._summary(off, True), now)
        last = self.round >= self.cfg["rounds"]
        if self.auto and not last:
            return self._begin_slew(new, "settle", now)
        self.proposal = new
        if last:      # out of rounds: offer the final step (Apply), do not loop on
            self.state, self.t_phase = "done", now
            self.text = self._summary(off, False)
        else:
            self.state, self.t_phase = "proposal", now
            self.text = "Lean %+.2f deg: the balance point looks like %.2f deg" % (mean, new)
        self.push(now)

    def _summary(self, off, ok):
        if self.orig is not None and abs(off - self.orig) > 0.004:
            t = "Balance point %.2f deg (was %.2f)" % (off, self.orig)
        else:
            t = "Balance point %.2f deg is right" % off
        return t if ok else t + ", but the lean is still %+.2f deg" % self.last["mean"]
