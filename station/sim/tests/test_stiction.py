"""Issue #69: stick-slip rock with static friction in the sim's TT motors, and the stiction kick (mot.bk_*)."""
import math
import unittest

from station.bridge import link_protocol as lp
from station.sim.sim import FloorWheel, SimRobot, StictionKick

# the robot's motor setup at the 2026-11-21 tuning session (brake 1 kHz, model 1, new motors)
SETUP = {"mot.model": 1, "mot.drive_mode": 1, "mot.pwm_hz": 1000, "mot.vcomp": 1,
         "mot.dz_l_fwd": 0.82, "mot.dz_l_rev": 0.76, "mot.dz_r_fwd": 0.85, "mot.dz_r_rev": 0.88}


def balance(bk_gain, seed=2, secs=10.0, settle=4.0):
    """Balance in place; returns pitch rms (deg), % of steps with the left wheel stalled (speed ~0 while
    |cmd| > 5 %), and wheel direction flips per second (|speed| > 20 mm/s hysteresis)."""
    r = SimRobot(seed=seed)
    r.use_floor_wheels()
    for n, v in dict(SETUP, **{"mot.bk_gain": bk_gain}).items():
        r.params[r.pidx[n]][1] = v
    r.set_mode(lp.MODE_BALANCE, arm=True)
    th, stall, flips, sgn, n = [], 0, 0, 0, 0
    while r.t < secs:
        r.step()
        if r.t < settle:
            continue
        assert r.ctl.out.get("upright"), "fell over"
        n += 1
        th.append(math.degrees(r.theta))
        if abs(r.vl) < 1.0 and abs(r.motor_l) > 0.05 * 4095:
            stall += 1
        s = 1 if r.vl > 20 else (-1 if r.vl < -20 else sgn)
        flips += s != sgn and sgn != 0
        sgn = s
    m = sum(th) / len(th)
    rms = math.sqrt(sum((x - m) ** 2 for x in th) / len(th))
    return rms, 100.0 * stall / n, flips / (n * r.dt)


class TestFloorWheel(unittest.TestCase):
    def test_breakaway_then_kinetic(self):
        w = FloorWheel(v_start=(2.5, 2.1), v_stop=(0.8, 0.8))
        for _ in range(200):
            w.step(2.0, 0.0025)                # below breakaway: stuck
        self.assertEqual(w.v, 0.0)
        w.step(2.6, 0.0025)
        for _ in range(200):
            w.step(1.0, 0.0025)                # once turning, 1 V (> kinetic 0.8) keeps it going
        self.assertGreater(w.v, 15.0)
        for _ in range(200):
            w.step(0.5, 0.0025)                # below kinetic: stops and sticks
        self.assertEqual(w.v, 0.0)
        for _ in range(100):
            w.step(-2.0, 0.0025)               # reverse below its breakaway (2.1): stuck
        self.assertEqual(w.v, 0.0)


class TestKickPort(unittest.TestCase):
    def test_kick_fade_band(self):
        k, t = StictionKick(), 0
        args = (2.5, 2.1, 1.0, 80.0, 0.03, 15.0)
        for _ in range(20):
            k.apply(0.0, 0.0, 0.0, True, t, *args)
            t += 2500
        self.assertEqual(k.apply(0.02, 1.0, 0.0, True, t, *args), 1.0)      # inside the band: no kick
        self.assertEqual(k.apply(-0.1, -1.9, 0.0, True, t, *args), -2.1)    # stopped: breakaway, reverse
        t += 2500
        v = k.apply(-0.1, -1.9, -30.0, True, t, *args)                       # moving: fades from there
        self.assertEqual(k.state, StictionKick.FADE)
        self.assertTrue(-2.1 <= v <= -1.9)
        self.assertEqual(k.apply(-0.1, -1.9, 0.0, False, t, *args), -1.9)    # stale feedback: off


class TestStickSlipRock(unittest.TestCase):
    def test_kick_reduces_rock_without_buzz(self):
        rms0, stall0, flips0 = balance(0.0)
        rms1, stall1, flips1 = balance(1.0)
        print("\nstick-slip sim: bk_gain 0: rms %.2f deg, stalled %.1f %%, flips %.2f/s | bk_gain 1: rms %.2f deg, "
              "stalled %.1f %%, flips %.2f/s" % (rms0, stall0, flips0, rms1, stall1, flips1))
        self.assertGreater(stall0, 8.0)            # the stick-slip rock reproduces without the kick
        self.assertLess(rms1, 0.6 * rms0)
        self.assertLess(stall1, 0.3 * stall0)
        self.assertLess(flips1, 2.0)               # no buzz (a 5 Hz buzz = ~10 flips/s)
        self.assertLessEqual(flips1, flips0 + 0.5)


if __name__ == "__main__":
    unittest.main()
