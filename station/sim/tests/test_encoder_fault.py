"""Issue #109 in the simulator (the firmware's ModeManager rules, mirrored in station/sim/sim.py): an encoder fault
while driving GOTO stops it and the robot balances in place; SET_MODE GOTO / STATION and arming are refused until it
clears; nothing resumes by itself.
Run: python -m unittest discover -s station -t ."""
import math
import unittest

from station.bridge import link_protocol as lp
from station.sim.tests.test_nav import NavRun


class TestEncoderFault(unittest.TestCase):
    def test_goto_stops_and_holds_in_place(self):
        n = NavRun()
        n.stand_up(lp.MODE_BALANCE)
        n.frame(lp.goto_frames([(3000.0, 0.0)])[0])
        n.frame(lp.encode_set_mode(lp.MODE_GOTO))
        self.assertTrue(n.run(10.0, lambda s: s.r.mode == lp.MODE_GOTO and abs(s.r.speed_f) > 150))
        n.r.fault = 3                                      # the left AS5600 drops out at speed
        n.run(0.2)
        self.assertEqual(n.r.mode, lp.MODE_BALANCE)
        self.assertTrue(n.r.armed)
        self.assertTrue(any("GOTO stopped -> BALANCE" in l for l in n.logs), n.logs[-5:])
        self.assertTrue(any(l.startswith("enc L: FAULT") for l in n.logs))
        n.run(3.0)                                         # ramps down, then holds
        x0, y0 = n.pos()
        n.run(5.0)
        x1, y1 = n.pos()
        self.assertTrue(n.r.ctl.upright)                   # still balancing (never dropped)
        self.assertLess(math.hypot(x1 - x0, y1 - y0), 150.0)   # in place, not driving on
        self.assertLess(abs(n.r.speed_f), 60.0)
        self.assertEqual(n.tlm[-1]["fault_flags"] & lp.LINK_FAULT_ENC_L, lp.LINK_FAULT_ENC_L)
        self.assertEqual(n.tlm[-1]["error_code"], 3)
        # the planner / station asks again: refused and said so
        for m in (lp.MODE_GOTO, lp.MODE_EXPLORE, lp.MODE_STATION, lp.MODE_HOLD):
            n.frame(lp.encode_set_mode(m))
        n.run(0.5)
        self.assertEqual(n.r.mode, lp.MODE_BALANCE)
        self.assertEqual(sum("ignored: left encoder fault" in l for l in n.logs), 4)
        # cleared: still BALANCE until someone asks
        n.r.fault = 0
        n.run(1.0)
        self.assertEqual(n.r.mode, lp.MODE_BALANCE)
        self.assertTrue(any(l.startswith("enc L: ok again") for l in n.logs))
        n.frame(lp.encode_set_mode(lp.MODE_GOTO))
        n.run(0.2)
        self.assertEqual(n.r.mode, lp.MODE_GOTO)

    def test_no_arming_on_fault(self):
        n = NavRun()
        n.run(0.5)
        n.r.fault = 4
        n.frame(lp.encode_event(lp.EVT_LINK_ARM))
        n.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        n.run(0.5)
        self.assertFalse(n.r.armed)
        self.assertEqual(n.r.mode, lp.MODE_IDLE)
        self.assertTrue(any("ignored: right encoder fault" in l for l in n.logs))
        n.r.fault = 0
        n.frame(lp.encode_event(lp.EVT_LINK_ARM))
        n.frame(lp.encode_set_mode(lp.MODE_BALANCE))
        n.run(0.2)
        self.assertTrue(n.r.armed)


if __name__ == "__main__":
    unittest.main()
