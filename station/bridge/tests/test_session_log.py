"""session_log: prune (cap, active file kept), CSV export (partial last line, big file), disk errors (#49)."""
import json
import os
import shutil
import tempfile
import unittest

from station.bridge import link_protocol as lp
from station.bridge import session_log
from station.bridge.session_log import SessionLog, to_csv


class TestSessionLog(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _old(self, name, size):
        with open(os.path.join(self.tmp, name), "wb") as f:
            f.write(b"x" * size)

    def test_prune_removes_oldest_first_and_keeps_the_active_file(self):
        log = SessionLog(self.tmp, max_total_mb=0.001)           # ~1 KB cap
        for i in range(1, 4):
            self._old("sbr3-20200101-00000%d.jsonl" % i, 600)
        self._old("notes.txt", 5000)                              # not a session file: never touched
        self.assertTrue(log.start())
        names = [d["name"] for d in log.list()]
        self.assertIn(log.name, names)
        self.assertNotIn("sbr3-20200101-000001.jsonl", names)    # oldest gone
        self.assertNotIn("sbr3-20200101-000002.jsonl", names)
        self.assertTrue(os.path.exists(os.path.join(self.tmp, "notes.txt")))
        # even when the active file alone is over the cap it is kept
        log.max_bytes = 1
        log.prune()
        self.assertEqual([d["name"] for d in log.list() if d["active"]], [log.name])
        log.stop()

    def test_no_prune_under_the_cap(self):
        log = SessionLog(self.tmp, max_total_mb=1)
        self._old("sbr3-20200101-000001.jsonl", 600)
        log.start()
        self.assertEqual(len(log.list()), 2)
        log.stop()

    def test_same_second_sessions_get_distinct_names(self):
        log = SessionLog(self.tmp)
        names = set()
        for _ in range(3):
            self.assertTrue(log.start())
            names.add(log.name)
        log.stop()
        self.assertEqual(len(names), 3)

    def test_path_only_accepts_session_names(self):
        log = SessionLog(self.tmp)
        log.start()
        self.assertIsNotNone(log.path(log.name))
        for bad in ("../x.jsonl", "settings.json", "", None, "sbr3-1-2.jsonl\\..\\x"):
            self.assertIsNone(log.path(bad), bad)
        log.stop()

    def test_csv_skips_a_partial_last_line_and_other_kinds(self):
        log = SessionLog(self.tmp)
        log.start()
        d = {c: 1.5 for c in session_log.TLM_COLS}
        log.tlm(d)
        log.tlm(d)
        log.record("log", "hello")
        log.stop()
        p = log.path(log.name)
        with open(p, "a", encoding="utf-8") as f:
            f.write('["tlm",9.9,1.0,2.')                         # the live file's half-written last line
        rows = list(to_csv(p))
        self.assertEqual(rows[0].split(",")[0], "t_s")
        self.assertEqual(len(rows), 3)                           # header + 2 complete tlm rows
        self.assertEqual(len(rows[1].split(",")), len(session_log.TLM_COLS) + 1)
        self.assertEqual(len(list(to_csv(p, "rng"))), 1)         # header only

    def test_csv_big_file_streams(self):
        p = os.path.join(self.tmp, "sbr3-20200101-000000.jsonl")
        row = ["tlm", 0.0] + [0.25] * len(session_log.TLM_COLS)
        with open(p, "w", encoding="utf-8") as f:
            for i in range(20000):
                row[1] = i / 100
                f.write(json.dumps(row) + "\n")
        n = sum(1 for _ in to_csv(p))
        self.assertEqual(n, 20001)

    def test_unusable_dir_sets_error_and_the_log_methods_stay_safe(self):
        blocker = os.path.join(self.tmp, "file")
        open(blocker, "w").close()
        log = SessionLog(self.tmp)
        log.dir = os.path.join(blocker, "sub")                   # a path under a regular file: cannot be made
        self.assertFalse(log.start())
        self.assertTrue(log.error)
        log.tlm({c: 0 for c in session_log.TLM_COLS})            # no-ops, no exception
        log.record("log", "x")
        log.flush()
        self.assertEqual(log.list(), [])

    def test_write_failure_stops_logging_instead_of_raising(self):
        log = SessionLog(self.tmp)
        log.start()

        class Full:
            def write(self, s):
                raise OSError(28, "No space left on device")

            def close(self):
                pass
        real = log.f
        log.f = Full()
        log.record("log", "x")
        self.assertIsNone(log.f)
        self.assertIn("space", log.error)
        real.close()


if __name__ == "__main__":
    unittest.main()