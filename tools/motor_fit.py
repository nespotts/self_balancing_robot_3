#!/usr/bin/env python3
"""Fit the motor correction from a wheels-up motor test (mot.test, issue #16). Stdlib only.

  python3 tools/motor_fit.py                     newest ~/sbr3-logs/*.jsonl (run on the Pi)
  python3 tools/motor_fit.py --ssh robot         same, fetched from the Pi (run on the PC)
  python3 tools/motor_fit.py FILE [FILE ...]     session .jsonl, or any text with "mt,..." lines
  options: --vref 11.5 (model-1 reference voltage)  --vbatt 12.0 (use if the battery reading was bad)
           --config all | latest | DRIVE:HZ   (default all: every drive config separately + a comparison)

Reads the "mt,..." MSG_LOG lines (format in docs/firmware.md). Runs are grouped by drive config
(mot.drive_mode, mot.pwm_hz from the begin line, issue #22; older logs = 0:36621); per config it
uses the newest completed run of each test (a thresholds, b speed curve, c reversal), and prints:
  dead zone start/stop per wheel + direction (duty and volts), speed vs volts with a linear fit,
  left/right mismatch, reversal lag, motor time constant, v3 curve vs volts model, and the params
  to try.  Everything is in volts (duty / 4095 * pack voltage at that moment), so a fit made at
  one charge level holds at another (with mot.vcomp = 1).
The parsing and the fit live in station/bridge/motor_fit.py (the ground station's Motor test card
shows the same results).
"""
import argparse
import glob
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from station.bridge import motor_fit as mf  # noqa: E402


def read_local(paths):
    if not paths:
        files = sorted(glob.glob(os.path.expanduser("~/sbr3-logs/*.jsonl")), key=os.path.getmtime)
        if not files:
            sys.exit("no session logs in ~/sbr3-logs (give a file, or --ssh robot)")
        paths = [files[-1]]
    out = []
    for p in paths:
        out += mf.read_file(p)
    return out, paths


def read_ssh(host, name=None):
    remote = name or "$(ls -t ~/sbr3-logs/*.jsonl | head -1)"
    cmd = "f=%s; echo \"$f\"; grep -h -e '\"mt,' -e '^\\[\"param\"' \"$f\"" % remote
    r = subprocess.run(["ssh", host, cmd], capture_output=True, text=True)
    if r.returncode not in (0, 1):
        sys.exit("ssh failed: " + r.stderr.strip())
    lines = r.stdout.splitlines()
    return list(mf.mt_items(lines[1:])), [host + ":" + (lines[0] if lines else "?")]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("files", nargs="*")
    ap.add_argument("--ssh", metavar="HOST", help="read the newest session log on HOST (e.g. robot)")
    ap.add_argument("--vref", type=float, default=None, help="model-1 reference voltage (default mot.v_ref or 11.5)")
    ap.add_argument("--vbatt", type=float, default=None, help="pack voltage to use when the logged one is invalid")
    ap.add_argument("--band", type=float, default=None, help="mot.dz_band for the comparison (default from log or 0.01)")
    ap.add_argument("--config", default="all", metavar="all|latest|DRIVE:HZ",
                    help="drive config(s) to fit: all (default, each separately + a comparison), latest, "
                         "or e.g. 1:20000 = mot.drive_mode 1 at mot.pwm_hz 20000")
    a = ap.parse_args()

    if a.ssh:
        items, src = read_ssh(a.ssh, a.files[0] if a.files else None)
    else:
        items, src = read_local(a.files)
    runs, params = mf.split_runs(items)
    print("source: %s" % ", ".join(src))
    if not runs:
        sys.exit("no motor test runs (mt,begin ...) found")
    for r in runs:
        print("  run test %s: %-8s %s  (%d a / %d b / %d c lines)" % (r["test"], mf.cfg_name(r["cfg"]), r["status"],
                                                                    len(r["a"]), len(r["b"]), len(r["c"])))
    configs = mf.configs(runs)
    if a.config == "all":
        sel = configs
    elif a.config == "latest":
        sel = configs[-1:]
    else:
        try:
            d, hz = a.config.split(":")
            want = (int(d), float(hz))
        except ValueError:
            sys.exit("--config: all, latest or DRIVE:HZ (e.g. 1:20000)")
        sel = [c for c in configs if c[0] == want[0] and abs(c[1] - want[1]) <= 0.01 * want[1]]
        if not sel:
            sys.exit("no runs with %s (have: %s)" % (mf.cfg_name(want), ", ".join(mf.cfg_name(c) for c in configs)))
    results = []
    for cfg in sel:
        print("\n" + "=" * 100)
        print("=== %s   (newest run of each test with this config)" % mf.cfg_label(cfg))
        R = mf.analyze([r for r in runs if r["cfg"] == cfg], params, cfg, a.vref, a.vbatt, a.band)
        print("\n".join(mf.report(R)))
        results.append(R)
    if len(results) > 1:
        print("\n".join(mf.compare(results)))


if __name__ == "__main__":
    main()
