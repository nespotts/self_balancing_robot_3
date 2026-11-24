#!/usr/bin/env python3
"""SIMULATOR ONLY: arm the sim robot and put it in STATION (Manual) mode, as the owner would on the phone, so Nav2's
/cmd_vel can drive it in the end-to-end test. The ROS adapter itself never arms; neither should anything else that
talks to the real robot, so this refuses any bridge that is not on this machine.

  python3 ros2/scripts/sim_arm.py [ws://127.0.0.1:8080/ws]
"""
import json
import sys
import time
from urllib.parse import urlparse

import websocket

MODES = ["IDLE", "BALANCE", "MANUAL", "STATION", "GOTO", "EXPLORE", "HOLD"]


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else "ws://127.0.0.1:8080/ws"
    if urlparse(url).hostname not in ("127.0.0.1", "localhost"):
        sys.exit("sim_arm.py: refusing %s - this is for the local simulator only" % url)
    ws = websocket.create_connection(url, timeout=5, suppress_origin=True)
    ws.send(json.dumps({"t": "hi", "name": "sim_arm"}))
    ws.send(json.dumps({"t": "set_mode", "mode": MODES.index("BALANCE"), "arm": True}))
    time.sleep(3.0)                                  # stand up
    ws.send(json.dumps({"t": "set_mode", "mode": MODES.index("STATION")}))
    end = time.time() + 5
    while time.time() < end:
        m = json.loads(ws.recv())
        if m.get("t") == "tlm" and MODES[m["mode"]] == "STATION" and m["status"] & 1:
            print("sim robot armed, mode STATION")
            ws.close()
            return 0
    ws.close()
    sys.exit("sim_arm.py: the sim robot did not reach armed STATION")


if __name__ == "__main__":
    main()
