#!/usr/bin/env python3
"""SIMULATOR ONLY: save places / zones in the local sim's station, as the owner would on the phone (tests).
Refuses any bridge that is not on this machine, like sim_arm.py.

  python3 ros2/scripts/sim_station.py place Kitchen 3300 1300       odometry mm (as the phone sends them)
  python3 ros2/scripts/sim_station.py zone wall "1500,700 1500,-1500"
  python3 ros2/scripts/sim_station.py zone nogo "x,y x,y x,y ..."
  python3 ros2/scripts/sim_station.py delete w3                     delete one zone (id)
  python3 ros2/scripts/sim_station.py clear                         delete all zones
  python3 ros2/scripts/sim_station.py list
URL: SBR3_URL (default ws://127.0.0.1:8080/ws).
"""
import json
import os
import sys
import time
from urllib.parse import urlparse

import websocket


def connect():
    url = os.environ.get("SBR3_URL", "ws://127.0.0.1:8080/ws")
    if urlparse(url).hostname not in ("127.0.0.1", "localhost"):
        sys.exit("sim_station.py: refusing %s - this is for the local simulator only" % url)
    ws = websocket.create_connection(url, timeout=5, suppress_origin=True)
    ws.send(json.dumps({"t": "hi", "name": "sim_station"}))
    return ws


def wait_for(ws, t, secs=4.0, after=None):
    end = time.time() + secs
    last = None
    while time.time() < end:
        try:
            m = json.loads(ws.recv())
        except Exception:
            break
        if m.get("t") == "error":
            sys.exit("bridge: %s" % m.get("text"))
        if m.get("t") == t:
            last = m
            if after is None or after(m):
                return m
    return last


def main(a):
    if not a:
        sys.exit(__doc__)
    ws = connect()
    wait_for(ws, "zones", 3.0)                     # the hello burst ends with zones
    if a[0] == "place":
        ws.send(json.dumps({"t": "places", "action": "add", "name": a[1], "x": float(a[2]), "y": float(a[3])}))
        m = wait_for(ws, "places", 4.0, lambda m: any(p["name"].lower() == a[1].lower() for p in m.get("items", [])))
        print("places:", [(p["name"], p["x"], p["y"]) for p in (m or {}).get("items", [])])
    elif a[0] == "zone":
        pts = [[float(v) for v in p.split(",")] for p in a[2].split()]
        ws.send(json.dumps({"t": "zones", "action": "add", "kind": a[1], "pts": pts}))
        m = wait_for(ws, "zones", 4.0, lambda m: any(z["kind"] == a[1] for z in m.get("items", [])))
        print("zones:", [(z["id"], z["kind"], len(z.get("pts") or [])) for z in (m or {}).get("items", [])])
    elif a[0] == "delete":
        ws.send(json.dumps({"t": "zones", "action": "delete", "id": a[1]}))
        m = wait_for(ws, "zones", 4.0, lambda m: all(z["id"] != a[1] for z in m.get("items", [])))
        print("zones:", [(z["id"], z["kind"]) for z in (m or {}).get("items", [])])
    elif a[0] == "clear":
        ws.send(json.dumps({"t": "zones", "action": "clear"}))
        m = wait_for(ws, "zones", 4.0, lambda m: not m.get("items"))
        print("zones cleared" if m is not None else "no reply")
    elif a[0] == "list":
        ws.send(json.dumps({"t": "places", "action": "list"}))
        print(json.dumps(wait_for(ws, "places", 3.0)))
        ws.send(json.dumps({"t": "zones", "action": "list"}))
        print(json.dumps(wait_for(ws, "zones", 3.0)))
    ws.close()


if __name__ == "__main__":
    main(sys.argv[1:])
