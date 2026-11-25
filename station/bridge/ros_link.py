"""ROS inside the station (issue #106): the bridge relays what the ROS 2 adapter says about ROS (map, pose, Nav2 plan and
status, localization, node health, the #107 explorer) to the station UI, and forwards the station's ROS commands to it.

Adapter -> bridge (only from a socket subscribed to the ROS stream, ros_stream.py): ros_map, ros_pose, ros_plan,
ros_nav, ros_status, ros_explore, ros_reply (ros2/sbr3_ros/sbr3_ros/stationlink.py). Each is kept (the newest of
each kind goes to a client that connects later) and relayed to every station client.

Station -> bridge (WebSocket or POST /api/cmd): {"t": "ros", "action": ...}
  goal          x, y (m), yaw (rad, optional), frame "map" (default) | "station" (x, y in mm)
  place         name: a saved station place (the adapter puts it in the ROS map with TF map <- station_map)
  patrol        places [names], loops (0 = forever), dwell (s)
  cancel        every Nav2 goal (also RViz's) + the station's patrol
  initial_pose  x, y, yaw, frame: map (m) or station (mm; through the last good map <- station_map alignment)
  explore       run true | false (#107 explorer)
  save_map      name (#107 explorer; optional, default explore-<date>)
Bridge -> adapter: {"t": "ros_cmd", "cmd": <action>, "id": n, "who": ..., the fields above}.

Nothing here drives: goals reach the robot only as the adapter's /cmd_vel, which it forwards while the robot is armed
in STATION (Manual). Goals that would drive are refused here unless the robot is armed in STATION
("Switch to Manual to let ROS drive"); the bridge never arms and never changes the mode for ROS.
With no adapter connected nothing changes: no ROS messages go out and status["ros"] is null.
"""
import json
import math
import time

from . import link_protocol as lp
from .ros_stream import WS_SLOW_BYTES

TIMEOUT_S = 3.0                # no ROS message for this long -> "ROS not connected" (the adapter sends ros_status at 1 Hz)
UPLINK = ("ros_map", "ros_pose", "ros_plan", "ros_nav", "ros_status", "ros_explore", "ros_reply")
KEEP = ("ros_map", "ros_plan", "ros_nav", "ros_status", "ros_explore", "ros_pose")
DROPPABLE = ("ros_pose",)
ACTIONS = ("goal", "place", "patrol", "cancel", "initial_pose", "explore", "save_map")
DRIVES = ("goal", "place", "patrol")
MANUAL_TEXT = "Switch to Manual to let ROS drive"


def _js(d):
    return json.dumps(d, separators=(",", ":"))


class RosLink:
    """Hooks (bridge.py): from_adapter(ws, m) for UPLINK messages, command(m, who) for {"t": "ros", action in ACTIONS},
    welcome(ws) for a new client, drop(ws), tick(now), status(now) -> status["ros"], station_pose_set(...) (#105)."""

    def __init__(self, bridge, clock=time.monotonic):
        self.b = bridge
        self.clock = clock
        self.ws = None                 # the adapter's socket (the last one that sent ROS data)
        self.seen = None               # clock() of its last message (None: never this run)
        self.since = None              # connected since
        self.last = {}                 # t -> JSON text of the newest message of that kind
        self.cache = {}                # t -> dict (ros_nav, ros_status, ros_explore), for status / health
        self.was_connected = False
        self.next_id = 0
        self.n = {"in": 0, "out": 0, "cmds": 0}

    # ------------------------------------------------------------ adapter -> station
    def connected(self, now=None):
        now = self.clock() if now is None else now
        return (self.ws is not None and not self.ws.closed and self.seen is not None and now - self.seen < TIMEOUT_S)

    def from_adapter(self, ws, m):
        t = m.get("t")
        if t not in UPLINK:
            return
        if ws not in self.b.ros.clients:   # opt-in: only the ROS stream's own subscribers speak for ROS
            return
        now = self.clock()
        if self.ws is not ws or not self.connected(now):
            self.since = now
            if self.ws is not ws:
                self.b.say("ROS link: adapter %s is talking (map / pose / Nav2)" % (
                    ws.peer[0] if getattr(ws, "peer", None) else "?"))
        self.ws, self.seen = ws, now
        self.n["in"] += 1
        text = _js(m)
        if t in KEEP:
            self.last[t] = text
        if t in ("ros_nav", "ros_status", "ros_explore"):
            self.cache[t] = m
        if t == "ros_reply" and not m.get("ok"):
            self.b.say("ROS: %s refused: %s" % (m.get("cmd"), m.get("text")))
        if not self.was_connected:
            self.was_connected = True
            self.b.push_status()
        self._relay(text, droppable=t in DROPPABLE)

    def _relay(self, text, droppable=False):
        """To the station clients (browsers, phones): not back to ROS stream subscribers."""
        skip = self.b.ros.clients
        for c in list(self.b.clients):
            if c.closed or c in skip:
                continue
            if droppable and c.buffered() > WS_SLOW_BYTES:
                continue
            c.send(text)
            self.n["out"] += 1

    def welcome(self, ws):
        """A new station client: the newest ROS picture, if ROS is connected."""
        if not self.connected():
            return
        for t in ("ros_status", "ros_map", "ros_plan", "ros_nav", "ros_explore", "ros_pose"):
            if t in self.last:
                ws.send(self.last[t])

    def drop(self, ws):
        if ws is self.ws:
            self.ws = None
            self.b.say("ROS link: adapter disconnected")
            self.tick(self.clock())

    def tick(self, now):
        c = self.connected(now)
        if c != self.was_connected:
            self.was_connected = c
            if not c:
                self.since = None
                self.b.say("ROS link: no word from the adapter for %.0f s" % TIMEOUT_S if self.ws is not None else
                           "ROS link: off")
            self.b.push_status()

    def status(self, now=None):
        """status["ros"]: None until an adapter has spoken this run (the station shows nothing about ROS)."""
        if self.seen is None:
            return None
        now = self.clock() if now is None else now
        st = self.cache.get("ros_status") or {}
        nav = self.cache.get("ros_nav") or {}
        ex = self.cache.get("ros_explore")
        return {"connected": self.connected(now), "age_s": round(now - self.seen, 1),
                "since_s": round(now - self.since, 1) if self.since is not None else None,
                "nav": nav.get("state"), "loc": (st.get("loc") or {}).get("state"),
                "nav2": (st.get("nodes") or {}).get("nav2"),
                "explore": bool((st.get("nodes") or {}).get("explore")) or bool(ex),
                "drivable": st.get("drivable")}

    def health(self):
        """What the Health tile looks at (None: no adapter ever)."""
        if self.seen is None:
            return None
        return {"connected": self.connected(), "age_s": round(self.clock() - self.seen, 1),
                "status": self.cache.get("ros_status"), "nav": self.cache.get("ros_nav")}

    # ------------------------------------------------------------ station -> adapter
    def manual_problem(self):
        """Why ROS could not drive the robot now ("" = it can): armed, STATION, no E-stop, live."""
        b = self.b
        t = b.tlm
        if b.estop or b.robot_estop:
            return "E-stop is on: release it, Arm & Balance, then tap Manual to let ROS drive"
        if not t or b.link_state != "connected" or self.clock() - b.tlm_time > 1.0:
            return "the robot is not connected"
        mode = lp.MODE_NAMES[t["mode"]] if 0 <= t.get("mode", -1) < lp.MODE_COUNT else "?"
        if not t.get("status", 0) & lp.TLM_ARMED:
            return "The robot is disarmed: Arm & Balance, then tap Manual to let ROS drive"
        if mode != "STATION":
            return MANUAL_TEXT
        return ""

    def command(self, m, who="?"):
        """-> None (forwarded) or an error / reply dict."""
        act = m.get("action")
        if act not in ACTIONS:
            return {"t": "error", "text": "unknown ros action %r" % act}
        if not self.connected():
            return {"t": "error", "code": "ros_offline", "text": "ROS is not connected (no adapter talking to the bridge)"}
        if act in DRIVES or (act == "explore" and m.get("run", True)):
            why = self.manual_problem()
            if why:
                return {"t": "error", "code": "not_manual", "text": why}
        out = {k: v for k, v in m.items() if k not in ("t", "action", "id", "cmd", "who")}
        if act == "place" and not str(m.get("name") or "").strip():
            return {"t": "error", "text": "place needs a name"}
        self.next_id += 1
        out.update(t="ros_cmd", cmd=act, id=self.next_id, who=who)
        self.n["cmds"] += 1
        self.ws.send(_js(out))
        if act not in ("goal",):
            self.b.say("%s: ROS %s%s" % (who, act, " " + str(m.get("name") or m.get("places") or "")
                                          if act in ("place", "patrol", "save_map") else ""))
        return {"t": "ros_sent", "action": act, "id": self.next_id}

    def station_pose_set(self, x_mm, y_mm, heading_deg, who="station"):
        """Hook for the station's "Set position" (#105): give ROS the same pose as its initial pose, in the station
        frame (the adapter maps it with the last good map <- station_map alignment). False when ROS is not connected."""
        if not self.connected():
            return False
        r = self.command({"t": "ros", "action": "initial_pose", "frame": "station", "x": float(x_mm), "y": float(y_mm),
                          "yaw": math.radians((float(heading_deg) + 180.0) % 360.0 - 180.0)}, who)
        return bool(r and r.get("t") == "ros_sent")
