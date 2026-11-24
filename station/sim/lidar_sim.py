"""Simulated LDROBOT LD19 (issue #76): real 47-byte packets (station/bridge/ld19.py encodes them, CRC and all) from the
sim's virtual room, so the bridge's whole lidar path (parser, scan assembly, pose interpolation, masks, pitch gate,
mapping) runs on a PC before any hardware is mounted.

Like the real sensor: 4500 points/s at 10 Hz (450 points a turn, 0.8 deg apart, 12 per packet, so a packet straddles
the turn's start every other revolution), angles increase CLOCKWISE from the sensor front, every packet is measured at
the robot pose of that moment (a driving / turning robot smears the scan unless the bridge deskews it), distance 0 =
no return. Imperfections: range noise (6 mm + 0.3 %), dropped points, intensity falling with range, two thin cage posts
at the robot's sides (robot-frame +-90 deg, ~3 deg wide, 55 mm out), floor hits when the robot leans (a beam pitched down
by e meets the floor at height / tan(e); e = lean * cos(azimuth)), optional corrupted packets / lost packets.
"""
import math
import random

from station.bridge import ld19

TURN_HZ = 10.0
PTS_PER_TURN = int(ld19.POINT_RATE / TURN_HZ)          # 450
STEP_DEG = 360.0 / PTS_PER_TURN                         # 0.8
MAX_RANGE_MM = 12000.0                                  # LD19 reach (the sim room is 5 m)
POST_MM = 55.0
POST_HALF_DEG = 1.5


class LidarSim:
    def __init__(self, seed=None, x_mm=0.0, y_mm=0.0, yaw_deg=0.0, flip=False, height_mm=200.0, posts=True,
                 noise_mm=6.0, dropout=0.01, bad_packets=0.0, lost_packets=0.0, lever_mm=0.0):
        self.rng = random.Random(seed)
        self.x, self.y, self.yaw, self.flip, self.height = x_mm, y_mm, yaw_deg, flip, height_mm
        self.lever = lever_mm       # lidar this far above the axle: leaning moves it lever * sin(lean) forward (#79)
        self.posts = posts
        self.noise, self.dropout = noise_mm, dropout
        self.bad_packets, self.lost_packets = bad_packets, lost_packets
        self.n = None               # index of the next point to emit (global, 4500 per second of sim time)
        self.packets = 0

    def step(self, robot, active=True):
        """Packets for the sim time that passed since the last call; robot: a SimRobot (true pose X, Y, TH, lean theta)."""
        due = int(robot.t * ld19.POINT_RATE)
        if self.n is None or not active:
            self.n = due - due % ld19.POINTS            # nobody listening: skip ahead, no backlog
            return b""
        out = bytearray()
        while self.n + ld19.POINTS <= due:
            out += self._packet(robot, self.n)
            self.n += ld19.POINTS
        return bytes(out)

    def _packet(self, robot, n0):
        th = math.radians(robot.TH)
        c, s = math.cos(th), math.sin(th)
        fx = self.x + self.lever * math.sin(robot.theta)
        ox, oy = robot.X + c * fx - s * self.y, robot.Y + s * fx + c * self.y
        lean = math.degrees(robot.theta)
        rng = self.rng
        room = robot.room
        pts = []
        for i in range(ld19.POINTS):
            a = ((n0 + i) % PTS_PER_TURN) * STEP_DEG                 # sensor angle, clockwise
            b = self.yaw + (a if self.flip else -a)                  # robot frame, CCW from forward
            d = None
            q = 0
            if self.posts and (abs((b - 90.0 + 180.0) % 360.0 - 180.0) < POST_HALF_DEG
                               or abs((b + 90.0 + 180.0) % 360.0 - 180.0) < POST_HALF_DEG):
                d, q = POST_MM, 250
            else:
                d = room.raycast(ox, oy, th + math.radians(b), MAX_RANGE_MM)
                if d is not None:
                    q = max(12, min(255, 225 - d / 40.0 + rng.gauss(0, 12)))
                e = lean * math.cos(math.radians(b))                 # beam pitched down by e degrees
                if e > 0.3:
                    df = self.height / math.tan(math.radians(e))
                    if d is None or df < d:
                        d, q = df, 70
            if d is None or rng.random() < self.dropout:
                pts.append((0, 0))
            else:
                pts.append((max(1.0, d + rng.gauss(0, self.noise + 0.003 * d)), q))
        a0 = (n0 % PTS_PER_TURN) * STEP_DEG
        a1 = a0 + (ld19.POINTS - 1) * STEP_DEG
        ts = int((n0 + ld19.POINTS - 1) / ld19.POINT_RATE * 1000.0)
        self.packets += 1
        if self.lost_packets and rng.random() < self.lost_packets:
            return b""
        raw = ld19.encode_packet(pts, a0 % 360.0, a1 % 360.0, speed_dps=3600 + rng.gauss(0, 3), ts_ms=ts)
        if self.bad_packets and rng.random() < self.bad_packets:
            k = rng.randrange(2, ld19.PKT_LEN - 1)
            raw = raw[:k] + bytes([raw[k] ^ 0x5A]) + raw[k + 1:]     # a flipped byte: the CRC catches it
        return raw
