"""Sim-only extras for the ROS 2 tests (issue #98): start pose, kidnapping, ground truth, surprise boxes.

  python -m station.sim --lidar --start 1200,-400,90         the robot starts there (world mm, deg); odometry reads 0
  python -m station.sim --lidar --teleport-file /tmp/tp       write "x_mm y_mm deg" into /tmp/tp: the robot is carried
                                                              there (the odometry does not notice, "kidnapped robot")
  python -m station.sim --lidar --truth-log /tmp/truth.csv    10 Hz: wall time, true X, Y (mm), TH (deg), odometry x, y, th
  python -m station.sim --lidar --box 2200,-300,300,300       a box that is not on any saved map (repeatable)
"""
import asyncio
import math
import os
import time

from .sim import _box


def parse_pose(text):
    """'x,y,deg' (mm, mm, deg; spaces or commas) -> (x, y, deg). Raises ValueError."""
    v = [float(s) for s in str(text).replace(",", " ").split()]
    if len(v) == 2:
        v.append(0.0)
    if len(v) != 3 or not all(math.isfinite(a) for a in v):
        raise ValueError("pose: x,y[,deg] in mm / degrees")
    return tuple(v)


def parse_box(text):
    """'cx,cy,w,h' mm -> four wall segments. Raises ValueError."""
    v = [float(s) for s in str(text).replace(",", " ").split()]
    if len(v) != 4 or v[2] <= 0 or v[3] <= 0:
        raise ValueError("box: cx,cy,w,h in mm")
    return _box(*v)


def add_boxes(robot, boxes):
    """Extra boxes as real walls, kept when the room's extra walls change (set_extra)."""
    extra = robot.room.segs[len(robot.room.base):]
    for b in boxes:
        robot.room.base = robot.room.base + parse_box(b)
    robot.room.segs = list(robot.room.base) + extra


def teleport(robot, x, y, deg, reset_odom=False):
    """Move the robot's true pose to (x, y, deg) world mm / degrees.
    reset_odom False: the odometry keeps reading where it was (the robot was picked up and carried);
    True: the odometry reads 0, 0, 0 there (a start pose)."""
    if reset_odom:
        robot.X, robot.Y, robot.TH = float(x), float(y), float(deg)
        robot.ox, robot.oy, robot.oth = robot.X, robot.Y, robot.TH
        for a in ("dx_o", "dy_o", "dth_o"):            # DriftRobot's own odometry
            if hasattr(robot, a):
                setattr(robot, a, 0.0)
        if robot.hw:
            robot.odx = robot.ody = 0.0
    else:
        if robot.hw is None and not getattr(robot, "drifting", lambda: False)():
            ox, oy, oth = robot.odom()                 # keep this odometry pose at the new true pose
            robot.oth = float(deg) - oth
            c, s = math.cos(math.radians(robot.oth)), math.sin(math.radians(robot.oth))
            robot.ox = float(x) - (c * ox - s * oy)
            robot.oy = float(y) - (s * ox + c * oy)
        else:
            robot.oth += float(deg) - robot.TH          # gyro heading = TH - oth + drift: keep it
        robot.X, robot.Y, robot.TH = float(x), float(y), float(deg)
    if hasattr(robot, "_prev"):                        # DriftRobot integrates true motion: not this jump
        robot._prev = (robot.t, robot.X, robot.Y, robot.TH)
    robot.log("sim: teleported to %.0f, %.0f, %.0f deg" % (robot.X, robot.Y, robot.TH))


async def teleport_watch(robot, path, every=0.25):
    """Poll `path`: when it holds 'x y deg', carry the robot there and delete the file."""
    while True:
        await asyncio.sleep(every)
        try:
            with open(path) as f:
                text = f.read()
        except OSError:
            continue
        try:
            os.remove(path)
        except OSError:
            pass
        try:
            x, y, d = parse_pose(text)
        except ValueError:
            print("[sim] teleport file: expected 'x_mm y_mm deg', got %r" % text[:40], flush=True)
            continue
        teleport(robot, x, y, d)
        print("[sim] kidnapped: robot carried to %.0f, %.0f, %.0f deg" % (x, y, d), flush=True)


def truth_row(robot, now=None):
    ox, oy, oth = robot.odom()
    return "%.3f,%.1f,%.1f,%.2f,%.1f,%.1f,%.2f\n" % (time.time() if now is None else now, robot.X, robot.Y,
                                                     robot.TH, ox, oy, oth)


async def truth_log(robot, path, hz=10.0):
    """Ground truth for the ROS tests: wall time (s), true X, Y mm, TH deg, odometry x, y mm, th deg."""
    with open(path, "w") as f:
        f.write("t,X,Y,TH,ox,oy,oth\n")
        while True:
            f.write(truth_row(robot))
            f.flush()
            await asyncio.sleep(1.0 / hz)
