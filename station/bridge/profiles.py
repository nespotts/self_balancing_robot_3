"""Robot profiles (issue #97): one Pi + LD19 moves between robots (balancer, Shark, the sim), and each robot keeps
its own maps, places, zones, lidar.json, planner.json, tuning presets and fall records:

    ~/sbr3-maps/<robot>/      maps (*.json / *.png / *.places.json / *.zones.json), settings.json, planner.json, lidar.json
    ~/sbr3-presets/<robot>/   tuning presets
    ~/sbr3-falls/<robot>/     fall records

identify() names the robot from what the Teensy reports; migrate_flat() moves the old flat layout (everything
directly in ~/sbr3-maps etc.) into balancer/ the first time, without ever losing a file.
"""
import hashlib
import os
import re
import shutil

DEFAULT_ROBOT = "balancer"
PLATFORM_NAMES = {1: "balancer", 2: "shark"}        # the read-only param sys.platform (#85)
ROBOT_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
PLATFORM_PARAM = "sys.platform"


def clean_name(s):
    """A folder-safe robot name (lower case letters, digits, - and _), or None."""
    s = str(s or "").strip().lower()
    return s if ROBOT_RE.match(s) else None


def identify(hello, params):
    """(name, source) from the HELLO frame ({"firmware": ...}) and the param values ({name: value}).
    sys.platform (1 balancer, 2 shark) first; else the firmware string; a simulator ("sim ...") has its own profile
    ("sim", or "sim-shark" for a simulated Shark) so it never touches a real robot's maps; default "balancer"."""
    plat = None
    v = (params or {}).get(PLATFORM_PARAM)
    try:
        code = int(round(float(v)))
        if code > 0:
            plat = PLATFORM_NAMES.get(code, "platform%d" % code)
    except (TypeError, ValueError, OverflowError):
        pass
    fw = str((hello or {}).get("firmware") or "").strip().lower()
    if fw.startswith("sim"):
        return ("sim" if plat in (None, DEFAULT_ROBOT) else "sim-" + plat), "firmware"
    if plat:
        return plat, PLATFORM_PARAM
    return DEFAULT_ROBOT, "default"


def robot_dir(root, robot):
    return os.path.join(os.path.abspath(os.path.expanduser(root)), robot)


# ---------------------------------------------------------------------------------------------- migration
def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _movable(name):
    return not name.startswith(".") and not name.endswith((".tmp", ".migrating"))


def migrate_flat(root, robot=DEFAULT_ROBOT, say=None):
    """Move the regular files that sit directly in `root` (the layout before profiles) into root/<robot>/.
    Per file: copy to <dest>.migrating, check size and SHA-256 against the original, rename into place, only then
    delete the original. A crash at any point leaves a complete copy of every file (re-running finishes the job).
    A file that already exists in the profile folder is never overwritten: identical -> the old one is dropped,
    different -> both stay and it is reported. Folders (other robots' profiles) are never touched.
    Returns {"moved": n, "dup": n, "kept": [names], "failed": [names]}."""
    say = say or (lambda text: None)
    out = {"moved": 0, "dup": 0, "kept": [], "failed": []}
    root = os.path.abspath(os.path.expanduser(root))
    try:
        names = sorted(n for n in os.listdir(root) if _movable(n) and os.path.isfile(os.path.join(root, n)))
    except OSError:
        return out
    if not names:
        return out
    dest_dir = os.path.join(root, robot)
    try:
        os.makedirs(dest_dir, exist_ok=True)
    except OSError as e:
        say("profiles: cannot create %s (%s) - old files stay where they are" % (dest_dir, e))
        out["failed"] = names
        return out
    for n in names:
        src, dst = os.path.join(root, n), os.path.join(dest_dir, n)
        tmp = dst + ".migrating"
        try:
            if os.path.exists(dst):
                if os.path.getsize(src) == os.path.getsize(dst) and _sha(src) == _sha(dst):
                    os.remove(src)               # an earlier run already copied it
                    out["dup"] += 1
                else:
                    out["kept"].append(n)        # two different versions: keep both, the profile's one is used
                continue
            shutil.copy2(src, tmp)
            if os.path.getsize(tmp) != os.path.getsize(src) or _sha(tmp) != _sha(src):
                raise OSError("copy does not match the original")
            os.replace(tmp, dst)
            os.remove(src)
            out["moved"] += 1
        except OSError as e:
            out["failed"].append(n)
            say("profiles: could not move %s (%s) - left in place" % (n, e))
            try:
                if os.path.exists(tmp):
                    os.remove(tmp)
            except OSError:
                pass
    if out["moved"] or out["dup"]:
        say("profiles: moved %d file%s from %s into %s/ (%s)" % (
            out["moved"] + out["dup"], "" if out["moved"] + out["dup"] == 1 else "s", root, robot, "verified"))
    if out["kept"]:
        say("profiles: %d file%s in %s differ from the copy in %s/ and stay there: %s" % (
            len(out["kept"]), "" if len(out["kept"]) == 1 else "s", root, robot, ", ".join(out["kept"][:5])))
    return out
