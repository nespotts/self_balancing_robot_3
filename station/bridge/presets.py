"""Tuning presets (issue #16): named sets of bal.* / vel.* / str.* / drv.* / mot.* values, kept as JSON
files on the Pi (~/sbr3-presets/<name>.json). Two built-ins ship read-only.

Applying a preset = set_param for each value that differs from the robot's; it never touches
the wheels-up test (mot.test, test_max, test_wheels), the L298N drive mode and its limits (mot.drive_mode,
brake_max_hz, brake_duty_max: issues #23 / #26) or cfg.* (actions).
"""
import json
import os
import re
import time

GROUPS = ("bal.", "vel.", "str.", "drv.", "mot.")
# test controls and L298N safety limits (issues #22 / #23 / #26) are never part of a preset
NEVER = frozenset(("mot.test", "mot.test_max", "mot.test_wheels", "mot.drive_mode", "mot.brake_max_hz",
                   "mot.brake_duty_max"))
NAME_MAX = 60

_V3 = {
    "bal.kp": 250.0, "bal.ki": 2010.0, "bal.kd": 4.63, "bal.offset_deg": 3.15, "bal.cutoff_deg": 45.0,
    "bal.reengage_deg": 15.0, "bal.knob_trim_deg": 0.0, "bal.d_src": 0.0,
    "str.kp": 20.0, "str.ki": 5.9, "str.kd": 0.075, "str.max_err_deg": 45.0,
    "vel.kp": 0.006, "vel.ki": 0.00869, "vel.kd": 0.00007, "vel.limit_deg": 20.0,
    "drv.max_speed": 450.0, "drv.max_turn": 180.0, "drv.accel": 1500.0, "drv.turn_accel": 1000.0,
    "drv.turn_deadband": 2.0,
    "mot.left_scale": 0.95, "mot.right_scale": 1.0, "mot.max": 4095.0, "mot.curve_exp": 3.92,
    "mot.knee_in": 1250.0, "mot.knee_out": 2750.0, "mot.model": 0.0, "mot.vcomp": 0.0, "mot.pwm_hz": 36621.1,
}
BUILTIN = [
    {"name": "v3 known good", "builtin": True, "created": "2026-11-21T00:00:00",
     "note": "v3 gains (kp 250, ki 2010, kd 4.63, d_src 0), v3 motor curve. Balanced first try on v4.",
     "values": dict(_V3)},
    {"name": "2026-11-21 (kd 7.15, ki 1500)", "builtin": True, "created": "2026-11-21T04:00:00",
     "note": "Owner's live session: more D, less I. Pushes settle faster; feels better driving.",
     "values": dict(_V3, **{"bal.ki": 1500.0, "bal.kd": 7.15})},
]


def tunable(name):
    return name.startswith(GROUPS) and name not in NEVER


def snapshot(params):
    """Tunable values from a param list [{name, value, ...}] or {id: {...}}."""
    items = params.values() if isinstance(params, dict) else params
    return {p["name"]: float(p["value"]) for p in items if tunable(p["name"])}


def _same(a, b):
    return abs(a - b) <= 1e-5 * max(1.0, abs(a), abs(b))


def plan_apply(values, params):
    """What applying `values` changes: {"changes": [{id, name, from, to}], "skipped": [names], "same": n}.
    Values outside the robot's min/max are clamped (as the robot would). Never NEVER / cfg.*."""
    items = params.values() if isinstance(params, dict) else params
    by_name = {p["name"]: p for p in items}
    changes, skipped, same = [], [], 0
    for name, v in values.items():
        p = by_name.get(name)
        if not tunable(name) or p is None:
            skipped.append(name)
            continue
        v = max(p["min"], min(p["max"], float(v)))
        if _same(v, float(p["value"])):
            same += 1
        else:
            changes.append({"id": p["id"], "name": name, "from": p["value"], "to": v})
    changes.sort(key=lambda c: c["id"])
    return {"changes": changes, "skipped": sorted(skipped), "same": same}


def diff(a, b, order=None):
    """Rows {name, a, b, same} for every name in either value set (None = not in that set).
    order: param names in robot order (others after, alphabetical)."""
    names = set(a) | set(b)
    pos = {n: i for i, n in enumerate(order or [])}
    rows = []
    for n in sorted(names, key=lambda n: (pos.get(n, 1e9), n)):
        va, vb = a.get(n), b.get(n)
        rows.append({"name": n, "a": va, "b": vb, "same": va is not None and vb is not None and _same(va, vb)})
    return rows


def _slug(name):
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("._")
    return s[:NAME_MAX] or "preset"


class PresetStore:
    def __init__(self, folder):
        self.dir = os.path.abspath(os.path.expanduser(folder))

    def _path(self, name):
        return os.path.join(self.dir, _slug(name) + ".json")

    def list(self):
        out = [dict(p) for p in BUILTIN]
        if os.path.isdir(self.dir):
            user = []
            for fn in os.listdir(self.dir):
                if not fn.endswith(".json"):
                    continue
                try:
                    with open(os.path.join(self.dir, fn), encoding="utf-8") as f:
                        d = json.load(f)
                    vals = {k: float(v) for k, v in d.get("values", {}).items() if tunable(k)}
                except (OSError, ValueError, TypeError, AttributeError):
                    continue
                user.append({"name": str(d.get("name") or fn[:-5]), "builtin": False,
                             "created": d.get("created", ""), "note": str(d.get("note", "")), "values": vals})
            user.sort(key=lambda d: d["created"], reverse=True)
            out += user
        return out

    def get(self, name):
        return next((p for p in self.list() if p["name"] == name), None)

    def save(self, name, values, note=""):
        name = (name or "").strip()[:NAME_MAX]
        if not name:
            raise ValueError("give the preset a name")
        if any(p["name"] == name for p in BUILTIN):
            raise ValueError("'%s' is a built-in preset" % name)
        vals = {k: float(v) for k, v in values.items() if tunable(k)}
        if not vals:
            raise ValueError("no tuning parameters to save (robot not connected?)")
        os.makedirs(self.dir, exist_ok=True)
        d = {"name": name, "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "note": str(note)[:200], "values": vals}
        tmp = self._path(name) + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=1, sort_keys=True)
        os.replace(tmp, self._path(name))
        return dict(d, builtin=False)

    def delete(self, name):
        if any(p["name"] == name for p in BUILTIN):
            raise ValueError("built-in presets can't be deleted")
        p = self.get(name)
        if not p:
            raise ValueError("no preset '%s'" % name)
        os.remove(self._path(name))
