"""Saved places on the map (issue #53): "Kitchen", "Couch", "Home". Stored with the map
(<map>.places.json next to <map>.json in ~/sbr3-maps), in the map frame (the occupancy grid's frame).

"Home" is the one special place: save a place called Home to move it, delete it to fall back to the
default (where the robot's pose was last reset, the odometry origin).
"""
import json
import os
import re

HOME = "Home"
MAX_PLACES = 50
MAX_NAME = 30
FORMAT = "sbr3-places-1"
_BAD = re.compile(r"[\x00-\x1f\x7f<>]")


def clean_name(name):
    """Trim / collapse spaces; raises ValueError for an empty or too long name. 'home' (any case) -> 'Home'."""
    s = " ".join(_BAD.sub(" ", str(name if name is not None else "")).split())
    if not s:
        raise ValueError("a place needs a name")
    if len(s) > MAX_NAME:
        raise ValueError("place name: %d characters at most" % MAX_NAME)
    return HOME if s.lower() == HOME.lower() else s


class Places:
    def __init__(self):
        self.items = []          # [{"name", "x", "y"}] map frame, mm (Home is one of them once saved)
        self.origin = None       # [x, y] map frame: where the pose was last reset (None = the map origin)

    # ------------------------------------------------------------ queries
    def find(self, name):
        key = str(name or "").strip().lower()
        return next((p for p in self.items if p["name"].lower() == key), None)

    def home(self, default_xy=(0.0, 0.0)):
        """(x, y, is_default)"""
        p = self.find(HOME)
        if p:
            return p["x"], p["y"], False
        o = self.origin if self.origin else default_xy
        return float(o[0]), float(o[1]), True

    def listing(self, default_xy=(0.0, 0.0)):
        """What the browsers get: Home first (always there), then the others in the order saved."""
        hx, hy, dflt = self.home(default_xy)
        out = [{"name": HOME, "x": round(hx), "y": round(hy), "home": True, "default": dflt}]
        out += [{"name": p["name"], "x": round(p["x"]), "y": round(p["y"])} for p in self.items if p["name"] != HOME]
        return out

    # ------------------------------------------------------------ changes
    def add(self, name, x, y):
        name = clean_name(name)
        x, y = float(x), float(y)
        if not (abs(x) < 1e7 and abs(y) < 1e7):
            raise ValueError("bad position")
        p = self.find(name)
        if p:
            if name != HOME:
                raise ValueError("there is already a place called %s" % p["name"])
            p["x"], p["y"] = x, y
        else:
            if len(self.items) >= MAX_PLACES:
                raise ValueError("%d places is the limit" % MAX_PLACES)
            self.items.append({"name": name, "x": x, "y": y})
        return name

    def rename(self, old, new):
        p = self.find(old)
        if not p:
            raise ValueError("no place called %s" % old)
        if p["name"] == HOME:
            raise ValueError("Home keeps its name (save a new Home to move it)")
        new = clean_name(new)
        if new == HOME:
            raise ValueError("Home is the special place: use Set home")
        other = self.find(new)
        if other and other is not p:
            raise ValueError("there is already a place called %s" % other["name"])
        p["name"] = new
        return new

    def delete(self, name):
        p = self.find(name)
        if not p:
            raise ValueError("no place called %s" % name)
        self.items.remove(p)
        return p["name"]

    # ------------------------------------------------------------ files
    def to_json(self):
        return json.dumps({"format": FORMAT, "origin": self.origin,
                           "places": [{"name": p["name"], "x": round(p["x"], 1), "y": round(p["y"], 1)}
                                      for p in self.items]}, indent=1)

    @classmethod
    def from_json(cls, text):
        d = json.loads(text)
        if not isinstance(d, dict) or d.get("format") != FORMAT:
            raise ValueError("not a places file")
        pl = cls()
        for p in d.get("places") or []:
            try:
                pl.add(p["name"], p["x"], p["y"])
            except (KeyError, TypeError, ValueError):
                continue                      # one bad entry must not lose the others
        o = d.get("origin")
        if isinstance(o, (list, tuple)) and len(o) == 2:
            try:
                pl.origin = [float(o[0]), float(o[1])]
            except (TypeError, ValueError):
                pass
        return pl

    @staticmethod
    def write(path, pl):
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(pl.to_json())
        os.replace(tmp, path)

    @classmethod
    def read(cls, path):
        try:
            with open(path, encoding="utf-8") as f:
                return cls.from_json(f.read())
        except (OSError, ValueError):
            return cls()
