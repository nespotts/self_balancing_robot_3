"""Fusion 360 script: builds the LD19 top mount (issue #75) into Self_Balancing_Model_Rev3.

Adds NEW components only (never touches the owner's parts):
  LD19 Top base v1            printed
  LD19 Deck v1                printed
  LD19 proxy (not printed)    simple LD19 model (datasheet dims)
  Reference - cables & battery (not printed)  battery, Velcro strap, leads, USB, LD19 cable
  LD19 scan plane (check only) thin disc at the scan height + fall-line sketches
Re-running deletes and rebuilds those five components. Units in this file: mm (Fusion API: cm).
Robot frame: origin = axle centre, X fore/aft (+X = Teensy side), Y = axle, Z up.
"""
import adsk.core, adsk.fusion, math

V = adsk.core.ValueInput.createByReal
NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# ---- key heights (mm above axle) ----
ZF0 = 101.25            # underside of the top piece = top of the Frame (unchanged interface)
ZTUN0, ZTUN1 = 103.25, 107.25   # Velcro strap tunnel in the floor (4 mm, like the original)
ZF1 = 109.25            # floor top = battery bottom
BATT = (35.0, 109.5, 27.5)       # battery x, y, z (CAD '2200mAh Lipo')
BX0, BX1 = -16.0, 19.0           # battery x range (same as the original part)
ZBT = ZF1 + BATT[2]     # 136.75 battery top
ZEW = ZBT + 0.5         # end-wall top (strap + USB pass over it)
ZD0 = ZBT + 6.0         # 142.75 deck underside: 6 mm gap for strap (y<0) and USB cable (y>=0)
ZD1 = ZD0 + 4.0         # 146.75 deck top = LD19 base
LD_HOLES = [(23.7, 0.0), (-8.2, 23.4), (-8.2, -23.4)]   # LD19 M2 holes, connector toward -X
BOSSES = [(-38, -61.5), (-38, 61.5), (38, -61.5), (40.5, 61.5)]   # +Y rear boss moved out of the lead path
FRAME_SCREWS = [(x, y) for x in (-15, 15) for y in (-39, 39)]
# original frame_top features, measured on the owner's part, re-used relative to the battery floor
STRAP_Y = (-30.5, -10.5)                 # 20 mm Velcro strap
LEAD_Y, LEAD_Z = (43.0, 57.0), (ZF1 + 3.5, ZF1 + 23.5)   # original battery-lead opening 14 x 20, R2.5 corners


def P(sk, x, y, z=0.0):
    return sk.modelToSketchSpace(adsk.core.Point3D.create(x / 10, y / 10, z / 10))


def rrect_pts(sk, a0, b0, a1, b1, r, f):
    """rounded rectangle in a sketch; f(a, b) -> model (x, y, z)."""
    L = sk.sketchCurves.sketchLines; A = sk.sketchCurves.sketchArcs
    Q = lambda a, b: P(sk, *f(a, b))
    if r <= 0:
        p = [Q(a0, b0), Q(a1, b0), Q(a1, b1), Q(a0, b1)]
        l0 = L.addByTwoPoints(p[0], p[1]); l1 = L.addByTwoPoints(l0.endSketchPoint, p[2])
        l2 = L.addByTwoPoints(l1.endSketchPoint, p[3]); L.addByTwoPoints(l2.endSketchPoint, l0.startSketchPoint)
        return
    # 3-point arcs (orientation independent)
    c = math.cos(math.pi / 4) * r
    arcs = []
    for (ca, cb, sa, sb) in ((a1 - r, b1 - r, 1, 1), (a0 + r, b1 - r, -1, 1), (a0 + r, b0 + r, -1, -1), (a1 - r, b0 + r, 1, -1)):
        arcs.append((Q(ca + sa * r, cb), Q(ca + sa * c, cb + sb * c), Q(ca, cb + sb * r)))
    made = [A.addByThreePoints(*arc) for arc in arcs]
    # connect: arc0 end (a1-r, b1) -> arc1 end? use nearest endpoints
    def ends(a):
        return [a.startSketchPoint, a.endSketchPoint]
    for i in range(4):
        e0, e1 = made[i], made[(i + 1) % 4]
        best = None
        for p in ends(e0):
            for q in ends(e1):
                dd = p.geometry.distanceTo(q.geometry)
                if best is None or dd < best[0]: best = (dd, p, q)
        L.addByTwoPoints(best[1], best[2])


def rrect(sk, x0, y0, x1, y1, r):
    rrect_pts(sk, x0, y0, x1, y1, r, lambda a, b: (a, b, 0.0))


def circ(sk, x, y, d):
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(sk, x, y), d / 20)


def profs(sk, min_area=None):
    oc = adsk.core.ObjectCollection.create()
    for p in sk.profiles:
        oc.add(p)
    return oc


def ext(comp, sk, start, h, op, body=None, name=None):
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(profs(sk), op)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(V(start / 10))
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(h / 10)),
                         adsk.fusion.ExtentDirections.PositiveExtentDirection)
    if body is not None:
        inp.participantBodies = [body]
    f = ef.add(inp)
    if name: f.name = name
    return f


def sketch(comp, plane, name):
    sk = comp.sketches.add(plane); sk.name = name; sk.isVisible = False; return sk


# ---------------- printed part 1: top base ----------------
def build_base(root):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component; c.name = 'LD19 Top base v1'
    XY = c.xYConstructionPlane
    sk = sketch(c, XY, 'outline'); rrect(sk, -45, -60, 45, 60, 3)
    body = ext(c, sk, ZF0, ZD0 - ZF0, NEW, name='outline').bodies.item(0)
    body.name = 'LD19 Top base'
    sk = sketch(c, XY, 'bay'); rrect(sk, -46, -57, 46, 57, 0)
    ext(c, sk, ZF1, ZD0 - ZF1 + 1, CUT, body, 'battery bay')
    sk = sketch(c, XY, 'end wall'); rrect(sk, BX1, -57, BX1 + 3, 57, 0)
    ext(c, sk, ZF1, ZEW - ZF1, JOIN, body, 'end wall (battery stop)')
    # rear wall: turns the rear floor rail (a fall-impact edge) into a deep L-beam tied to the deck;
    # runs side wall to side wall at x 42..45; the lead path (x 23..36) stays open
    sk = sketch(c, XY, 'rear wall'); rrect(sk, 42, -57, 45, 57, 0)
    ext(c, sk, ZF1, ZD0 - ZF1, JOIN, body, 'rear wall (stiffens the rear rail)')
    sk = sketch(c, c.yZConstructionPlane, 'rear wall window')
    pts = [(-14, ZF1 + 4), (14, ZF1 + 4), (14, ZF1 + 15), (0, ZF1 + 29), (-14, ZF1 + 15)]
    sp = [P(sk, 0.0, y, z) for y, z in pts]
    Ls = sk.sketchCurves.sketchLines
    first = Ls.addByTwoPoints(sp[0], sp[1]); prev = first
    for q in sp[2:]:
        prev = Ls.addByTwoPoints(prev.endSketchPoint, q)
    Ls.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
    ext(c, sk, 41, 5, CUT, body, 'rear wall window, 45 deg top (view/reach the Teensy USB)')
    sk = sketch(c, XY, 'bosses')
    for x, y in BOSSES: circ(sk, x, y, 9)
    ext(c, sk, ZF0, ZD0 - ZF0, JOIN, body, 'deck bosses')
    sk = sketch(c, XY, 'M3 insert holes')
    for x, y in BOSSES: circ(sk, x, y, 4.05)
    ext(c, sk, ZD0 - 6.5, 7, CUT, body, 'M3 insert 4.05 x 6.5')
    sk = sketch(c, XY, 'M3 clearance')
    for x, y in FRAME_SCREWS: circ(sk, x, y, 3.5)
    ext(c, sk, ZF0 - 1, 12, CUT, body, 'M3 clearance 3.5 (to Frame)')
    sk = sketch(c, XY, 'M3 counterbore')
    for x, y in FRAME_SCREWS: circ(sk, x, y, 6.0)
    ext(c, sk, ZF1 - 3.2, 4, CUT, body, 'M3 counterbore 6.0 x 3.2')
    sk = sketch(c, XY, 'front slot'); rrect(sk, -29, -32.5, -17, 32.5, 4)
    ext(c, sk, ZF0 - 1, 12, CUT, body, 'front slot (strap, Pi USB plug, LD19 cable) - as original')
    sk = sketch(c, XY, 'strap tunnel'); rrect(sk, -19, STRAP_Y[0], 25, STRAP_Y[1], 0)
    ext(c, sk, ZTUN0, ZTUN1 - ZTUN0, CUT, body, 'Velcro strap tunnel 20.5 x 4 - as original')
    sk = sketch(c, XY, 'rear window'); rrect(sk, 23, -52, 35, 52, 4)   # leaves a 10 mm rear rail
    ext(c, sk, ZF0 - 1, 12, CUT, body, 'rear floor window (Teensy USB plug, strap, leads)')
    sk = sketch(c, XY, 'centre window'); rrect(sk, -8, -6, 8, 30, 4)
    ext(c, sk, ZF0 - 1, 12, CUT, body, 'centre lightening window')
    # battery-lead opening through the end wall (same size/position vs battery as the original)
    YZ = c.yZConstructionPlane
    sk = sketch(c, YZ, 'battery lead opening')
    # same width and bottom height as the original 14 x 20 opening, but open at the top so the leads
    # lift out sideways when the battery is swapped (and no bridge to print)
    rrect_pts(sk, LEAD_Y[0], LEAD_Z[0], LEAD_Y[1], ZEW + 5, 2.5, lambda a, b: (0.0, a, b))
    ext(c, sk, BX1 - 2, 7, CUT, body, 'battery lead slot 14 wide, open top (original position)')
    # battery swap door: the +Y side wall is open over the battery's cross-section (open top, no bridge)
    # (+Y wall open from the battery front to the rear boss: battery, leads, T-plug and balance plug slide out together)
    sk = sketch(c, XY, 'battery swap opening'); rrect(sk, BX0 - 1, 56, 36, 61, 0)
    ext(c, sk, ZF1, ZD0 - ZF1 + 1, CUT, body, 'battery + leads swap opening (+Y side)')
    sk = sketch(c, XY, 'lead floor notch'); rrect(sk, 23, 40, 36, 61, 0)
    ext(c, sk, ZF0 - 1, 12, CUT, body, 'lead floor notch, open to +Y (T-plug passes)')
    # 1 mm chamfers on the vertical edges of the lead / swap openings (edge by edge; tiny edges that
    # cannot take 1 mm are skipped)
    def opening_edges():
        out = []
        for e in body.edges:
            g = e.geometry
            if not isinstance(g, adsk.core.Line3D): continue
            a, b = g.startPoint, g.endPoint
            if abs(a.x - b.x) > 1e-4 or abs(a.y - b.y) > 1e-4 or abs(a.z - b.z) < 0.3: continue
            x, y = a.x * 10, a.y * 10
            if y > 39 and (-18.5 < x < -15.5 or 18 < x < 37):
                out.append((round(x, 2), round(y, 2), round(min(a.z, b.z) * 10, 2), e))
        return out
    keys = [k[:3] for k in opening_edges()]
    done = 0
    for k in keys:
        cur = [e for e in opening_edges() if e[:3] == k]
        if not cur: continue
        oc = adsk.core.ObjectCollection.create(); oc.add(cur[0][3])
        ci = c.features.chamferFeatures.createInput2()
        ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(oc, V(0.1), False)
        try:
            c.features.chamferFeatures.add(ci).name = 'opening chamfer 1.0'
            done += 1
        except RuntimeError:
            pass
    print('opening edges chamfered %d of %d' % (done, len(keys)))
    sk = sketch(c, YZ, 'USB notch')
    rrect_pts(sk, -5, ZBT - 3.25, 7, ZEW + 2, 0, lambda a, b: (0.0, a, b))
    ext(c, sk, BX1 - 2, 7, CUT, body, 'USB cable notch in end wall')
    return occ, body


# ---------------- printed part 2: deck ----------------
def build_deck(root):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component; c.name = 'LD19 Deck v1'
    XY = c.xYConstructionPlane
    T = ZD1 - ZD0
    sk = sketch(c, XY, 'outline'); rrect(sk, -45, -66, 45, 66, 6)
    body = ext(c, sk, ZD0, T, NEW, name='deck plate').bodies.item(0)
    body.name = 'LD19 Deck'
    sk = sketch(c, XY, 'M3 clearance')
    for x, y in BOSSES: circ(sk, x, y, 3.4)
    ext(c, sk, ZD0 - 1, T + 2, CUT, body, 'M3 clearance 3.4')
    sk = sketch(c, XY, 'M2 insert holes')
    for x, y in LD_HOLES: circ(sk, x, y, 3.2)
    ext(c, sk, ZD0 - 1, T + 2, CUT, body, 'M2 insert 3.2 thru')
    sk = sketch(c, XY, 'cable hole'); rrect(sk, -33, -7, -24, 7, 4)
    ext(c, sk, ZD0 - 1, T + 2, CUT, body, 'LD19 cable hole')
    sk = sketch(c, XY, 'windows')
    rrect(sk, -12, 32, 14, 52, 5); rrect(sk, -12, -52, 14, -32, 5); rrect(sk, 29, -40, 39, 40, 4)
    ext(c, sk, ZD0 - 1, T + 2, CUT, body, 'windows (rear one = reach the Teensy USB plug)')
    edges = adsk.core.ObjectCollection.create()
    for e in body.edges:
        bb = e.boundingBox
        if abs(bb.minPoint.z - ZD1 / 10) < 1e-6 and abs(bb.maxPoint.z - ZD1 / 10) < 1e-6:
            if bb.maxPoint.x > 4.49 or bb.minPoint.x < -4.49 or bb.maxPoint.y > 6.59 or bb.minPoint.y < -6.59:
                edges.add(e)
    ci = c.features.chamferFeatures.createInput2()
    ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, V(0.15), True)
    c.features.chamferFeatures.add(ci).name = 'top edge chamfer 1.5'
    return occ, body


# ---------------- LD19 proxy ----------------
def build_proxy(root):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component; c.name = 'LD19 proxy (not printed)'
    XY = c.xYConstructionPlane
    sk = sketch(c, XY, 'body'); rrect(sk, -19.3, -19.3, 19.3, 19.3, 4)
    lb = ext(c, sk, ZD1, 22.2, NEW, name='LD19 body').bodies.item(0); lb.name = 'LD19'
    sk = sketch(c, XY, 'ears')
    rrect(sk, 15, -3.5, 23.7, 3.5, 0)
    for s in (1, -1): rrect(sk, -11.7, min(15 * s, 23.4 * s), -4.7, max(15 * s, 23.4 * s), 0)
    for x, y in LD_HOLES: circ(sk, x, y, 7)
    ext(c, sk, ZD1, 3, JOIN, lb, 'ears')
    sk = sketch(c, XY, 'ear holes')
    for x, y in LD_HOLES: circ(sk, x, y, 2.5)
    ext(c, sk, ZD1 - 1, 5, CUT, lb, 'ear holes 2.5')
    sk = sketch(c, XY, 'head'); circ(sk, 0, 0, 35.29)
    ext(c, sk, ZD1 + 22.2, 12.6, JOIN, lb, 'rotating head 35.29')
    sk = sketch(c, XY, 'plug'); rrect(sk, -27, -3.5, -19.3, 3.5, 0)
    ext(c, sk, ZD1 + 3.5, 4.5, JOIN, lb, 'ZH1.5 connector + plug')
    return occ, lb


# ---------------- reference bodies (BRep, not printed) ----------------
def tb():
    return adsk.fusion.TemporaryBRepManager.get()


def p3(x, y, z):
    return adsk.core.Point3D.create(x / 10, y / 10, z / 10)


def box(x0, y0, z0, x1, y1, z1):
    c = p3((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    ob = adsk.core.OrientedBoundingBox3D.create(c, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
                                                (x1 - x0) / 10, (y1 - y0) / 10, (z1 - z0) / 10)
    return tb().createBox(ob)


def union(bodies):
    t = tb(); out = bodies[0]
    for b in bodies[1:]:
        t.booleanOperation(out, b, adsk.fusion.BooleanTypes.UnionBooleanType)
    return out


def pipe(pts, r):
    t = tb(); parts = []
    for a, b in zip(pts[:-1], pts[1:]):
        if math.dist(a, b) < 1e-6: continue
        parts.append(t.createCylinderOrCone(p3(*a), r / 10, p3(*b), r / 10))
    for p in pts[1:-1]:
        parts.append(t.createSphere(p3(*p), r / 10))
    return union(parts)


def arc(c, r, a0, a1, plane='xz', y=0.0, n=8):
    """points on an arc in the XZ plane (angles in deg, 0 = +X, 90 = +Z)."""
    out = []
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        out.append((c[0] + r * math.cos(a), y, c[1] + r * math.sin(a)))
    return out


def build_reference(root):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component; c.name = 'Reference - cables & battery (not printed)'
    items = {}
    items['battery 35 x 109.5 x 27.5'] = box(BX0, -BATT[1] / 2, ZF1, BX1, BATT[1] / 2, ZBT)
    sy0, sy1 = STRAP_Y[0] + 0.5, STRAP_Y[1] - 0.5
    items['Velcro strap 20 mm loop'] = union([
        box(-20, sy0, ZBT + 0.75, 25, sy1, ZBT + 2.75),       # over the battery and the end wall
        box(-20, sy0, ZTUN0 + 1, -18, sy1, ZBT + 2.75),      # down the battery front, through the front slot
        box(-20, sy0, ZTUN0 + 1, 25, sy1, ZTUN0 + 3),         # in the floor tunnel
        box(23, sy0, ZTUN0 + 1, 25, sy1, ZBT + 2.75)])        # up behind the end wall
    # battery leads: bundle from the battery +X/+Y corner, through the lead opening, down the rear window
    ly, lz = 47.0, (LEAD_Z[0] + LEAD_Z[1]) / 2
    leads = [(BX1, ly, lz), (22.5, ly, lz)] + [(p[0], ly, p[2]) for p in arc((22.5, lz - 6), 6, 90, 0, n=4)][1:] + \
            [(28.5, ly - 1, 100), (28.5, 46, 96)]
    items['battery main leads (bundle)'] = union([pipe(leads, 3.5), box(24, 41.5, 70, 33, 50.5, 96)])
    items['balance lead + plug'] = union([pipe([(BX1, 46, lz + 6), (28, 46, lz + 6)], 1.5),
                                          box(24, 41, lz + 3, 34, 51, lz + 9)])
    # Pi USB-C (port top at z 92.6, y 21.55) -> over the battery under the deck -> Teensy micro-USB (y 0.2)
    py, ty = 21.55, 0.2
    ux, tx = -22.1, 27.7
    zt = ZBT + 3.5         # cable centre over the battery (4.5 mm cable, 0.25 mm under the deck)
    fr = 10.0              # front bend radius
    br = 10.0              # rear bend radius
    pts = [(ux, py, 120.0), (ux, py, zt - fr)]
    pts += [(p[0], py, p[2]) for p in arc((ux + fr, zt - fr), fr, 180, 90)][1:]
    pts += [(tx - br, ty, zt)]
    pts += [(p[0], ty, p[2]) for p in arc((tx - br, zt - br), br, 90, 0)][1:]
    pts += [(tx, ty, 116.0)]
    usb = pipe(pts, 2.25)
    items['USB cable Pi -> Teensy'] = union([usb,
        box(ux - 3.5, py - 6, 92.6, ux + 3.5, py + 6, 114.6), tb().createCylinderOrCone(p3(ux, py, 114.6), 0.35, p3(ux, py, 120), 0.3),
        box(tx - 3.5, ty - 5.5, 90.0, tx + 3.5, ty + 5.5, 110.0), tb().createCylinderOrCone(p3(tx, ty, 110), 0.35, p3(tx, ty, 116), 0.3)])
    # LD19 cable: plug -> deck hole -> front slot -> Pi header (bottom edge of the Pi)
    lz0 = ZD1 + 5.75
    items['LD19 cable (4-pin)'] = pipe([(-27, 0, lz0), (-29.5, 0, lz0), (-29.5, 0, ZD0 - 3), (-27, 0, ZF0 + 2),
                                        (-27, 0, 95), (-27.5, -5, 70), (-26, -8, 66)], 1.5)
    bf = c.features.baseFeatures.add()
    bf.startEdit()
    for name, b in items.items():
        nb = c.bRepBodies.add(b, bf)
        nb.name = name
    bf.finishEdit()
    bf.name = 'reference bodies'
    return occ


# ---------------- scan plane + fall-line sketches ----------------
def build_check(root):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    c = occ.component; c.name = 'LD19 scan plane (check only)'
    zs = ZD1 + 27.0
    sk = sketch(c, c.xYConstructionPlane, 'scan disc'); circ(sk, 0, 0, 240); circ(sk, 0, 0, 36.5)
    prof = max(sk.profiles, key=lambda p: p.areaProperties().area)   # the annulus
    ef = c.features.extrudeFeatures
    inp = ef.createInput(prof, NEW)
    inp.startExtent = adsk.fusion.OffsetStartDefinition.create(V((zs - 0.1) / 10))
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(V(0.02)), adsk.fusion.ExtentDirections.PositiveExtentDirection)
    b = ef.add(inp).bodies.item(0); b.name = 'scan plane z=%.2f (r 18.25-120)' % zs
    b.isLightBulbOn = False
    # fall lines: fore/aft (wheel tangent -> floor corner of the base), sideways (tyre corner -> deck corner)
    sk = sketch(c, c.xZConstructionPlane, 'fall lines (side view)')
    L = sk.sketchCurves.sketchLines
    sk.sketchCurves.sketchCircles.addByCenterRadius(P(sk, 0, 0, 0), 3.446)
    for s in (1, -1):
        t = (34.28 * s, -3.51); q = (45 * s, ZF0)
        k = (205 - t[1]) / (q[1] - t[1])
        L.addByTwoPoints(P(sk, t[0] - (q[0] - t[0]) * 0.25, 0, t[1] - (q[1] - t[1]) * 0.25), P(sk, t[0] + (q[0] - t[0]) * k, 0, 205))
    L.addByTwoPoints(P(sk, -120, 0, zs), P(sk, 120, 0, zs)); L.addByTwoPoints(P(sk, -120, 0, -34.46), P(sk, 120, 0, -34.46))
    sk2 = sketch(c, c.yZConstructionPlane, 'side-tip lines (front view)')
    L2 = sk2.sketchCurves.sketchLines
    for s in (1, -1):
        a = (75.9 * s, 34.46); b = (66 * s, ZD1 - 1.5)
        k = (205 - a[1]) / (b[1] - a[1])
        L2.addByTwoPoints(P(sk2, 0, 75.9 * s, -34.46), P(sk2, 0, a[0], a[1]))
        L2.addByTwoPoints(P(sk2, 0, a[0], a[1]), P(sk2, 0, a[0] + (b[0] - a[0]) * k, 205))
    L2.addByTwoPoints(P(sk2, 0, -120, zs), P(sk2, 0, 120, zs))
    return occ, zs


def appearance(app, d, words):
    for lib in app.materialLibraries:
        if 'Appearance' not in lib.name: continue
        for i in range(lib.appearances.count):
            a = lib.appearances.item(i)
            if all(w.lower() in a.name.lower() for w in words):
                return d.appearances.itemByName(a.name) or d.appearances.addByCopy(a, a.name)
    return None


def run(_context: str):
    app = adsk.core.Application.get()
    d = adsk.fusion.Design.cast(app.activeProduct)
    if d.rootComponent.name != 'Self_Balancing_Model_Rev3' and 'Self_Balancing' not in app.activeDocument.name:
        raise Exception('wrong document active: ' + app.activeDocument.name)
    root = d.rootComponent
    mine = ('LD19 Top base', 'LD19 Deck', 'LD19 proxy', 'LD19 scan plane', 'Reference - cables')
    for o in [o for o in root.occurrences if o.component.name.startswith(mine)]:
        o.deleteMe()
    base_occ, base = build_base(root)
    deck_occ, deck = build_deck(root)
    prox_occ, ld = build_proxy(root)
    ref_occ = build_reference(root)
    chk_occ, zs = build_check(root)
    col = {'LD19 Top base': ['Plastic', 'Glossy', 'Yellow'], 'LD19 Deck': ['Plastic', 'Glossy', 'Blue'],
           'LD19': ['Plastic', 'Glossy', 'Black']}
    for b in (base, deck, ld):
        a = appearance(app, d, col[b.name])
        if a: b.appearance = a
    refcol = {'battery main': ['Plastic', 'Glossy', 'Black'], 'battery': ['Plastic', 'Matte', 'Green'],
              'Velcro': ['Plastic', 'Glossy', 'Red'], 'balance': ['Plastic', 'Matte', 'White'],
              'USB': ['Plastic', 'Matte', 'Grey'], 'LD19 cable': ['Plastic', 'Glossy', 'Purple']}
    for b in ref_occ.component.bRepBodies:
        for k, w in refcol.items():
            if b.name.startswith(k):
                a = appearance(app, d, w) or appearance(app, d, ['Plastic', 'Glossy', 'White'])
                if a: b.appearance = a
                break
    a = appearance(app, d, ['Glass', 'Clear'])
    if a: chk_occ.component.bRepBodies.item(0).appearance = a
    for b in (base, deck, ld):
        bb = b.boundingBox
        print(b.name, 'vol cm3 %.2f' % b.physicalProperties.volume,
              'bbox', [round(v * 10, 2) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)])
    print('scan plane z', zs, 'deck top', ZD1)
