"""Fusion 360 script: dump geometry + CAD mass properties of the robot to JSON.

Read-only: it inspects the active design and writes one JSON file; it changes nothing.
Run it from Utilities > Scripts and Add-Ins (or the Fusion MCP), with the robot
design active. Set OUT_PATH first.

Units in the JSON: mm, g, kg*m^2 (Fusion internal units are cm, kg, kg*cm^2).
Inertia tensors are about the WORLD ORIGIN, in tensor form (off-diagonals = -integral(xy dm)).
CAD masses use whatever material Fusion has on each body (mostly the default "Steel"),
so they are NOT real masses; build_model.py replaces them with estimates.
"""
import adsk.core, adsk.fusion, json, datetime, os

# Writes next to this script (Fusion sets __file__); change if you run it from elsewhere.
OUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fusion_dump.json")

CM = 10.0            # cm -> mm
KGCM2 = 1e-4         # kg*cm^2 -> kg*m^2


def body_props(b, acc):
    pp = b.getPhysicalProperties(acc)
    c = pp.centerOfMass
    r = pp.getXYZMomentsOfInertia()  # (ok, xx, yy, zz, xy, yz, xz)
    mn, mx = b.boundingBox.minPoint, b.boundingBox.maxPoint
    return dict(
        mass=pp.mass, volume=pp.volume, com=(c.x, c.y, c.z), I=list(r[1:7]),
        bb=((mn.x, mn.y, mn.z), (mx.x, mx.y, mx.z)),
        material=b.material.name if b.material else None,
        density_g_cm3=pp.density * 1000.0,
    )


def aggregate(items):
    m = sum(i["mass"] for i in items)
    v = sum(i["volume"] for i in items)
    com = [sum(i["mass"] * i["com"][k] for i in items) / m if m else 0 for k in range(3)]
    I = [sum(i["I"][k] for i in items) for k in range(6)]
    lo = [min(i["bb"][0][k] for i in items) for k in range(3)]
    hi = [max(i["bb"][1][k] for i in items) for k in range(3)]
    mats = sorted({i["material"] or "None" for i in items})
    return dict(
        n_bodies=len(items),
        materials=mats,
        volume_cm3=round(v, 4),
        cad_mass_g=round(m * 1000, 3),
        com_mm=[round(x * CM, 3) for x in com],
        I_origin_cad_kgm2=[x * KGCM2 for x in I],
        bbox_mm=dict(min=[round(x * CM, 2) for x in lo], max=[round(x * CM, 2) for x in hi]),
        size_mm=[round((hi[k] - lo[k]) * CM, 2) for k in range(3)],
    )


def solid_bodies(occ):
    out = []
    for b in occ.bRepBodies:
        if b.isSolid:
            out.append(b)
    for c in occ.childOccurrences:
        out.extend(solid_bodies(c))
    return out


def frame(occ):
    o, x, y, z = occ.transform2.getAsCoordinateSystem()
    r = lambda v: [round(v.x, 6), round(v.y, 6), round(v.z, 6)]
    return dict(origin_mm=[round(o.x * CM, 3), round(o.y * CM, 3), round(o.z * CM, 3)],
                x_axis=r(x), y_axis=r(y), z_axis=r(z))


def run(_context: str):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    doc = app.activeDocument
    root = design.rootComponent
    acc = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy

    comps = []
    for top in root.occurrences:
        bodies = solid_bodies(top)
        props = [body_props(b, acc) for b in bodies]
        entry = dict(name=top.name, component=top.component.name,
                     visible=top.isLightBulbOn, frame=frame(top))
        if props:
            entry.update(aggregate(props))
        # one level of children (useful for the PCB stack-up)
        kids = []
        for c in top.childOccurrences:
            cp = [body_props(b, acc) for b in solid_bodies(c)]
            if cp:
                k = dict(name=c.name, frame=frame(c))
                k.update(aggregate(cp))
                kids.append(k)
        if kids:
            entry["children"] = kids
        # per-body detail for small parts (wheels: tyre vs rim, IMU chip, ...)
        if len(bodies) <= 6:
            entry["bodies"] = []
            for b, p in zip(bodies, props):
                d = aggregate([p])
                d["name"] = b.assemblyContext.fullPathName + "/" + b.name if b.assemblyContext else b.name
                entry["bodies"].append(d)
        comps.append(entry)

    df = doc.dataFile
    out = dict(
        document=doc.name,
        version=df.versionNumber if df else None,
        date_modified=(datetime.datetime.utcfromtimestamp(df.dateModified).isoformat() + "Z") if df else None,
        dumped_utc=datetime.datetime.utcnow().isoformat() + "Z",
        units=dict(length="mm", mass="g", inertia="kg*m^2 about world origin, tensor form"),
        axes="Fusion world: Z up, Y = wheel axle, X = fore/aft (pitch plane is XZ). Origin = axle centre.",
        components=comps,
    )
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print("wrote", OUT_PATH, len(comps), "components")
