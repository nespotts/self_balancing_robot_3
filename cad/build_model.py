"""Build cad/robot_model.json from cad/fusion_dump.json (written by fusion_dump.py).

Almost every body in the Fusion design carries the default material (Steel), so the CAD
masses are meaningless. This script keeps the CAD geometry (volumes, centres of mass,
inertia shapes) and swaps in ESTIMATED masses (catalogue values or volume x assumed density).
Each component's CAD inertia tensor is scaled by est_mass / cad_mass (uniform density inside
the component). Replace the estimates with weighed values in MASS_G and re-run:

    python cad/build_model.py
"""
import json, math, os, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
DUMP = os.path.join(HERE, "fusion_dump.json")
OUT = os.path.join(HERE, "robot_model.json")
G = 9.81

PLA = 1.24          # g/cm^3 solid
FILL_BIG = 0.75     # assumed solid fraction of large FDM prints (walls + infill) -- unknown
FILL_SMALL = 0.90

# (estimated grams, confidence, basis). Keys are top-level occurrence names in the design.
# Weigh the parts and put the real numbers here.
MASS_G = {
    "Frame:1":            (None, "low", f"printed; CAD volume x PLA {PLA} g/cm3 x assumed fill {FILL_BIG}"),
    "frame_top:1":        (None, "low", f"printed; CAD volume x PLA {PLA} g/cm3 x assumed fill {FILL_BIG}"),
    "2200mAh Lipo:1":     (185.0, "medium", "3S 2200 mAh LiPo catalogue mass (firmware monitors 3 cells)"),
    "motor:1":            (30.0, "medium", "yellow 'TT' dual-shaft gear motor, catalogue ~28-32 g"),
    "motor:2":            (30.0, "medium", "yellow 'TT' dual-shaft gear motor, catalogue ~28-32 g"),
    "circuit board assembly:1": (None, "low", "sum of per-part guesses, see children"),
    "Motor Driver:1":     (26.0, "low", "43x43x27 mm H-bridge module (L298N-size), catalogue ~26-30 g"),
    "Orange_Pi_Zero_2W:1": (15.0, "low", "Orange Pi Zero 2W + soldered headers, guess"),
    "SMALLSERVO:1":       (9.0, "medium", "micro servo (SG90 class) catalogue 9 g"),
    "SMALLSERVOHORN:1":   (0.3, "medium", "servo horn"),
    "servo_Lidar_Mount:1": (None, "low", f"printed; CAD volume x PLA x fill {FILL_SMALL}"),
    "vl53l0x:1":          (0.5, "medium", "VL53L0X breakout"),
    "vl53l0x:2":          (0.5, "medium", "VL53L0X breakout"),
    "GY-BNO085:1":        (2.0, "medium", "GY-BNO08x breakout + header"),
    "AS5600 v1:1":        (1.5, "medium", "AS5600 encoder breakout"),
    "AS5600 v1:2":        (1.5, "medium", "AS5600 encoder breakout"),
    "magnet mount:1":     (None, "low", f"printed; CAD volume x PLA x fill {FILL_SMALL}"),
    "magnet mount:2":     (None, "low", f"printed; CAD volume x PLA x fill {FILL_SMALL}"),
    "magnet:1":           (None, "high", "4x1 mm NdFeB disc, 7.5 g/cm3"),
    "magnet:2":           (None, "high", "4x1 mm NdFeB disc, 7.5 g/cm3"),
    # wheels are split into tyre + rim below
    "Wheel D65x25:2":     (None, "medium", "tyre rubber 1.15 g/cm3 + rim plastic 1.05 g/cm3 (CAD volumes)"),
    "Wheel D65x25:3":     (None, "medium", "tyre rubber 1.15 g/cm3 + rim plastic 1.05 g/cm3 (CAD volumes)"),
    "stand:1":            (0.0, "n/a", "parking stand, not part of the robot (excluded)"),
}
VOL_DENSITY = {  # for the None entries above: g/cm^3 applied to CAD volume
    "Frame:1": PLA * FILL_BIG, "frame_top:1": PLA * FILL_BIG,
    "servo_Lidar_Mount:1": PLA * FILL_SMALL,
    "magnet mount:1": PLA * FILL_SMALL, "magnet mount:2": PLA * FILL_SMALL,
    "magnet:1": 7.5, "magnet:2": 7.5,
}
PCB_CHILD_G = {  # circuit board assembly children (guesses)
    "circuit board:1": 13.4,               # 50x91x1.6 FR4
    "Teensy 4.1 (With Headers):1": 6.0,
    "Buck Converter:1": 11.0,              # LM2596 module
    "NRF24L01:1": 2.0,
    "Header_Female_single_15:1": 1.0, "Header_Female_single_15:2": 1.0,
    "2 Pin Terminal Block:1": 1.5, "2 Pin Terminal Block:2": 1.5,
    "Fuse - Mini ATC:1": 1.0, "Fuse - Mini ATC:2": 1.0,
    "Capacitor_470u:1": 2.0, "Capacitor_10uF:1": 0.5, "Capacitor_10uF:2": 0.5,
    "Buzzer:1": 1.0, "Push_Button:1": 0.5, "Push_Button:2": 0.5,
    "CONNECTOR WHITE JST XH 4 PIN - 2,54 mm:1": 0.5,
}
PCB_HEADER_G_PER_CM3 = 1.6  # remaining pin headers
ALLOWANCE = dict(name="unmodelled wires/screws/nuts (allowance)", mass_g=25.0,
                 com_mm=[0.0, 0.0, 70.0], confidence="low",
                 basis="nothing in CAD; guess, spread along the frame")
WHEEL_PARTS = ("Wheel D65x25:2", "Wheel D65x25:3")
ROTATING = WHEEL_PARTS + ("magnet mount:1", "magnet mount:2", "magnet:1", "magnet:2")


def r(x, n=2):
    return round(x, n)


def scaled(I, factor):
    return [i * factor for i in I]


def main():
    d = json.load(open(DUMP, encoding="utf-8"))
    comps = {c["name"]: c for c in d["components"]}
    table, parts = [], []   # parts: (name, mass_kg, com_m, I_origin_kgm2, group)

    for name, c in comps.items():
        est, conf, basis = MASS_G.get(name, (None, "low", "no estimate; CAD volume x 1.2 g/cm3"))
        vol = c.get("volume_cm3", 0.0)
        cad_g = c.get("cad_mass_g", 0.0)
        I = c.get("I_origin_cad_kgm2", [0] * 6)
        com = c.get("com_mm", [0, 0, 0])
        group = "wheel" if name in ROTATING else ("excluded" if name == "stand:1" else "body")
        if name in WHEEL_PARTS:
            est = 0.0
            for b in c["bodies"]:
                dens = 1.15 if "Tire" in b["name"] else 1.05
                m = b["volume_cm3"] * dens
                est += m
                parts.append((name + " " + ("tyre" if "Tire" in b["name"] else "rim"), m / 1000,
                              [x / 1000 for x in b["com_mm"]],
                              scaled(b["I_origin_cad_kgm2"], m / b["cad_mass_g"]), group))
        elif name == "circuit board assembly:1":
            est = 0.0
            for k in c["children"]:
                m = PCB_CHILD_G.get(k["name"], k["volume_cm3"] * PCB_HEADER_G_PER_CM3)
                est += m
                parts.append((name + "/" + k["name"], m / 1000, [x / 1000 for x in k["com_mm"]],
                              scaled(k["I_origin_cad_kgm2"], m / k["cad_mass_g"]), group))
        else:
            if est is None:
                est = vol * VOL_DENSITY.get(name, 1.2)
            if group != "excluded" and cad_g > 0:
                parts.append((name, est / 1000, [x / 1000 for x in com],
                              scaled(I, est / cad_g), group))
        table.append(dict(
            name=name, component=c["component"], group=group,
            est_mass_g=r(est, 1), mass_confidence=conf, mass_basis=basis,
            cad_material=c.get("materials"), cad_mass_g_meaningless=r(cad_g, 1),
            volume_cm3=r(vol, 2), com_mm=com, bbox_mm=c.get("bbox_mm"), size_mm=c.get("size_mm"),
            frame=c["frame"]))

    a = ALLOWANCE
    parts.append((a["name"], a["mass_g"] / 1000, [x / 1000 for x in a["com_mm"]],
                  [0] * 6, "body"))
    table.append(dict(name=a["name"], group="body", est_mass_g=a["mass_g"],
                      mass_confidence=a["confidence"], mass_basis=a["basis"], com_mm=a["com_mm"]))

    def lump(sel):
        ps = [p for p in parts if p[4] in sel]
        m = sum(p[1] for p in ps)
        c = [sum(p[1] * p[2][k] for p in ps) / m for k in range(3)]
        I = [sum(p[3][k] for p in ps) for k in range(6)]  # about world origin (= axle centre)
        # parallel-axis to CoM (tensor form: diag -= m(d^2 - di^2), offdiag += m di dj)
        x, y, z = c
        Ic = [I[0] - m * (y * y + z * z), I[1] - m * (x * x + z * z), I[2] - m * (x * x + y * y),
              I[3] + m * x * y, I[4] + m * y * z, I[5] + m * x * z]
        return m, c, I, Ic

    mb, cb, Ib, Icb = lump({"body"})
    mw, cw, Iw, Icw = lump({"wheel"})
    mt, ct, It, Ict = lump({"body", "wheel"})

    # geometry
    tyre = [b for b in comps["Wheel D65x25:2"]["bodies"] if "Tire" in b["name"]][0]
    tyre2 = [b for b in comps["Wheel D65x25:3"]["bodies"] if "Tire" in b["name"]][0]
    wheel_d = tyre["size_mm"][0]
    tyre_w = tyre["size_mm"][1]
    track = abs(tyre2["com_mm"][1] - tyre["com_mm"][1])
    R = wheel_d / 2
    robot = [c for n, c in comps.items() if n != "stand:1" and "bbox_mm" in c]
    lo = [min(c["bbox_mm"]["min"][k] for c in robot) for k in range(3)]
    hi = [max(c["bbox_mm"]["max"][k] for c in robot) for k in range(3)]
    imu_chip = [b for b in comps["GY-BNO085:1"]["bodies"] if b["size_mm"][2] < 2 and b["size_mm"][0] < 6][0]
    horn = comps["SMALLSERVOHORN:1"]["frame"]
    v1, v2 = comps["vl53l0x:1"], comps["vl53l0x:2"]

    l = math.hypot(cb[0], cb[2])                  # axle -> body CoM distance (m)
    I_axle = Ib[1]                                # body pitch inertia about the axle (kg m^2)
    I_com = Icb[1]
    tau = math.sqrt(I_axle / (mb * G * l))        # linearised fall time constant (wheels held)
    tau_point = math.sqrt(l / G)
    wheel_I_each = Iw[1] / 2                      # about the axle, each wheel (tyre+rim+magnet parts)

    def mm(v): return [r(x * 1000, 1) for x in v]

    out = dict(
        source=dict(
            document=d["document"], fusion_version=d["version"], document_saved_utc=d["date_modified"],
            extracted_utc=d["dumped_utc"], generated_utc=datetime.datetime.utcnow().isoformat() + "Z",
            tools="cad/fusion_dump.py (read-only Fusion script) -> cad/fusion_dump.json -> cad/build_model.py",
            note="Geometry is from CAD (high confidence). Masses are ESTIMATES: nearly every body in the "
                 "design has the default 'Steel' material, so Fusion's own masses (6.8 kg total) are meaningless."),
        frame=dict(origin="axle centre (midpoint between the wheels)", x="fore/aft, +X = PCB/Teensy side "
                   "(which side the firmware calls 'forward' is not in the CAD - confirm)",
                   y="along the axle (wheel axis)", z="up", pitch_axis="Y", units="mm, g, kg*m^2"),
        key=dict(
            wheel_diameter_mm=dict(value=r(wheel_d, 1), confidence="medium",
                                   notes="tyre tread tips in CAD ('Wheel D65x25', nominal 65 mm); rolling diameter "
                                         "is a little less - measure the distance per wheel turn"),
            wheel_width_mm=r(tyre_w, 1),
            track_width_mm=dict(value=r(track, 1), confidence="high",
                                notes="tyre centre to tyre centre"),
            axle_height_mm=dict(value=r(R, 1), confidence="medium", notes="= wheel radius (axle above ground)"),
            overall_mm=dict(height_above_ground=r(hi[2] + R, 1), width_y=r(hi[1] - lo[1], 1),
                            depth_x=r(hi[0] - lo[0], 1),
                            bbox_min=[r(x, 1) for x in lo], bbox_max=[r(x, 1) for x in hi],
                            notes="robot only (parking stand excluded)"),
            mass_total_g=dict(value=r(mt * 1000, 0), confidence="low", notes="estimate; weigh the robot"),
            com_total_mm=dict(value=mm(ct), confidence="low"),
            com_height_above_axle_mm=dict(value=r(cb[2] * 1000, 1), confidence="low",
                                          notes="body (everything except wheels); depends mostly on the "
                                                "printed frame masses and the battery"),
            com_fore_aft_offset_mm=dict(value=r(cb[0] * 1000, 1), confidence="low",
                                        notes="+X; gives a static balance tilt of atan(x/z)"),
            static_balance_tilt_deg=dict(value=r(math.degrees(math.atan2(cb[0], cb[2])), 1), confidence="low",
                                         notes="the CoM is not straight above the axle in the CAD pose; to balance, the body "
                                               "tilts by this angle so the CoM moves over the axle (top toward -X "
                                               "when the offset is +X)"),
            pitch_inertia_body_about_axle_kgm2=dict(value=r(I_axle, 6), confidence="low"),
            pitch_inertia_body_about_com_kgm2=dict(value=r(I_com, 6), confidence="low"),
        ),
        sensors=dict(
            imu=dict(part="GY-BNO085 (BNO08x)", chip_centre_mm=imu_chip["com_mm"],
                     height_above_axle_mm=r(imu_chip["com_mm"][2], 1),
                     orientation="board flat (horizontal, chip up), CAD rotation = identity; header on the +Y "
                                 "edge. Chip axes vs robot axes are not knowable from the CAD box model - "
                                 "check by tilting (pitch is about CAD Y = axle)",
                     confidence="high for position, unknown for axis signs"),
            range_head=dict(parts="2x VL53L0X back to back on a micro servo (SMALLSERVO)",
                            servo_axis=dict(point_mm=horn["origin_mm"], direction=[0, 0, 1],
                                            notes="vertical axis through the horn"),
                            sensor_centres_mm=[v1["com_mm"], v2["com_mm"]],
                            facing=["+X", "-X"],
                            radius_from_servo_axis_mm=r(abs(v1["com_mm"][0] - horn["origin_mm"][0]), 1),
                            height_above_axle_mm=r(v1["com_mm"][2], 1),
                            height_above_ground_mm=r(v1["com_mm"][2] + R, 1),
                            fore_aft_offset_from_axle_mm=r(horn["origin_mm"][0], 1)),
        ),
        placements_mm={k: comps[n]["com_mm"] for k, n in {
            "teensy_4_1": None, "orange_pi_zero_2w": "Orange_Pi_Zero_2W:1", "battery_3s_2200": "2200mAh Lipo:1",
            "motor_neg_y": "motor:1", "motor_pos_y": "motor:2", "motor_driver": "Motor Driver:1",
            "encoder_as5600_pos_y": "AS5600 v1:1", "encoder_as5600_neg_y": "AS5600 v1:2",
            "servo": "SMALLSERVO:1"}.items() if n},
        inverted_pendulum=dict(
            m_body_kg=r(mb, 3), m_wheels_total_kg=r(mw, 4),
            l_axle_to_body_com_m=r(l, 4),
            I_body_pitch_about_com_kgm2=r(I_com, 6),
            I_body_pitch_about_axle_kgm2=r(I_axle, 6),
            I_wheel_each_about_axle_kgm2=r(wheel_I_each, 8),
            wheel_radius_m=r(R / 1000, 4),
            fall_time_constant_s=r(tau, 3),
            fall_time_constant_point_mass_s=r(tau_point, 3),
            notes=["tau = sqrt(I_axle / (m g l)): tilt error grows ~e^(t/tau) with wheels held still",
                   "wheel inertia here is tyre + rim + magnet mount only; the TT gearbox (~1:48) reflects "
                   "motor rotor inertia roughly N^2 times larger at the wheel, likely bigger than the wheel itself "
                   "- not in the CAD",
                   "all masses are estimates (confidence low) until parts are weighed"]),
        components=table,
        to_weigh_or_measure=[
            "whole robot (kitchen scale) - fixes the total",
            "Frame and frame_top prints (biggest uncertainty: unknown infill)",
            "battery (3S 2200 mAh)",
            "one wheel with tyre; one TT motor",
            "PCB with everything on it; motor driver; Orange Pi",
            "CoM height: balance the robot on its side on a pencil/edge, or hang it from two points",
            "rolling diameter: mark the tyre, roll 10 turns, distance / (10*pi)",
            "track width: tyre centre to tyre centre with calipers",
            "IMU axis signs: tilt forward and log pitch",
        ],
    )
    # Teensy lives inside the PCB assembly
    pcb = comps["circuit board assembly:1"]
    teensy = [k for k in pcb["children"] if k["name"].startswith("Teensy")][0]
    out["placements_mm"]["teensy_4_1"] = teensy["com_mm"]
    out["placements_mm"]["pcb_assembly"] = pcb["com_mm"]
    out["key"]["firmware_compare"] = dict(
        wheel_diameter=dict(firmware_mm=66.0, cad_mm=r(wheel_d, 1), diff_pct=r((wheel_d / 66.0 - 1) * 100, 1)),
        track_width=dict(firmware_mm=123.0, cad_mm=r(track, 1), diff_pct=r((track / 123.0 - 1) * 100, 1)),
        notes="firmware: src/Odometry/wheel.h wheel_diameter=66.0, src/Odometry/Odometry.h wheel_separation=123.0")
    json.dump(out, open(OUT, "w", encoding="utf-8", newline="\n"), indent=2)
    print(json.dumps(out["key"], indent=1))
    print(json.dumps(out["inverted_pendulum"], indent=1))
    print(json.dumps(out["sensors"], indent=1))
    print(json.dumps(out["placements_mm"]))
    for t in table:
        print(f'{t["name"]:<32} {t["est_mass_g"]:>7} g  {t["mass_confidence"]}')


if __name__ == "__main__":
    main()
