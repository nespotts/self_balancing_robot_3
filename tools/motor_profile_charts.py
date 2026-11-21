#!/usr/bin/env python3
"""Charts for docs/motor-profile.md (issue #26). Stdlib only, writes SVG.

  python3 tools/motor_profile_charts.py MTLINES RAMPS.csv OUTDIR [--vbatt 12]

MTLINES: text with the "mt,..." log lines of the runs (e.g. grep of a session log).
RAMPS.csv: cfg,t,duty,speed,phase rows of the test-1 ramps (right wheel, from the telemetry).
Writes OUTDIR/motor-deadzone.svg (start/stop volts per drive config) and
OUTDIR/motor-speed.svg (wheel speed vs motor volts while turning).
"""
import collections, csv, re, sys

COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]  # dataviz categorical slots 1-5
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"


def label(cfg):
    d, hz = cfg.split(":")
    hz = int(float(hz))
    return ("brake" if d == "1" else "coast") + " " + ("%.3g kHz" % (hz / 1000.0))


def parse(mtfile, vb):
    runs, cur = [], None
    for ln in open(mtfile, encoding="utf-8", errors="ignore"):
        i = ln.find("mt,")
        if i < 0:
            continue
        f = ln[i:].strip().split(",")
        if f[1] == "begin":
            hz = re.search(r"hz=([0-9.]+)", ln)
            dr = re.search(r"drive=([0-9])", ln)
            cur = {"cfg": "%s:%d" % (dr.group(1) if dr else "0", round(float(hz.group(1))) if hz else 36621),
                   "a": {}, "b": [], "ok": False}
            runs.append(cur)
        elif cur is None:
            continue
        elif f[1] == "a" and f[2] == "right":
            duty = float(f[5])
            if duty >= 0:
                cur["a"][(f[3], f[4])] = duty / 4095 * vb
        elif f[1] == "b" and f[2] == "right" and f[3] == "fwd":
            cur["b"].append((float(f[4]) / 4095 * vb, abs(float(f[7]))))
        elif f[1] == "end":
            cur["ok"] = True
            cur = None
        elif f[1] == "abort":
            cur = None
    best = collections.OrderedDict()
    for r in runs:
        if r["ok"]:
            best[r["cfg"]] = r
    return best


def svg(w, h, body, title):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
            'font-family="system-ui,Segoe UI,Arial,sans-serif" font-size="13">\n<title>%s</title>\n'
            '<rect width="100%%" height="100%%" fill="%s"/>\n%s</svg>\n' % (w, h, w, h, title, SURF, body))


def deadzone_chart(runs):
    cfgs = list(runs)
    W, H, L, R, T, rowh = 720, 70 + 46 * len(cfgs), 120, 30, 46, 46
    x = lambda v: L + (W - L - R) * v / 8.0
    b = ['<text x="%d" y="22" fill="%s" font-size="15" font-weight="600">Dead zone, right wheel: '
         'stops below / starts above (motor volts)</text>' % (L - 100, INK)]
    for v in range(0, 9):
        b.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s"/>' % (x(v), T - 6, x(v), H - 34, GRID))
        b.append('<text x="%.1f" y="%d" fill="%s" text-anchor="middle">%d V</text>' % (x(v), H - 16, INK2, v))
    for i, c in enumerate(cfgs):
        a = runs[c]["a"]
        st = [a[k] for k in a if k[1] == "start"]
        sp = [a[k] for k in a if k[1] == "stop"]
        if not st or not sp:
            continue
        s0, s1 = sum(sp) / len(sp), sum(st) / len(st)
        y = T + 14 + i * rowh
        col = COLORS[i % len(COLORS)]
        b.append('<text x="%d" y="%d" fill="%s" text-anchor="end">%s</text>' % (L - 12, y + 5, INK, label(c)))
        b.append('<rect x="%.1f" y="%d" width="%.1f" height="14" rx="4" fill="%s" opacity="0.35"/>' %
                 (x(s0), y - 7, x(s1) - x(s0), col))
        for v in (s0, s1):
            b.append('<circle cx="%.1f" cy="%d" r="6" fill="%s" stroke="%s" stroke-width="2"/>' % (x(v), y, col, SURF))
        b.append('<text x="%.1f" y="%d" fill="%s" text-anchor="end">%.1f</text>' % (x(s0) - 10, y + 5, INK2, s0))
        b.append('<text x="%.1f" y="%d" fill="%s">%.1f</text>' % (x(s1) + 10, y + 5, INK2, s1))
    return svg(W, H, "\n".join(b), "Dead zone per drive config")


def speed_chart(runs, ramps, vb):
    W, H, L, R, T, B = 720, 420, 70, 110, 46, 50
    vmax, smax = 12.0, 1500.0
    x = lambda v: L + (W - L - R) * v / vmax
    y = lambda s: H - B - (H - B - T) * s / smax
    down = collections.defaultdict(lambda: collections.defaultdict(list))
    for r in ramps:
        if r["phase"] == "down":
            v = abs(float(r["duty"])) / 4095 * vb
            down[r["cfg"]][round(v * 4) / 4].append(abs(float(r["speed"])))
    b = ['<text x="%d" y="22" fill="%s" font-size="15" font-weight="600">Right wheel speed vs motor volts '
         'while turning</text>' % (L, INK)]
    for v in range(0, 13, 2):
        b.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s"/>' % (x(v), T, x(v), H - B, GRID))
        b.append('<text x="%.1f" y="%d" fill="%s" text-anchor="middle">%d V</text>' % (x(v), H - B + 20, INK2, v))
    for s in range(0, 1501, 250):
        b.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="%s"/>' % (L, y(s), W - R, y(s), GRID))
        b.append('<text x="%d" y="%.1f" fill="%s" text-anchor="end">%d</text>' % (L - 8, y(s) + 4, INK2, s))
    b.append('<text x="%d" y="%d" fill="%s">mm/s</text>' % (L - 40, T - 10, INK2))
    labels = []
    for i, c in enumerate(runs):
        col = COLORS[i % len(COLORS)]
        steady = [(v, s) for v, s in runs[c]["b"] if s > 20]
        if not steady:
            continue
        vmove = min(v for v, _ in steady)
        low = []
        for v, ss in sorted(down[c].items()):  # rising part only (the top of the ramp still has the start kick in it)
            s = sum(ss) / len(ss)
            if len(ss) <= 2 or v <= 0 or v >= vmove - 0.3:
                continue
            if low and s < low[-1][1]:
                break
            low.append((v, s))
        pts = sorted(low + steady)
        path = " ".join("%s%.1f,%.1f" % ("M" if j == 0 else "L", x(v), y(s)) for j, (v, s) in enumerate(pts))
        b.append('<path d="%s" fill="none" stroke="%s" stroke-width="2" stroke-linejoin="round"/>' % (path, col))
        for v, s in steady:
            b.append('<circle cx="%.1f" cy="%.1f" r="4" fill="%s" stroke="%s" stroke-width="2"/>' % (x(v), y(s), col, SURF))
        if pts[-1][0] < vmax - 1:  # ends early (duty cap): label above the line, part way along
            mv, ms = pts[int(len(pts) * 0.6)]
            b.append('<text x="%.1f" y="%.1f" fill="%s" text-anchor="end" stroke="%s" stroke-width="4" '
                     'paint-order="stroke">%s</text>' % (x(mv) - 4, y(ms) - 12, INK, SURF, label(c)))
            continue
        labels.append([y(pts[-1][1]), x(pts[-1][0]), label(c), col])
    labels.sort()
    for j in range(1, len(labels)):  # keep end labels apart
        labels[j][0] = max(labels[j][0], labels[j - 1][0] + 16)
    for yy, xx, txt, col in labels:
        b.append('<text x="%.1f" y="%.1f" fill="%s" stroke="%s" stroke-width="4" paint-order="stroke">%s</text>' %
                 (xx + 10, yy + 4, INK, SURF, txt))
    b.append('<text x="%d" y="%d" fill="%s" font-size="12">dots: steady 1.2 s steps from rest (test 2); '
             'line below the first dot: slow ramp down while still turning (test 1)</text>' % (L, H - 8, INK2))
    return svg(W, H, "\n".join(b), "Wheel speed vs motor volts")


def main():
    a = sys.argv[1:]
    vb = 12.0
    if "--vbatt" in a:
        i = a.index("--vbatt")
        vb = float(a[i + 1])
        del a[i:i + 2]
    mtfile, rampfile, outdir = a
    runs = parse(mtfile, vb)
    ramps = list(csv.DictReader(open(rampfile)))
    open(outdir + "/motor-deadzone.svg", "w", encoding="utf-8").write(deadzone_chart(runs))
    open(outdir + "/motor-speed.svg", "w", encoding="utf-8").write(speed_chart(runs, ramps, vb))
    print("configs:", ", ".join(label(c) for c in runs))


if __name__ == "__main__":
    main()
