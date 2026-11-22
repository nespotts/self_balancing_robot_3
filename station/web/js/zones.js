// Virtual walls, no-go zones, fence, slow zones and mazes on the map (issue #70).
// Map ⋯ → Draw: pick a tool, tap points (or drag a rectangle), ✓ to add; tap a shape to select it (drag its
// handles, change a slow zone's speed, delete it); Maze: tap a spot → Generate, then Solve. The bridge stores
// them with the map and enforces them (planner + watchdog); this file only draws and edits.
import { state, on, flag } from "./conn.js";
import { toast } from "./ui.js";
import { mapToOdom, cmd, plan } from "./occmap.js";

export const zones = { items: [], undo: 0 };
const $ = (id) => document.getElementById(id);
const COL = { wall: "#a3241c", wallEdge: "#ff8f7a", nogo: "#e0413f", fence: "#4fd1a1", slow: "#e8a317", draft: "#f0efec",
  entry: "#22a447", exit: "#e8a317" };
const NAMES = { wall: "Virtual wall", nogo: "No-go zone", fence: "Fence (keep-in)", slow: "Slow zone", maze: "Maze" };
const HINT = {
  wall: "Tap points along the wall, then ✓",
  nogo: "Drag a rectangle (▭) or tap corners (✎), then ✓",
  fence: "The area the robot must stay in: drag or tap corners, ✓",
  slow: "Speed cap inside: drag or tap corners, ✓",
  maze: "Tap where the maze goes",
  delete: "Tap a wall or zone to delete it",
};
const AREA = new Set(["nogo", "fence", "slow"]);

let api = null;            // {sx, sy, wx, wy, redraw} from map.js
let tool = null;           // null = not drawing
let shape = "rect";        // area tools: rect (drag) | pts (tap corners)
let draft = [];            // odometry mm
let sel = null;            // selected item id
let drag = null;           // {kind: "rect"|"handle", ...}
let mazeAt = null;         // [x, y] odometry: where a new maze goes
let edit = null;           // {id, pts} while a handle is dragged (local preview)

export function bindZones(a) { api = a; }
export function zoneDrawing() { return tool !== null; }
const redraw = () => api && api.redraw();

on("zones", (m) => {
  zones.items = m.items || []; zones.undo = m.undo || 0;
  if (sel && !byId(sel)) sel = null;
  renderSheet(); redraw();
});
const byId = (id) => zones.items.find((z) => z.id === id);
const od = (p) => mapToOdom(p[0], p[1]);

// ---------------------------------------------------------------- drawing
let hatch = null;
function hatchPattern(ctx) {
  if (hatch) return hatch;
  const c = document.createElement("canvas"); c.width = c.height = 12;
  const g = c.getContext("2d");
  g.strokeStyle = "rgba(224,65,63,.75)"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(-3, 15); g.lineTo(15, -3); g.moveTo(-3, 3); g.lineTo(3, -3); g.moveTo(9, 15); g.lineTo(15, 9); g.stroke();
  return (hatch = ctx.createPattern(c, "repeat"));
}
function poly(ctx, pts, close) {
  ctx.beginPath();
  pts.forEach((p, i) => { const X = api.sx(p[0]), Y = api.sy(p[1]); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); });
  if (close) ctx.closePath();
}
function wallLine(ctx, segs, w) {
  ctx.lineCap = "round"; ctx.lineJoin = "round";
  for (const [lw, col] of [[w + 3, COL.wallEdge], [w, COL.wall]]) {
    ctx.lineWidth = lw; ctx.strokeStyle = col;
    ctx.beginPath();
    for (const s of segs) { ctx.moveTo(api.sx(s[0][0]), api.sy(s[0][1])); for (const p of s.slice(1)) ctx.lineTo(api.sx(p[0]), api.sy(p[1])); }
    ctx.stroke();
  }
}
function label(ctx, text, X, Y, col) {
  ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.lineWidth = 3; ctx.strokeStyle = "rgba(21,21,20,.9)"; ctx.strokeText(text, X, Y);
  ctx.fillStyle = col; ctx.fillText(text, X, Y);
}
function centroid(pts) { let x = 0, y = 0; for (const p of pts) { x += p[0]; y += p[1]; } return [x / pts.length, y / pts.length]; }
function ptsOf(z) { return (edit && edit.id === z.id ? edit.pts : z.pts.map(od)); }

export function drawZones(ctx, sx, sy, s) {
  if (!api || (!zones.items.length && !tool)) return;
  ctx.save();
  const wW = Math.max(4, 30 * s);
  for (const z of zones.items) {
    if (z.kind === "slow") {
      const P = ptsOf(z);
      poly(ctx, P, true); ctx.fillStyle = "rgba(232,163,23,.16)"; ctx.fill();
      ctx.setLineDash([]); ctx.lineWidth = 2; ctx.strokeStyle = COL.slow; ctx.stroke();
      const [cx, cy] = centroid(P); label(ctx, `≤ ${z.speed} mm/s`, sx(cx), sy(cy), COL.slow);
    }
  }
  for (const z of zones.items) {
    if (z.kind === "nogo") {
      const P = ptsOf(z);
      poly(ctx, P, true); ctx.fillStyle = hatchPattern(ctx); ctx.fill();
      ctx.lineWidth = 2.5; ctx.strokeStyle = COL.nogo; ctx.stroke();
    } else if (z.kind === "fence") {
      const P = ptsOf(z);
      poly(ctx, P, true); ctx.setLineDash([12, 8]); ctx.lineWidth = 3; ctx.strokeStyle = COL.fence; ctx.stroke(); ctx.setLineDash([]);
    }
  }
  for (const z of zones.items) {
    if (z.kind === "wall") wallLine(ctx, [ptsOf(z)], wW);
    else if (z.kind === "maze") {
      wallLine(ctx, z.walls.map((w) => [od([w[0], w[1]]), od([w[2], w[3]])]), Math.max(3, 25 * s));
      const f = 0.5 / 0.9, [ex, ey] = od([z.entry_in[0] + (z.entry[0] - z.entry_in[0]) * f, z.entry_in[1] + (z.entry[1] - z.entry_in[1]) * f]);
      const [xx, xy] = od(z.exit);
      label(ctx, "IN", sx(ex), sy(ey), COL.entry);       // in the entrance gap
      ctx.beginPath(); ctx.arc(sx(xx), sy(xy), Math.max(6, 90 * s), 0, 7); ctx.fillStyle = "rgba(232,163,23,.25)"; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = COL.exit; ctx.stroke();
      label(ctx, z.best_s != null ? `EXIT · best ${z.best_s} s` : "EXIT", sx(xx), sy(xy), COL.exit);
    }
  }
  // selection: outline + handles
  const z = sel && byId(sel);
  if (z && tool) {
    if (z.kind === "maze") {
      const a = od([z.x0, z.y0]), b = od([z.x0 + z.cols * z.cell, z.y0 + z.rows * z.cell]);
      ctx.setLineDash([6, 5]); ctx.lineWidth = 2; ctx.strokeStyle = "#f0efec";
      ctx.strokeRect(Math.min(sx(a[0]), sx(b[0])) - 6, Math.min(sy(a[1]), sy(b[1])) - 6, Math.abs(sx(b[0]) - sx(a[0])) + 12, Math.abs(sy(b[1]) - sy(a[1])) + 12);
      ctx.setLineDash([]);
    } else {
      for (const p of ptsOf(z)) handle(ctx, sx(p[0]), sy(p[1]));
    }
  }
  // draft (being drawn) and the rectangle being dragged
  const D = drag && drag.kind === "rect" ? rectPts(drag.a, drag.b) : draft;
  if (tool && D.length) {
    poly(ctx, D, AREA.has(tool) && D.length > 2);
    ctx.setLineDash([8, 6]); ctx.lineWidth = 2.5; ctx.strokeStyle = COL[tool] || COL.draft; ctx.stroke(); ctx.setLineDash([]);
    for (const p of D) { ctx.beginPath(); ctx.arc(sx(p[0]), sy(p[1]), 5, 0, 7); ctx.fillStyle = COL.draft; ctx.fill(); }
  }
  if (tool === "maze" && mazeAt) {
    const m = mazeForm(), w = m.w * 1000 / 2, h = m.h * 1000 / 2;
    ctx.setLineDash([8, 6]); ctx.lineWidth = 2; ctx.strokeStyle = COL.wallEdge;
    ctx.strokeRect(sx(mazeAt[0] - w), sy(mazeAt[1] + h), 2 * w * s, 2 * h * s); ctx.setLineDash([]);
  }
  ctx.restore();
}
function handle(ctx, X, Y) {
  ctx.beginPath(); ctx.arc(X, Y, 9, 0, 7); ctx.fillStyle = "#f0efec"; ctx.fill();
  ctx.lineWidth = 2.5; ctx.strokeStyle = "#151514"; ctx.stroke();
}
const rectPts = (a, b) => [[a[0], a[1]], [b[0], a[1]], [b[0], b[1]], [a[0], b[1]]];

// ---------------------------------------------------------------- hit test (px)
function segDist(px, py, ax, ay, bx, by) {
  const ex = bx - ax, ey = by - ay, L = ex * ex + ey * ey;
  const u = L ? Math.max(0, Math.min(1, ((px - ax) * ex + (py - ay) * ey) / L)) : 0;
  return Math.hypot(ax + u * ex - px, ay + u * ey - py);
}
function insidePx(px, py, P) {
  let c = false;
  for (let i = 0, j = P.length - 1; i < P.length; j = i++) {
    const [xi, yi] = P[i], [xj, yj] = P[j];
    if ((yi > py) !== (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) c = !c;
  }
  return c;
}
function hitItem(px, py) {
  const { sx, sy } = api;
  for (const z of [...zones.items].reverse()) {
    if (z.kind === "maze") {
      const a = od([z.x0, z.y0]), b = od([z.x0 + z.cols * z.cell, z.y0 + z.rows * z.cell]);
      const X0 = Math.min(sx(a[0]), sx(b[0])), X1 = Math.max(sx(a[0]), sx(b[0])), Y0 = Math.min(sy(a[1]), sy(b[1])), Y1 = Math.max(sy(a[1]), sy(b[1]));
      if (px > X0 - 10 && px < X1 + 10 && py > Y0 - 10 && py < Y1 + 10) return z;
      continue;
    }
    const P = z.pts.map(od).map((p) => [sx(p[0]), sy(p[1])]);
    const n = z.kind === "wall" ? P.length - 1 : P.length;
    for (let i = 0; i < n; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      if (segDist(px, py, a[0], a[1], b[0], b[1]) < 14) return z;
    }
    if (z.kind !== "wall" && z.kind !== "fence" && insidePx(px, py, P)) return z;
  }
  return null;
}
function hitHandle(px, py) {
  const z = sel && byId(sel);
  if (!z || z.kind === "maze") return -1;
  const P = z.pts.map(od);
  let best = -1, bd = 22;
  P.forEach((p, i) => { const d = Math.hypot(px - api.sx(p[0]), py - api.sy(p[1])); if (d < bd) { bd = d; best = i; } });
  return best;
}

// ---------------------------------------------------------------- pointer (map.js forwards these while drawing)
export const zoneDraw = {
  /** pointerdown: true = this gesture is ours (handle drag / rectangle); false = map.js pans as usual */
  down(px, py) {
    if (!tool) return false;
    const h = hitHandle(px, py);
    if (h >= 0) { const z = byId(sel); edit = { id: z.id, pts: z.pts.map(od) }; drag = { kind: "handle", i: h }; return true; }
    if (AREA.has(tool) && shape === "rect" && !draft.length) {
      const p = [api.wx(px), api.wy(py)];
      drag = { kind: "rect", a: p, b: p, px, py, moved: false };
      return true;
    }
    return false;
  },
  move(px, py) {
    if (!drag) return;
    const p = [api.wx(px), api.wy(py)];
    if (drag.kind === "handle") edit.pts[drag.i] = p;
    else { drag.b = p; drag.moved = drag.moved || Math.hypot(px - drag.px, py - drag.py) > 8; }
    redraw();
  },
  up(px, py) {
    const d = drag; drag = null;
    if (!d) return;
    if (d.kind === "handle") {
      const pts = edit.pts.map((p) => [Math.round(p[0]), Math.round(p[1])]);
      cmd({ t: "zones", action: "update", id: edit.id, pts });
      setTimeout(() => { edit = null; redraw(); }, 400);   // keep the preview until the bridge's copy is back
    } else if (d.moved) {
      const P = rectPts(d.a, d.b);
      if (Math.abs(d.b[0] - d.a[0]) < 100 || Math.abs(d.b[1] - d.a[1]) < 100) toast("Too small - drag a bigger box");
      else addShape(P);
    } else this.tap(px, py);
    redraw();
  },
  cancel() { drag = null; edit = null; redraw(); },
  /** a plain tap while drawing */
  tap(px, py) {
    if (!tool) return;
    const p = [api.wx(px), api.wy(py)];
    if (tool === "delete") {
      const z = hitItem(px, py);
      if (z) { cmd({ t: "zones", action: "delete", id: z.id }); toast(`${NAMES[z.kind]} deleted - Undo brings it back`, 2500); }
      return;
    }
    if (!draft.length) {
      const z = hitItem(px, py);
      if (z && (tool !== "maze" || z.kind === "maze")) { sel = z.id; mazeAt = null; renderSheet(); redraw(); return; }
      if (sel) { sel = null; renderSheet(); }
    }
    if (tool === "maze") { mazeAt = p; renderSheet(); redraw(); return; }
    if (AREA.has(tool) && draft.length > 2 && Math.hypot(px - api.sx(draft[0][0]), py - api.sy(draft[0][1])) < 18) { finish(); return; }
    draft.push(p); updateBar(); redraw();
  },
};

function addShape(P) {
  const pts = P.map((p) => [Math.round(p[0]), Math.round(p[1])]);
  const m = { t: "zones", action: "add", kind: tool, pts };
  if (tool === "slow") m.speed = +($("zs-speed-new")?.value || lastSpeed);
  if (tool === "fence" && zones.items.some((z) => z.kind === "fence") && !confirm("Replace the fence?")) return;
  cmd(m);
  draft = []; updateBar();
}
function finish() {
  if (tool === "wall" ? draft.length < 2 : draft.length < 3) return toast(tool === "wall" ? "A wall needs 2 points" : "Tap at least 3 corners");
  addShape(draft);
}

// ---------------------------------------------------------------- draw bar + sheet
const bar = $("draw-bar"), hint = $("draw-hint"), sheet = $("zone-sheet");
const bShape = $("draw-shape"), bUndo = $("draw-undo"), bFin = $("draw-finish");
let lastSpeed = 120;
function setTool(t) {
  tool = t; draft = []; drag = null; mazeAt = null;
  for (const b of bar.querySelectorAll("[data-tool]")) b.classList.toggle("on", b.dataset.tool === t);
  updateBar(); renderSheet(); redraw();
}
function updateBar() {
  hint.textContent = draft.length ? `${draft.length} point${draft.length > 1 ? "s" : ""}` + (AREA.has(tool) && draft.length > 2 ? " · tap the first to close" : "") : HINT[tool] || "";
  bShape.hidden = !AREA.has(tool);
  bShape.textContent = shape === "rect" ? "▭" : "✎";
  bShape.title = shape === "rect" ? "Rectangle: drag (tap for points)" : "Points: tap corners (tap for rectangle)";
  bFin.hidden = !draft.length;
  bUndo.disabled = !draft.length && !zones.undo;
}
export function startDrawing() {
  document.body.classList.add("drawing"); bar.hidden = false; $("map-menu").hidden = true;
  setTool(tool || "wall");
}
function stopDrawing() {
  tool = null; sel = null; draft = []; mazeAt = null;
  document.body.classList.remove("drawing"); bar.hidden = true; sheet.hidden = true; redraw();
}
for (const b of bar.querySelectorAll("[data-tool]")) b.onclick = () => setTool(b.dataset.tool);
bShape.onclick = () => { shape = shape === "rect" ? "pts" : "rect"; draft = []; updateBar(); redraw(); };
bUndo.onclick = () => { if (draft.length) { draft.pop(); updateBar(); redraw(); } else cmd({ t: "zones", action: "undo" }); };
bFin.onclick = finish;
$("draw-done").onclick = stopDrawing;
$("map-draw").onclick = startDrawing;
on("zones", updateBar);

function mazeForm() {
  const g = (id, d) => { const v = parseFloat($(id)?.value); return isFinite(v) ? v : d; };
  return { w: g("zm-w", 3), h: g("zm-h", 3), cell: g("zm-cell", 700) };
}
function esc(s) { return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c])); }
function renderSheet() {
  const z = sel && byId(sel);
  if (!tool || (!z && !(tool === "maze" && mazeAt))) { sheet.hidden = true; return; }
  let html;
  if (!z) {          // new maze
    const m = mazeForm();
    html = `<div class="ps-head"><b>New maze</b><small>entrance faces the robot</small><button class="ghost small" data-x="close">✕</button></div>
      <div class="zs-form"><label>W <input id="zm-w" type="number" min="1" max="8" step="0.5" value="${m.w}"> m</label>
      <label>H <input id="zm-h" type="number" min="1" max="8" step="0.5" value="${m.h}"> m</label>
      <label>Cell <input id="zm-cell" type="number" min="400" max="1500" step="50" value="${m.cell}"> mm</label></div>
      <div class="ps-row"><button class="primary" data-x="gen">Generate</button></div>`;
  } else if (z.kind === "maze") {
    html = `<div class="ps-head"><b>Maze ${z.cols} × ${z.rows}</b><small>${z.cell} mm cells${z.best_s != null ? " · best " + z.best_s + " s" : ""}</small><button class="ghost small" data-x="close">✕</button></div>
      <div class="ps-row"><button class="primary" data-x="solve">Solve ▶</button><button data-x="regen">New maze</button><button class="warn" data-x="del">Clear</button></div>`;
  } else {
    html = `<div class="ps-head"><b>${esc(NAMES[z.kind])}</b><small>drag the handles</small><button class="ghost small" data-x="close">✕</button></div>
      <div class="ps-row">${z.kind === "slow" ? `<label class="zs-speed">≤ <input id="zs-speed" type="number" min="30" max="500" step="10" value="${z.speed}"> mm/s</label>` : ""}
      <button class="warn" data-x="del">Delete</button></div>`;
  }
  sheet.innerHTML = html; sheet.hidden = false;
  for (const id of ["zm-w", "zm-h", "zm-cell"]) { const el = $(id); if (el) el.oninput = redraw; }
  const sp = $("zs-speed");
  if (sp) sp.onchange = () => { lastSpeed = +sp.value; cmd({ t: "zones", action: "update", id: z.id, speed: +sp.value }); };
  sheet.querySelectorAll("[data-x]").forEach((b) => { b.onclick = () => sheetAction(b.dataset.x, z, b); });
}
function sheetAction(x, z, btn) {
  if (x === "close") { sel = null; mazeAt = null; renderSheet(); redraw(); return; }
  if (x === "del") { cmd({ t: "zones", action: "delete", id: z.id }); sel = null; renderSheet(); return; }
  if (x === "gen") {
    const m = mazeForm();
    if (m.cell < 400) return toast("Cell width: 400 mm at least (the robot has to fit)");
    cmd({ t: "zones", action: "maze", x: Math.round(mazeAt[0]), y: Math.round(mazeAt[1]), w: m.w * 1000, h: m.h * 1000, cell: m.cell });
    mazeAt = null; renderSheet(); return;
  }
  if (x === "regen") {
    const c = [z.x0 + z.cols * z.cell / 2, z.y0 + z.rows * z.cell / 2], o = od(c);
    cmd({ t: "zones", action: "delete", id: z.id });
    cmd({ t: "zones", action: "maze", x: Math.round(o[0]), y: Math.round(o[1]), w: z.cols * z.cell, h: z.rows * z.cell, cell: z.cell });
    sel = null; renderSheet(); return;
  }
  if (x === "solve") {
    if (!state.connected) return toast("Not connected");
    const t = state.tlm, disarmed = t && !flag("armed");
    let arm = false;
    if (disarmed && !t.radio_ok) {
      if (!confirm("The robot is disarmed.\nArm it and drive through the maze to the exit?")) return;
      arm = true;
    } else if (!confirm("Solve: the robot drives through the maze to the exit by itself. STOP ends it. Go?")) return;
    cmd({ t: "plan", action: "solve", maze: z.id, arm });
    stopDrawing();
  }
}

// ---------------------------------------------------------------- maze timer + watchdog alerts
const timer = $("maze-timer");
let t0 = 0;
on("plan", (m) => {
  if (m.solve) { t0 = performance.now() - m.solve.elapsed * 1000; timer.hidden = false; }
  else timer.hidden = true;
});
setInterval(() => { if (!timer.hidden) timer.textContent = `Maze ⏱ ${((performance.now() - t0) / 1000).toFixed(1)} s`; }, 100);
let lastAlert = null;           // (no toast for an old alert when the page connects)
on("status", (m) => {
  if (lastAlert !== null && m.zone_alert && m.zone_alert !== lastAlert) toast("Zones: " + m.zone_alert, 3500);
  lastAlert = m.zone_alert || "";
});
export function solving() { return !!plan.solve; }
