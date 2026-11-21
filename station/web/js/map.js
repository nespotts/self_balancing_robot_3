// Map view: robot pose, track, range points (fading with age), waypoints. Pan / zoom / tap to add.
import { state, on, send, modeName, flag } from "./conn.js";
import { toast } from "./ui.js";
import { SENS_RGB, SENS_NEW } from "./chart.js";
import { drawOccMap, drawPlan, planActive, plan, onChange, routeOn, hasMap } from "./occmap.js";

const C = {
  bg: "#151514", grid: "#262624", grid2: "#30302d", axis: "#45453f",
  track: "#3987e5",
  robot: "#f0efec", heading: "#3987e5", wp: "#9085e9", wpPending: "#c3c2b7", target: "#d95926", text: "#f0efec",
};
const ROBOT = { l: 90, w: 152, track: 126.8, wheel: 66 };  // mm, from the CAD
const FADE_MS = 120000;

const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const view = { cx: 0, cy: 0, s: 0.12, follow: true };   // s = px per mm
const layers = { ranges: true, track: true, grid: true };
let pending = [];
let dirty = true, W = 0, H = 0, dpr = 1;
let piNowBase = 0, piNowAt = 0;

export function invalidate() { dirty = true; }

function resize() {
  dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth; H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  dirty = true;
}
new ResizeObserver(resize).observe(canvas);

const sx = (x) => W / 2 + (x - view.cx) * view.s;
const sy = (y) => H / 2 - (y - view.cy) * view.s;
const wx = (px) => view.cx + (px - W / 2) / view.s;
const wy = (py) => view.cy - (py - H / 2) / view.s;

function piNow() { return piNowBase + (performance.now() - piNowAt); }

function niceStep() {
  for (const st of [50, 100, 250, 500, 1000, 2000, 5000, 10000]) if (st * view.s >= 45) return st;
  return 20000;
}

function drawGrid() {
  const st = niceStep();
  const x0 = Math.floor(wx(0) / st) * st, x1 = wx(W), y0 = Math.floor(wy(H) / st) * st, y1 = wy(0);
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.strokeStyle = C.grid;
  for (let x = x0; x <= x1; x += st) { const p = Math.round(sx(x)) + 0.5; ctx.moveTo(p, 0); ctx.lineTo(p, H); }
  for (let y = y0; y <= y1; y += st) { const p = Math.round(sy(y)) + 0.5; ctx.moveTo(0, p); ctx.lineTo(W, p); }
  ctx.stroke();
  ctx.beginPath();
  ctx.strokeStyle = C.axis;
  const ox = Math.round(sx(0)) + 0.5, oy = Math.round(sy(0)) + 0.5;
  ctx.moveTo(ox, 0); ctx.lineTo(ox, H); ctx.moveTo(0, oy); ctx.lineTo(W, oy);
  ctx.stroke();
  // scale bar (bottom-left, above the waypoint bar)
  const len = st * view.s, bx = 16, by = H - 86;
  ctx.strokeStyle = C.text; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(bx, by - 5); ctx.lineTo(bx, by); ctx.lineTo(bx + len, by); ctx.lineTo(bx + len, by - 5); ctx.stroke();
  ctx.fillStyle = C.text; ctx.font = "12px system-ui, sans-serif"; ctx.textBaseline = "bottom";
  ctx.fillText(st >= 1000 ? st / 1000 + " m" : st + " mm", bx + len + 6, by + 2);
}

function drawTrack() {
  const tr = state.track;
  if (tr.length < 2) return;
  ctx.strokeStyle = C.track; ctx.globalAlpha = 0.7; ctx.lineWidth = 2; ctx.lineJoin = "round";
  ctx.beginPath();
  ctx.moveTo(sx(tr[0][0]), sy(tr[0][1]));
  for (let i = 1; i < tr.length; i++) ctx.lineTo(sx(tr[i][0]), sy(tr[i][1]));
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawRanges() {
  const pts = state.ranges, now = piNow();
  const size = Math.max(2.5, Math.min(6, 25 * view.s));
  // per sensor (p[8]: 0 front, 1 rear; older bridges send no id -> front colour)
  const buckets = [0, 1].map(() => [[], [], [], [], [], []]), fresh = [[], []];
  for (const p of pts) {
    if (!p[5] || p[6]) continue;   // invalid or taken while tilted
    const s = p[8] ? 1 : 0, age = now - p[7];
    if (age < 1200) { fresh[s].push(p); continue; }
    const a = Math.max(0, Math.min(5, Math.floor((1 - age / FADE_MS) * 6)));
    buckets[s][a].push(p);
  }
  const hs = size / 2;
  buckets.forEach((bs, s) => {
    const [r, g, b] = SENS_RGB[s];
    bs.forEach((list, k) => {
      if (!list.length) return;
      ctx.fillStyle = `rgba(${r},${g},${b},${0.16 + k * 0.13})`;
      for (const p of list) ctx.fillRect(sx(p[1]) - hs, sy(p[2]) - hs, size, size);
    });
  });
  // the current sweep: rays + bright points
  fresh.forEach((list, s) => {
    if (!list.length) return;
    const [r, g, b] = SENS_RGB[s];
    ctx.strokeStyle = `rgba(${r},${g},${b},0.25)`; ctx.lineWidth = 1;
    ctx.beginPath();
    for (const p of list) { ctx.moveTo(sx(p[3]), sy(p[4])); ctx.lineTo(sx(p[1]), sy(p[2])); }
    ctx.stroke();
    ctx.fillStyle = SENS_NEW[s];
    const fs = size + 1.5;
    for (const p of list) ctx.fillRect(sx(p[1]) - fs / 2, sy(p[2]) - fs / 2, fs, fs);
  });
}

function drawWaypoints() {
  const sent = state.waypoints || [], nav = state.nav;
  const navIdx = nav && nav.waypoint_count === sent.length ? nav.nav_index : -1;
  const r = 11;
  ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  const path = (list, color, dash) => {
    if (list.length < 1) return;
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.setLineDash(dash);
    ctx.beginPath();
    list.forEach((p, i) => (i ? ctx.lineTo(sx(p[0]), sy(p[1])) : ctx.moveTo(sx(p[0]), sy(p[1]))));
    ctx.stroke(); ctx.setLineDash([]);
  };
  // dashed line from the robot to the current target
  if (state.tlm && navIdx >= 0 && navIdx < sent.length && modeName(state.tlm.mode) === "GOTO") {
    path([[state.tlm.x_mm, state.tlm.y_mm], sent[navIdx]], C.target, [6, 6]);
  }
  if (planActive()) {   // planned route: the robot's waypoints are just the route's corners (drawPlan)
    ctx.textAlign = "start"; ctx.textBaseline = "alphabetic";
  } else {
  path(sent, C.wp, []);
  sent.forEach((p, i) => {
    const x = sx(p[0]), y = sy(p[1]), done = navIdx >= 0 && i < navIdx;
    ctx.fillStyle = done ? "#3a3a37" : C.wp;
    ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill();
    if (i === navIdx) { ctx.strokeStyle = C.target; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(x, y, r + 4, 0, 7); ctx.stroke(); }
    ctx.fillStyle = "#fff"; ctx.fillText(String(i + 1), x, y + 0.5);
  });
  }
  if (pending.length) {
    path(pending, C.wpPending, [5, 5]);
    pending.forEach((p, i) => {
      const x = sx(p[0]), y = sy(p[1]);
      ctx.fillStyle = "#151514"; ctx.strokeStyle = C.wpPending; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = C.wpPending; ctx.fillText(String(i + 1), x, y + 0.5);
    });
  }
  ctx.textAlign = "start"; ctx.textBaseline = "alphabetic";
}

function drawRobot() {
  const t = state.tlm;
  if (!t) return;
  const x = sx(t.x_mm), y = sy(t.y_mm), h = -t.heading_deg * Math.PI / 180;
  // real footprint (CAD): 90 mm long x 152 mm wide, wheels 66 mm on a 126.8 mm track; never under ~30 px wide
  const k = Math.max(view.s, 30 / ROBOT.w);
  const L = ROBOT.l * k, Wd = ROBOT.w * k, tr = ROBOT.track * k, wd = ROBOT.wheel * k, ww = Math.max(3, (ROBOT.w - ROBOT.track) * k);
  ctx.save();
  ctx.translate(x, y); ctx.rotate(h);
  ctx.fillStyle = "rgba(240,239,236,0.16)"; ctx.strokeStyle = C.robot; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.rect(-L / 2, -(tr - ww) / 2, L, tr - ww); ctx.fill(); ctx.stroke();
  ctx.fillStyle = C.robot;   // wheels
  ctx.fillRect(-wd / 2, -Wd / 2, wd, ww); ctx.fillRect(-wd / 2, Wd / 2 - ww, wd, ww);
  ctx.fillStyle = flag("obstacle") ? C.target : C.heading;   // heading arrow
  const a = Math.max(10, L * 0.55);
  ctx.beginPath(); ctx.moveTo(L / 2 + a, 0); ctx.lineTo(L / 2, -a * 0.6); ctx.lineTo(L / 2, a * 0.6); ctx.closePath(); ctx.fill();
  ctx.restore();
}

function draw() {
  requestAnimationFrame(draw);
  if (!dirty || canvas.offsetParent === null) return;
  dirty = false;
  if (view.follow && state.tlm) { view.cx = state.tlm.x_mm; view.cy = state.tlm.y_mm; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);
  if (layers.grid) drawGrid();
  drawOccMap(ctx, sx, sy, view.s);
  if (layers.track) drawTrack();
  drawPlan(ctx, sx, sy);
  if (layers.ranges) drawRanges();
  drawWaypoints();
  drawRobot();
}
requestAnimationFrame(draw);

// ---------- info box + waypoint bar ----------
const info = document.getElementById("map-info");
function updateInfo() {
  const t = state.tlm, n = state.nav;
  if (!t) { info.textContent = state.connected ? "Waiting for the robot…" : "Not connected to the bridge"; return; }
  let s = `x ${(t.x_mm / 1000).toFixed(2)}  y ${(t.y_mm / 1000).toFixed(2)} m  ·  ${Math.round(((t.heading_deg % 360) + 540) % 360 - 180)}°`;
  const mode = modeName(t.mode);
  if (n && (mode === "GOTO" || mode === "EXPLORE")) {
    const st = state.hello?.nav_states?.[n.state] ?? "state " + n.state;
    s += `\n${mode}: ${st}`;
    if (n.waypoint_count) s += ` · ${Math.min(n.nav_index + 1, n.waypoint_count)}/${n.waypoint_count} · ${(n.distance_mm / 1000).toFixed(2)} m`;
  }
  if (flag("obstacle")) s += "\n⚠ obstacle ahead";
  info.textContent = s;
  info.style.whiteSpace = "pre-line";
}

const wpHint = document.getElementById("wp-hint"), wpCount = document.getElementById("wp-count");
const bUndo = document.getElementById("wp-undo"), bClear = document.getElementById("wp-clear"), bGo = document.getElementById("wp-go");
const bExplore = document.getElementById("wp-explore");
function updateBar() {
  const sent = state.waypoints?.length || 0;
  const planning = plan.mode !== "idle", exploring = plan.mode === "explore";
  const hasAny = pending.length || sent || planning;
  wpHint.hidden = !!hasAny;
  wpCount.hidden = !hasAny;
  wpCount.textContent = pending.length ? `${pending.length} new point${pending.length > 1 ? "s" : ""}`
    : planning ? (plan.status || (exploring ? "Exploring" : "Route"))
    : `${sent} waypoint${sent > 1 ? "s" : ""}`;
  bUndo.hidden = !pending.length;
  bClear.hidden = !hasAny || exploring;
  const inGoto = state.tlm && modeName(state.tlm.mode) === "GOTO";
  bGo.hidden = exploring || !(pending.length || (sent && !inGoto && !planning));
  bExplore.hidden = !!pending.length || (planning && !exploring);
  bExplore.textContent = exploring ? "Stop exploring" : "Explore";
  bExplore.classList.toggle("on", exploring);
}

bUndo.onclick = () => { pending.pop(); updateBar(); dirty = true; };
bClear.onclick = () => {
  if (pending.length) pending = [];
  else if (plan.mode !== "idle") send({ t: "plan", action: "stop" });
  else send({ t: "event", event: state.hello.events.indexOf("CLEAR_WAYPOINTS") });
  updateBar(); dirty = true;
};
bGo.onclick = () => {
  if (!state.connected) return toast("Not connected");
  const t = state.tlm, disarmed = t && !flag("armed");
  if (disarmed && !t.radio_ok) {   // no remote: SET_MODE GOTO arms the motors
    if (!confirm("The robot is disarmed.\nArm it and drive to the waypoints?")) return;
  } else if (disarmed) toast("Waypoints sent - arm with the RC remote to go", 3500);
  if (pending.length) {
    const pts = pending.map((p) => [Math.round(p[0]), Math.round(p[1])]);
    // with a map: the bridge plans a route around obstacles through the tapped points (#12)
    if (routeOn.get() && hasMap()) send({ t: "plan", action: "goto", points: pts });
    else send({ t: "goto", points: pts, replace: true, go: true });
    pending = [];
  } else {
    send({ t: "set_mode", mode: state.hello.modes.indexOf("GOTO") });
  }
  updateBar(); dirty = true;
};

bExplore.onclick = () => {
  if (!state.connected) return toast("Not connected");
  if (plan.mode === "explore") { send({ t: "plan", action: "stop" }); send({ t: "set_mode", mode: state.hello.modes.indexOf("BALANCE") }); return; }
  const t = state.tlm, disarmed = t && !flag("armed");
  if (!confirm(disarmed && !t.radio_ok
    ? "Explore: the robot arms and drives by itself to unmapped places until the room is mapped.\nSTOP ends it. Start?"
    : "Explore: the robot drives by itself to unmapped places until the room is mapped.\nSTOP ends it. Start?")) return;
  if (disarmed && t.radio_ok) toast("Arm with the RC remote to go", 3500);
  send({ t: "plan", action: "explore" });
};
onChange(() => { dirty = true; updateBar(); });

// ---------- pan / zoom / tap ----------
const ptrs = new Map();
let gesture = null;   // {x, y, t, moved, multi, dist, mid}
function zoomAt(px, py, f) {
  const x = wx(px), y = wy(py);
  view.s = Math.max(0.01, Math.min(3, view.s * f));
  view.cx = x - (px - W / 2) / view.s; view.cy = y + (py - H / 2) / view.s;
  dirty = true;
}
function setFollow(v) { view.follow = v; document.getElementById("map-follow").classList.toggle("on", v); dirty = true; }

canvas.addEventListener("pointerdown", (e) => {
  canvas.setPointerCapture(e.pointerId);
  ptrs.set(e.pointerId, { x: e.offsetX, y: e.offsetY });
  if (ptrs.size === 1) gesture = { x: e.offsetX, y: e.offsetY, t: performance.now(), moved: 0, multi: false };
  else if (gesture) gesture.multi = true;
});
canvas.addEventListener("pointermove", (e) => {
  const p = ptrs.get(e.pointerId);
  if (!p) return;
  if (ptrs.size === 1) {
    const dx = e.offsetX - p.x, dy = e.offsetY - p.y;
    if (gesture) gesture.moved += Math.abs(dx) + Math.abs(dy);
    if (gesture && gesture.moved > 8) {
      if (view.follow) setFollow(false);
      view.cx -= dx / view.s; view.cy += dy / view.s; dirty = true;
    }
  } else if (ptrs.size === 2) {
    const [a, b] = [...ptrs.values()];
    const d0 = Math.hypot(a.x - b.x, a.y - b.y);
    p.x = e.offsetX; p.y = e.offsetY;
    const [a2, b2] = [...ptrs.values()];
    const d1 = Math.hypot(a2.x - b2.x, a2.y - b2.y);
    if (d0 > 0) zoomAt((a2.x + b2.x) / 2, (a2.y + b2.y) / 2, d1 / d0);
    return;
  }
  p.x = e.offsetX; p.y = e.offsetY;
});
function endPtr(e) {
  if (!ptrs.has(e.pointerId)) return;
  ptrs.delete(e.pointerId);
  if (ptrs.size === 0 && gesture) {
    if (!gesture.multi && gesture.moved <= 8 && performance.now() - gesture.t < 600 && e.type === "pointerup") {
      pending.push([wx(e.offsetX), wy(e.offsetY)]);
      updateBar(); dirty = true;
    }
    gesture = null;
  }
}
canvas.addEventListener("pointerup", endPtr);
canvas.addEventListener("pointercancel", endPtr);
canvas.addEventListener("wheel", (e) => { e.preventDefault(); zoomAt(e.offsetX, e.offsetY, Math.pow(1.0015, -e.deltaY)); }, { passive: false });

document.getElementById("map-zin").onclick = () => zoomAt(W / 2, H / 2, 1.4);
document.getElementById("map-zout").onclick = () => zoomAt(W / 2, H / 2, 1 / 1.4);
document.getElementById("map-follow").onclick = () => setFollow(!view.follow);
const menu = document.getElementById("map-menu");
document.getElementById("map-more").onclick = (e) => { e.stopPropagation(); menu.hidden = !menu.hidden; };
document.addEventListener("pointerdown", (e) => { if (!menu.hidden && !menu.contains(e.target) && e.target.id !== "map-more") menu.hidden = true; });
for (const k of ["ranges", "track", "grid"]) {
  document.getElementById("lay-" + k).onchange = (e) => { layers[k] = e.target.checked; dirty = true; };
}
document.getElementById("map-clear-track").onclick = () => { state.track = []; state.ranges = []; menu.hidden = true; dirty = true; };
document.getElementById("map-reset-pose").onclick = () => {
  if (!confirm("Reset the robot's pose to 0,0 heading 0? Waypoints are cleared.")) return;
  send({ t: "event", event: state.hello.events.indexOf("RESET_POSE") });
  state.track = []; menu.hidden = true; dirty = true;
};

on("tlm", (m) => { piNowBase = m.pt; piNowAt = performance.now(); dirty = true; updateInfo(); updateBar(); });
for (const t of ["ranges", "waypoints", "nav", "reset", "hello", "track", "plan"]) on(t, () => { dirty = true; updateBar(); updateInfo(); });
on("conn", updateInfo);
setInterval(() => { dirty = true; }, 500);   // keep fading even when the robot is silent
updateBar(); updateInfo();

export function mapDebug() { return { view, pending }; }
