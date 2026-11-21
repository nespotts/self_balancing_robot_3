// Occupancy grid layer + planned route (issue #12). The bridge sends the map as 1 m PNG tiles
// ("map" messages: full on connect / reset / load, changed tiles ~2 Hz) and the planner's route ("plan").
import { state, on, send } from "./conn.js";
import { toast } from "./ui.js";

const occ = { info: null, v: -1, canvas: null, cx: null, drawn: new Set(), bbox: null, corr: [0, 0, 0], settings: {}, stats: {}, name: null };
export const plan = { mode: "idle", status: "", path: [], goals: [], settings: {} };
let redraw = () => {};
export function onChange(fn) { redraw = fn; }

const store = {
  get(k, d) { try { const v = localStorage.getItem("sbr3." + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("sbr3." + k, JSON.stringify(v)); } catch {} },
};
export const occLayer = { on: store.get("occ.on", true), opacity: store.get("occ.opacity", 0.8) };

function fillUnknown(tx, ty) {
  const T = occ.info.tile;
  occ.cx.fillStyle = "#808080";
  occ.cx.fillRect(tx * T, (occ.info.nty - 1 - ty) * T, T, T);
}

function growBbox(tx, ty) {
  const b = occ.bbox;
  if (!b) { occ.bbox = [tx, ty, tx, ty]; fillUnknown(tx, ty); return; }
  const nb = [Math.min(b[0], tx), Math.min(b[1], ty), Math.max(b[2], tx), Math.max(b[3], ty)];
  if (nb.every((v, i) => v === b[i])) return;
  // unknown grey under the whole explored rectangle (tiles never touched stay grey)
  for (let y = nb[1]; y <= nb[3]; y++) for (let x = nb[0]; x <= nb[2]; x++) {
    if ((x < b[0] || x > b[2] || y < b[1] || y > b[3]) && !occ.drawn.has(x + "," + y)) fillUnknown(x, y);
  }
  occ.bbox = nb;
}

on("map", (m) => {
  occ.corr = m.corr || [0, 0, 0]; occ.settings = m.settings || {}; occ.stats = m.stats || {}; occ.name = m.name;
  if (m.full || m.v !== occ.v || !occ.canvas) {
    if (!m.w) return;   // a delta for a map we don't have: the full one follows
    occ.info = { res: m.res_mm, x0: m.x0_mm, y0: m.y0_mm, w: m.w, h: m.h, tile: m.tile, ntx: m.ntx, nty: m.nty };
    occ.v = m.v;
    occ.canvas = occ.canvas || document.createElement("canvas");
    occ.canvas.width = m.w; occ.canvas.height = m.h;
    occ.cx = occ.canvas.getContext("2d");
    occ.cx.clearRect(0, 0, m.w, m.h);
    occ.drawn.clear(); occ.bbox = null;
  }
  const T = occ.info.tile, v = occ.v;
  for (const [tx, ty, b64] of m.tiles || []) {
    growBbox(tx, ty);
    occ.drawn.add(tx + "," + ty);
    const img = new Image();
    img.onload = () => { if (occ.v === v) { occ.cx.drawImage(img, tx * T, (occ.info.nty - 1 - ty) * T); redraw(); } };
    img.src = "data:image/png;base64," + b64;
  }
  syncSettings();
  redraw();
});

on("plan", (m) => {
  const was = plan.mode;
  Object.assign(plan, m);
  if (was !== "idle" && m.mode === "idle" && m.status) toast(m.status, 3000);
  syncSettings();
  redraw();
});

export function planActive() { return plan.mode !== "idle" && plan.path.length > 1; }
export function hasMap() { return !!(occ.canvas && occ.drawn.size); }
// map frame (places, grid) -> odometry frame (what the canvas draws): the inverse of the map's pose correction
export function mapToOdom(x, y) {
  const [tx, ty, th] = occ.corr, a = -th * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [c * (x - tx) - s * (y - ty), s * (x - tx) + c * (y - ty)];
}

// ---------- drawing (called from map.js with its view: odometry mm -> screen px) ----------
export function drawOccMap(ctx, sx, sy, s) {
  if (!occLayer.on || !occ.canvas || !occ.drawn.size) return;
  const i = occ.info, [tx, ty, th] = occ.corr;
  ctx.save();
  ctx.globalAlpha = occLayer.opacity;
  ctx.imageSmoothingEnabled = s * i.res < 2;          // crisp cells when zoomed in
  ctx.translate(sx(0), sy(0));
  ctx.scale(s, -s);                                   // odometry mm, y up
  if (th || tx || ty) { ctx.rotate(-th * Math.PI / 180); ctx.translate(-tx, -ty); }   // map frame -> odometry
  ctx.translate(i.x0, i.y0 + i.h * i.res);
  ctx.scale(i.res, -i.res);                           // canvas pixels (rows top-down)
  ctx.drawImage(occ.canvas, 0, 0);
  ctx.restore();
}

export function drawPlan(ctx, sx, sy) {
  if (!planActive()) return;
  const p = plan.path;
  ctx.save();
  ctx.strokeStyle = "#b4a9ff"; ctx.lineWidth = 3; ctx.lineJoin = "round"; ctx.setLineDash([10, 6]);
  ctx.beginPath();
  p.forEach((q, k) => (k ? ctx.lineTo(sx(q[0]), sy(q[1])) : ctx.moveTo(sx(q[0]), sy(q[1]))));
  ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = "#b4a9ff";
  for (const q of p.slice(1, -1)) { ctx.beginPath(); ctx.arc(sx(q[0]), sy(q[1]), 4, 0, 7); ctx.fill(); }
  // goals: flag-ish target rings
  const goals = plan.mode === "explore" ? p.slice(-1) : plan.goals;
  ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  goals.forEach((g, k) => {
    const x = sx(g[0]), y = sy(g[1]);
    ctx.fillStyle = "#9085e9"; ctx.beginPath(); ctx.arc(x, y, 11, 0, 7); ctx.fill();
    ctx.strokeStyle = "#f0efec"; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, 11, 0, 7); ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.fillText(plan.mode === "explore" ? "?" : String(k + 1), x, y + 0.5);
  });
  ctx.restore();
}

// ---------- menu: layer, save / load / clear, settings ----------
const $ = (id) => document.getElementById(id);
let lastCmd = 0;
export function cmd(m) { lastCmd = performance.now(); if (!send(m)) toast("Not connected"); }
on("error", (m) => { if (performance.now() - lastCmd < 3000 && !m.code) toast(m.text, 3500); });   // (code: main.js says it)

$("lay-occ").checked = occLayer.on;
$("occ-opacity").value = occLayer.opacity;
$("lay-occ").onchange = (e) => { occLayer.on = e.target.checked; store.set("occ.on", occLayer.on); redraw(); };
$("occ-opacity").oninput = (e) => { occLayer.opacity = +e.target.value; store.set("occ.opacity", occLayer.opacity); if (!occLayer.on) { occLayer.on = true; $("lay-occ").checked = true; } redraw(); };

$("map-save").onclick = () => {
  const d = new Date(), z = (n) => String(n).padStart(2, "0");
  const def = occ.name && occ.name !== "autosave" ? occ.name : `map-${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}-${z(d.getHours())}${z(d.getMinutes())}`;
  const name = prompt("Save the map as", def);
  if (name) cmd({ t: "map", action: "save", name: name.trim().replace(/[^A-Za-z0-9_-]+/g, "-").slice(0, 40) });
};
on("map_saved", (m) => toast("Map saved: " + m.name));

const list = $("map-list");
$("map-load").onclick = () => {
  if (!list.hidden) { list.hidden = true; return; }
  cmd({ t: "map", action: "list" });
};
on("map_list", (m) => {
  list.replaceChildren();
  if (!m.maps.length) { const p = document.createElement("small"); p.textContent = "No saved maps"; list.append(p); }
  for (const it of m.maps.slice(0, 12)) {
    const b = document.createElement("button"), when = new Date(it.mtime * 1000);
    b.innerHTML = `<span></span><small>${when.toLocaleDateString(undefined, { month: "short", day: "numeric" })} ${when.toLocaleTimeString(undefined, { hour: "2-digit", minute: "2-digit" })}</small>`;
    b.firstChild.textContent = it.name;
    b.onclick = () => {
      if (hasMap() && !confirm(`Load "${it.name}"? The current map is replaced (save it first if you want it).`)) return;
      cmd({ t: "map", action: "load", name: it.name });
      list.hidden = true; $("map-menu").hidden = true;
    };
    list.append(b);
  }
  list.hidden = false;
});

$("map-reset").onclick = () => {
  if (!confirm("Clear the map and its places? (Saved maps are kept.)")) return;
  cmd({ t: "map", action: "reset" });
  $("map-menu").hidden = true;
};

// settings (map: mapper, route: planner)
const NUM = [["set-margin", "plan", "margin_mm"], ["set-radius", "plan", "robot_radius_mm"],
             ["set-pitch", "map", "pitch_limit_deg"], ["set-notarget", "map", "no_target_mm"]];
export const routeOn = { get: () => $("set-route").checked };
$("set-route").checked = store.get("route.on", true);
$("set-route").onchange = (e) => store.set("route.on", e.target.checked);
for (const [id, t, key] of NUM) {
  $(id).onchange = (e) => {
    const v = parseFloat(e.target.value);
    if (isFinite(v)) cmd({ t, action: "set", settings: { [key]: v } });
  };
}
$("set-correct").onchange = (e) => cmd({ t: "map", action: "set", settings: { correct: e.target.checked } });

function syncSettings() {
  for (const [id, t, key] of NUM) {
    const src = t === "plan" ? plan.settings : occ.settings, el = $(id);
    if (src && key in src && document.activeElement !== el) el.value = src[key];
  }
  if (occ.settings && "correct" in occ.settings) $("set-correct").checked = occ.settings.correct;
  const st = occ.stats || {};
  $("map-stats").textContent = st.readings != null
    ? `${occ.name ? occ.name + " · " : ""}${st.used} of ${st.readings} readings used · ${st.tilt} tilted · bridge CPU ${st.cpu_pct}%`
    : "";
}
