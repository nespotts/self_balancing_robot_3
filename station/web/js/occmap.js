// Occupancy grid layer + planned route (issue #12). The bridge sends the map as 1 m PNG tiles
// ("map" messages: full on connect / reset / load, changed tiles ~2 Hz; the map grows as the robot explores) and the planner's route ("plan").
import { state, on, send } from "./conn.js";
import { toast } from "./ui.js";

// tiles: key "tx,ty" (absolute: world mm / tile size, floor, so they stay valid while the map grows) -> Image
const occ = { info: null, v: -1, tiles: new Map(), bbox: null, corr: [0, 0, 0], settings: {}, stats: {}, name: null };
export const plan = { mode: "idle", status: "", path: [], goals: [], settings: {}, retrace: null };
let redraw = () => {};
export function onChange(fn) { redraw = fn; }

const store = {
  get(k, d) { try { const v = localStorage.getItem("sbr3." + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("sbr3." + k, JSON.stringify(v)); } catch {} },
};
export const occLayer = { on: store.get("occ.on", true), opacity: store.get("occ.opacity", 0.8) };

function growBbox(tx, ty) {
  const b = occ.bbox;
  occ.bbox = b ? [Math.min(b[0], tx), Math.min(b[1], ty), Math.max(b[2], tx), Math.max(b[3], ty)] : [tx, ty, tx, ty];
}

on("map", (m) => {
  occ.corr = m.corr || [0, 0, 0]; occ.settings = m.settings || {}; occ.stats = m.stats || {}; occ.name = m.name;
  if (m.full || m.v !== occ.v || !occ.info) {
    if (!m.tile) return;   // a delta for a map we don't have: the full one follows
    occ.info = { res: m.res_mm, tile: m.tile };
    occ.v = m.v;
    occ.tiles.clear(); occ.bbox = null;
  }
  const v = occ.v;
  for (const [tx, ty, b64] of m.tiles || []) {
    growBbox(tx, ty);
    const key = tx + "," + ty, img = new Image();
    img.onload = () => { if (occ.v === v) { occ.tiles.set(key, img); redraw(); } };
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

export function planActive() { return plan.mode !== "idle" && (plan.path.length > 1 || !!plan.retrace); }
export function hasMap() { return occ.tiles.size > 0; }
// map frame (places, grid) -> odometry frame (what the canvas draws): the inverse of the map's pose correction
// (the ROS view, #106, draws in the ROS map frame instead: setStationView gives places / zones that transform)
let stationView = null;
export function setStationView(fn) { stationView = fn; }
export function mapToOdom(x, y) { return stationView ? stationView(x, y) : rawMapToOdom(x, y); }
export function rawMapToOdom(x, y) {
  const [tx, ty, th] = occ.corr, a = -th * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [c * (x - tx) - s * (y - ty), s * (x - tx) + c * (y - ty)];
}
// odometry (the robot's x_mm, y_mm, the lidar points) -> the station map frame: map pose = corr o odometry pose
export function odomToMap(x, y) {
  const [tx, ty, th] = occ.corr, a = th * Math.PI / 180, c = Math.cos(a), s = Math.sin(a);
  return [tx + c * x - s * y, ty + s * x + c * y];
}

// ---------- drawing (called from map.js with its view: odometry mm -> screen px) ----------
export function drawOccMap(ctx, sx, sy, s) {
  if (!occLayer.on || !occ.tiles.size) return;
  const i = occ.info, [tx, ty, th] = occ.corr;
  ctx.save();
  ctx.globalAlpha = occLayer.opacity;
  ctx.imageSmoothingEnabled = s * i.res < 2;          // crisp cells when zoomed in
  ctx.translate(sx(0), sy(0));
  ctx.scale(s, -s);                                   // odometry mm, y up
  if (th || tx || ty) { ctx.rotate(-th * Math.PI / 180); ctx.translate(-tx, -ty); }   // map frame -> odometry
  ctx.scale(i.res, -i.res);                           // cell units; tile images are rows top-down
  const T = i.tile, b = occ.bbox;
  ctx.fillStyle = "#808080";                          // unknown grey under the whole explored rectangle
  ctx.fillRect(b[0] * T, -(b[3] + 1) * T, (b[2] - b[0] + 1) * T, (b[3] - b[1] + 1) * T);
  for (const [key, img] of occ.tiles) {
    const k = key.indexOf(",");
    ctx.drawImage(img, +key.slice(0, k) * T, -(+key.slice(k + 1) + 1) * T);
  }
  ctx.restore();
}

// pose lost, retracing steps (retrace.py): the breadcrumbs back (odometry frame, next one first), rings at the
// well-mapped spots where it stops to find itself
function drawRetrace(ctx, sx, sy) {
  const R = plan.retrace, p = R && R.trail;
  if (!p || !p.length) return;
  ctx.save();
  const pts = state.tlm ? [[state.tlm.x_mm, state.tlm.y_mm], ...p] : p;
  ctx.strokeStyle = "#f0a040"; ctx.lineWidth = 3; ctx.lineJoin = "round"; ctx.setLineDash([4, 6]);
  ctx.beginPath();
  pts.forEach((q, k) => (k ? ctx.lineTo(sx(q[0]), sy(q[1])) : ctx.moveTo(sx(q[0]), sy(q[1]))));
  ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = "#f0a040";
  for (const q of p) { ctx.beginPath(); ctx.arc(sx(q[0]), sy(q[1]), 2.5, 0, 7); ctx.fill(); }
  ctx.strokeStyle = "#f0a040"; ctx.lineWidth = 2.5;
  for (const k of R.stops || []) {
    const q = p[k];
    if (q) { ctx.beginPath(); ctx.arc(sx(q[0]), sy(q[1]), 9, 0, 7); ctx.stroke(); }
  }
  ctx.restore();
}

export function drawPlan(ctx, sx, sy) {
  if (!planActive()) return;
  if (plan.retrace) return drawRetrace(ctx, sx, sy);
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
  if (!confirm("Clear the map, its places and its walls / zones? (Saved maps are kept.)")) return;
  cmd({ t: "map", action: "reset" });
  $("map-menu").hidden = true;
};

// settings (map: mapper, route: planner)
const NUM = [["set-margin", "plan", "margin_mm"], ["set-soft", "plan", "soft_mm"],
             ["set-squeeze", "plan", "squeeze_mm"], ["set-radius", "plan", "robot_radius_mm"],
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
$("set-stopscan").onchange = (e) => cmd({ t: "map", action: "set", settings: { stop_scan: e.target.checked } });
$("set-lmatch").onchange = (e) => cmd({ t: "map", action: "set", settings: { lidar_match: e.target.checked } });
$("set-lslip").onchange = (e) => cmd({ t: "map", action: "set", settings: { match_slip: e.target.checked } });
$("map-trust").onclick = () => { cmd({ t: "map", action: "trust_pose" }); toast("Pose trusted: mapping from here", 2500); $("map-menu").hidden = true; };
$("map-find").onclick = () => { cmd({ t: "map", action: "relocalize" }); toast("Searching the whole map…", 2500); $("map-menu").hidden = true; };
$("set-scanpause").onchange = (e) => cmd({ t: "plan", action: "set", settings: { scan_pause: e.target.checked } });
$("set-retrace").onchange = (e) => cmd({ t: "plan", action: "set", settings: { retrace_lost: e.target.checked } });

// LD19 scan matching (#78): one line for the map info box ("" when there is no lidar / it is off)
export function matchLine() {
  const L = occ.stats && occ.stats.lmatch;
  if (!L || !L.lidar || !L.on || !L.state || L.state === "off") return "";
  // wheel slip / lost (#79): the pose is held / searched for, nothing goes into the map meanwhile
  if (L.mode === "slip") return `⚠ Wheel slip · holding position · grip ${Math.round(100 * (L.grip || 0))} %`;
  // relocalisation (#105): the whole-map search, Set position
  const R = L.reloc, pct = (v) => Math.round(100 * (v || 0)) + " %";
  if (L.mode === "lost" && plan.retrace && !(R && (R.running || ["pending", "searching", "confirming", "set"].includes(R.phase))))
    return "⚠ Pose lost · retracing steps to a well-mapped spot · not mapping";
  if (R && L.mode === "lost") {
    const set = R.by === "set";
    if (R.running || R.phase === "pending" || R.phase === "searching")
      return set ? "Set position: matching the scan…"
        : R.by === "retrace" ? "⚠ Pose lost · retracing steps · matching the scan here…" : "⚠ Pose lost · searching the whole map…";
    if (R.phase === "set") return "Set position: matching the scan…";
    if (R.phase === "confirming")
      return `${set ? "Set position: fits" : "Found on the map"} · fit ${pct(R.score)} · margin ${pct(R.margin)} · confirming…`;
    if (R.phase === "ambiguous") return `⚠ Ambiguous: ${R.cands > 1 ? R.cands + " places fit" : "the room looks alike"} · use Set position (⋯)`;
    if (R.phase === "none") return "⚠ Pose lost · not found on the map · use Set position (⋯)";
    if (R.phase === "set_poor") return `⚠ Set position: poor fit (${pct(R.score)}) · not mapping · drive a little or set it again`;
    if (R.phase === "unconfirmed") return "⚠ Found pose not confirmed · not mapping · searching again…";
  }
  if (R && R.phase === "found" && L.mode === "track")
    return R.by === "set" ? "Scan match ✓ position confirmed · mapping again"
      : `Scan match ✓ found on the map · fit ${pct(R.score)} · margin ${pct(R.margin)}`;
  if (L.mode === "lost" && L.checking) return "Scan match: checking the pose after a lidar gap…";
  if (L.mode === "lost") {
    const w = L.window ? ` ±${Math.round(L.window[0] / 10)} cm` : "";
    return `⚠ Pose lost ${Math.round(L.lost_s || 0)} s · searching${w} · not mapping` + (L.lost_s > 20 ? " (⋯ Set position)" : "");
  }
  if (L.state.startsWith("matched")) {
    const a = L.applied, fix = a && (a[0] || a[1] || a[2])
      ? `fix ${Math.round(Math.hypot(a[0], a[1]))} mm ${Math.abs(a[2]).toFixed(1)}°` : "pose fine";
    return `Scan match ✓ ${Math.round(100 * (L.score || 0))} % · ${fix} · ${L.ms} ms`;
  }
  const why = L.state.replace(/^kept: /, "");
  return `Scan match: ${why.length > 38 ? why.slice(0, 37) + "…" : why}`;
}

// stop-and-scan pose corrections (#71): a toast when one is applied, the history behind the menu
const fmtCorr = (e) => `${(e.mm / 10).toFixed(e.mm < 100 ? 1 : 0)} cm / ${Math.abs(e.dth).toFixed(1)}°`;
let scanSeen = null;
on("scan", (m) => {
  const h = m.history || [], top = h[0];
  if (scanSeen !== null && top && top.time + "/" + top.who !== scanSeen && top.ok) toast("Pose corrected " + fmtCorr(top), 3000);
  scanSeen = top ? top.time + "/" + top.who : "";
  const list = $("scan-list");
  list.replaceChildren();
  for (const e of h) {
    const row = document.createElement("small"), t = new Date(e.time * 1000);
    row.textContent = `${t.toLocaleTimeString(undefined, { hour: "2-digit", minute: "2-digit", second: "2-digit" })}  ` +
      (e.ok ? "corrected " + fmtCorr(e) : "kept: " + e.why) + (e.who !== "auto" ? " (asked)" : "");
    list.append(row);
  }
  $("scan-hist").hidden = !h.length;
});
$("map-scan").onclick = () => { cmd({ t: "map", action: "scan" }); toast("Hold still: scanning…", 2500); $("map-menu").hidden = true; };
$("set-zblock").onchange = (e) => cmd({ t: "plan", action: "set", settings: { zones_block_manual: e.target.checked } });

function syncSettings() {
  for (const [id, t, key] of NUM) {
    const src = t === "plan" ? plan.settings : occ.settings, el = $(id);
    if (src && key in src && document.activeElement !== el) el.value = src[key];
  }
  if (occ.settings && "correct" in occ.settings) $("set-correct").checked = occ.settings.correct;
  if (occ.settings && "stop_scan" in occ.settings) $("set-stopscan").checked = occ.settings.stop_scan;
  if (occ.settings && "lidar_match" in occ.settings) $("set-lmatch").checked = occ.settings.lidar_match;
  $("set-lmatch-row").hidden = !(occ.stats && occ.stats.lmatch && occ.stats.lmatch.lidar);
  if (occ.settings && "match_slip" in occ.settings) $("set-lslip").checked = occ.settings.match_slip;
  $("set-lslip-row").hidden = $("set-lmatch-row").hidden || !(occ.settings && occ.settings.lidar_match);
  const lmt = occ.stats && occ.stats.lmatch;
  $("map-trust").hidden = !(lmt && lmt.mode === "lost");
  $("map-find").hidden = !(lmt && lmt.lidar && lmt.on && occ.tiles.size);
  if (plan.settings && "scan_pause" in plan.settings) $("set-scanpause").checked = plan.settings.scan_pause;
  if (plan.settings && "retrace_lost" in plan.settings) $("set-retrace").checked = plan.settings.retrace_lost;
  $("set-retrace-row").hidden = $("set-lslip-row").hidden;
  if (plan.settings && "zones_block_manual" in plan.settings) $("set-zblock").checked = plan.settings.zones_block_manual;
  const st = occ.stats || {};
  $("map-stats").textContent = st.readings != null
    ? `${occ.name ? occ.name + " · " : ""}${st.scans ? st.scans + " lidar scans · " : ""}${st.used - (st.scans || 0)} of ${st.readings - (st.sweep_skipped || 0)} readings used · ${st.tilt} tilted · bridge CPU ${st.cpu_pct}%`
    : "";
}
