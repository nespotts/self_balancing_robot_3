// ROS inside the station (issue #106): slam_toolbox / Nav2 on the PC, watched and steered from here (no RViz).
// Everything here is hidden until a ROS adapter talks to the bridge (status.ros.connected).
// The ROS button on the Map page switches the view to the ROS map frame: the ROS map, pose and Nav2 plan, the station's
// places and zones put on it. Tap: a Nav2 goal (drag its arrow for the heading), Go ▶ sends it. Hold: places, or
// "Set ROS position here". Place sheet: Go here (ROS), Patrol…. ROS never arms: the robot must be armed in Manual.
import { state, on, send, modeName, flag } from "./conn.js";
import { toast, el } from "./ui.js";
import { setStationView, rawMapToOdom, odomToMap, occLayer } from "./occmap.js";
import { places, setSheetHook, closeSheet } from "./places.js";

export const ros = { map: null, pose: null, plan: [], nav: null, navAt: 0, status: null, explore: null, exploreAt: 0 };
const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { const v = localStorage.getItem("sbr3." + k); return v == null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("sbr3." + k, JSON.stringify(v)); } catch {} },
};
let viewOn = store.get("ros.view", false);
let pending = null;          // {x, y (ROS map mm), yaw (rad), kind: "goal" | "pose"}
let redraw = () => {};
export function onRosChange(fn) { redraw = fn; }
const changed = () => { syncView(); redraw(); };

export const rosConnected = () => !!(state.connected && state.status?.ros?.connected);
export const rosView = () => viewOn && rosConnected();
export const rosPending = () => pending;
const MANUAL = "Switch to Manual to let ROS drive";

// ---------- frames: ROS map (m) / station map (mm) / odometry (mm). T = map <- station_map [x m, y m, yaw rad] ----------
function T() { return ros.pose?.T || null; }
export function stationToRos(x, y) {
  const t = T();
  if (!t) return [NaN, NaN];
  const c = Math.cos(t[2]), s = Math.sin(t[2]);
  return [t[0] * 1000 + c * x - s * y, t[1] * 1000 + s * x + c * y];
}
export function rosToStation(x, y) {
  const t = T();
  if (!t) return [NaN, NaN];
  const c = Math.cos(t[2]), s = Math.sin(t[2]), dx = x - t[0] * 1000, dy = y - t[1] * 1000;
  return [c * dx + s * dy, -s * dx + c * dy];
}
export function odomToRos(x, y) { const [a, b] = odomToMap(x, y); return stationToRos(a, b); }
export function rosToOdom(x, y) { const [a, b] = rosToStation(x, y); return rawMapToOdom(a, b); }
function syncView() { setStationView(rosView() ? stationToRos : null); }
export function rosRobot() { return ros.pose ? { x: ros.pose.p[0] * 1000, y: ros.pose.p[1] * 1000, h: ros.pose.p[2] * 180 / Math.PI } : null; }

// ---------- messages ----------
function decodeRle(b64, n) {
  const bin = atob(b64), out = new Uint8Array(n);
  let i = 0, k = 0;
  while (i < bin.length && k < n) {
    const v = bin.charCodeAt(i++);
    let c = 0, sh = 0, b;
    do { b = bin.charCodeAt(i++); c += (b & 127) * 2 ** sh; sh += 7; } while (b & 128 && i < bin.length);
    out.fill(v, k, Math.min(n, k + c)); k += c;
  }
  return out;
}
const GREY = [128, 255, 0, 190];             // unknown, free, occupied, maybe (as the station's own map tiles)
on("ros_map", (m) => {
  const n = m.w * m.h;
  if (!n || n > 16e6) return;
  const cls = decodeRle(m.rle, n), cv = document.createElement("canvas");
  cv.width = m.w; cv.height = m.h;
  const c2 = cv.getContext("2d"), img = c2.createImageData(m.w, m.h), d = img.data;
  for (let j = 0; j < m.h; j++) {             // grid row 0 = the map's bottom row: image rows go top-down
    const src = j * m.w, dst = (m.h - 1 - j) * m.w;
    for (let i = 0; i < m.w; i++) { const g = GREY[cls[src + i]] ?? 128, o = (dst + i) * 4; d[o] = d[o + 1] = d[o + 2] = g; d[o + 3] = 255; }
  }
  c2.putImageData(img, 0, 0);
  ros.map = { w: m.w, h: m.h, res: m.res, origin: m.origin, seq: m.seq, img: cv };
  redraw();
});
on("ros_pose", (m) => { ros.pose = m; syncView(); redraw(); });
on("ros_plan", (m) => { ros.plan = m.pts || []; redraw(); });
on("ros_status", (m) => { ros.status = m; changed(); });
on("ros_explore", (m) => { ros.explore = m; ros.exploreAt = performance.now(); changed(); });
on("ros_nav", (m) => {
  const old = ros.nav;
  ros.nav = m;
  if (!old || old.seq !== m.seq) {
    ros.navAt = performance.now();
    const what = m.place || (m.kind === "patrol" ? "patrol" : "the goal");
    if (old && m.state === "succeeded") toast(m.kind === "patrol" ? "Patrol done ✓" + (m.reason ? " (" + m.reason + ")" : "") : "Reached " + what + " ✓", 3000);
    else if (old && m.state === "failed") toast("Nav2 failed: " + (m.reason || what), 4500);
    else if (old && m.state === "cancelled" && m.reason && !m.reason.startsWith("cancelled from")) toast("ROS goal " + m.reason, 4500);
  }
  changed();
});
on("ros_reply", (m) => { if (!m.ok && m.text) toast("ROS: " + m.text, 4500); else if (m.ok && m.cmd === "initial_pose") toast("ROS position set", 2500); });
on("error", (m) => { if ((m.code === "not_manual" || m.code === "ros_offline") && m.text) toast(m.text, 4000); });
on("status", changed);
on("conn", changed);

// ---------- drawing (map.js passes its view: ROS map mm -> screen px) ----------
export function drawRosMap(ctx, sx, sy, s) {
  const M = ros.map;
  if (!M || !occLayer.on) return;
  const [ox, oy, oth] = M.origin, r = M.res * 1000;
  ctx.save();
  ctx.globalAlpha = occLayer.opacity;
  ctx.imageSmoothingEnabled = s * r < 2;
  ctx.translate(sx(ox * 1000), sy(oy * 1000));
  ctx.scale(s, -s);                                 // mm, y up
  if (oth) ctx.rotate(oth);
  ctx.scale(r, -r);                                 // cells, image rows top-down
  ctx.drawImage(M.img, 0, -M.h);
  ctx.restore();
}

function arrow(ctx, X, Y, yaw, len, color, dash) {
  const ex = X + len * Math.cos(yaw), ey = Y - len * Math.sin(yaw);
  ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.setLineDash(dash || []);
  ctx.beginPath(); ctx.moveTo(X, Y); ctx.lineTo(ex, ey); ctx.stroke(); ctx.setLineDash([]);
  const a = 9, h = Math.atan2(-(ey - Y), ex - X);
  ctx.fillStyle = color; ctx.beginPath();
  ctx.moveTo(ex + a * Math.cos(h), ey - a * Math.sin(h));
  ctx.lineTo(ex + a * Math.cos(h + 2.4), ey - a * Math.sin(h + 2.4));
  ctx.lineTo(ex + a * Math.cos(h - 2.4), ey - a * Math.sin(h - 2.4));
  ctx.closePath(); ctx.fill();
  return [ex, ey];
}
const HANDLE = 46;    // px from the goal to its heading handle

export function drawRosPlan(ctx, sx, sy) {
  const p = ros.plan, n = ros.nav;
  ctx.save();
  if (p && p.length > 1) {
    ctx.strokeStyle = "#7fd4ff"; ctx.lineWidth = 3; ctx.lineJoin = "round"; ctx.setLineDash([10, 6]);
    ctx.beginPath();
    p.forEach((q, k) => (k ? ctx.lineTo(sx(q[0] * 1000), sy(q[1] * 1000)) : ctx.moveTo(sx(q[0] * 1000), sy(q[1] * 1000))));
    ctx.stroke(); ctx.setLineDash([]);
  }
  const busy = n && (n.state === "navigating" || n.state === "dwell");
  const g = busy && n.goal ? n.goal : busy && p && p.length ? [...p[p.length - 1], null] : null;
  if (g) {
    const X = sx(g[0] * 1000), Y = sy(g[1] * 1000);
    if (g[2] != null) arrow(ctx, X, Y, g[2], 30, "#7fd4ff");
    ctx.fillStyle = "#2a7fb8"; ctx.strokeStyle = "#f0efec"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(X, Y, 10, 0, 7); ctx.fill(); ctx.stroke();
  }
  // the explorer's target (#107)
  const ex = exploreFresh() ? ros.explore : null;
  const tm = ex && (ex.target_map ? [ex.target_map[0] * 1000, ex.target_map[1] * 1000] : ex.target ? stationToRos(ex.target[0], ex.target[1]) : null);
  if (tm && isFinite(tm[0])) {
    const X = sx(tm[0]), Y = sy(tm[1]);
    ctx.fillStyle = "#9085e9"; ctx.beginPath(); ctx.arc(X, Y, 11, 0, 7); ctx.fill();
    ctx.strokeStyle = "#f0efec"; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText("?", X, Y + 0.5);
  }
  ctx.restore();
}

export function drawRosPending(ctx, sx, sy) {
  if (!pending) return;
  const X = sx(pending.x), Y = sy(pending.y), col = pending.kind === "pose" ? "#4fd1a1" : "#c3c2b7";
  ctx.save();
  const [hx, hy] = arrow(ctx, X, Y, pending.yaw, HANDLE - 8, col, [6, 4]);
  ctx.fillStyle = "#151514"; ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
  ctx.beginPath(); ctx.arc(X, Y, 11, 0, 7); ctx.fill(); ctx.stroke(); ctx.setLineDash([]);
  ctx.beginPath(); ctx.arc(X + HANDLE * Math.cos(pending.yaw), Y - HANDLE * Math.sin(pending.yaw), 9, 0, 7);
  ctx.fillStyle = "rgba(195,194,183,.25)"; ctx.fill(); ctx.stroke();
  ctx.fillStyle = col; ctx.font = "bold 11px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
  ctx.fillText(pending.kind === "pose" ? "P" : "G", X, Y + 0.5);
  void hx; void hy;
  ctx.restore();
}

// ---------- taps (map.js, ROS view) ----------
export function rosTap(x, y) {
  const r = rosRobot();
  const yaw = r && Math.hypot(x - r.x, y - r.y) > 50 ? Math.atan2(y - r.y, x - r.x) : (pending?.yaw ?? (r ? r.h * Math.PI / 180 : 0));
  pending = { x, y, yaw, kind: pending?.kind === "pose" ? "pose" : "goal" };
  changed();
}
// pointer down on the heading handle (or the goal itself): -> "turn" / "move" / null
export function rosGrab(px, py, sx, sy) {
  if (!pending) return null;
  const X = sx(pending.x), Y = sy(pending.y);
  if (Math.hypot(px - (X + HANDLE * Math.cos(pending.yaw)), py - (Y - HANDLE * Math.sin(pending.yaw))) < 22) return "turn";
  if (Math.hypot(px - X, py - Y) < 18) return "move";
  return null;
}
export function rosDrag(kind, x, y) {
  if (!pending) return;
  if (kind === "turn") { if (Math.hypot(x - pending.x, y - pending.y) > 1) pending.yaw = Math.atan2(y - pending.y, x - pending.x); }
  else { pending.x = x; pending.y = y; }
  redraw();
}
let pressAt = null;
export function rosPress(x, y) { pressAt = [x, y]; $("pm-rospose").hidden = !rosView(); }
$("pm-rospose").onclick = () => {
  $("place-menu").hidden = true;
  if (!pressAt) return;
  const r = rosRobot();
  pending = { x: pressAt[0], y: pressAt[1], yaw: r ? r.h * Math.PI / 180 : 0, kind: "pose" };
  toast("Drag the arrow to the way the robot faces, then Set ▶", 3500);
  changed();
};

// #105 hook: the station's "Set position" can call this to give ROS the same pose (station frame, mm / rad)
export function rosSetPose(x, y, yaw, frame = "station") {
  if (!rosConnected()) return false;
  return send({ t: "ros", action: "initial_pose", frame, x: frame === "map" ? x / 1000 : x, y: frame === "map" ? y / 1000 : y, yaw });
}

// ---------- sending ----------
function manualProblem() {
  const t = state.tlm;
  if (!t || !state.status?.robot_alive) return "The robot is not connected";
  if (state.status?.estop || state.status?.robot_estop) return "E-stop is on: release it first";
  if (!flag("armed")) return "Arm & Balance, then tap Manual to let ROS drive";
  if (modeName(t.mode) !== "STATION") return MANUAL;
  return "";
}
function rosCmd(m, drives) {
  if (!rosConnected()) { toast("ROS is not connected"); return false; }
  if (drives) { const why = manualProblem(); if (why) { toast(why, 4000); return false; } }
  if (!send(Object.assign({ t: "ros" }, m))) { toast("Not connected"); return false; }
  return true;
}
function resetBtn(b) { clearTimeout(b._tm); b.dataset.ask = ""; b.classList.remove("ask"); b.textContent = b.dataset.label; }
function twoTap(b, fire) {     // same two-tap Go as the station's own (places.js); ROS never arms
  if (b.dataset.ask !== "1") {
    b.dataset.ask = "1"; b.classList.add("ask"); b.textContent = "Tap again to go";
    clearTimeout(b._tm); b._tm = setTimeout(() => resetBtn(b), 4000);
    return;
  }
  resetBtn(b); fire();
}

// ---------- the bar at the bottom (ROS view) ----------
const bGo = $("ros-go"), bClear = $("ros-clear"), bCancel = $("ros-cancel"), bPatrol = $("ros-patrol"), bExplore = $("ros-explore");
const BAR = [bGo, bClear, bCancel, bPatrol, bExplore];
export function navBusy() { const n = ros.nav; return !!(n && (n.state === "navigating" || n.state === "dwell")); }
function exploreFresh() { return ros.explore && performance.now() - ros.exploreAt < 10000; }
export function exploreAvailable() { return !!(ros.status?.nodes?.explore || exploreFresh()); }
function exploring() { return exploreFresh() && ["exploring", "navigating", "dwell", "saving", "waiting"].includes(ros.explore.state); }
export function hideRosBar(hint) {
  for (const b of BAR) b.hidden = true;
  if (hint && hint.dataset.orig) { hint.textContent = hint.dataset.orig; delete hint.dataset.orig; }
}
export function updateRosBar(hint, count) {
  const busy = navBusy(), ex = exploring();
  hint.hidden = !!(pending || busy);
  count.hidden = !hint.hidden;
  if (!hint.dataset.orig) hint.dataset.orig = hint.textContent;
  hint.textContent = "ROS · tap: Nav2 goal · hold: place";
  if (pending) count.textContent = pending.kind === "pose" ? "Set ROS position" : "Nav2 goal";
  else if (busy) count.textContent = navShort();
  bClear.hidden = !pending;
  bGo.hidden = !pending;
  bGo.textContent = pending?.kind === "pose" ? "Set ▶" : "Go ▶";
  bCancel.hidden = pending || !busy;
  bPatrol.hidden = !!(pending || busy || ex) || places.items.length < 1;
  bExplore.hidden = !!(pending || busy) || !exploreAvailable();
  bExplore.textContent = ex ? "Stop exploring" : "Explore";
  bExplore.classList.toggle("on", ex);
}
bClear.onclick = () => { pending = null; changed(); };
bGo.onclick = () => {
  if (!pending) return;
  const p = pending;
  if (p.kind === "pose") {
    if (rosCmd({ action: "initial_pose", frame: "map", x: p.x / 1000, y: p.y / 1000, yaw: p.yaw }, false)) pending = null;
  } else if (rosCmd({ action: "goal", frame: "map", x: +(p.x / 1000).toFixed(3), y: +(p.y / 1000).toFixed(3), yaw: +p.yaw.toFixed(3) }, true)) pending = null;
  changed();
};
bCancel.onclick = () => { rosCmd({ action: "cancel" }, false); };
bPatrol.onclick = () => openPatrol();
bExplore.onclick = () => {
  if (exploring()) { rosCmd({ action: "explore", run: false }, false); return; }
  if (!confirm("Explore (ROS): Nav2 drives the robot to unmapped places until the room is mapped.\nSTOP ends it. Start?")) return;
  rosCmd({ action: "explore", run: true }, true);
};

// ---------- the ROS button, ⋯ menu ----------
const bView = $("map-ros"), bSave = $("ros-savemap");
bView.onclick = () => {
  viewOn = !viewOn; store.set("ros.view", viewOn);
  if (!viewOn) pending = null;
  toast(viewOn ? "ROS view: the ROS map, pose and plan" : "Station view", 1800);
  changed();
};
bSave.onclick = () => {
  $("map-menu").hidden = true;
  const d = new Date(), z = (n) => String(n).padStart(2, "0");
  const name = prompt("Save the ROS map as", `explore-${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}-${z(d.getHours())}${z(d.getMinutes())}`);
  if (name) rosCmd({ action: "save_map", name: name.trim().replace(/[^A-Za-z0-9_-]+/g, "-").slice(0, 40) }, false);
};
// drawing walls / zones works in the station frame: leave the ROS view first
$("map-menu").addEventListener("click", (e) => {
  if (e.target.closest("#map-draw") && viewOn) { viewOn = false; store.set("ros.view", false); pending = null; changed(); }
}, true);
function syncButtons() {
  const c = rosConnected();
  bView.hidden = !c;
  bView.classList.toggle("on", rosView());
  bSave.hidden = !(c && exploreAvailable());
}

// ---------- place sheet: Go here (ROS), Patrol…, Cancel ----------
const psRos = $("ps-ros"), psPatrol = $("ps-patrol"), psCancel = $("ps-cancel"), psGo = $("ps-go");
psRos.dataset.label = "Go here (ROS) ▶";
let sheetPlace = null;
setSheetHook((p) => {
  sheetPlace = p;
  const c = rosConnected(), rv = rosView();
  psRos.hidden = !c;
  psGo.hidden = rv;                                  // ROS view: ROS drives
  psRos.classList.toggle("primary", rv);
  psPatrol.hidden = !c;
  psCancel.hidden = !(c && navBusy());
  if (rv && p && ros.pose) {                         // distance on the ROS map
    const [x, y] = stationToRos(p.x, p.y), r = rosRobot();
    if (isFinite(x)) $("ps-dist").textContent = (Math.hypot(x - r.x, y - r.y) / 1000).toFixed(1) + " m away";
  }
});
psRos.onclick = () => {
  if (!sheetPlace) return;
  const why = manualProblem();
  if (why) { toast(why, 4000); return; }
  twoTap(psRos, () => { if (rosCmd({ action: "place", name: sheetPlace.name }, true)) closeSheet(); });
};
psPatrol.onclick = () => { const n = sheetPlace?.name; closeSheet(); openPatrol(n); };
psCancel.onclick = () => { rosCmd({ action: "cancel" }, false); closeSheet(); };

// ---------- patrol picker ----------
const pSheet = $("rp-sheet"), pList = $("rp-list");
let order = [];
function openPatrol(first) {
  const names = places.items.map((p) => p.name);
  order = first ? [first] : names.filter((n) => !places.items.find((p) => p.name === n)?.home);
  if (!order.length) order = names.slice();
  renderPatrol();
  pSheet.hidden = false;
}
function renderPatrol() {
  pList.replaceChildren(...places.items.map((p) => {
    const k = order.indexOf(p.name);
    return el("button", { class: "rp-chip" + (k >= 0 ? " on" : ""), onclick: () => {
      if (k >= 0) order.splice(k, 1); else order.push(p.name);
      renderPatrol();
    } }, k >= 0 ? el("i", {}, String(k + 1)) : null, p.name);
  }));
  $("rp-sum").textContent = order.length ? order.join(" → ") : "tap places in order";
  $("rp-go").disabled = !order.length;
}
$("rp-close").onclick = () => { pSheet.hidden = true; };
$("rp-go").onclick = () => {
  if (!order.length) return;
  const loops = +$("rp-loops").value, dwell = Math.max(0, Math.min(600, +$("rp-dwell").value || 0));
  if (rosCmd({ action: "patrol", places: order.slice(), loops, dwell }, true)) pSheet.hidden = true;
};
on("places", () => { if (!pSheet.hidden) { order = order.filter((n) => places.items.some((p) => p.name === n)); renderPatrol(); } });

// ---------- the info line ----------
const m1 = (v) => (v == null ? "?" : v.toFixed(1));
function navShort() {
  const n = ros.nav;
  if (!n) return "";
  const to = n.place || "goal";
  if (n.state === "dwell") return `At ${to} · ${Math.ceil(n.patrol?.dwell_left ?? 0)} s`;
  return `→ ${to}${n.dist_m != null ? " · " + m1(n.dist_m) + " m" : ""}`;
}
export function rosInfo() {
  if (!rosConnected()) return "";
  const L = [], n = ros.nav, st = ros.status, age = performance.now() - ros.navAt;
  if (n) {
    const to = n.place || (n.kind === "external" ? "a goal (RViz / script)" : "the goal");
    const pt = n.patrol, pat = pt ? ` · ${pt.i + 1}/${pt.places.length}, loop ${pt.loop + 1}/${pt.loops || "∞"}` : "";
    if (n.state === "navigating") {
      let s = `Nav2 → ${to}${pat}`;
      if (n.dist_m != null) s += ` · ${m1(n.dist_m)} m`;
      if (n.eta_s) s += ` · ~${n.eta_s} s`;
      if (n.recoveries) s += ` · ${n.recoveries} recover${n.recoveries > 1 ? "ies" : "y"}`;
      L.push(s);
    } else if (n.state === "dwell") L.push(`Patrol: at ${to}${pat} · next in ${Math.ceil(pt?.dwell_left ?? 0)} s`);
    else if (n.state === "succeeded" && age < 10000) L.push(n.kind === "patrol" ? `Patrol done ✓${n.reason ? " · " + n.reason : ""}` : `Nav2: reached ${to} ✓`);
    else if (n.state === "failed" && age < 20000) L.push(`⚠ Nav2 failed${n.reason ? ": " + n.reason : ""}`);
    else if (n.state === "cancelled" && age < 8000) L.push(`Nav2: ${n.reason && !n.reason.startsWith("cancelled from") ? n.reason : "cancelled"}`);
    if (navBusy() && st && st.drivable === false) L.push("⚠ " + (st.why || MANUAL));
  }
  const loc = st?.loc;
  if (loc && loc.state && loc.state !== "ok") {
    const fit = loc.quality != null ? ` · fit ${Math.round(100 * loc.quality)} %` : "";
    L.push(({ searching: "ROS pose: searching the map…", lost: "⚠ ROS pose lost: searching", weak: "ROS pose: weak",
      waiting: "ROS: waiting for scans (stand the robot up)", no_tf: "⚠ ROS: no pose (map -> laser transform missing)" }[loc.state]
      || "ROS pose: " + loc.state) + fit);
  } else if (rosView() && loc?.state === "ok" && loc.quality != null) L.push(`ROS pose ✓ fit ${Math.round(100 * loc.quality)} %`);
  else if (rosView() && st && !st.tf) L.push("ROS: no pose yet (map -> base_link)");
  if (exploreFresh()) {
    const e = ros.explore;
    let s = `Explore: ${e.state}`;
    if (e.frontiers != null) s += ` · ${e.frontiers} frontier${e.frontiers === 1 ? "" : "s"}`;
    if (e.area_m2 != null) s += ` · ${(+e.area_m2).toFixed(1)} m²`;
    if (e.reason) s += ` · ${e.reason}`;
    L.push(s);
  }
  return L.join("\n");
}
export function rosPoseLine() {
  const r = rosRobot();
  if (!r) return "ROS map · no pose yet";
  return `ROS map · x ${(r.x / 1000).toFixed(2)}  y ${(r.y / 1000).toFixed(2)} m  ·  ${Math.round(((r.h % 360) + 540) % 360 - 180)}°`;
}

on("status", syncButtons); on("conn", syncButtons); on("ros_status", syncButtons); on("ros_explore", syncButtons);
syncButtons();
