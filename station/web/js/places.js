// Saved places on the map (issue #53): pins, long-press / right-click -> "Save place here…",
// tap a pin -> "Go here" (the bridge plans a route around obstacles). Go needs two taps, like Go ▶.
import { state, on, flag } from "./conn.js";
import { toast } from "./ui.js";
import { mapToOdom, cmd } from "./occmap.js";

export const places = { items: [] };
const $ = (id) => document.getElementById(id);
let redraw = () => {};
export function onPlacesChange(fn) { redraw = fn; }

const HOME_COLOR = "#22a447", PIN_COLOR = "#d55181";
let sel = null;          // name of the place whose sheet is open

on("places", (m) => {
  places.items = m.items || [];
  if (sel && !byName(sel)) closeSheet(); else if (sel) renderSheet();
  redraw();
});

const byName = (n) => places.items.find((p) => p.name === n);
const odom = (p) => mapToOdom(p.x, p.y);

// ---------- drawing + hit test (map.js passes its mm -> px functions) ----------
export function drawPlaces(ctx, sx, sy) {
  ctx.save();
  ctx.font = "bold 12px system-ui, sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "middle";
  ctx.lineJoin = "round";
  for (const p of places.items) {
    const [ox, oy] = odom(p), X = sx(ox), Y = sy(oy);
    if (X < -80 || Y < -40 || X > 4000 || Y > 4000) continue;
    const col = p.home ? HOME_COLOR : PIN_COLOR;
    ctx.fillStyle = col; ctx.strokeStyle = "#f0efec"; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(X, Y); ctx.lineTo(X - 7.5, Y - 12); ctx.lineTo(X + 7.5, Y - 12); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.arc(X, Y - 18, 10, 0, 7); ctx.fill(); ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.beginPath(); ctx.arc(X, Y - 18, 3.2, 0, 7); ctx.fill();
    ctx.strokeStyle = "rgba(21,21,20,.9)"; ctx.lineWidth = 3; ctx.strokeText(p.name, X + 14, Y - 18);
    ctx.fillStyle = "#f0efec"; ctx.fillText(p.name, X + 14, Y - 18);
  }
  ctx.restore();
}

export function hitPlace(px, py, sx, sy) {
  let best = null, bd = 24;
  for (const p of places.items) {
    const [ox, oy] = odom(p), d = Math.hypot(px - sx(ox), py - (sy(oy) - 16));
    if (d < bd) { best = p; bd = d; }
  }
  return best;
}

// ---------- two-tap Go ----------
function resetBtn(b) { clearTimeout(b._tm); b.dataset.ask = ""; b.classList.remove("ask"); b.textContent = b.dataset.label; }
function goConfirm(btn, what, fire) {
  if (!state.connected) return toast("Not connected");
  const t = state.tlm, disarmed = t && !flag("armed");
  if (disarmed && !t.radio_ok) {            // no remote: the station arms, after this explicit confirm
    if (!confirm(`The robot is disarmed.\nArm it and drive ${what}?`)) return;
    resetBtn(btn); fire(true); return;
  }
  if (btn.dataset.ask !== "1") {            // first tap
    btn.dataset.ask = "1"; btn.classList.add("ask"); btn.textContent = "Tap again to go";
    clearTimeout(btn._tm); btn._tm = setTimeout(() => resetBtn(btn), 4000);
    return;
  }
  resetBtn(btn);                            // second tap
  if (disarmed) toast("Route sent - arm with the RC remote to go", 3500);
  fire(false);
}

// ---------- the sheet for a tapped pin ----------
const sheet = $("place-sheet"), goBtn = $("ps-go"), extra = $("ps-extra"), moreBtn = $("ps-more");
export function closeSheet() { sel = null; sheet.hidden = true; extra.hidden = true; resetBtn(goBtn); }
let sheetHook = () => {};
export function setSheetHook(fn) { sheetHook = fn; }     // ros.js (#106): Go here (ROS), Patrol…, Cancel
function renderSheet() {
  const p = byName(sel);
  if (!p) return closeSheet();
  $("ps-name").textContent = p.name;
  const t = state.tlm;
  let d = "";
  if (t) { const [ox, oy] = odom(p); d = (Math.hypot(ox - t.x_mm, oy - t.y_mm) / 1000).toFixed(1) + " m away"; }
  $("ps-dist").textContent = p.home && p.default ? "start" + (d ? " · " + d : "") : d;
  goBtn.dataset.label = p.home ? "Go home ▶" : "Go here ▶";
  if (goBtn.dataset.ask !== "1") goBtn.textContent = goBtn.dataset.label;
  $("ps-rename").hidden = !!p.home;
  $("ps-del").hidden = p.home && p.default;
  $("ps-del").textContent = p.home ? "Reset Home to the start…" : "Delete…";
  moreBtn.hidden = p.home && p.default;
  sheetHook(p);
  sheet.hidden = false;
}
export function openSheet(p) { menu.hidden = true; resetBtn(goBtn); extra.hidden = true; sel = p.name; renderSheet(); }

goBtn.onclick = () => {
  const p = byName(sel);
  if (!p) return;
  goConfirm(goBtn, p.home ? "home" : "to " + p.name, (arm) => {
    cmd(p.home ? { t: "plan", action: "home", arm } : { t: "plan", action: "goto", place: p.name, arm });
    closeSheet();
  });
};
moreBtn.onclick = () => { extra.hidden = !extra.hidden; };
$("ps-close").onclick = closeSheet;
$("ps-rename").onclick = () => {
  const n = prompt("Rename the place", sel);
  if (n && n.trim() && n.trim() !== sel) cmd({ t: "places", action: "rename", name: sel, new: n.trim() });
  extra.hidden = true;
};
$("ps-del").onclick = () => {
  const p = byName(sel);
  if (!p || !confirm(p.home ? "Reset Home to where the robot started?" : `Delete the place "${p.name}"?`)) return;
  cmd({ t: "places", action: "delete", name: p.name });
  closeSheet();
};

// ---------- long-press / right-click on the map: save a place here ----------
const menu = $("place-menu");
let at = null;           // odometry mm of the press
export function openPlaceMenu(px, py, x, y, W, H) {
  closeSheet();
  at = [Math.round(x), Math.round(y)];
  menu.hidden = false;
  menu.style.left = Math.max(8, Math.min(W - menu.offsetWidth - 8, px - 20)) + "px";
  menu.style.top = Math.max(8, Math.min(H - menu.offsetHeight - 70, py - 10)) + "px";
}
export function closePlaceMenu() { menu.hidden = true; }
document.addEventListener("pointerdown", (e) => {
  if (!menu.hidden && !menu.contains(e.target)) menu.hidden = true;
});
$("pm-save").onclick = () => {
  menu.hidden = true;
  const n = prompt("Name this place", "");
  if (n && n.trim() && at) cmd({ t: "places", action: "add", name: n.trim(), x: at[0], y: at[1] });
};
$("pm-home").onclick = () => {
  menu.hidden = true;
  if (at) cmd({ t: "places", action: "add", name: "Home", x: at[0], y: at[1] });
};

// ---------- "Go home" in the ⋯ menu ----------
const homeBtn = $("map-home");
homeBtn.dataset.label = homeBtn.textContent;
homeBtn.onclick = () => goConfirm(homeBtn, "home", (arm) => { cmd({ t: "plan", action: "home", arm }); $("map-menu").hidden = true; });

on("tlm", () => { if (sel && !sheet.hidden) renderSheet(); });
