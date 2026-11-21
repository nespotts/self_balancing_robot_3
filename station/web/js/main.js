// Entry: tabs, top bar, STOP. Views live in their own modules.
import { state, on, send, connect, modeName, flag, robotAlive } from "./conn.js";
import { toast } from "./ui.js";
import { invalidate as mapInvalidate } from "./map.js";
import "./drive.js";
import "./native.js";   // Android app extras (tilt, gamepad, haptics); no-op in browsers
import { plotsShown } from "./plots.js";
import { sensorsShown } from "./sensors.js";
import { tuneShown } from "./tune.js";
import { loadSessions } from "./logs.js";

const VIEWS = ["map", "drive", "sensors", "plots", "tune", "logs"];
const tabs = document.getElementById("tabs");

function show(view) {
  if (!VIEWS.includes(view)) view = "map";
  document.body.dataset.view = view;
  for (const b of tabs.children) b.classList.toggle("on", b.dataset.view === view);
  if (location.hash.slice(1) !== view) history.replaceState(null, "", "#" + view);
  if (view === "map" || view === "drive") mapInvalidate();
  if (view === "plots") plotsShown();
  sensorsShown(view === "sensors");
  tuneShown(view === "tune");
  if (view === "logs") loadSessions();
}
tabs.addEventListener("click", (e) => { const b = e.target.closest("button"); if (b) show(b.dataset.view); });
window.addEventListener("hashchange", () => show(location.hash.slice(1)));
show(location.hash.slice(1) || "map");

// ---------- STOP ----------
// STOP latches (bridge + robot, link v4): nothing restarts the motors until "Release E-stop" (asks first),
// and even then the robot stays disarmed until it is armed again.
const stopBtn = document.getElementById("stop");
const estopBar = document.getElementById("estop-bar");
function estop() {
  if (!send({ t: "estop" })) toast("Not connected - STOP not sent!", 3000);
  stopBtn.classList.remove("flash"); void stopBtn.offsetWidth; stopBtn.classList.add("flash");
}
// latched: the bridge's flag, or the robot's own (remote_flags LINK_RF_LINK_ESTOP 0x08)
function estopLatched() {
  const s = state.status, t = state.tlm;
  return !!(s?.estop || s?.robot_estop || (t && robotAlive() && (t.remote_flags & 0x08)));
}
document.getElementById("estop-release").onclick = () => {
  if (!confirm("Release the E-stop?\n\nThe robot stays disarmed: arm it again (Arm & Balance, or the RC remote's ARMED switch off -> on) to drive.")) return;
  // the bar goes away when the bridge / robot report the latch cleared; no link -> a no_link error toast
  if (!send({ t: "estop_release" })) toast("Not connected - release not sent", 3000);
};
stopBtn.addEventListener("pointerdown", (e) => { e.preventDefault(); estop(); });   // fire on touch-down, not release
window.addEventListener("keydown", (e) => {
  if ((e.code === "Space" || e.code === "Escape") && !e.target.closest("input, textarea")) { e.preventDefault(); estop(); }
});

// The bridge refuses robot commands while it has no link to the robot (and says so): show it once, here.
on("error", (m) => {
  if ((m.code === "no_link" || m.code === "estop") && m.text) toast(m.text.charAt(0).toUpperCase() + m.text.slice(1), 3500);
});

// ---------- top bar ----------
const pLink = document.getElementById("pill-link"), pMode = document.getElementById("pill-mode");
const pArmed = document.getElementById("pill-armed"), pBatt = document.getElementById("pill-batt");
let rcLost = false, navHandover = false;
function setPill(p, text, cls) {
  p.querySelector(".txt").textContent = text;
  p.className = "pill " + (cls || "");
}
function updateTop() {
  const s = state.status, t = state.tlm;
  if (!state.connected) setPill(pLink, "OFFLINE", "bad");
  else if (!s || s.link === "waiting") setPill(pLink, "NO ROBOT", "warn");
  else if (s.link === "released") setPill(pLink, "FLASHING", "warn");
  else if (!s.robot_alive) setPill(pLink, "SILENT", "warn");
  else if (t && !flag("pi_link")) setPill(pLink, "LINK?", "warn");
  // The failsafe bit is always set with no RC remote: only an alarm if a remote was there and went quiet.
  else if (t && s.remote_seen && !t.radio_ok) setPill(pLink, "RC LOST", "warn");
  else setPill(pLink, "LINK", "good");
  const lost = !!(t && s?.remote_seen && !t.radio_ok && robotAlive());
  if (lost && !rcLost) toast("RC remote lost - the robot stops driving and keeps balancing", 4000);
  rcLost = lost;
  const alive = robotAlive();
  setPill(pMode, t && alive ? modeName(t.mode) : "—", t && alive && t.mode !== 0 ? "mode-on" : "");
  const latched = estopLatched();
  estopBar.hidden = !latched;
  // GOTO / EXPLORE / HOLD paused because the RC remote was lost: needs it back, or a mode from here (link v4: 0x10)
  const handover = !!(t && alive && (t.remote_flags & 0x10));
  if (handover && !navHandover) toast("Remote lost: the robot stopped and waits. Pick the mode here to take over.", 5000);
  navHandover = handover;
  if (latched) setPill(pArmed, "E-STOP", "estop");
  else if (t && alive && (t.remote_flags & 1)) setPill(pArmed, "RC E-STOP", "estop");   // link v3: LINK_RF_ESTOP
  else if (t && alive) setPill(pArmed, flag("armed") ? (flag("upright") ? "ARMED" : "ARMED·DOWN") : "SAFE", flag("armed") ? "armed" : "");
  else setPill(pArmed, "—");
  if (t && alive) setPill(pBatt, t.battery_v.toFixed(1) + " V", flag("low_batt") ? "lowbatt" : "");
  else setPill(pBatt, "— V");
}
pLink.onclick = () => show("logs");
pMode.onclick = () => show(window.matchMedia("(min-width: 900px)").matches ? "map" : "drive");
let lastTop = 0;
on("tlm", () => { const n = performance.now(); if (n - lastTop > 200) { lastTop = n; updateTop(); } });
on("status", updateTop);
on("conn", (up) => { updateTop(); if (!up) toast("Connection to the bridge lost - retrying"); });
updateTop();

connect();
window.sbr3 = { state, send, show };   // for debugging from the console
