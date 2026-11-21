// Drive panel: readouts, mode buttons, joystick + keyboard -> cmd_vel (MODE_STATION only).
import { state, on, send, modeName, robotAlive, flag, tlmFresh, STALE_MS } from "./conn.js";
import { el, toast } from "./ui.js";

const MODE_HELP = {
  IDLE: "Motors off - safe to pick up", BALANCE: "Balance in place", MANUAL: "Drive with the RC remote (only with a remote)",
  STATION: "Manual: drive with the joystick / tilt", GOTO: "Follow the waypoints", EXPLORE: "Wander and map", HOLD: "Hold position",
};
// Controller mode buttons: friendly names. IDLE is the Disarm button (top bar); MANUAL (the RC remote's sticks) shows only with a remote.
const MODE_LABEL = { BALANCE: "Balance", HOLD: "Hold", STATION: "Manual", GOTO: "Go to", EXPLORE: "Explore", MANUAL: "RC drive", IDLE: "Idle" };
const MODE_ORDER = ["BALANCE", "HOLD", "STATION", "GOTO", "EXPLORE", "MANUAL", "IDLE"];
const EVENT_LABEL =  { MARK_WAYPOINT: "Mark waypoint here", CLEAR_WAYPOINTS: "Clear waypoints", GO_HOME: "Go home",
  RESET_POSE: "Reset pose to 0,0", START_SCAN: "Stop and scan" };

const modesEl = document.getElementById("modes");
const joy = document.getElementById("joy"), knob = document.getElementById("joy-knob");
const lock = document.getElementById("joy-lock"), lockText = document.getElementById("joy-lock-text");
const take = document.getElementById("joy-take");
let limit = 0.66;
let stick = { x: 0, y: 0, active: false };
const keys = new Set();
let sending = false, lastSend = 0, warned = false;

function paramValue(re, dflt) {
  const p = state.params.find((p) => re.test(p.name));
  return p ? p.value : dflt;
}

// GOTO and EXPLORE start a drive by themselves (old waypoints, unmapped places): two taps, like Arm & Balance.
const TWO_TAP = new Set(["GOTO", "EXPLORE"]);
let tapBtn = null, tapTimer = 0;
function tapReset() {
  clearTimeout(tapTimer);
  if (tapBtn) { tapBtn.classList.remove("confirm"); tapBtn.textContent = tapBtn.dataset.label; }
  tapBtn = null;
}
function modeTap(btn, name, i) {
  if (TWO_TAP.has(name) && tapBtn !== btn) {
    tapReset();
    tapBtn = btn;
    btn.classList.add("confirm");
    btn.textContent = "Tap again";
    tapTimer = setTimeout(tapReset, 3000);
    return;
  }
  tapReset();
  send({ t: "set_mode", mode: i });
}

function buildModes() {
  modesEl.innerHTML = "";
  tapReset();
  const names = state.hello?.modes || [];
  [...names.keys()].sort((a, b) => MODE_ORDER.indexOf(names[a]) - MODE_ORDER.indexOf(names[b])).forEach((i) => {
    const name = names[i], label = MODE_LABEL[name] || name;
    const b = el("button", { class: name === "IDLE" ? "idle" : "", "data-mode": i, "data-name": name, "data-label": label,
      title: MODE_HELP[name] || "" }, label);
    b.onclick = () => modeTap(b, name, i);
    modesEl.append(b);
  });
  updateModeVisibility();
  const menu = document.getElementById("drive-menu");
  menu.innerHTML = "";
  (state.hello?.events || []).forEach((name, i) => {
    if (i === 0) return;
    menu.append(el("button", { onclick: () => {
      menu.hidden = true;
      if (name === "RESET_POSE" && !confirm("Reset the robot's pose to 0,0?")) return;
      send({ t: "event", event: i });
      toast(EVENT_LABEL[name] || name);
    } }, EVENT_LABEL[name] || name));
  });
}

export function isStation() { return state.tlm && modeName(state.tlm.mode) === "STATION"; }
// With no RC remote the station arms the robot, but only from an explicit arm step: a set_mode with
// "arm": true (two-tap Arm & Balance, the map's Go / Explore after a confirm). A plain set_mode never arms.
export function remotePresent() { return !!state.tlm?.radio_ok; }
export function disarmedNoRemote() { return robotAlive() && !flag("armed") && !remotePresent(); }

// ---------- arming: an explicit two-tap step ----------
const armBox = document.getElementById("arm-box"), armBtn = document.getElementById("arm-btn");
const armHint = document.getElementById("arm-hint"), whoEl = document.getElementById("ctl-who");
let armTimer = 0;
function armReset() { clearTimeout(armTimer); armBtn.classList.remove("confirm"); armBtn.textContent = "Arm & Balance"; }
armBtn.onclick = () => {
  if (!armBtn.classList.contains("confirm")) {
    armBtn.classList.add("confirm");
    armBtn.textContent = "Tap again to arm";
    armTimer = setTimeout(armReset, 3000);
    return;
  }
  armReset();
  send({ t: "set_mode", mode: state.hello.modes.indexOf("BALANCE"), arm: true });
  toast("Arming - stand the robot up");
};

// Who is in charge: with an RC remote present its ARMED switch rules (the phone can still pick modes);
// without one the phone is the only controller and arms / disarms by itself.
function updateWho() {
  const alive = robotAlive();
  whoEl.hidden = !alive;
  if (!alive) return;
  const rc = remotePresent();
  whoEl.className = "ctl-who " + (rc ? "rc" : "phone");
  whoEl.textContent = rc ? "Controlled by: RC remote (its switch arms)" : "Controlled by: this phone";
}
// IDLE is the Disarm button; the RC remote's own drive mode only exists with a remote
function updateModeVisibility() {
  const rc = remotePresent();
  for (const b of modesEl.children) b.hidden = b.dataset.name === "IDLE" || (b.dataset.name === "MANUAL" && !rc);
}

function updateArm() {
  const alive = robotAlive(), armed = flag("armed");
  const needArm = alive && !armed && !remotePresent();
  armBox.hidden = !(alive && !armed);
  armBtn.hidden = !needArm;
  armHint.textContent = !alive || armed ? "" : needArm ? "Motors are off. Arming starts balancing." :
    "Arm with the RC remote's switch.";
  if (!needArm) armReset();
  modesEl.hidden = needArm;   // mode buttons don't arm: show them once armed
  updateWho();
  updateModeVisibility();
}

function updateLock() {
  let txt = "";
  if (!state.connected) txt = "Not connected to the bridge";
  else if (!robotAlive()) txt = "Robot not responding";
  else if (disarmedNoRemote()) txt = "Arm the robot first";
  else if (!isStation()) txt = "Joystick drives in Manual mode";
  lock.hidden = !txt;
  joy.classList.toggle("locked", !!txt);
  lockText.textContent = txt;
  take.hidden = !(state.connected && robotAlive() && !isStation() && !disarmedNoRemote());
  updateArm();
}

// Numbers only look live while telemetry is: when the robot goes silent (flashing, unplugged, reset) they
// grey out with a "no data" badge, and the mode tiles stop claiming a mode and can't be tapped.
const tilesEl = document.querySelector("#view-drive .tiles");
function updateReadouts() {
  const t = state.tlm, fresh = tlmFresh();
  tilesEl.classList.toggle("stale", !fresh);
  for (const b of modesEl.children) {
    b.disabled = !fresh;
    b.classList.toggle("on", fresh && !!t && +b.dataset.mode === t.mode);
  }
  if (!fresh) tapReset();
  if (!t) return;
  document.getElementById("rd-speed").textContent = Math.round(t.speed_mms);
  document.getElementById("rd-turn").textContent = Math.round(t.turn_rate_dps);
  document.getElementById("rd-pitch").textContent = t.pitch_deg.toFixed(1);
  document.getElementById("rd-head").textContent = Math.round(((t.heading_deg % 360) + 540) % 360 - 180);
}

take.onclick = () => send({ t: "set_mode", mode: state.hello.modes.indexOf("STATION") });

// ---------- joystick ----------
function setKnob(x, y) {
  knob.style.transform = `translate(${x * 100}%, ${-y * 100}%)`;
}
function joyFromEvent(e) {
  const r = joy.getBoundingClientRect();
  const R = r.width * 0.36;
  let x = (e.clientX - (r.left + r.width / 2)) / R, y = -(e.clientY - (r.top + r.height / 2)) / R;
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  stick.x = x; stick.y = y;
  setKnob(x * 1.2, y * 1.2);   // knob travel = 0.36 of the pad = 1.2 knob widths
}
joy.addEventListener("pointerdown", (e) => {
  if (!lock.hidden) return;
  joy.setPointerCapture(e.pointerId);
  joy.classList.add("active");
  stick.active = true;
  joyFromEvent(e);
  pump(true);
});
joy.addEventListener("pointermove", (e) => { if (stick.active) joyFromEvent(e); });
function joyEnd() {
  if (!stick.active) return;
  stick = { x: 0, y: 0, active: false };
  joy.classList.remove("active");
  setKnob(0, 0);
  pump(true);
}
joy.addEventListener("pointerup", joyEnd);
joy.addEventListener("pointercancel", joyEnd);

// ---------- keyboard ----------
const KEYMAP = { KeyW: "f", ArrowUp: "f", KeyS: "b", ArrowDown: "b", KeyA: "l", ArrowLeft: "l", KeyD: "r", ArrowRight: "r" };
window.addEventListener("keydown", (e) => {
  if (e.target.closest("input, textarea, select") || e.ctrlKey || e.metaKey || e.altKey) return;
  const k = KEYMAP[e.code];
  if (!k) return;
  e.preventDefault();
  if (!isStation()) { if (!warned) { toast("Switch to Manual to drive with the keys"); warned = true; } return; }
  if (!keys.has(k)) { keys.add(k); pump(true); }
});
window.addEventListener("keyup", (e) => { const k = KEYMAP[e.code]; if (k && keys.delete(k)) pump(true); });
window.addEventListener("blur", () => { keys.clear(); joyEnd(); pump(true); });

function command() {
  let x = stick.x, y = stick.y;
  const ext = stick.active || keys.size ? null : extInput();
  if (ext) { x = ext.x; y = ext.y; }
  if (keys.size) {
    y = (keys.has("f") ? 1 : 0) - (keys.has("b") ? 1 : 0);
    x = (keys.has("r") ? 1 : 0) - (keys.has("l") ? 1 : 0);
  }
  const curve = (v) => Math.sign(v) * Math.pow(Math.abs(v), 1.6);   // fine control near the centre
  const maxV = paramValue(/max_?speed/i, 450), maxW = paramValue(/max_?turn/i, 180);
  return { speed: Math.round(curve(y) * maxV * limit), turn: Math.round(-curve(x) * maxW * Math.max(limit, 0.5)) };
}

// Sends cmd_vel at 10 Hz while the stick/keys are held (the bridge stops forwarding after 300 ms of silence).
function pump(now) {
  const active = stick.active || keys.size > 0 || !!extInput();
  if (active && isStation()) {
    if (now || performance.now() - lastSend > 90) { send({ t: "cmd_vel", ...command() }); lastSend = performance.now(); }
    sending = true;
  } else if (sending) {
    send({ t: "cmd_vel", speed: 0, turn: 0 });
    sending = false;
  }
}
setInterval(() => pump(false), 100);

// Extra inputs (Android app: tilt / gamepad, native.js). Each returns {x, y} (-1..1, before the curve) while it drives, else null.
const inputs = [];
export function addInput(fn) { inputs.push(fn); }
function extInput() { for (const f of inputs) { const v = f(); if (v) return v; } return null; }
export function driveNow() { pump(true); }

// ---------- options ----------
document.getElementById("speed-seg").addEventListener("click", (e) => {
  const b = e.target.closest("button");
  if (!b) return;
  limit = +b.dataset.v;
  for (const x of b.parentElement.children) x.classList.toggle("on", x === b);
});
const dmenu = document.getElementById("drive-menu");
document.getElementById("drive-more").onclick = (e) => { e.stopPropagation(); dmenu.hidden = !dmenu.hidden; };
document.addEventListener("pointerdown", (e) => { if (!dmenu.hidden && !dmenu.contains(e.target) && e.target.id !== "drive-more") dmenu.hidden = true; });

on("hello", buildModes);
on("tlm", () => { updateReadouts(); updateLock(); if (!isStation() && (stick.active || keys.size)) { keys.clear(); joyEnd(); } });
on("status", () => { updateLock(); updateReadouts(); });
on("conn", () => { updateLock(); updateReadouts(); });
setInterval(updateReadouts, STALE_MS / 4);   // telemetry stopping is not a message: look at the clock
updateLock();
updateReadouts();
