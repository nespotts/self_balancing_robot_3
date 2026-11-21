// Android app extras (station/android), only when window.SBR3Native exists; plain browsers get nothing.
//   Tilt:    hold the pad (dead-man) and tilt the phone -> cmd_vel. Zeroed at the press; release = stop.
//   Gamepad: left stick drives (STATION mode); B = STOP, X = IDLE, Y = BALANCE (twice to arm), A = STATION,
//            L1 / R1 = speed limit.
//   Haptics: E-stop, link loss, obstacle. Keeps the screen on while connected. Screen orientation in the Drive ⋯ menu.
// Native -> page events: window.__sbr3.emit(name, json): tilt, gamepad, gamepadButton, gamepads, pause, resume.
import { state, on, send, robotAlive, flag } from "./conn.js";
import { addInput, driveNow, isStation, disarmedNoRemote } from "./drive.js";
import { el, toast } from "./ui.js";

const N = window.SBR3Native;
export const TILT = { dead: 3, pitchFull: 18, rollFull: 22, staleMs: 300 };   // degrees; ms without a sample = stop
export const PAD = { dead: 0.12, staleMs: 400 };

const handlers = {};
function onNative(name, fn) { (handlers[name] ||= []).push(fn); }

// dead-band, then linear to 1 at `full` (drive.js applies its expo curve on top)
export function shape(v, dead, full) {
  const a = Math.abs(v);
  return a <= dead ? 0 : Math.sign(v) * Math.min(1, (a - dead) / (full - dead));
}
const wrap180 = (d) => ((d + 540) % 360) - 180;

if (N) init();

function init() {
  window.__sbr3 = {
    emit(name, json) {
      let d = {};
      try { d = typeof json === "string" ? JSON.parse(json) : json || {}; } catch { return; }
      for (const fn of handlers[name] || []) { try { fn(d); } catch (e) { console.error("native", name, e); } }
    },
  };
  let info = {};
  try { info = JSON.parse(N.appInfo()); } catch {}
  document.body.classList.add("native");
  document.head.append(el("link", { rel: "stylesheet", href: "native.css" }));
  const buzz = (p) => { try { N.vibrate(JSON.stringify(p)); } catch {} };

  const joy = document.getElementById("joy"), knob = document.getElementById("joy-knob");
  const wrapEl = joy.closest(".joy-wrap"), opts = document.querySelector(".drive-opts");
  const hint = el("div", { class: "tilt-hint" }, el("b", {}, "Hold"), el("span", {}, "& tilt to drive"));
  joy.insertBefore(hint, document.getElementById("joy-lock"));

  // ---------- input mode: Stick / Tilt ----------
  let mode = "stick";
  try { if (localStorage.getItem("sbr3.input") === "tilt" && info.tilt) mode = "tilt"; } catch {}
  const seg = el("div", { class: "seg", id: "input-seg", title: "Drive input" },
    el("button", { "data-v": "stick" }, "Stick"), el("button", { "data-v": "tilt" }, "Tilt"));
  seg.hidden = !info.tilt;
  seg.addEventListener("click", (e) => { const b = e.target.closest("button"); if (b) setMode(b.dataset.v); });
  opts.prepend(seg);
  const padChip = el("button", { class: "ghost pad-chip", id: "pad-chip", hidden: true, title: "Gamepad",
    onclick: () => toast("Gamepad: stick drives · B STOP · X IDLE · Y BALANCE (twice to arm) · A STATION · L1/R1 speed", 5000) },
    padIcon(), el("span", {}, "Pad"));
  opts.append(padChip);

  function setMode(m) {
    mode = m === "tilt" && info.tilt ? "tilt" : "stick";
    try { localStorage.setItem("sbr3.input", mode); } catch {}
    for (const b of seg.children) b.classList.toggle("on", b.dataset.v === mode);
    joy.classList.toggle("tilt", mode === "tilt");
    release();
    if (mode === "tilt") N.tiltStart(30); else N.tiltStop();
  }

  // ---------- tilt + dead-man ----------
  let tilt = null, hold = null;
  onNative("tilt", (d) => {
    tilt = { pitch: d.pitch, roll: d.roll, at: performance.now() };
    if (hold) { const v = tiltVec(); setKnob(v.x, v.y); }
  });
  function tiltVec() {
    return { x: shape(wrap180(tilt.roll - hold.r0), TILT.dead, TILT.rollFull),
             y: shape(wrap180(tilt.pitch - hold.p0), TILT.dead, TILT.pitchFull) };
  }
  const tiltFresh = () => tilt && performance.now() - tilt.at < TILT.staleMs;
  addInput(() => (hold && tiltFresh() ? tiltVec() : null));

  function setKnob(x, y) { knob.style.transform = `translate(${x * 120}%, ${-y * 120}%)`; }
  function release() {
    if (!hold) return;
    hold = null;
    joy.classList.remove("active", "held");
    setKnob(0, 0);
    driveNow();          // the zero cmd_vel goes out now, not on the next 100 ms tick
  }
  // capture phase on the wrapper: in Tilt mode the pad is the dead-man, drive.js never sees the touch
  wrapEl.addEventListener("pointerdown", (e) => {
    if (mode !== "tilt" || !joy.contains(e.target) || joy.classList.contains("locked")) return;
    e.stopPropagation();
    e.preventDefault();
    if (hold) return;
    if (!tiltFresh()) { toast("Tilt sensor not ready"); return; }
    try { joy.setPointerCapture(e.pointerId); } catch {}
    hold = { id: e.pointerId, p0: tilt.pitch, r0: tilt.roll };
    joy.classList.add("active", "held");
    buzz([0, 18]);
    driveNow();
  }, true);
  for (const t of ["pointerup", "pointercancel", "lostpointercapture"]) {
    joy.addEventListener(t, (e) => { if (hold && e.pointerId === hold.id) release(); });
  }
  window.addEventListener("blur", release);
  document.addEventListener("visibilitychange", () => { if (document.hidden) release(); });
  window.addEventListener("pagehide", release);   // leaving the page: the zero goes out before the socket closes
  onNative("pause", () => { release(); pad = null; driveNow(); });
  // watchdog: tilt samples stopped while held (sensor / app trouble) -> let go
  setInterval(() => { if (hold && !tiltFresh()) { release(); toast("Tilt sensor stopped - released"); } }, 100);

  // ---------- gamepad ----------
  let pad = null, padHint = false, armAsk = 0;
  onNative("gamepad", (d) => { pad = { ...d, at: performance.now() }; });
  addInput(() => {
    if (!pad || performance.now() - pad.at > PAD.staleMs) return null;
    const x = pad.lx, y = -pad.ly, m = Math.hypot(x, y);
    if (m < PAD.dead) return null;
    if (!isStation()) {
      if (!padHint) { toast(disarmedNoRemote() ? "Gamepad: Y twice arms, then A to drive" : "Gamepad: press A to drive (STATION)"); padHint = true; }
      return null;
    }
    const k = Math.min(1, (m - PAD.dead) / (1 - PAD.dead)) / m;
    return { x: x * k, y: y * k };
  });
  const modeIdx = (n) => state.hello?.modes?.indexOf(n) ?? -1;
  function setModeName(n, arm) { const i = modeIdx(n); if (i >= 0) send({ t: "set_mode", mode: i, arm: !!arm }); }
  onNative("gamepadButton", (d) => {
    if (!d.down) return;
    switch (d.name) {
      case "B": {
        if (!send({ t: "estop" })) toast("Not connected - STOP not sent!", 3000);
        buzz([0, 250]);
        const s = document.getElementById("stop");
        s.classList.remove("flash"); void s.offsetWidth; s.classList.add("flash");
        break;
      }
      case "X": setModeName("IDLE"); toast("IDLE - motors off"); break;
      case "Y":
        if (disarmedNoRemote() && performance.now() - armAsk > 2500) { armAsk = performance.now(); toast("Press Y again to arm & balance", 2500); break; }
        armAsk = 0;
        setModeName("BALANCE", disarmedNoRemote());   // the second Y press is the explicit arm step
        toast(disarmedNoRemote() ? "Arming - stand the robot up" : "BALANCE");
        break;
      case "A":
        if (disarmedNoRemote()) { toast("Arm first: press Y twice"); break; }
        setModeName("STATION"); padHint = false;
        break;
      case "L1": case "R1": {
        const bs = [...document.querySelectorAll("#speed-seg button")];
        const i = bs.findIndex((b) => b.classList.contains("on"));
        const n = Math.max(0, Math.min(bs.length - 1, i + (d.name === "R1" ? 1 : -1)));
        if (bs[n]) { bs[n].click(); toast("Speed: " + bs[n].textContent); }
        break;
      }
      case "START": location.hash = "#drive"; break;
    }
  });
  onNative("gamepads", (d) => {
    const had = !padChip.hidden;
    padChip.hidden = !(d.list && d.list.length);
    if (padChip.hidden) pad = null;
    if (had !== !padChip.hidden) toast(padChip.hidden ? "Gamepad disconnected" : "Gamepad connected: " + d.list[0].name);
  });
  try { window.__sbr3.emit("gamepads", N.gamepads()); } catch {}

  // ---------- release + haptics on robot state ----------
  let wasEstop = false, wasAlive = false, wasObstacle = false, lastObstacleBuzz = 0;
  on("status", (s) => {
    if (s.estop && !wasEstop) buzz([0, 300]);
    wasEstop = !!s.estop;
  });
  on("tlm", () => {
    if (hold && !isStation()) release();
    const ob = flag("obstacle"), now = performance.now();
    if (ob && !wasObstacle && now - lastObstacleBuzz > 2000) { buzz([0, 60, 50, 60]); lastObstacleBuzz = now; }
    wasObstacle = ob;
  });
  setInterval(() => {
    const alive = !!robotAlive();
    if (wasAlive && !alive) { buzz([0, 120, 80, 120, 80, 120]); release(); }
    wasAlive = alive;
  }, 250);
  on("conn", (up) => {
    try { N.keepAwake(!!up); } catch {}
    if (!up) release();
  });

  // ---------- Drive ⋯ menu: screen orientation, find another robot ----------
  function addMenu() {
    const menu = document.getElementById("drive-menu");
    if (menu.querySelector(".native-menu")) return;
    let cur = info.orientation || "auto";
    const o = el("div", { class: "seg native-orient" },
      ...[["auto", "Auto"], ["landscape", "Landscape"], ["portrait", "Portrait"]].map(([v, t]) =>
        el("button", { "data-v": v, class: v === cur ? "on" : "" }, t)));
    o.addEventListener("click", (e) => {
      const b = e.target.closest("button");
      if (!b) return;
      cur = b.dataset.v;
      N.setOrientation(cur);
      for (const x of o.children) x.classList.toggle("on", x === b);
    });
    menu.append(el("div", { class: "native-menu" },
      el("span", { class: "muted" }, "Screen"), o,
      el("button", { onclick: () => { menu.hidden = true; N.openFinder(); } }, "Find another robot…"),
      el("span", { class: "muted small" }, "Balance Bot " + (info.version || "") +
        (info.robot && info.robot.label ? " · " + info.robot.label : ""))));
  }
  on("hello", addMenu);
  addMenu();
  setMode(mode);
}

function padIcon() {
  const ns = "http://www.w3.org/2000/svg";
  const s = document.createElementNS(ns, "svg");
  s.setAttribute("viewBox", "0 0 24 24");
  s.innerHTML = '<path d="M7 8h10a4 4 0 0 1 4 4v1.5a3.5 3.5 0 0 1-6.3 2.1L14 15h-4l-.7.6A3.5 3.5 0 0 1 3 13.5V12a4 4 0 0 1 4-4z"/>' +
    '<path d="M8 10.5v3M6.5 12h3"/><circle cx="15.5" cy="11" r=".8" class="fill"/><circle cx="17.5" cy="13" r=".8" class="fill"/>';
  return s;
}
