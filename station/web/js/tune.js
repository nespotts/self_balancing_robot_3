// Tuning: live balance-loop chart (full-rate "tune" stream) + controls generated from the robot's
// ParamInfoFrames (names/ranges come from the firmware). cfg.* params are actions -> buttons.
import { state, on, send } from "./conn.js";
import { el, fmt, toast } from "./ui.js";
import { Strip, S1, S2, S3, SUM } from "./chart.js";

const GROUPS = { bal: "Balance", str: "Heading", vel: "Velocity", drv: "Driving", mot: "Motors",
  ctl: "Control loop", batt: "Battery", tlm: "Telemetry" };
const OPEN_BY_DEFAULT = new Set(["bal", "str", "vel"]);
const DISCRETE = {   // small integer params shown as segmented buttons
  "bal.d_src": ["d(pitch)/dt", "Gyro"],
  "ctl.loops": ["Balance", "+Heading", "+Velocity", "All"],
};
const HIDDEN = (name) => name.startsWith("cfg.");   // actions, not settings

const groupsEl = document.getElementById("tune-groups"), info = document.getElementById("tune-info");
const saveBtn = document.getElementById("tune-save");
const rows = {};   // id -> {slider, num, revert, row, seg}
let builtKey = "", unsaved = false, saveAskedAt = -1e9;

let openGroups;
try { openGroups = new Set(JSON.parse(localStorage.getItem("sbr3.tuneOpen") || "null") || OPEN_BY_DEFAULT); }
catch { openGroups = new Set(OPEN_BY_DEFAULT); }

function param(name) { return state.params.find((p) => p.name === name); }
function steps(p) { return (p.max - p.min) / 1000 || 0.001; }

function setUnsaved(v) {
  unsaved = v;
  saveBtn.classList.toggle("unsaved", v);
  saveBtn.textContent = v ? "Save to robot •" : "Save to robot";
}

function apply(id, v, r) {
  send({ t: "set_param", id, value: v });
  setUnsaved(true);
  if (!r) return;
  r.row.classList.add("sent");
  clearTimeout(r.flash);
  r.flash = setTimeout(() => r.row.classList.remove("sent"), 400);
}

function sliderRow(p, label) {
  const num = el("input", { type: "number", step: "any", min: p.min, max: p.max, inputmode: "decimal" });
  const slider = el("input", { type: "range", min: p.min, max: p.max, step: steps(p) });
  const revert = el("button", { class: "revert", title: "Back to the value at page load" }, "↺");
  const row = el("div", { class: "prow" }, el("span", { class: "pname", title: p.name }, label), revert, num, slider,
    el("div", { class: "range" }, el("span", {}, fmt(p.min)), el("span", {}, fmt(p.max))));
  const r = { slider, num, revert, row, last: 0, timer: 0 };
  slider.addEventListener("input", () => {
    const v = +slider.value;
    num.value = +v.toPrecision(5);
    const now = performance.now();   // live apply, throttled to 10 Hz while dragging
    clearTimeout(r.timer);
    if (now - r.last > 100) { r.last = now; apply(p.id, v, r); }
    else r.timer = setTimeout(() => { r.last = performance.now(); apply(p.id, +slider.value, r); }, 100);
  });
  num.addEventListener("change", () => {
    let v = +num.value;
    if (!isFinite(v)) return;
    v = Math.max(p.min, Math.min(p.max, v));
    num.value = v; slider.value = v;
    apply(p.id, v, r);
  });
  num.addEventListener("keydown", (e) => { if (e.key === "Enter") num.blur(); });
  revert.onclick = () => {
    const v = state.paramsFirst[p.id];
    num.value = v; slider.value = v;
    apply(p.id, v, r);
  };
  return r;
}

function segRow(p, label, names) {
  const seg = el("div", { class: "seg pseg" });
  names.forEach((n, i) => seg.append(el("button", { "data-v": p.min + i, onclick: () => apply(p.id, p.min + i, r) }, n)));
  const revert = el("button", { class: "revert", title: "Back to the value at page load",
    onclick: () => apply(p.id, state.paramsFirst[p.id], r) }, "↺");
  const row = el("div", { class: "prow prow-seg" }, el("span", { class: "pname", title: p.name }, label), revert, seg);
  const r = { seg, revert, row };
  return r;
}

function build() {
  const list = state.params.filter((p) => !HIDDEN(p.name));
  const key = list.map((p) => p.id + p.name).join("|");
  if (key === builtKey) return update();
  builtKey = key;
  groupsEl.innerHTML = "";
  for (const k in rows) delete rows[k];
  const groups = new Map();
  for (const p of list) {
    const g = p.name.includes(".") ? p.name.split(".")[0] : "other";
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(p);
  }
  for (const [g, ps] of groups) {
    const card = el("details", { class: "card pgroup" });
    card.open = openGroups.has(g);
    card.append(el("summary", {}, el("h3", {}, GROUPS[g] || g, el("small", {}, ps.length + ""))));
    card.addEventListener("toggle", () => {
      card.open ? openGroups.add(g) : openGroups.delete(g);
      try { localStorage.setItem("sbr3.tuneOpen", JSON.stringify([...openGroups])); } catch {}
    });
    for (const p of ps) {
      const label = p.name.includes(".") ? p.name.slice(p.name.indexOf(".") + 1) : p.name;
      const r = DISCRETE[p.name] ? segRow(p, label, DISCRETE[p.name]) : sliderRow(p, label);
      rows[p.id] = r;
      card.append(r.row);
    }
    groupsEl.append(card);
  }
  update();
}

function update() {
  for (const p of state.params) {
    const r = rows[p.id];
    if (!r) continue;
    if (r.seg) for (const b of r.seg.children) b.classList.toggle("on", +b.dataset.v === Math.round(p.value));
    else {
      if (document.activeElement !== r.num) r.num.value = +(+p.value).toPrecision(6);
      if (document.activeElement !== r.slider) r.slider.value = p.value;
    }
    r.revert.hidden = state.paramsFirst[p.id] === undefined || Math.abs(state.paramsFirst[p.id] - p.value) < 1e-9;
  }
  const n = state.params.filter((p) => !HIDDEN(p.name)).length;
  info.textContent = n ? "Changes apply live"
    : state.connected ? "Waiting for parameters from the robot…" : "Not connected";
  saveBtn.disabled = !param("cfg.save");
  const hz = param("tlm.hz"), rate = param("ctl.rate_hz");
  fullBtn.classList.toggle("on", !!(hz && rate && hz.value >= rate.value));
}

// ---------- save / defaults / reload ----------
saveBtn.onclick = () => {
  if (!send({ t: "cfg", action: "save" })) return toast("Not connected");
  saveAskedAt = performance.now();
  toast("Saving…");
};
const menu = document.getElementById("tune-menu");
document.getElementById("tune-more").onclick = (e) => { e.stopPropagation(); menu.hidden = !menu.hidden; };
document.addEventListener("pointerdown", (e) => { if (!menu.hidden && !menu.contains(e.target) && e.target.id !== "tune-more") menu.hidden = true; });
document.getElementById("tune-reload").onclick = () => { menu.hidden = true; send({ t: "get_params" }); toast("Reloading…"); };
document.getElementById("tune-defaults").onclick = () => {
  menu.hidden = true;
  if (!confirm("Restore every parameter to the firmware defaults?\n\nThey apply now; tap Save to robot to keep them after a reboot.")) return;
  send({ t: "cfg", action: "defaults" });
  setUnsaved(true);
};
on("log", (m) => {
  if (m.src !== "robot" || performance.now() - saveAskedAt > 60000) return;
  if (m.text === "params: saved") { toast("Saved to the robot"); setUnsaved(false); saveAskedAt = -1e9; }
  else if (m.text.includes("save waits")) toast("Saves when the motors are off (disarm)", 3500);
  else if (m.text.includes("SAVE FAILED")) toast("Save FAILED", 4000);
});

// ---------- balance-loop chart (full-rate tune stream) ----------
const WIN_S = 5;
const pid = new Strip({ title: "Balance output (motor counts)", minSpan: 400, series: [   // total drawn first (behind)
  ["out", "total", SUM, (m) => Math.max(-4095, Math.min(4095, m.bal_p + m.bal_i + m.bal_d))],
  ["bal_p", "P", S1], ["bal_i", "I", S2], ["bal_d", "D", S3]] }, 6000);
const pitch = new Strip({ title: "Pitch °", minSpan: 2, series: [
  ["pitch_deg", "pitch", S1], ["balance_target_deg", "target", S2]] }, 6000);
const tcPlots = document.getElementById("tc-plots"), tcRate = document.getElementById("tc-rate");
tcPlots.append(pid.box, pitch.box);
const fullBtn = document.getElementById("tc-full"), pauseBtn = document.getElementById("tc-pause");
let shown = false, paused = false, pausedEnd = 0, dirty = true, fullByMe = false, rowTimes = [];
pid.onhover = pitch.onhover = () => (dirty = true);

on("tune", (m) => {
  const ci = {};
  m.cols.forEach((c, i) => (ci[c] = i));
  for (const r of m.rows) {
    const o = {};
    for (const c in ci) o[c] = r[ci[c]];
    pid.push(o.rt, o); pitch.push(o.rt, o);
  }
  const now = performance.now();
  rowTimes.push([now, m.rows.length]);
  while (rowTimes.length && now - rowTimes[0][0] > 2000) rowTimes.shift();
  if (!paused) dirty = true;
});
setInterval(() => {
  const n = rowTimes.reduce((a, x) => a + x[1], 0);
  tcRate.textContent = shown && state.connected ? (n ? `${Math.round(n / 20) * 10} Hz` : "no data") : "";
}, 500);

function frame() {
  requestAnimationFrame(frame);
  if (!dirty || !shown) return;
  dirty = false;
  const tEnd = paused ? pausedEnd : pid.last();
  pid.draw(tEnd, WIN_S); pitch.draw(tEnd, WIN_S);
}
requestAnimationFrame(frame);

fullBtn.onclick = () => {
  const hz = param("tlm.hz"), rate = param("ctl.rate_hz");
  if (!hz) return;
  const on = !(rate && hz.value >= rate.value);
  send({ t: "set_param", id: hz.id, value: on ? (rate ? rate.value : 400) : 100 });
  fullByMe = on;
};
pauseBtn.onclick = () => {
  paused = !paused;
  pausedEnd = pid.last();
  pauseBtn.textContent = paused ? "Resume" : "Pause";
  pauseBtn.classList.toggle("on", paused);
  dirty = true;
};

// called by main.js when the view changes
export function tuneShown(v) {
  if (v === shown) return;
  shown = v;
  send({ t: "sub", tune: v });
  if (!v && fullByMe) {   // back to the normal rate (smaller logs) when we turned it up
    const hz = param("tlm.hz");
    if (hz) send({ t: "set_param", id: hz.id, value: 100 });
    fullByMe = false;
  }
  dirty = true;
}
on("conn", (up) => { if (up && shown) send({ t: "sub", tune: true }); update(); });
on("hello", () => { pid.clear(); pitch.clear(); });
window.addEventListener("resize", () => (dirty = true));
on("params", build);
on("robot", update);
build();
