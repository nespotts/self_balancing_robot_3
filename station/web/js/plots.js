// Live scrolling strip charts. Data: every telemetry message the bridge sends (~25 Hz).
// (Full-rate balance data is on the Tune view.)
import { on } from "./conn.js";
import { el } from "./ui.js";
import { Strip, S1, S2, S3 } from "./chart.js";

const CHARTS = [
  { id: "pitch", title: "Pitch °", series: [["pitch_deg", "pitch", S1], ["balance_target_deg", "target", S2]], minSpan: 4 },
  { id: "speed", title: "Speed mm/s", series: [["speed_mms", "measured", S1], ["cmd_speed_mms", "command", S2]], minSpan: 100 },
  { id: "turn", title: "Turn °/s", series: [["turn_rate_dps", "measured", S1], ["cmd_turn_dps", "command", S2]], minSpan: 40 },
  { id: "motors", title: "Motors", series: [["motor_left", "left", S1], ["motor_right", "right", S3]], fixed: [-1, 1] },
  { id: "pid", chip: "P/I/D", title: "Balance P/I/D", series: [["bal_p", "P", S1], ["bal_i", "I", S2], ["bal_d", "D", S3]], minSpan: 200 },
  { id: "wheels", title: "Wheels mm/s", series: [["wheel_left_mms", "left", S1], ["wheel_right_mms", "right", S3]], minSpan: 100 },
  { id: "rate", chip: "Pitch rate", title: "Pitch rate °/s", series: [["pitch_rate_dps", "rate", S1]], minSpan: 20 },
  { id: "batt", title: "Battery V", series: [["battery_v", "pack", S1]], minSpan: 0.5 },
  { id: "loop", title: "Loop Hz", series: [["loop_hz", "loop", S1]], minSpan: 50 },
];
const N = 1600;   // > 60 s at 25 Hz
let paused = false, pausedEnd = 0, winS = 10, dirty = true;

let shown;
try { shown = new Set(JSON.parse(localStorage.getItem("sbr3.plots") || "null") || ["pitch", "speed", "motors"]); }
catch { shown = new Set(["pitch", "speed", "motors"]); }

const chipsEl = document.getElementById("plot-chips"), plotsEl = document.getElementById("plots");
const charts = {};

function build() {
  chipsEl.innerHTML = "";
  for (const c of CHARTS) {
    chipsEl.append(el("button", { class: "chip" + (shown.has(c.id) ? " on" : ""), onclick: (e) => {
      shown.has(c.id) ? shown.delete(c.id) : shown.add(c.id);
      e.target.classList.toggle("on", shown.has(c.id));
      try { localStorage.setItem("sbr3.plots", JSON.stringify([...shown])); } catch {}
      layout();
    } }, c.chip || c.title.replace(/ .*/, "")));
    charts[c.id] = new Strip(c, N);
    charts[c.id].onhover = () => (dirty = true);
  }
  layout();
}

function layout() {
  plotsEl.innerHTML = "";
  for (const c of CHARTS) if (shown.has(c.id)) plotsEl.append(charts[c.id].box);
  dirty = true;
}

function frame() {
  requestAnimationFrame(frame);
  if (!dirty || plotsEl.offsetParent === null) return;
  dirty = false;
  const tEnd = paused ? pausedEnd : charts.pitch.last();
  for (const c of CHARTS) if (shown.has(c.id)) charts[c.id].draw(tEnd, winS);
}
requestAnimationFrame(frame);

on("tlm", (m) => {
  for (const c of CHARTS) charts[c.id].push(m.pt, m);
  if (!paused) dirty = true;
});
on("hello", () => { for (const c of CHARTS) charts[c.id].clear(); dirty = true; });   // bridge restarted: new clock

const pauseBtn = document.getElementById("plot-pause");
pauseBtn.onclick = () => {
  paused = !paused;
  pausedEnd = charts.pitch.last();
  pauseBtn.textContent = paused ? "Resume" : "Pause";
  pauseBtn.classList.toggle("primary", !paused);
  dirty = true;
};
document.getElementById("plot-win").addEventListener("click", (e) => {
  const b = e.target.closest("button");
  if (!b) return;
  winS = +b.dataset.v;
  for (const x of b.parentElement.children) x.classList.toggle("on", x === b);
  dirty = true;
});
window.addEventListener("resize", () => (dirty = true));
export function plotsShown() { dirty = true; }
build();
