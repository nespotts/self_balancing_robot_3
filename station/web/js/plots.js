// Live scrolling strip charts on canvas. Data: every telemetry message the bridge sends (~25 Hz).
import { on } from "./conn.js";
import { el } from "./ui.js";

const S1 = "#3987e5", S2 = "#d95926", S3 = "#199e70";   // categorical slots 1-3 (dark surface)
const CHARTS = [
  { id: "pitch", title: "Pitch °", series: [["pitch_deg", "pitch", S1], ["balance_target_deg", "target", S2]], minSpan: 4 },
  { id: "speed", title: "Speed mm/s", series: [["speed_mms", "measured", S1], ["cmd_speed_mms", "command", S2]], minSpan: 100 },
  { id: "turn", title: "Turn °/s", series: [["turn_rate_dps", "measured", S1], ["cmd_turn_dps", "command", S2]], minSpan: 40 },
  { id: "motors", title: "Motors", series: [["motor_left", "left", S1], ["motor_right", "right", S3]], fixed: [-1, 1] },
  { id: "wheels", title: "Wheels mm/s", series: [["wheel_left_mms", "left", S1], ["wheel_right_mms", "right", S3]], minSpan: 100 },
  { id: "rate", chip: "Pitch rate", title: "Pitch rate °/s", series: [["pitch_rate_dps", "rate", S1]], minSpan: 20 },
  { id: "batt", title: "Battery V", series: [["battery_v", "pack", S1]], minSpan: 0.5 },
  { id: "loop", title: "Loop Hz", series: [["loop_hz", "loop", S1]], minSpan: 50 },
];
const N = 1600;   // > 60 s at 25 Hz
const buf = { t: new Float64Array(N) };
for (const c of CHARTS) for (const [k] of c.series) buf[k] = new Float32Array(N);
let head = 0, count = 0;
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
    const canvas = el("canvas");
    const legend = el("div", { class: "legend" }, el("span", { class: "title" }, c.title));
    const vals = c.series.map(([, label, color]) => {
      const v = el("b", {}, "—");
      legend.append(el("span", {}, el("i", { style: `background:${color}` }), label + " ", v));
      return v;
    });
    const box = el("div", { class: "plot" }, canvas, legend);
    charts[c.id] = { def: c, box, canvas, ctx: canvas.getContext("2d"), vals, hoverX: null };
    canvas.addEventListener("pointermove", (e) => { charts[c.id].hoverX = e.offsetX; dirty = true; });
    canvas.addEventListener("pointerleave", () => { charts[c.id].hoverX = null; dirty = true; });
  }
  layout();
}

function layout() {
  plotsEl.innerHTML = "";
  for (const c of CHARTS) if (shown.has(c.id)) plotsEl.append(charts[c.id].box);
  dirty = true;
}

function fmtV(v) {
  const a = Math.abs(v);
  return v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : 2);
}

function niceStep(span, n) {
  const raw = span / n, p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p;
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
}

function idx(i) { return (head - count + i + N) % N; }   // i-th oldest sample

function drawChart(ch, tEnd) {
  const { canvas, ctx, def } = ch;
  const dpr = window.devicePixelRatio || 1, W = canvas.clientWidth, H = canvas.clientHeight;
  if (!W || !H) return;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const padL = 46, padR = 12, padT = 38, padB = 22;
  const pw = W - padL - padR, ph = H - padT - padB;
  const t0 = tEnd - winS * 1000;
  // first sample in the window
  let first = count;
  for (let i = count - 1; i >= 0; i--) { if (buf.t[idx(i)] < t0) { first = i + 1; break; } if (i === 0) first = 0; }
  let lo = Infinity, hi = -Infinity;
  if (def.fixed) [lo, hi] = def.fixed;
  else {
    for (const [k] of def.series) {
      const a = buf[k];
      for (let i = first; i < count; i++) { const v = a[idx(i)]; if (v < lo) lo = v; if (v > hi) hi = v; }
    }
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    const mid = (lo + hi) / 2, span = Math.max(hi - lo, def.minSpan || 1) * 1.15;
    lo = mid - span / 2; hi = mid + span / 2;
  }
  const step = niceStep(hi - lo, Math.max(2, Math.floor(ph / 40)));
  if (!def.fixed) { lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step; }
  const X = (t) => padL + ((t - t0) / (winS * 1000)) * pw;
  const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;
  // grid + y labels
  ctx.font = "11px system-ui, sans-serif"; ctx.fillStyle = "#8d8c84"; ctx.textAlign = "right"; ctx.textBaseline = "middle";
  ctx.lineWidth = 1;
  for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.strokeStyle = Math.abs(v) < step * 1e-6 ? "#45453f" : "#2a2a28";
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
    ctx.fillText(String(+v.toFixed(Math.max(0, -Math.floor(Math.log10(step) + 1e-9)))), padL - 6, y);
  }
  ctx.textBaseline = "top";
  const tStep = winS <= 10 ? 2 : winS <= 30 ? 5 : 10;
  for (let s = 0; s <= winS; s += tStep) {
    const x = padL + (s / winS) * pw;
    ctx.textAlign = s === 0 ? "left" : s === winS ? "right" : "center";
    ctx.fillText(s === winS ? "now" : "-" + (winS - s) + "s", x, H - padB + 6);
  }
  // series
  ctx.save();
  ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
  ctx.lineWidth = 2; ctx.lineJoin = "round";
  def.series.forEach(([k, , color]) => {
    const a = buf[k];
    ctx.strokeStyle = color;
    ctx.beginPath();
    for (let i = Math.max(0, first - 1); i < count; i++) {
      const j = idx(i), x = X(buf.t[j]), y = Y(a[j]);
      i === Math.max(0, first - 1) ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.stroke();
  });
  ctx.restore();
  // crosshair + values (hover) or latest values
  let at = count - 1;
  if (ch.hoverX != null && ch.hoverX >= padL && count) {
    const th = t0 + ((ch.hoverX - padL) / pw) * winS * 1000;
    for (let i = count - 1; i >= first; i--) { if (buf.t[idx(i)] <= th) { at = i; break; } }
    const x = Math.round(X(buf.t[idx(at)])) + 0.5;
    ctx.strokeStyle = "#8d8c84"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
    def.series.forEach(([k, , color]) => {
      const y = Y(buf[k][idx(at)]);
      ctx.fillStyle = color; ctx.strokeStyle = "#1a1a19"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill(); ctx.stroke();
    });
  }
  def.series.forEach(([k], n) => { ch.vals[n].textContent = count ? fmtV(buf[k][idx(at)]) : "—"; });
}

function frame() {
  requestAnimationFrame(frame);
  if (!dirty || plotsEl.offsetParent === null) return;
  dirty = false;
  const tEnd = paused ? pausedEnd : (count ? buf.t[idx(count - 1)] : 0);
  for (const c of CHARTS) if (shown.has(c.id)) drawChart(charts[c.id], tEnd);
}
requestAnimationFrame(frame);

on("tlm", (m) => {
  buf.t[head] = m.pt;
  for (const c of CHARTS) for (const [k] of c.series) buf[k][head] = m[k];
  head = (head + 1) % N;
  if (count < N) count++;
  if (!paused) dirty = true;
});
on("hello", () => { count = 0; head = 0; dirty = true; });   // bridge restarted: its clock restarted too

const pauseBtn = document.getElementById("plot-pause");
pauseBtn.onclick = () => {
  paused = !paused;
  pausedEnd = count ? buf.t[idx(count - 1)] : 0;
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
