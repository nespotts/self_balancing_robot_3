// Sensors view (issue #19): the two VL53L0X range sensors on the servo head.
// Live readout per sensor, polar view of the last sweep (robot frame, forward up), range vs time,
// and setup (park / trim / sweep / direction / hand test) behind the ⋯.
// Data: "ranges" entries [i, hx, hy, rx, ry, valid, tilt, t_ms, sensor_id, sensor_angle_deg, range_mm].
import { state, on, send, tlmFresh } from "./conn.js";
import { el, toast } from "./ui.js";
import { SENS, SENS_NEW, SENS_RGB } from "./chart.js";

const NAMES = ["Front", "Rear"], XSHUT = [34, 28];
const WIN_MS = 10000, KEEP_MS = 60000, STALE_MS = 2000;
const NO_TARGET = 8190;

const param = (name) => state.params.find((p) => p.name === name);
const pv = (name, d) => { const p = param(name); return p ? p.value : d; };
const fmtA = (a) => (a > 0.05 ? "+" : "") + (Math.abs(a) < 10 ? a.toFixed(1) : Math.round(a)) + "°";
const fmtMm = (mm) => (mm >= 1000 ? (mm / 1000).toFixed(mm % 1000 ? 1 : 0) + " m" : mm + " mm");

// ---------- live data ----------
const S = [0, 1].map(() => ({ last: null, hist: [], dirs: new Map(), base: new Map(), hitUntil: 0 }));
let lastIdx = -1, shown = false, paused = false, pausedEnd = 0, dirty = true;
let piBase = 0, piAt = 0, handTest = false;
const piNow = () => (piAt ? piBase + performance.now() - piAt : 0);

function ingest(p) {
  if (p.length < 11) return;                     // bridge without sensor ids (before issue #19)
  const s = S[p[8] ? 1 : 0];
  const r = { t: p[7], ok: !!p[5], tilt: !!p[6], a: p[9], mm: p[10] };
  s.last = r;
  s.hist.push(r);
  if (s.hist.length > 4000) s.hist.splice(0, s.hist.length - 3000);
  s.dirs.set(Math.round(r.a / 2), r);           // latest reading per 2° direction -> polar view
  if (handTest) hand(s, r);
}

// "Which sensor is which": a reading much closer than what that sensor usually sees in that
// direction (baseline per 6°: follows increases at once, decreases over ~3 s) lights its card.
function hand(s, r) {
  if (piNow() - r.t > 1000) return;              // replayed history: no flashes
  const far = pv("rng.max_mm", 2000), v = r.ok ? r.mm : far, key = Math.round(r.a / 6);
  const b = s.base.get(key);
  if (!b) { s.base.set(key, { v, t: r.t }); return; }
  if (r.ok && r.mm < 0.6 * b.v && b.v - r.mm > 120) { s.hitUntil = performance.now() + 1500; dirty = true; }
  const k = v > b.v ? 0.6 : Math.min(1, Math.max(0, r.t - b.t) / 3000);
  b.v += (v - b.v) * k; b.t = r.t;
}

on("ranges", (m) => {
  for (const p of m.pts) if (p[0] > lastIdx) { lastIdx = p[0]; ingest(p); }
  if (!paused) dirty = true;
});
on("hello", () => { lastIdx = -1; for (const s of S) { s.last = null; s.hist = []; s.dirs.clear(); s.base.clear(); } dirty = true; });
on("tlm", (m) => { piBase = m.pt; piAt = performance.now(); });

// ---------- readout cards ----------
const cardsEl = document.getElementById("sens-cards");
const cards = [0, 1].map((i) => {
  const big = el("b", {}, "—"), unit = el("span", { class: "unit" }, "mm");
  const ok = el("span", { class: "sc-ok" }, "—"), ang = el("span", {}, "—"), rate = el("span", {}, "—");
  const card = el("div", { class: "card sens-card", style: `--sc:${SENS[i]}` },
    el("div", { class: "sc-head" }, el("i", { class: "sw" }), el("b", {}, NAMES[i]),
      el("span", { class: "nodata" }, "no data"),
      el("span", { class: "muted" }, `sensor ${i} · XSHUT ${XSHUT[i]}`), el("span", { class: "sc-badge" }, "this one")),
    el("div", { class: "sc-range" }, big, unit),
    el("div", { class: "sc-meta" }, ok, ang, rate));
  cardsEl.append(card);
  return { card, big, unit, ok, ang, rate };
});

function updateCards() {
  const now = piNow(), nowP = performance.now(), max = pv("rng.max_mm", 2000);
  const hz = state.status?.rng_hz;
  cards.forEach((c, i) => {
    // stale: the robot went quiet (the last numbers are old) or this sensor stopped reporting
    const r = S[i].last, stale = !r || !tlmFresh() || (piAt && now - r.t > STALE_MS);
    c.card.classList.toggle("stale", !!stale);
    c.card.classList.toggle("hit", handTest && S[i].hitUntil > nowP);
    if (!r) { c.big.textContent = "—"; c.unit.textContent = ""; c.ok.textContent = "no readings"; c.ok.className = "sc-ok"; c.ang.textContent = ""; c.rate.textContent = ""; return; }
    c.big.textContent = r.ok ? r.mm : "—";
    c.unit.textContent = r.ok ? "mm" : r.mm > 0 && r.mm < NO_TARGET ? `${r.mm} mm` : "";
    const why = stale ? "no readings" : r.ok ? "valid" : r.mm === 0 ? "timeout" : r.mm > max && r.mm < NO_TARGET ? "too far" : "no target";
    c.ok.textContent = why + (r.ok && r.tilt ? " · tilted" : "");
    c.ok.className = "sc-ok " + (stale ? "" : r.ok ? "good" : "bad");
    c.ang.textContent = fmtA(r.a);
    c.rate.textContent = hz ? `${Math.round(hz[i])}/s` : "";
  });
}

// ---------- status bar ----------
const statusEl = document.getElementById("sens-status"), onBtn = document.getElementById("sens-on");
const polarTitle = document.getElementById("sens-polar-title");
function servoHead(hold) {   // firmware: command = hold + trim, clamped; reported head = dir*scale*(cmd-90-trim)
  const trim = pv("rng.servo_trim_deg", 0), dir = pv("rng.servo_dir", 1) < 0 ? -1 : 1, sc = pv("rng.servo_deg_per_cmd", 1);
  const lo = pv("rng.servo_min_deg", 0), hi = pv("rng.servo_max_deg", 180), raw = hold + trim;
  const cmd = Math.max(lo, Math.min(hi, raw));
  return { head: dir * sc * (cmd - 90 - trim), clamped: Math.abs(cmd - raw) > 0.01, cmd };
}
function updateStatus() {
  const en = param("rng.enable");
  let txt;
  if (!state.connected) txt = "Not connected";
  else if (!state.params.length) txt = "Waiting for the robot…";
  else if (!en) txt = "No range sensor params on this firmware";
  else if (en.value < 0.5) txt = "Range sensor off";
  else {
    const hold = pv("rng.servo_hold_deg", -1);
    if (hold >= 0) {
      const h = servoHead(hold);
      txt = `Parked at ${Math.round(hold)}° · head ${fmtA(h.head)}${h.clamped ? " (limit)" : ""}`;
    } else txt = `Sweeping ${Math.round(pv("rng.sweep_deg", 170))}° · ${Math.round(pv("rng.steps", 16))} steps`;
  }
  statusEl.textContent = txt;
  polarTitle.textContent = pv("rng.servo_hold_deg", -1) >= 0 ? "Parked head" : "Last sweep";
  onBtn.hidden = !(en && en.value < 0.5 && state.connected);
}
onBtn.onclick = () => setParam("rng.enable", 1);

// ---------- setup menu (⋯) ----------
const menu = document.getElementById("sens-menu"), moreBtn = document.getElementById("sens-more");
const syncs = [];
function setParam(name, v) {
  const p = param(name);
  if (!p) return toast("The robot has no " + name);
  if (!send({ t: "set_param", id: p.id, value: v })) return toast("Not connected");
  p.value = v;   // optimistic; the robot's echo follows
  syncAll();
}
function throttled(fn) {   // live apply while dragging, at most 10 per second
  let last = 0, timer = 0;
  return (v) => {
    clearTimeout(timer);
    const now = performance.now();
    if (now - last > 100) { last = now; fn(v); } else timer = setTimeout(() => { last = performance.now(); fn(v); }, 100);
  };
}
function sliderRow(name, label, { step = 1, lo, hi, idle, nudge, show = (v) => fmtA(v).replace("+", "") } = {}) {
  const val = el("b", {}, "—");
  const input = el("input", { type: "range", step });
  const apply = throttled((v) => setParam(name, v));
  input.addEventListener("input", () => { val.textContent = show(+input.value); apply(+input.value); });
  const kids = [input];
  if (nudge) {
    const nb = (d, t) => el("button", { class: "small nudge", title: `${d > 0 ? "+" : "−"}${nudge}°`, onclick: () => {
      const p = param(name); if (!p) return;
      setParam(name, Math.max(p.min, Math.min(p.max, Math.round((p.value + d * nudge) * 100) / 100)));
    } }, t);
    kids.unshift(nb(-1, "−")); kids.push(nb(1, "+"));
  }
  const row = el("div", { class: "srow" }, el("div", { class: "sl" }, el("span", {}, label), val), el("div", { class: "sctl" }, ...kids));
  syncs.push(() => {
    const p = param(name);
    input.disabled = !p;
    for (const b of row.querySelectorAll("button")) b.disabled = !p;
    if (!p) { val.textContent = "—"; return; }
    input.min = lo ?? p.min; input.max = hi ?? p.max;
    if (document.activeElement !== input) input.value = p.value < +input.min ? idle ?? input.min : p.value;   // (hold -1 = sweeping)
    val.textContent = show(p.value);
  });
  return row;
}
function segRow(name, label, opts, isOn = (p, v) => Math.abs(p.value - v) < 0.01) {
  const seg = el("div", { class: "seg" });
  for (const [t, v] of opts) seg.append(el("button", { "data-v": v, onclick: () => setParam(name, v) }, t));
  syncs.push(() => {
    const p = param(name);
    for (const b of seg.children) { b.disabled = !p; b.classList.toggle("on", !!p && isOn(p, +b.dataset.v)); }
  });
  return el("div", { class: "srow" }, label ? el("div", { class: "sl" }, el("span", {}, label)) : null, seg);
}
function toggleRow(label, get, set) {
  const cb = el("input", { type: "checkbox" });
  cb.onchange = () => set(cb.checked);
  syncs.push(() => { const v = get(); cb.disabled = v == null; cb.checked = !!v; });
  return el("label", { class: "srow stog" }, cb, el("span", {}, label));
}
function buildMenu() {
  menu.append(
    el("h4", {}, "Range sensor setup"),
    toggleRow("Sensors on", () => { const p = param("rng.enable"); return p ? p.value >= 0.5 : null; },
      (v) => setParam("rng.enable", v ? 1 : 0)),
    segRow("rng.servo_hold_deg", "Head", [["Sweep", -1], ["0°", 0], ["5°", 5], ["90°", 90], ["175°", 175], ["180°", 180]],
      (p, v) => (v < 0 ? p.value < 0 : Math.abs(p.value - v) < 0.01)),
    sliderRow("rng.servo_hold_deg", "Park at (servo command)", { lo: 0, hi: 180, idle: 90,
      show: (v) => (v < 0 ? "sweeping" : Math.round(v) + "°") }),
    sliderRow("rng.servo_trim_deg", "Trim", { step: 0.5, nudge: 0.5 }),
    sliderRow("rng.sweep_deg", "Sweep width", { step: 5, show: (v) => Math.round(v) + "°" }),
    segRow("rng.servo_dir", "Direction", [["Normal", 1], ["Reversed", -1]], (p, v) => (p.value < 0 ? -1 : 1) === v),
    toggleRow("Hand test: which sensor is which", () => handTest, setHand),
    el("button", { class: "ghost small more-link", onclick: () => { menu.hidden = true; window.sbr3?.show("tune"); } },
      "All range settings in Tune →"),
  );
}
function syncAll() { for (const f of syncs) f(); updateStatus(); }
moreBtn.onclick = (e) => { e.stopPropagation(); menu.hidden = !menu.hidden; if (!menu.hidden) syncAll(); };
document.addEventListener("pointerdown", (e) => { if (!menu.hidden && !menu.contains(e.target) && e.target !== moreBtn) menu.hidden = true; });

const handEl = document.getElementById("sens-hand");
function setHand(v) {
  handTest = v;
  handEl.hidden = !v;
  for (const s of S) { s.base.clear(); s.hitUntil = 0; }
  if (v && pv("rng.servo_hold_deg", -1) < 0) toast("Tip: park the head (90°) so both sensors stay still", 3500);
  syncAll(); dirty = true;
}
document.getElementById("sens-hand-done").onclick = () => setHand(false);

const pauseBtn = document.getElementById("sens-pause");
pauseBtn.onclick = () => {
  paused = !paused;
  pausedEnd = piNow();
  pauseBtn.textContent = paused ? "Resume" : "Pause";
  pauseBtn.classList.toggle("on", paused);
  dirty = true;
};

// ---------- canvases ----------
function fit(c) {
  const dpr = window.devicePixelRatio || 1, W = c.clientWidth, H = c.clientHeight;
  if (!W || !H) return null;
  if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  return { ctx, W, H };
}

let scale;
try { scale = +localStorage.getItem("sbr3.sensScale") || 2000; } catch { scale = 2000; }
const scaleSeg = document.getElementById("sens-scale");
function syncScale() { for (const b of scaleSeg.children) b.classList.toggle("on", +b.dataset.v === scale); }
scaleSeg.addEventListener("click", (e) => {
  const b = e.target.closest("button"); if (!b) return;
  scale = +b.dataset.v; syncScale(); dirty = true;
  try { localStorage.setItem("sbr3.sensScale", String(scale)); } catch {}
});
syncScale();

// how long one pass (360° with both sensors) takes: what "last sweep" keeps
function keepMs() {
  if (pv("rng.servo_hold_deg", -1) >= 0) return 800;
  const pass = Math.max(1, pv("rng.steps", 16)) * (pv("rng.settle_ms", 40) + pv("rng.timing_budget_ms", 33) + 7);
  return Math.max(1500, pass * 1.25 + 300);
}

const polar = document.getElementById("sens-polar");
function drawPolar(tEnd) {
  const f = fit(polar); if (!f) return;
  const { ctx, W, H } = f;
  const geo = state.hello?.range_geom || { fwd_mm: 26, radius_mm: 12.9 };
  const cx = W / 2, cy = H / 2, R = Math.min(W, H) / 2 - 22, k = R / scale;   // px per mm; centre = servo axis
  const XY = (a, d) => [cx - Math.sin(a * Math.PI / 180) * d * k, cy - Math.cos(a * Math.PI / 180) * d * k];
  // rings + labels
  const step = scale <= 500 ? 100 : scale <= 1000 ? 250 : scale <= 2000 ? 500 : 1000;
  ctx.lineWidth = 1; ctx.font = "11px system-ui, sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "middle";
  for (let d = step; d <= scale + 1e-6; d += step) {
    ctx.strokeStyle = d === scale ? "#45453f" : "#2a2a28";
    ctx.beginPath(); ctx.arc(cx, cy, d * k, 0, 7); ctx.stroke();
    ctx.fillStyle = "#8d8c84";
    const [lx, ly] = XY(-45, d);
    ctx.fillText(fmtMm(d), lx + 3, ly - 7);
  }
  ctx.strokeStyle = "#2a2a28";
  ctx.beginPath(); ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R); ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy); ctx.stroke();
  ctx.fillStyle = "#c3c2b7"; ctx.textAlign = "center";
  ctx.fillText("forward", cx, cy - R - 11);
  ctx.fillStyle = "#8d8c84";
  ctx.textAlign = "right"; ctx.fillText("L", cx - R - 6, cy); ctx.textAlign = "left"; ctx.fillText("R", cx + R + 6, cy);
  // robot (real size, at least ~24 px wide): axle geo.fwd_mm behind the servo axis
  const kk = Math.max(k, 24 / 152), L = 90 * kk, Wd = 152 * kk, tr = 126.8 * kk, wd = 66 * kk, ww = Math.max(3, 25 * kk);
  const ay = cy + geo.fwd_mm * kk;
  ctx.fillStyle = "rgba(240,239,236,0.14)"; ctx.strokeStyle = "#f0efec"; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.rect(cx - (tr - ww) / 2, ay - L / 2, tr - ww, L); ctx.fill(); ctx.stroke();
  ctx.fillStyle = "#f0efec";
  ctx.fillRect(cx - Wd / 2, ay - wd / 2, ww, wd); ctx.fillRect(cx + Wd / 2 - ww, ay - wd / 2, ww, wd);
  ctx.fillStyle = "#3987e5";   // forward arrow (map heading colour)
  const a = Math.max(8, L * 0.45), ty = ay - L / 2;
  ctx.beginPath(); ctx.moveTo(cx, ty - a); ctx.lineTo(cx - a * 0.6, ty); ctx.lineTo(cx + a * 0.6, ty); ctx.closePath(); ctx.fill();
  // points of the last sweep, fading with age; beyond the scale -> on the edge ring
  const keep = keepMs(), hs = 3.2;
  for (let i = 0; i < 2; i++) {
    const [r, g, b] = SENS_RGB[i];
    for (const [key, p] of S[i].dirs) {
      const age = tEnd - p.t;
      if (age > keep * 2.5) { if (age > KEEP_MS) S[i].dirs.delete(key); continue; }
      if (age < -50 || age > keep) continue;
      const al = 0.35 + 0.65 * (1 - age / keep);
      if (!p.ok) {   // no target in that direction: tick on the edge
        const [x1, y1] = XY(p.a, scale), [x2, y2] = XY(p.a, scale * 0.94);
        ctx.strokeStyle = `rgba(${r},${g},${b},${al * 0.5})`; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
        continue;
      }
      const d = p.mm + geo.radius_mm, over = d > scale;
      const [x, y] = XY(p.a, Math.min(d, scale));
      ctx.fillStyle = `rgba(${r},${g},${b},${al})`;
      if (over) { ctx.strokeStyle = ctx.fillStyle; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, hs, 0, 7); ctx.stroke(); }
      else { ctx.beginPath(); ctx.arc(x, y, hs, 0, 7); ctx.fill(); }
    }
  }
  // current head direction: a line per sensor + the newest reading
  for (let i = 0; i < 2; i++) {
    const p = S[i].last;
    if (!p || tEnd - p.t > STALE_MS) continue;
    const [ex, ey] = XY(p.a, scale);
    ctx.strokeStyle = SENS[i]; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke(); ctx.setLineDash([]);
    if (p.ok) {
      const [x, y] = XY(p.a, Math.min(p.mm + geo.radius_mm, scale));
      ctx.fillStyle = SENS_NEW[i]; ctx.strokeStyle = "#1a1a19"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, 6, 0, 7); ctx.fill(); ctx.stroke();
    }
  }
}

// range vs time, one strip per sensor: dots (valid), ticks along the top (no target)
const strips = [0, 1].map((i) => {
  const c = document.getElementById("sens-t" + i), o = { c, hoverX: null };
  c.addEventListener("pointermove", (e) => { o.hoverX = e.offsetX; dirty = true; });
  c.addEventListener("pointerleave", () => { o.hoverX = null; dirty = true; });
  return o;
});
function drawStrip(i, tEnd) {
  const o = strips[i], f = fit(o.c); if (!f) return;
  const { ctx, W, H } = f, hist = S[i].hist;
  const padL = 44, padR = 10, padT = 30, padB = 18, pw = W - padL - padR, ph = H - padT - padB, t0 = tEnd - WIN_MS;
  const X = (t) => padL + ((t - t0) / WIN_MS) * pw, Y = (mm) => padT + (1 - Math.min(mm, scale) / scale) * ph;
  ctx.font = "11px system-ui, sans-serif"; ctx.lineWidth = 1;
  ctx.textAlign = "right"; ctx.textBaseline = "middle"; ctx.fillStyle = "#8d8c84";
  const step = scale / 2;
  for (let v = 0; v <= scale + 1e-6; v += step) {
    const y = Math.round(Y(v)) + 0.5;
    ctx.strokeStyle = v === 0 ? "#45453f" : "#2a2a28";
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
    ctx.fillText(fmtMm(v).replace(" mm", ""), padL - 6, y);
  }
  ctx.textBaseline = "top";
  for (let s = 0; s <= 10; s += 2) {
    ctx.textAlign = s === 0 ? "left" : s === 10 ? "right" : "center";
    ctx.fillText(s === 10 ? "now" : `-${10 - s}s`, padL + (s / 10) * pw, H - padB + 4);
  }
  // title (text ink; the swatch carries the colour)
  ctx.textAlign = "left"; ctx.textBaseline = "middle";
  ctx.fillStyle = SENS[i]; ctx.fillRect(padL, 7, 12, 4);
  ctx.fillStyle = "#f0efec"; ctx.font = "600 12px system-ui, sans-serif";
  ctx.fillText(`${NAMES[i]} (sensor ${i})`, padL + 18, 9);
  let lo = 0, hi = hist.length;   // first reading in the window
  while (lo < hi) { const m = (lo + hi) >> 1; if (hist[m].t < t0) lo = m + 1; else hi = m; }
  ctx.save(); ctx.beginPath(); ctx.rect(padL - 4, padT - 4, pw + 8, ph + 8); ctx.clip();
  const [r, g, b] = SENS_RGB[i];
  ctx.strokeStyle = `rgba(${r},${g},${b},0.45)`; ctx.lineWidth = 1.5; ctx.beginPath();
  let prev = null;
  for (let j = lo; j < hist.length; j++) {
    const p = hist[j]; if (p.t > tEnd) break;
    if (!p.ok) { prev = null; continue; }
    const x = X(p.t), y = Y(p.mm);
    if (prev && p.t - prev.t < 400) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    prev = p;
  }
  ctx.stroke();
  ctx.fillStyle = SENS[i];
  const bad = [];
  for (let j = lo; j < hist.length; j++) {
    const p = hist[j]; if (p.t > tEnd) break;
    if (!p.ok) { bad.push(p); continue; }
    ctx.beginPath(); ctx.arc(X(p.t), Y(p.mm), 2.2, 0, 7); ctx.fill();
  }
  ctx.fillStyle = "#6a6a64";
  for (const p of bad) ctx.fillRect(X(p.t) - 0.5, padT - 9, 1.5, 5);   // no target: ticks above the plot
  ctx.restore();
  // hover: the reading under the pointer
  if (o.hoverX != null && o.hoverX >= padL && lo < hist.length) {
    const th = t0 + ((o.hoverX - padL) / pw) * WIN_MS;
    let best = null;
    for (let j = lo; j < hist.length && hist[j].t <= tEnd; j++) if (!best || Math.abs(hist[j].t - th) < Math.abs(best.t - th)) best = hist[j];
    if (best) {
      const x = Math.round(X(best.t)) + 0.5;
      ctx.strokeStyle = "#8d8c84"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      const txt = `${best.ok ? best.mm + " mm" : "no target"}  ${fmtA(best.a)}  ${((best.t - tEnd) / 1000).toFixed(1)} s`;
      ctx.font = "12px system-ui, sans-serif"; ctx.textAlign = "right"; ctx.fillStyle = "#f0efec";
      ctx.fillText(txt, W - padR, 9);
    }
  }
}

function frame() {
  requestAnimationFrame(frame);
  if (!shown || !dirty) return;
  dirty = false;
  updateCards();
  const tEnd = paused ? pausedEnd : piNow() || Math.max(S[0].last?.t || 0, S[1].last?.t || 0);
  drawPolar(tEnd);
  drawStrip(0, tEnd); drawStrip(1, tEnd);
}
requestAnimationFrame(frame);
setInterval(() => { dirty = true; }, 250);   // keep scrolling / fading when the robot is silent
window.addEventListener("resize", () => (dirty = true));
on("params", syncAll);
on("status", () => { dirty = true; });
on("conn", syncAll);

buildMenu();
syncAll();

export function sensorsShown(v) { shown = v; if (v) { syncAll(); dirty = true; } }
