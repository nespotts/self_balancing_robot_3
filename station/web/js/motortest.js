// Tune page → Motor test (issue #16): wheels-up motor test (mot.test 1-4), live status from the robot's
// "mt,..." lines (the bridge follows them: motor_state), and the fit (station/bridge/motor_fit.py):
// dead zone, speed vs volts chart, proposal with "Apply proposal".
import { state, on, send, flag } from "./conn.js";
import { el, fmt, toast } from "./ui.js";
import { S1, S2 } from "./chart.js";
import { onSub, tuneSub, menu } from "./tunetools.js";

const root = document.getElementById("motor-view");
const param = (name) => state.params.find((p) => p.name === name);
const TESTS = [[4, "All", "≈ 80 s"], [1, "Dead zone", "≈ 45 s"], [2, "Speed curve", "≈ 40 s"], [3, "Reversal", "≈ 15 s"]];
const WAIT = {
  estop: "E-stop is on",
  arm_the_remote_in_IDLE: "Arm the remote, mode IDLE",
  mode_not_IDLE: "Switch the mode to IDLE",
  no_remote: "Remote not connected (the test needs it)",
  imu: "IMU not ready", encoders: "Encoder fault",
  not_upright: "Stand it upright (pitch and roll within 20°)",
  not_still: "Hold it still", wheels_turning: "Wheels still turning",
};
const ABORT = {
  cancelled: "cancelled", estop: "E-stop", disarmed: "disarmed", mode_change: "mode changed", remote_lost: "remote lost",
  imu: "IMU fault", encoder: "encoder fault", encoder_stale: "encoder stale", tilt: "tilted over 20°",
  pitch_rate: "body moved (pitch rate)", timeout: "time limit", not_started_in_60s: "not started within 60 s",
};
const PART = { a: "Dead zone", b: "Speed curve", c: "Reversal" };
const KEYS = ["left_fwd", "left_rev", "right_fwd", "right_rev"];
const SER = { left_fwd: [S1, false, "Left fwd"], left_rev: [S1, true, "Left rev"], right_fwd: [S2, false, "Right fwd"], right_rev: [S2, true, "Right rev"] };

// ---------- run card ----------
let pick = 4, ms = { state: "idle" }, fit = null, asked = false;
const seg = el("div", { class: "seg mt-seg" });
for (const [n, label] of TESTS) seg.append(el("button", { "data-v": n, class: n === pick ? "on" : "" }, label));
seg.onclick = (e) => {
  const b = e.target.closest("button");
  if (!b) return;
  pick = +b.dataset.v;
  for (const x of seg.children) x.classList.toggle("on", x === b);
  dur.textContent = TESTS.find((t) => t[0] === pick)[2];
};
const dur = el("span", { class: "muted mt-dur" }, TESTS[0][2]);
const startBtn = el("button", { class: "primary" }, "Start test");
const cancelBtn = el("button", { class: "warn", hidden: true }, "Cancel");
const stTitle = el("b", {}, ""), stText = el("span", { class: "muted" }, "");
const bar = el("div", { class: "mt-bar" }, el("i"));
const status = el("div", { class: "mt-status", hidden: true }, el("div", { class: "mt-st" }, stTitle, stText), bar);
const noFw = el("p", { class: "muted", hidden: true }, "This firmware has no motor test (mot.test).");
const runCard = el("div", { class: "card mt-card" },
  el("h3", {}, "Wheels-up motor test"),
  el("p", { class: "mt-how" }, "Robot on a stand, ", el("b", {}, "both wheels off the ground"), ". Remote: mode IDLE, ARMED."),
  el("div", { class: "mt-row" }, seg, dur),
  el("div", { class: "mt-row" }, startBtn, cancelBtn), status, noFw);

startBtn.onclick = () => {
  const t = TESTS.find((x) => x[0] === pick);
  if (!confirm(`Start the motor test: ${t[1]} (${t[2]})?\n\nBoth wheels must be OFF the ground: they spin up to full speed.\nIt starts when the robot is still in IDLE with the remote armed; STOP / disarm aborts.`)) return;
  send({ t: "motor", action: "start", test: pick, confirm: true }) || toast("Not connected");
};
cancelBtn.onclick = () => send({ t: "motor", action: "cancel" });

function fmtS(s) { return s == null ? "" : Math.floor(s / 60) + ":" + String(Math.floor(s % 60)).padStart(2, "0"); }
let msAt = 0;
function drawState() {
  const has = !!param("mot.test");
  noFw.hidden = has || !state.params.length;
  startBtn.disabled = !has || ms.state === "waiting" || ms.state === "running";
  const pending = ms.state === "waiting" || ms.state === "running" || (param("mot.test")?.value || 0) >= 0.5;
  cancelBtn.hidden = !pending;
  status.hidden = ms.state === "idle" && !ms.result;
  const t = TESTS.find((x) => x[0] === ms.test);
  const name = t ? t[1] : "Test";
  const el_ = ms.now != null ? ms.now + (performance.now() - msAt) / 1000 : null;
  status.className = "mt-status " + ms.state;
  bar.firstChild.style.width = Math.round(100 * (ms.progress || 0)) + "%";
  bar.hidden = ms.state !== "running" && ms.state !== "done";
  if (ms.state === "waiting") {
    stTitle.textContent = "Waiting: ";
    stText.textContent = ms.wait ? WAIT[ms.wait] || ms.wait : "checking…";
  } else if (ms.state === "running") {
    const part = PART[ms.part] || "starting";
    stTitle.textContent = (ms.test === 4 ? `All · ${part}` : part) + " ";
    stText.textContent = `${Math.round(100 * ms.progress)}% · ${fmtS(el_ != null && ms.t_begin != null ? el_ - ms.t_begin : null)}`;
  } else if (ms.state === "done") {
    stTitle.textContent = `${name}: done `;
    stText.textContent = ms.t_begin != null && ms.t_end != null ? `in ${Math.round(ms.t_end - ms.t_begin)} s` : "";
  } else if (ms.state === "aborted") {
    const why = (ms.result || "").split(",")[0];
    stTitle.textContent = `${name}: stopped `;
    stText.textContent = "(" + (ABORT[why] || why) + "): motors off";
  } else if (ms.result === "cancelled") {
    stTitle.textContent = "Cancelled"; stText.textContent = "";
  }
}
on("motor_state", (m) => {
  const was = ms.state;
  ms = m; msAt = performance.now();
  drawState();
  if (m.state === "done" && was === "running") requestFit();
  if (m.state === "aborted" && was === "running") requestFit();
});
setInterval(() => { if (ms.state === "running" && tuneSub === "motor") drawState(); }, 500);
on("params", drawState);

// ---------- results card ----------
const resBody = el("div", { class: "mt-res" });
const srcEl = el("span", { class: "muted mt-src" }, "");
const moreBtn = el("button", { id: "mt-more", class: "ghost small", title: "Results from" }, "⋯");
const mMenu = el("div", { class: "menu", id: "mt-menu", hidden: true });
const resCard = el("div", { class: "card mt-card" },
  el("div", { class: "mt-head" }, el("h3", {}, "Results"), srcEl, moreBtn, mMenu), resBody);
root.append(runCard, resCard);
menu("mt-more", "mt-menu");
moreBtn.addEventListener("click", async () => {
  mMenu.innerHTML = "";
  mMenu.append(el("button", { onclick: () => { mMenu.hidden = true; requestFit(); } }, "This session (live)"));
  mMenu.append(el("h4", {}, "Earlier sessions"));
  try {
    const d = await (await fetch("/api/logs")).json();
    for (const s of d.sessions.filter((s) => !s.active).slice(0, 12)) {
      const m = /sbr3-(\d{4})(\d\d)(\d\d)-(\d\d)(\d\d)/.exec(s.name);
      mMenu.append(el("button", { onclick: () => { mMenu.hidden = true; requestFit(s.name); } },
        m ? `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}` : s.name));
    }
  } catch { mMenu.append(el("p", { class: "muted" }, "Could not list sessions")); }
});

let fitSession = null;
function requestFit(session) {
  fitSession = session || null;
  srcEl.textContent = "fitting…";
  send({ t: "motor", action: "fit", session: fitSession });
}
on("motor_fit", (m) => { if ((m.session || null) !== fitSession) return; fit = m; drawFit(); });

const v2 = (v) => (v == null ? "—" : v.toFixed(2));
const sessionName = (s) => { const m = /sbr3-(\d{4})(\d\d)(\d\d)-(\d\d)(\d\d)/.exec(s || ""); return m ? `${m[2]}-${m[3]} ${m[4]}:${m[5]}` : s || ""; };

function drawFit() {
  resBody.innerHTML = "";
  if (!fit) return;
  srcEl.textContent = fit.session ? "session " + sessionName(fit.session) : "this session";
  if (fit.error) { resBody.append(el("p", { class: "muted" }, fit.error)); return; }
  const R = fit.results[fit.results.length - 1];
  if (!R) {
    resBody.append(el("p", { class: "muted" }, fit.runs.length ? "No results yet." : "No motor test in this session yet. ⋯ shows earlier sessions."));
    return;
  }
  const run = fit.runs[fit.runs.length - 1];
  if (fit.results.length > 1) resBody.append(el("p", { class: "muted small" }, `${fit.results.length} drive configs in this session; showing the newest (${R.cfg_name}). tools/motor_fit.py compares them.`));
  if (run && run.status === "running") resBody.append(el("p", { class: "muted small" }, "A test is running; this shows the newest finished results."));
  else if (run && run.status !== "ok") resBody.append(el("p", { class: "muted small" }, "Newest run: " + run.status));
  // dead zone
  if (R.a) {
    const t = el("table", { class: "mt-table" }, el("thead", {}, el("tr", {}, el("th", {}, "Dead zone"), el("th", {}, "starts"), el("th", {}, "stops"))));
    const tb = el("tbody");
    for (const r of R.a.rows) tb.append(el("tr", {}, el("td", {}, `${r.wheel} ${r.dir}`),
      el("td", {}, r.start_v == null ? "never" : v2(r.start_v) + " V"), el("td", {}, r.stop_v == null ? "—" : v2(r.stop_v) + " V")));
    t.append(tb);
    resBody.append(el("div", { class: "mt-sec" }, t));
  }
  if (R.b) resBody.append(el("div", { class: "mt-sec" }, curveChart(R)));
  // facts
  const facts = [];
  if (R.b?.lr) facts.push(`Left wheel ${R.b.lr.left_pct > 0 ? "+" : ""}${R.b.lr.left_pct.toFixed(0)}% vs right (${R.b.lr.left.toFixed(0)} / ${R.b.lr.right.toFixed(0)} mm/s per V)`);
  if (R.b?.tau_ms && Object.keys(R.b.tau_ms).length) facts.push(`Time constant L/R ${["left", "right"].map((w) => R.b.tau_ms[w] == null ? "—" : Math.round(R.b.tau_ms[w])).join(" / ")} ms`);
  if (R.c?.rows?.length) {
    const dw = (w) => { const xs = R.c.rows.filter((r) => r.wheel === w && r.dwell_ms != null).map((r) => r.dwell_ms); return xs.length ? Math.round(xs.reduce((a, b) => a + b) / xs.length) : "—"; };
    facts.push(`Reversal free play L/R ~${dw("left")} / ${dw("right")} ms`);
  }
  if (facts.length) resBody.append(el("ul", { class: "mt-facts" }, ...facts.map((f) => el("li", {}, f))));
  if (R.proposal) resBody.append(proposal(R));
  for (const w of R.warnings || []) resBody.append(el("p", { class: "mt-warn" }, "⚠ " + w));
}

// ---------- proposal ----------
const LABEL = { "mot.model": "use the volts model", "mot.vcomp": "battery compensation", "mot.v_ref": "u = 1 → volts",
  "mot.k_l": "mm/s per V", "mot.k_r": "mm/s per V",
};
function proposal(R) {
  const P = R.proposal, box = el("div", { class: "mt-sec mt-prop" }, el("h4", {}, "Proposal"));
  const t = el("table", { class: "mt-table" }, el("thead", {}, el("tr", {}, el("th", {}, ""), el("th", {}, ""), el("th", {}, "now"), el("th", {}, "new"))));
  const tb = el("tbody"), boxes = {};
  const order = ["mot.dz_l_fwd", "mot.dz_l_rev", "mot.dz_r_fwd", "mot.dz_r_rev", "mot.left_scale", "mot.right_scale", "mot.v_ref",
    "mot.k_l", "mot.k_r", "mot.model", "mot.vcomp"];
  for (const k of order) {
    if (!(k in P.params)) continue;
    const cur = param(k), opt = k === "mot.model" || k === "mot.vcomp";
    const cb = el("input", { type: "checkbox" });
    cb.checked = !opt;
    boxes[k] = cb;
    const same = cur && Math.abs(cur.value - P.params[k]) < 1e-6;
    tb.append(el("tr", { class: (opt ? "opt" : "") + (same ? " same" : "") }, el("td", {}, cb),
      el("td", {}, el("label", { title: k }, k.slice(4), LABEL[k] ? el("small", {}, " " + LABEL[k]) : null)),
      el("td", {}, cur ? fmt(cur.value) : "—"), el("td", {}, fmt(P.params[k]))));
    cb.closest("tr").querySelector("label").onclick = () => { cb.checked = !cb.checked; };
  }
  t.append(tb);
  box.append(t);
  if (P.gain_ratio) box.append(el("p", { class: "muted small" },
    `The volts model gives ~${(1 / P.gain_ratio).toFixed(2)}× the v3 curve's motor gain near balance: start with bal.kp / ki / kd × ${P.gain_ratio.toFixed(2)}, then retune.`));
  box.append(el("p", { class: "muted small" }, "Dead zones and v_ref act only with mot.model 1; the scales act in both models. Save to robot keeps it."));
  const btn = el("button", { class: "primary" }, "Apply proposal");
  btn.onclick = () => {
    const params = {};
    for (const k in boxes) if (boxes[k].checked) params[k] = P.params[k];
    if (!Object.keys(params).length) return toast("Nothing ticked");
    const list = Object.entries(params).map(([k, v]) => `${k} → ${fmt(v)}`).join("\n");
    const armedNow = flag("armed");
    if (armedNow && !confirm(`The robot is ARMED. Apply?\n\n${list}`)) return;
    send({ t: "motor", action: "apply", params, confirm: armedNow }) || toast("Not connected");
  };
  box.append(btn);
  return box;
}
on("motor_applied", (m) => {
  toast(m.changes.length ? `Applied ${m.changes.length} motor param${m.changes.length === 1 ? "" : "s"}` : "No changes", 3000);
  if (m.changes.length) window.dispatchEvent(new Event("sbr3-unsaved"));
  drawFit();
});

let chartDraw = null;
window.addEventListener("resize", () => chartDraw && requestAnimationFrame(chartDraw));

// ---------- speed vs volts chart (canvas; wheel = colour, direction = solid / dashed) ----------
function curveChart(R) {
  const pts = R.b.points, fits = R.b.fits;
  const canvas = el("canvas", { class: "mt-chart" });
  const tip = el("div", { class: "mt-tip", hidden: true });
  const legend = el("div", { class: "mt-legend" }, ...KEYS.filter((k) => pts[k]).map((k) => el("span", {},
    el("i", { class: SER[k][1] ? "dash" : "", style: `--c:${SER[k][0]}` }), SER[k][2])));
  const wrap = el("div", { class: "mt-chart-wrap" }, el("div", { class: "mt-ctitle" }, "Wheel speed vs motor volts"), legend, canvas, tip);
  let hover = null;
  const draw = () => {
    const dpr = window.devicePixelRatio || 1, W = canvas.clientWidth, H = canvas.clientHeight;
    if (!W) return;
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    const c = canvas.getContext("2d");
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, W, H);
    const all = KEYS.flatMap((k) => pts[k] || []);
    const xMax = Math.ceil(Math.max(12, ...all.map((p) => p[0]))), yMax = Math.max(200, ...all.map((p) => p[1])) * 1.08;
    const yStep = yMax > 1200 ? 500 : yMax > 600 ? 200 : 100;
    const L = 46, Rr = 54, T = 10, B = 26, pw = W - L - Rr, ph = H - T - B;
    const X = (v) => L + (v / xMax) * pw, Y = (s) => T + ph - (s / yMax) * ph;
    // dead zone band (lowest stop .. highest start)
    const a = R.a?.rows || [];
    const lo = Math.min(...a.map((r) => r.stop_v).filter((v) => v != null)), hi = Math.max(...a.map((r) => r.start_v).filter((v) => v != null));
    if (isFinite(lo) && isFinite(hi) && hi > lo) {
      c.fillStyle = "rgba(201,133,0,.14)";
      c.fillRect(X(lo), T, X(hi) - X(lo), ph);
      c.fillStyle = "#c3c2b7"; c.font = "11px system-ui, sans-serif"; c.textAlign = "center";
      c.fillText("dead zone", (X(lo) + X(hi)) / 2, T + 12);
    }
    c.font = "11px system-ui, sans-serif"; c.fillStyle = "#8d8c84"; c.lineWidth = 1;
    c.textAlign = "right"; c.textBaseline = "middle";
    for (let s = 0; s <= yMax; s += yStep) {
      const y = Math.round(Y(s)) + 0.5;
      c.strokeStyle = s ? "#2a2a28" : "#45453f";
      c.beginPath(); c.moveTo(L, y); c.lineTo(L + pw, y); c.stroke();
      c.fillText(String(s), L - 6, y);
    }
    c.textAlign = "center"; c.textBaseline = "top";
    for (let v = 0; v <= xMax; v += 2) c.fillText(v + " V", X(v), T + ph + 6);
    c.textAlign = "left"; c.textBaseline = "middle"; c.fillText("mm/s", 4, T + 4);
    for (const k of KEYS) {
      const p = (pts[k] || []).slice().sort((x, y) => x[0] - y[0]);
      if (!p.length) continue;
      const [col, dash] = SER[k];
      c.strokeStyle = col; c.lineWidth = 2; c.setLineDash(dash ? [6, 4] : []);
      c.beginPath(); p.forEach(([v, s], i) => (i ? c.lineTo(X(v), Y(s)) : c.moveTo(X(v), Y(s)))); c.stroke();
      c.setLineDash([]);
      for (const [v, s] of p) {
        c.beginPath(); c.arc(X(v), Y(s), 3, 0, 7);
        c.fillStyle = dash ? "#1a1a19" : col; c.fill();
        c.strokeStyle = col; c.lineWidth = 1.5; c.stroke();
      }
    }
    // direct labels per wheel at the end of its forward line
    const ends = ["left", "right"].map((w) => { const p = (pts[w + "_fwd"] || []).slice().sort((x, y) => x[0] - y[0]).pop(); return p && [w, p]; }).filter(Boolean);
    ends.sort((x, y) => Y(x[1][1]) - Y(y[1][1]));
    let lastY = -1e9;
    c.fillStyle = "#c3c2b7"; c.font = "12px system-ui, sans-serif";
    for (const [w, p] of ends) {
      const y = Math.max(Y(p[1]), lastY + 14);
      c.fillText(w === "left" ? "Left" : "Right", X(p[0]) + 8, y);
      lastY = y;
    }
    if (hover) {
      c.fillStyle = SER[hover.k][0]; c.strokeStyle = "#1a1a19"; c.lineWidth = 2;
      c.beginPath(); c.arc(X(hover.p[0]), Y(hover.p[1]), 5, 0, 7); c.fill(); c.stroke();
    }
    canvas._geo = { X, Y };
  };
  canvas.addEventListener("pointermove", (e) => {
    const g = canvas._geo;
    if (!g) return;
    let best = null, bd = 400;
    for (const k of KEYS) for (const p of pts[k] || []) {
      const d = (g.X(p[0]) - e.offsetX) ** 2 + (g.Y(p[1]) - e.offsetY) ** 2;
      if (d < bd) { bd = d; best = { k, p }; }
    }
    hover = best;
    tip.hidden = !best;
    if (best) {
      const f = fits[best.k];
      tip.textContent = `${SER[best.k][2]}: ${best.p[0].toFixed(2)} V → ${Math.round(best.p[1])} mm/s` + (f ? ` (fit ${f.k.toFixed(0)} mm/s/V from ${f.v0.toFixed(2)} V)` : "");
      tip.style.left = Math.min(e.offsetX + 12, canvas.clientWidth - 220) + "px";
      tip.style.top = Math.max(0, e.offsetY - 34) + "px";
    }
    draw();
  });
  canvas.addEventListener("pointerleave", () => { hover = null; tip.hidden = true; draw(); });
  chartDraw = draw;
  requestAnimationFrame(draw);
  return wrap;
}

// ask for this session's fit when the Motor test view is first shown
function shown() {
  if (chartDraw) requestAnimationFrame(chartDraw);
  if (tuneSub === "motor" && state.connected && !asked) { asked = true; requestFit(); }
  drawState();
}
onSub(shown);
on("conn", (up) => { if (!up) asked = false; else shown(); });
on("hello", () => { asked = false; setTimeout(shown, 200); });
