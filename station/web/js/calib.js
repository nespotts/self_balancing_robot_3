// Calibrate odometry (issue #14): wheel diameter, left/right ratio, track width, gyro drift.
// The bridge does the work (station/bridge/odo_calib.py); this card is a short guided list:
// pick a step -> Start (the robot must be armed and standing; each run needs your tap) -> measure with the
// tape and type it in -> the proposed values -> Apply -> Save to robot.
import { state, on, send, flag } from "./conn.js";
import { el, fmt, toast } from "./ui.js";
import { menu } from "./tunetools.js";

const card = document.getElementById("cal-card"), body = document.getElementById("cal-body");
const status = document.getElementById("cal-status");
const resetBtn = document.getElementById("cal-reset");
menu("cal-more", "cal-menu");

const STEPS = [
  { id: "straight", name: "Straight run", unit: "cm", def: 200, lo: 50, hi: 400, key: "length_mm" },
  { id: "cw", name: "Square, clockwise", unit: "cm", def: 100, lo: 50, hi: 200, key: "length_mm" },
  { id: "ccw", name: "Square, counter-clockwise", unit: "cm", def: 100, lo: 50, hi: 200, key: "length_mm" },
  { id: "gyro", name: "Gyro drift at rest", unit: "s", def: 60, lo: 20, hi: 300, key: "secs" },
];
const RUNNING = ["running", "returning"];
let cal = null, at = 0, open = null, lastWhy = "", lastState = "", lastTest = "";
const form = {};   // typed values survive re-renders: form["straight.len"], form["cw.x"], ...

const cm = (mm) => (mm / 10).toFixed(1);
const num = (id) => { const v = parseFloat(form[id]); return isFinite(v) ? v : null; };

// Why Start would be refused right now (the bridge checks again): "" = ready
function notReady(step) {
  if (!state.connected || !state.tlm) return "Not connected";
  if (!cal) return "Waiting for the robot";
  if (cal.params && cal.params["odo.wheel_d_mm"] == null) return "Flash the new firmware (no odo.* params yet)";
  const mode = (state.hello?.modes || [])[state.tlm.mode];
  if (!flag("armed")) return "Arm the robot first (two taps)";
  const ok = step === "gyro" ? ["BALANCE", "HOLD"] : ["BALANCE", "HOLD", "STATION"];
  if (!ok.includes(mode)) return "Needs BALANCE or HOLD mode";
  if (!flag("upright")) return "The robot is not upright";
  if (Math.abs(state.tlm.speed_mms) > 80) return "Moving: let it stand still";
  return "";
}

function squareSvg(ccw) {
  // start mark bottom-left, robot arrow, path, "measure here" at the end
  const s = (t) => fig(`<svg viewBox="0 0 120 96" class="cal-svg" aria-hidden="true">${t}</svg>`);
  const path = ccw ? "M24 76 H92 V16 H24 V76" : "M24 76 V16 H92 V76 H24";
  const arrow = ccw ? "M60 76 l-7 -5 v10z" : "M24 46 l-5 7 h10z";
  return s(`<rect x="24" y="16" width="68" height="60" rx="2" fill="none" stroke="var(--line)" stroke-dasharray="3 3"/>` +
    `<path d="${path}" fill="none" stroke="var(--blue)" stroke-width="2.5"/>` +
    `<path d="${arrow}" fill="var(--blue)"/>` +
    `<circle cx="24" cy="76" r="5" fill="var(--warn)"/>` +
    `<text x="30" y="90" font-size="9" fill="var(--text-2)">start mark</text>` +
    `<text x="96" y="14" font-size="9" fill="var(--muted)">1 m</text>`);
}
function fig(html) { const d = el("div", { class: "cal-fig" }); d.innerHTML = html; return d; }
function lineSvg() {
  return fig(`<svg viewBox="0 0 120 40" class="cal-svg" aria-hidden="true"><path d="M14 22 H104" stroke="var(--blue)" stroke-width="2.5"/>` +
    `<path d="M104 22 l-7 -5 v10z" fill="var(--blue)"/><circle cx="14" cy="22" r="5" fill="var(--warn)"/>` +
    `<path d="M14 34 H104" stroke="var(--text-2)" stroke-width="1" marker-end="none"/>` +
    `<text x="40" y="12" font-size="9" fill="var(--text-2)">tape: mark to wheel centre</text></svg>`);
}

function field(id, label, unit, dflt) {
  const inp = el("input", { type: "number", inputmode: "decimal", step: "any", value: form[id] ?? dflt ?? "", "data-id": id });
  inp.oninput = () => { form[id] = inp.value; };
  return el("label", { class: "cal-field" }, el("span", {}, label), inp, el("em", {}, unit));
}

function stepChip(step) {
  const t = cal?.tests?.[step.id], r = t?.result;
  if (cal && RUNNING.includes(cal.state) && cal.test === step.id) return ["running", "running"];
  if (!r) return ["", "not run"];
  if (step.id === "gyro") return ["ok", `${r.drift_dpm >= 0 ? "+" : "−"}${Math.abs(r.drift_dpm).toFixed(2)}°/min`];
  if (t.measured) return ["ok", "measured"];
  return ["todo", "measure it"];
}

function runView(step) {
  const r = cal.run || {};
  const frac = step.id === "gyro" ? (r.elapsed || 0) / (r.target || 60)
    : (r.dist_mm || 0) / ((r.length_mm || 1000) * (step.id === "straight" ? 1 : 4));
  const pct = Math.max(0, Math.min(1, frac)) * 100;
  const out = [el("div", { class: "bp-prog" }, el("i", { style: `width:${pct.toFixed(1)}%` })),
    el("p", { class: "muted" }, step.id === "gyro"
      ? `${Math.round(r.elapsed || 0)} s of ${Math.round(r.target || 60)} s` + (r.live_drift_dpm != null ? ` · drift so far ${r.live_drift_dpm}°/min` : "")
      : `${cm(r.dist_mm || 0)} cm driven · turned ${r.turn_deg}°`),
    el("p", { class: "muted" }, cal.text)];
  out.push(el("div", { class: "bp-actions" }, el("button", { class: "stopbtn", onclick: () => send({ t: "calib", action: "stop" }) }, "Stop")));
  return out;
}

function measureView(step) {
  const t = cal.tests[step.id], r = t.result, m = r.measured;
  const out = [];
  if (step.id === "straight") {
    out.push(el("p", {}, `Robot says ${cm(r.reported_mm)} cm. Measure the real distance.`));
    out.push(field("straight.fwd", "Tape", "cm", m ? cm(m.forward) : ""));
    out.push(field("straight.side", "Sideways (left +)", "cm", m ? cm(m.side) : "0"));
  } else {
    const [ex, ey] = r.end_mm;
    out.push(el("p", {}, `Robot thinks it ended ${cm(ex)} cm ahead, ${cm(ey)} cm left of the mark. Where is it really?`));
    out.push(el("p", { class: "muted" }, "Measure from the start mark to the same spot on the robot: ahead of where it first faced, and to its left."));
    out.push(field(step.id + ".x", "Ahead (− = behind)", "cm", m ? cm(m.x) : ""));
    out.push(field(step.id + ".y", "Left (− = right)", "cm", m ? cm(m.y) : ""));
  }
  out.push(el("div", { class: "bp-actions" }, el("button", { class: "primary", onclick: () => saveMeasure(step) }, m ? "Update" : "Save")));
  return out;
}

function saveMeasure(step) {
  let m;
  if (step.id === "straight") {
    const f = num("straight.fwd");
    if (f == null) return toast("Type the distance first");
    m = { t: "calib", action: "measured", test: "straight", forward_cm: f, side_cm: num("straight.side") ?? 0 };
  } else {
    const x = num(step.id + ".x"), y = num(step.id + ".y");
    if (x == null || y == null) return toast("Type both numbers (0 if it is on the mark)");
    m = { t: "calib", action: "measured", test: step.id, x_cm: x, y_cm: y };
  }
  if (!send(m)) toast("Not connected");
}

function stepPanel(step) {
  const p = el("div", { class: "cal-panel" });
  const running = RUNNING.includes(cal.state) && cal.test === step.id;
  if (running) { p.append(...runView(step)); return p; }
  const t = cal.tests[step.id], r = t.result;
  if (step.id === "gyro") {
    p.append(el("p", { class: "muted" }, "Robot balancing, hands off, nothing moving nearby."));
    if (r) {
      p.append(el("div", { class: "bp-big" }, el("b", {}, `${r.drift_dpm >= 0 ? "+" : "−"}${Math.abs(r.drift_dpm).toFixed(2)}°/min`), el("span", {}, r.verdict)));
      p.append(el("p", { class: "muted" }, `over ${r.secs} s. Under 1°/min is fine for mapping; the scan matcher fixes the rest.`));
    }
  } else {
    p.append(step.id === "straight" ? lineSvg() : squareSvg(step.id === "ccw"));
    p.append(el("p", { class: "muted" }, step.id === "straight"
      ? "Put a tape mark under the wheel centres. The robot drives straight ahead, slowly, then stops."
      : "Put a mark under the wheel centres. The robot drives the square and comes back to the mark. It stops facing along the last side."));
  }
  const sid = step.id + ".len";
  p.append(field(sid, step.id === "gyro" ? "Time" : "Side", step.unit, step.def));
  const why = lastWhy = notReady(step.id);
  const go = el("button", { class: "primary", onclick: () => start(step) }, r ? "Run again" : "Start");
  go.disabled = !!why;
  go.dataset.cal = "go";
  p.append(el("div", { class: "bp-actions" }, go));
  p.append(el("p", { class: "muted bp-hint", id: "cal-why" }, why));
  if (cal.state === "aborted" && cal.test === step.id) p.append(el("p", { class: "bp-bad" }, cal.text));
  if (r && step.id !== "gyro") p.append(el("hr", { class: "cal-hr" }), ...measureView(step));
  return p;
}

function start(step) {
  const v = num(step.id + ".len") ?? step.def;
  const m = { t: "calib", action: "start", test: step.id };
  if (step.id === "gyro") m.secs = v; else m.length_mm = v * 10;
  if (!send(m)) toast("Not connected");
}

function proposalView() {
  const pr = cal.proposal;
  if (!pr) return null;
  const box = el("div", { class: "cal-prop" });
  const LABEL = { "odo.wheel_d_mm": ["Wheel diameter", "mm"], "odo.track_mm": ["Track width", "mm"], "odo.lr_ratio": ["Right / left wheel", ""] };
  for (const r of pr.rows) {
    const [name, unit] = LABEL[r.name] || [r.name, ""];
    const same = Math.abs(r.to - r.from) < 1e-6;
    box.append(el("div", { class: "cal-row" + (same ? " same" : "") }, el("b", {}, name),
      el("span", { class: "bp-big" }, el("b", {}, `${fmt(r.from)} → ${fmt(r.to)}`), el("span", {}, unit)),
      el("small", { class: "muted" }, r.why)));
  }
  for (const n of pr.notes || []) box.append(el("p", { class: "muted bp-hint" }, n));
  const cl = pr.closing_cm || {};
  if (cl.cw != null || cl.ccw != null) {
    box.append(el("p", { class: "muted bp-hint" }, "Square ends off the mark by " +
      [cl.cw != null ? `${cl.cw} cm (cw)` : null, cl.ccw != null ? `${cl.ccw} cm (ccw)` : null].filter(Boolean).join(", ")));
  }
  const um = pr.umbmark;
  if (um) {
    box.append(el("details", { class: "cal-det" }, el("summary", {}, "Square test numbers"),
      el("p", { class: "muted" }, `UMBmark: turn error ${um.alpha_deg.toFixed(2)}° per corner, curvature ${um.beta_deg.toFixed(2)}°, ` +
        `E_b ${um.E_b.toFixed(4)}, E_d ${um.E_d.toFixed(4)}. (Textbook values for a robot that steers by its wheels; this one steers by the gyro, so the values above come from the wheels against the gyro.)`)));
  }
  const rows = pr.rows.filter((r) => Math.abs(r.to - r.from) > 1e-6);
  const acts = [];
  if (rows.length) acts.push(el("button", { class: "primary", onclick: () => send({ t: "calib", action: "apply" }) || toast("Not connected") }, "Apply"));
  if (cal.changed) {
    acts.push(el("button", { class: "primary", onclick: () => document.getElementById("tune-save").click() }, "Save to robot"));
    acts.push(el("button", { class: "ghost", onclick: () => send({ t: "calib", action: "revert" }) }, "Undo"));
  }
  if (acts.length) box.append(el("div", { class: "bp-actions" }, ...acts));
  if (cal.changed) box.append(el("p", { class: "muted bp-hint" }, "Saving needs the motors off (disarm). Run the square again to check."));
  return box;
}

function render() {
  body.innerHTML = "";
  const busy = cal && RUNNING.includes(cal.state);
  status.textContent = busy ? "running" : "";
  card.classList.toggle("busy", !!busy);
  if (!cal) { body.append(el("p", { class: "muted bp-hint" }, "Waiting for the robot")); return; }
  if (busy) open = cal.test;
  for (const step of STEPS) {
    const [cls, chip] = stepChip(step);
    const row = el("button", { class: "cal-step" + (open === step.id ? " on" : ""), onclick: () => { open = open === step.id ? null : step.id; render(); } },
      el("span", {}, step.name), el("em", { class: cls }, chip));
    body.append(row);
    if (open === step.id) body.append(stepPanel(step));
  }
  const pv = proposalView();
  if (pv) body.append(el("h4", { class: "cal-h" }, "Proposed values"), pv);
  else if (cal.params) body.append(el("p", { class: "muted bp-hint" },
    `Now: wheel ${fmt(cal.params["odo.wheel_d_mm"])} mm, track ${fmt(cal.params["odo.track_mm"])} mm, right/left ${fmt(cal.params["odo.lr_ratio"])}`));
  for (const w of cal.warn || []) body.append(el("p", { class: "bp-warn" }, w));
}

on("calib_state", (m) => {
  const stateChanged = !cal || m.state !== lastState || m.test !== lastTest || m.changed !== cal.changed ||
    JSON.stringify(m.tests) !== JSON.stringify(cal.tests) || JSON.stringify(m.proposal) !== JSON.stringify(cal.proposal);
  cal = m; at = performance.now(); lastState = m.state; lastTest = m.test;
  if (m.changed && m.applied) window.dispatchEvent(new Event("sbr3-unsaved"));   // Save to robot lights up
  if (stateChanged || RUNNING.includes(m.state)) render();
});
// Start's readiness follows the robot (tlm is 25 Hz: re-check a few times a second, in place, no re-render)
setInterval(() => {
  const go = body.querySelector("[data-cal=go]");
  if (!go || !open) return;
  const why = notReady(open);
  if (why !== lastWhy) {
    lastWhy = why; go.disabled = !!why;
    const w = document.getElementById("cal-why"); if (w) w.textContent = why;
  }
}, 400);
resetBtn.onclick = () => { document.getElementById("cal-menu").hidden = true; if (confirm("Forget the runs of this session?")) send({ t: "calib", action: "reset" }); };
on("conn", (up) => { if (!up) { cal = null; } render(); });
render();
