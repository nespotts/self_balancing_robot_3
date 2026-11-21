// Find balance point (issue #52): measures the lean the velocity loop needs to hold the robot still and
// proposes the bal.offset_deg that cancels it. The bridge does the work (station/bridge/balance_point.py);
// this card is Start -> progress -> "3.15 -> 2.40" Apply -> Save to robot.
import { state, on, send, flag } from "./conn.js";
import { el, toast } from "./ui.js";

const card = document.getElementById("bp-card"), body = document.getElementById("bp-body");
const go = document.getElementById("bp-go"), status = document.getElementById("bp-status");
let bp = { state: "idle" }, at = 0, notified = null, lastWhy = "";

const deg = (v) => (v == null ? "-" : v.toFixed(2) + "°");
const lean = (v) => (v >= 0 ? "+" : "−") + Math.abs(v).toFixed(2) + "°";
const RUNNING = ["settle", "measure", "slew"];

// Why Start would be refused right now (the bridge checks again): "" = ready
function notReady() {
  if (!state.connected || !state.tlm) return "Not connected";
  const names = state.hello?.modes || [];
  const mode = names[state.tlm.mode];
  if (!flag("armed")) return "Arm and balance the robot first";
  if (mode !== "BALANCE" && mode !== "HOLD") return "Needs BALANCE or HOLD mode";
  if (!flag("upright")) return "The robot is not upright";
  if (Math.abs(state.tlm.speed_mms) > 80) return "Moving: let it stand still";
  return "";
}

function actions(...btns) { return el("div", { class: "bp-actions" }, ...btns); }
function chain() {
  const h = bp.history || [];
  if (!h.length) return null;
  if (h.length < 2) return null;
  const parts = h.map((x) => lean(x.mean));
  return el("p", { class: "muted bp-chain", title: "Lean the velocity loop needed, per round" }, "Lean  " + parts.join("  →  "));
}
function warns() { return (bp.warn || []).map((w) => el("p", { class: "bp-warn" }, w)); }

function render() {
  const s = bp.state, running = RUNNING.includes(s);
  const changed = bp.orig != null && bp.offset != null && Math.abs(bp.offset - bp.orig) > 0.004;
  body.innerHTML = "";
  status.textContent = "";
  go.hidden = false;
  go.classList.remove("recording");
  if (s === "proposal") {
    go.hidden = true;                          // Apply / Cancel are in the card
  } else if (running) {
    go.textContent = "Stop";
    go.classList.add("recording");
    go.classList.remove("primary");
    go.disabled = false;
  } else {
    go.textContent = s === "idle" ? "Start" : "Again";
    go.classList.add("primary");
    const why = lastWhy = notReady();
    go.disabled = !!why;
    if (why) body.append(el("p", { class: "muted bp-hint" }, why));
  }
  if (s === "idle") {
    if (!body.firstChild) body.append(el("p", { class: "muted bp-hint" },
      "Robot balancing on the floor, hands off. About 16 s a round."));
    if (bp.text) body.prepend(el("p", { class: "muted" }, bp.text));
    return;
  }
  if (s === "settle" || s === "measure") {
    const big = el("div", { class: "bp-big" }, el("b", {}, bp.live != null ? lean(bp.live) : "…"), el("span", {}, "lean"));
    const bar = el("i", { id: "bp-bar" });
    body.append(big, el("div", { class: "bp-prog" }, bar),
      el("p", { class: "muted" }, `Round ${bp.round} of ${bp.rounds} · ${s === "settle" ? "settling" : "measuring"}`));
    status.textContent = bp.auto ? "automatic" : "";
    tickBar();
  } else if (s === "slew") {
    body.append(el("p", {}, bp.text));
  } else if (s === "proposal" || (s === "done" && bp.proposal != null)) {
    body.append(el("div", { class: "bp-big" }, el("b", {}, `${deg(bp.offset)} → ${deg(bp.proposal)}`)),
      el("p", { class: "muted" }, s === "done" ? bp.text : `The robot leans ${lean(bp.last ? bp.last.mean : 0)} to stand still.`));
    if (s === "proposal" || s === "done") body.append(actions(
      el("button", { class: "primary", onclick: () => send({ t: "balpoint", action: "apply" }) || toast("Not connected") }, "Apply"),
      s === "proposal" ? el("button", { onclick: () => send({ t: "balpoint", action: "stop" }) }, "Cancel") : null,
      s === "done" && changed ? el("button", { class: "ghost", onclick: undo }, "Undo") : null));
  } else if (s === "done") {
    body.append(el("div", { class: "bp-big" }, el("b", {}, deg(bp.offset)),
      changed ? el("span", {}, `was ${deg(bp.orig)}`) : el("span", {}, "is right")),
      el("p", { class: "muted" }, changed ? "Applied. Save it to keep it after a reboot." : "Nothing to change."));
    if (changed) body.append(actions(
      el("button", { class: "primary", onclick: () => document.getElementById("tune-save").click() }, "Save to robot"),
      el("button", { class: "ghost", onclick: undo }, "Undo")),
      el("p", { class: "muted bp-hint" }, "Saving needs the motors off (disarm)."));
  } else if (s === "aborted") {
    body.append(el("p", { class: "bp-bad" }, bp.text));
    if (changed) body.append(actions(el("button", { onclick: undo }, `Undo (back to ${deg(bp.orig)})`)));
  }
  const c = chain();
  if (c) body.append(c);
  for (const w of warns()) body.append(w);
}
function undo() { send({ t: "balpoint", action: "revert" }) || toast("Not connected"); }

function tickBar() {
  const bar = document.getElementById("bp-bar");
  if (!bar || bp.length == null) return;
  const f = Math.min(1, ((bp.elapsed || 0) + (performance.now() - at) / 1000) / bp.length);
  bar.style.width = (f * 100).toFixed(1) + "%";
  bar.parentNode.classList.toggle("settle", bp.state === "settle");
}
setInterval(() => { if (bp.state === "settle" || bp.state === "measure") tickBar(); }, 250);

go.onclick = () => {
  const s = bp.state;
  if (RUNNING.includes(s) || s === "proposal") return void send({ t: "balpoint", action: "stop" });
  if (!send({ t: "balpoint", action: "start" })) toast("Not connected");
};

on("balpoint_state", (m) => {
  bp = m;
  at = performance.now();
  if (m.orig != null && m.offset != null && Math.abs(m.offset - m.orig) > 0.004 && notified !== m.offset) {
    notified = m.offset;
    window.dispatchEvent(new Event("sbr3-unsaved"));      // same as a preset apply: Save to robot lights up
  }
  render();
});
on("tlm", () => { if (!RUNNING.includes(bp.state) && bp.state !== "proposal") renderIdleSoon(); });
let soon = 0;
function renderIdleSoon() {      // armed / upright / mode changes enable Start (tlm comes at 25 Hz: cheap re-check)
  if (soon) return;
  soon = setTimeout(() => { soon = 0; if (notReady() !== lastWhy) render(); }, 400);
}
on("conn", (up) => { if (!up) { bp = { state: "idle" }; } render(); });
render();
