// Tuning: controls generated from the robot's ParamInfoFrames (names/ranges come from the firmware).
import { state, on, send } from "./conn.js";
import { el, fmt } from "./ui.js";

const groupsEl = document.getElementById("tune-groups"), info = document.getElementById("tune-info");
const rows = {};   // id -> {slider, num, revert, row, p}
let builtKey = "";

function steps(p) { return (p.max - p.min) / 1000 || 0.001; }

function apply(id, v, r) {
  send({ t: "set_param", id, value: v });
  r.row.classList.add("sent");
  clearTimeout(r.flash);
  r.flash = setTimeout(() => r.row.classList.remove("sent"), 400);
}

function build() {
  const list = state.params;
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
    const card = el("div", { class: "card" }, el("h3", {}, g));
    for (const p of ps) {
      const label = p.name.includes(".") ? p.name.slice(p.name.indexOf(".") + 1) : p.name;
      const num = el("input", { type: "number", step: "any", min: p.min, max: p.max, inputmode: "decimal" });
      const slider = el("input", { type: "range", min: p.min, max: p.max, step: steps(p) });
      const revert = el("button", { class: "revert", title: "Back to the value at page load" }, "↺");
      const row = el("div", { class: "prow" }, el("span", { class: "pname", title: p.name }, label), revert, num, slider,
        el("div", { class: "range" }, el("span", {}, fmt(p.min)), el("span", {}, fmt(p.max))));
      const r = { slider, num, revert, row, last: 0, timer: 0 };
      rows[p.id] = r;
      slider.addEventListener("input", () => {
        const v = +slider.value;
        num.value = +v.toPrecision(5);
        // live apply, throttled to 10 Hz while dragging
        const now = performance.now();
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
      card.append(row);
    }
    groupsEl.append(card);
  }
  update();
}

function update() {
  for (const p of state.params) {
    const r = rows[p.id];
    if (!r) continue;
    if (document.activeElement !== r.num) r.num.value = +(+p.value).toPrecision(6);
    if (document.activeElement !== r.slider) r.slider.value = p.value;
    r.revert.hidden = state.paramsFirst[p.id] === undefined || Math.abs(state.paramsFirst[p.id] - p.value) < 1e-9;
  }
  const fw = state.robot?.firmware;
  info.textContent = state.params.length
    ? `${state.params.length} parameters${fw ? " · " + fw : ""} · changes apply live`
    : state.connected ? "Waiting for parameters from the robot…" : "Not connected";
}

document.getElementById("tune-reload").onclick = () => send({ t: "get_params" });
on("params", build);
on("robot", update);
on("conn", update);
build();
