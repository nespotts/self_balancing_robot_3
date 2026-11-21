// Tune page tools (issue #16): Balance / Motor test switch, presets (saved on the Pi), scored runs.
// The bridge does the work (station/bridge/tuning_api.py); this is the UI.
import { state, on, send, flag } from "./conn.js";
import { el, fmt, toast } from "./ui.js";

const view = document.getElementById("view-tune");
const param = (name) => state.params.find((p) => p.name === name);
const armed = () => flag("armed");
const store = (k, v) => { try { localStorage.setItem(k, v); } catch {} };
const load = (k, d) => { try { return localStorage.getItem(k) ?? d; } catch { return d; } };

// ---------- Balance / Motor test ----------
const subSeg = document.getElementById("tune-sub");
export let tuneSub = load("sbr3.tuneSub", "balance");
const subListeners = [];
export function onSub(fn) { subListeners.push(fn); }
function setSub(v) {
  tuneSub = v === "motor" ? "motor" : "balance";
  view.dataset.sub = tuneSub;
  for (const b of subSeg.children) b.classList.toggle("on", b.dataset.v === tuneSub);
  store("sbr3.tuneSub", tuneSub);
  for (const f of subListeners) f(tuneSub);
}
subSeg.onclick = (e) => { const b = e.target.closest("button"); if (b) setSub(b.dataset.v); };
setSub(tuneSub);

on("error", (m) => { if (m.text && !m.need_confirm && !m.code) toast(m.text, 3500); });   // (code: main.js says it)
on("ok", (m) => { if (m.text) toast(m.text); });

// small ⋯ menus: one open at a time, closed by a tap elsewhere
export function menu(btnId, menuId) {
  const b = document.getElementById(btnId), m = document.getElementById(menuId);
  b.onclick = (e) => { e.stopPropagation(); m.hidden = !m.hidden; };
  document.addEventListener("pointerdown", (e) => { if (!m.hidden && !m.contains(e.target) && !b.contains(e.target)) m.hidden = true; });
  return m;
}

// ---------- presets ----------
const sel = document.getElementById("preset-sel"), applyBtn = document.getElementById("preset-apply");
const pMenu = menu("preset-more", "preset-menu");
const cmp = document.getElementById("preset-cmp"), cmpA = document.getElementById("cmp-a"), cmpB = document.getElementById("cmp-b");
const cmpTable = document.getElementById("cmp-table"), cmpAll = document.getElementById("cmp-all");
let presets = [], chosen = load("sbr3.preset", "");
const LIVE = "";   // compare: the robot's live values

function liveVals() {
  const o = {};
  for (const p of state.params) o[p.name] = p.value;
  return o;
}
function matches(p) {
  const live = liveVals();
  return state.params.length > 0 && Object.entries(p.values).every(([k, v]) => !(k in live) || Math.abs(live[k] - v) <= 1e-5 * Math.max(1, Math.abs(v)));
}
function fillPresets() {
  const cur = presets.find(matches);
  sel.innerHTML = "";
  if (!presets.find((p) => p.name === chosen)) chosen = cur ? cur.name : "";
  sel.append(el("option", { value: "", disabled: true }, presets.length ? "Preset…" : "No presets"));
  for (const group of [["Built-in", true], ["Saved", false]]) {
    const ps = presets.filter((p) => !!p.builtin === group[1]);
    if (!ps.length) continue;
    const og = el("optgroup", { label: group[0] });
    for (const p of ps) og.append(el("option", { value: p.name }, p.name + (cur && p.name === cur.name ? "  ✓ current" : "")));
    sel.append(og);
  }
  sel.value = chosen;
  const p = presets.find((x) => x.name === chosen);
  applyBtn.disabled = !p || matches(p);
  applyBtn.textContent = p && matches(p) ? "Active" : "Apply";
  document.getElementById("preset-delete").disabled = !p || p.builtin;
  for (const s of [cmpA, cmpB]) {
    const v = s.value;
    s.innerHTML = "";
    s.append(el("option", { value: LIVE }, "Robot now"));
    for (const x of presets) s.append(el("option", { value: x.name }, x.name));
    s.value = [...s.options].some((o) => o.value === v) ? v : LIVE;
  }
}
sel.onchange = () => { chosen = sel.value; store("sbr3.preset", chosen); fillPresets(); if (!cmp.hidden) compare(); };
on("presets", (m) => { presets = m.list || []; fillPresets(); });
let refill = 0;
on("params", () => { clearTimeout(refill); refill = setTimeout(fillPresets, 300); });

applyBtn.onclick = () => {
  if (!chosen) return;
  if (!send({ t: "preset", action: "plan", name: chosen })) toast("Not connected");
};
on("preset_plan", (m) => {
  if (!m.changes.length) return toast("Already at '" + m.name + "'");
  const list = m.changes.map((c) => `${c.name}  ${fmt(c.from)} → ${fmt(c.to)}`).join("\n");
  if (m.armed || armed()) {
    if (!confirm(`The robot is ARMED. Apply '${m.name}'?\n\n${list}`)) return;
  }
  send({ t: "preset", action: "apply", name: m.name, confirm: true });
});
on("preset_applied", (m) => {
  const n = m.changes.length;
  toast(`'${m.name}': ${n} change${n === 1 ? "" : "s"}` + (n && n <= 3 ? " (" + m.changes.map((c) => c.name.split(".")[1] + " " + fmt(c.to)).join(", ") + ")" : ""), 3500);
  if (n) window.dispatchEvent(new Event("sbr3-unsaved"));
});

document.getElementById("preset-save").onclick = () => {
  pMenu.hidden = true;
  const g = (n) => fmt(param(n)?.value);
  const d = new Date(), pad = (x) => String(x).padStart(2, "0");
  const dflt = `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} kp ${g("bal.kp")} ki ${g("bal.ki")} kd ${g("bal.kd")}`;
  const name = prompt("Save the robot's current tuning values (bal / vel / str / drv / mot) as:", dflt);
  if (!name) return;
  chosen = name.trim(); store("sbr3.preset", chosen);
  send({ t: "preset", action: "save", name: chosen });
};
document.getElementById("preset-delete").onclick = () => {
  pMenu.hidden = true;
  if (chosen && confirm(`Delete the preset '${chosen}'?`)) send({ t: "preset", action: "delete", name: chosen });
};
document.getElementById("preset-compare").onclick = () => {
  pMenu.hidden = true;
  cmp.hidden = false;
  cmpA.value = LIVE;
  cmpB.value = chosen || (presets[0] && presets[0].name) || LIVE;
  compare();
};
document.getElementById("cmp-close").onclick = () => { cmp.hidden = true; };
cmpA.onchange = cmpB.onchange = cmpAll.onchange = () => compare();
function compare() { send({ t: "preset", action: "diff", a: cmpA.value, b: cmpB.value }); }
on("preset_diff", (m) => {
  if (cmp.hidden) return;
  const both = (r) => r.a != null && r.b != null;
  const rows = m.rows.filter((r) => cmpAll.checked || (!r.same && both(r)));
  const oneSide = m.rows.filter((r) => !both(r)).length;
  const name = (n) => (n === LIVE ? "Robot now" : n);
  cmpTable.innerHTML = "";
  if (!rows.length) cmpTable.append(el("p", { class: "muted" }, "Same values."));
  if (oneSide && !cmpAll.checked) cmpTable.append(el("p", { class: "muted cmp-note" }, `${oneSide} value${oneSide === 1 ? "" : "s"} only on one side (not in that preset): hidden.`));
  if (!rows.length) return;
  const t = el("table", {}, el("thead", {}, el("tr", {}, el("th", {}, ""), el("th", {}, name(m.a)), el("th", {}, name(m.b)))));
  const tb = el("tbody");
  for (const r of rows) tb.append(el("tr", { class: r.same ? "same" : "" }, el("td", {}, r.name),
    el("td", {}, r.a == null ? "—" : fmt(r.a)), el("td", {}, r.b == null ? "—" : fmt(r.b))));
  t.append(tb);
  cmpTable.prepend(t);
});

// ---------- scored runs ----------
const runBtn = document.getElementById("score-run"), lastBtn = document.getElementById("score-last");
const sStatus = document.getElementById("score-status"), sHint = document.getElementById("score-hint");
const tilesEl = document.getElementById("score-tiles"), histEl = document.getElementById("score-hist");
const sMenu = menu("score-more", "score-menu"), winSeg = document.getElementById("score-win");
let sstate = { running: false }, sAt = 0, runs = [], metrics = [], win = +load("sbr3.scoreWin", 20);
const TILES = ["err_rms", "osc_hz", "motor_rms", "settle_mean", "over_mean", "push_n"];
const KEY_PARAMS = ["bal.kp", "bal.ki", "bal.kd", "bal.d_src"];
const SHOW = new Set(["osc_hz", "push_n"]);   // unranked metrics shown by default
let allRows = false;

function setWin(v) {
  win = v; store("sbr3.scoreWin", v);
  for (const b of winSeg.children) b.classList.toggle("on", +b.dataset.v === v);
  lastBtn.textContent = `Last ${v} s`;
}
setWin(win);
winSeg.onclick = (e) => { const b = e.target.closest("button"); if (b) { setWin(+b.dataset.v); sMenu.hidden = true; } };
document.getElementById("score-clear").onclick = () => { sMenu.hidden = true; if (confirm("Clear the run history?")) send({ t: "score", action: "clear" }); };
runBtn.onclick = () => send({ t: "score", action: sstate.running ? "stop" : "start" }) || toast("Not connected");
lastBtn.onclick = () => send({ t: "score", action: "last", seconds: win }) || toast("Not connected");

on("score_state", (m) => { sstate = m; sAt = performance.now(); updateScoreBar(); });
on("score_runs", (m) => { runs = m.runs || []; if (m.metrics) metrics = m.metrics; drawRuns(); });
function updateScoreBar() {
  const r = sstate.running;
  runBtn.textContent = r ? "Stop" : "Start";
  runBtn.classList.toggle("recording", r);
  lastBtn.disabled = runBtn.disabled = !!sstate.busy;
  if (sstate.busy) sStatus.textContent = "scoring…";
  else if (r && sstate.now != null) sStatus.textContent = "● " + Math.round(sstate.now - sstate.t0 + (performance.now() - sAt) / 1000) + " s";
  else sStatus.textContent = "";
  const hz = param("tlm.hz"), rate = param("ctl.rate_hz");
  sHint.textContent = hz && rate && hz.value < rate.value ? `Telemetry is ${hz.value} Hz: turn on Full rate (chart below) for real scores.`
    : !runs.length ? "Let it balance, then Last 20 s. Or Start, push it a few times, Stop." : "";
}
setInterval(() => { if (sstate.running) updateScoreBar(); }, 500);
on("params", updateScoreBar);

const mdef = (k) => metrics.find((m) => m[0] === k) || [k, k, "", null];
function mval(k, v) {
  if (v == null) return "—";
  if (k === "push_n") return String(v);
  if (k === "motor_rms") return v.toFixed(3);
  if (k === "settle_mean") return v.toFixed(2);
  return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);
}
function drawRuns() {
  updateScoreBar();
  tilesEl.innerHTML = ""; histEl.innerHTML = "";
  const last = runs[runs.length - 1];
  if (!last) return;
  const s = last.score;
  if (!s.ok) tilesEl.append(el("p", { class: "muted" }, `Run ${last.id}: ${s.reason} (${s.up} of ${s.rows} rows balancing)`));
  else for (const k of TILES) {
    const [, label, unit] = mdef(k);
    let v = mval(k, s[k]);
    if (k === "settle_mean" && s.unsettled) v += ` (+${s.unsettled} >2 s)`;
    tilesEl.append(el("div", { class: "tile" }, el("b", {}, v), el("span", {}, label + (unit ? " " + unit : ""))));
  }
  if (runs.length < 2) return;
  // side by side: one column per run (newest right); params that differ between runs on top, best values marked
  const names = new Set(KEY_PARAMS);
  for (const r of runs) for (const k in r.params) if (runs.some((q) => q.params[k] !== undefined && Math.abs(q.params[k] - r.params[k]) > 1e-9)) names.add(k);
  const t = el("table", {});
  const hr = el("tr", {}, el("th", {}, ""));
  for (const r of runs) hr.append(el("th", { title: `${r.label}, session ${r.session}` }, `#${r.id}`, el("small", {}, r.time.slice(0, 5) + " · " + r.seconds + " s")));
  t.append(el("thead", {}, hr));
  const tb = el("tbody");
  for (const n of names) {
    if (!runs.some((r) => r.params[n] !== undefined)) continue;
    const tr = el("tr", { class: "prm" }, el("td", {}, n));
    runs.forEach((r, i) => {
      const v = r.params[n], prev = i ? runs[i - 1].params[n] : v;
      const changed = r.changed?.filter((c) => c.name === n) || [];
      tr.append(el("td", { class: v !== undefined && prev !== undefined && Math.abs(v - prev) > 1e-9 ? "chg" : "",
        title: changed.length ? "changed during the run: " + changed.map((c) => `${fmt(c.from)}→${fmt(c.to)} at ${c.t}s`).join(", ") : "" },
        v === undefined ? "—" : fmt(v) + (changed.length ? " *" : "")));
    });
    tb.append(tr);
  }
  for (const [k, label, unit, better] of metrics) {
    if (!runs.some((r) => r.score.ok && r.score[k] != null)) continue;
    const vals = runs.map((r) => (r.score.ok ? r.score[k] : null));
    const ok = vals.filter((v) => v != null);
    if (!better && !SHOW.has(k) && !allRows) continue;         // P / I / D rms, mean error, Hz: behind "all metrics"
    const best = better === "low" && ok.length > 1 && Math.min(...ok) < Math.max(...ok) ? Math.min(...ok) : null;
    const tr = el("tr", {}, el("td", {}, label + (unit ? ` (${unit})` : "")));
    vals.forEach((v) => tr.append(el("td", { class: best != null && v === best ? "best" : "" }, mval(k, v))));
    tb.append(tr);
  }
  t.append(tb);
  const wrap = el("div", { class: "hist-wrap" }, t);
  const more = el("button", { class: "ghost small hist-more", onclick: () => { allRows = !allRows; drawRuns(); } }, allRows ? "Fewer metrics" : "All metrics");
  histEl.append(el("div", { class: "hist-cap muted" }, "Runs side by side · ", el("span", { class: "best-key" }, "best"), " · * = changed during the run"), wrap, more);
  requestAnimationFrame(() => { wrap.scrollLeft = wrap.scrollWidth; });
}
on("conn", (up) => { if (!up) { sstate = { running: false }; updateScoreBar(); } });
