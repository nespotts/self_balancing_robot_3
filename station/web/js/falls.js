// Falls list (Logs view): every fall the bridge recorded, with the likely cause, a mini plot of pitch / target /
// motors around it, and (tap) the full record: evidence, bigger plots, params then, download. Issue #50.
import { on } from "./conn.js";
import { el, toast } from "./ui.js";
import { S1, S2, S3 } from "./chart.js";

const listEl = document.getElementById("falls");
const infoEl = document.getElementById("falls-info");
let tagHelp = {};
const open = new Set();          // ids shown expanded (survive a refresh)
const cache = {};                // id -> full record

const KIND = { tip: "tipped over", cut: "motors cut", link: "link lost" };
const KEY_PARAMS = ["bal.kp", "bal.ki", "bal.kd", "bal.offset_deg", "bal.cutoff_deg", "vel.kp", "vel.ki", "vel.kd", "mot.max"];

function when(iso) {
  const m = /(\d{4})-(\d\d)-(\d\d)T(\d\d:\d\d:\d\d)/.exec(iso || "");
  if (!m) return iso || "?";
  const d = new Date(+m[1], +m[2] - 1, +m[3]), now = new Date();
  const day = d.toDateString() === now.toDateString() ? "Today"
    : d.toLocaleDateString(undefined, { month: "short", day: "numeric" });
  return `${day} ${m[4]}`;
}

// ---- drawing: stacked panes sharing one time axis (t = 0 is the fall) ----
// panes: [{ lo, hi, series: [{ v: [...], color, dash }], label }]
export function drawPanes(canvas, t, panes, opts = {}) {
  const dpr = window.devicePixelRatio || 1, W = canvas.clientWidth, H = canvas.clientHeight;
  if (!W || !H) return;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const big = !!opts.big;
  const padL = big ? 44 : 4, padR = 6, padT = 4, padB = big ? 18 : 2, gap = 4;
  const t0 = t[0], t1 = t[t.length - 1];
  const X = (s) => padL + ((s - t0) / (t1 - t0)) * (W - padL - padR);
  const total = panes.reduce((a, p) => a + (p.weight || 1), 0);
  const avail = H - padT - padB - gap * (panes.length - 1);
  let y = padT;
  ctx.font = "11px system-ui, sans-serif";
  for (const p of panes) {
    const h = avail * (p.weight || 1) / total;
    let lo = p.lo, hi = p.hi;
    if (lo == null) {
      lo = Infinity; hi = -Infinity;
      for (const s of p.series) for (const v of s.v) { if (v < lo) lo = v; if (v > hi) hi = v; }
      const span = Math.max(hi - lo, p.minSpan || 1) * 1.1, mid = (lo + hi) / 2;
      lo = mid - span / 2; hi = mid + span / 2;
    }
    const Y = (v) => y + (1 - (v - lo) / (hi - lo)) * h;
    ctx.fillStyle = "rgba(255,255,255,.035)";
    ctx.fillRect(padL, y, W - padL - padR, h);
    if (lo < 0 && hi > 0) {                                    // zero line
      ctx.strokeStyle = "#45453f"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(padL, Math.round(Y(0)) + 0.5); ctx.lineTo(W - padR, Math.round(Y(0)) + 0.5); ctx.stroke();
    }
    ctx.save();
    ctx.beginPath(); ctx.rect(padL, y, W - padL - padR, h); ctx.clip();
    ctx.lineJoin = "round"; ctx.lineWidth = big ? 1.8 : 1.5;
    for (const s of p.series) {
      ctx.strokeStyle = s.color; ctx.setLineDash(s.dash || []);
      ctx.beginPath();
      s.v.forEach((v, i) => (i ? ctx.lineTo(X(t[i]), Y(v)) : ctx.moveTo(X(t[i]), Y(v))));
      ctx.stroke();
    }
    ctx.restore(); ctx.setLineDash([]);
    if (big) {
      ctx.fillStyle = "#8d8c84"; ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.fillText(fmtAxis(hi, hi - lo), padL - 5, y + 6); ctx.fillText(fmtAxis(lo, hi - lo), padL - 5, y + h - 6);
      if (p.label) { ctx.textAlign = "left"; ctx.textBaseline = "top"; ctx.fillText(p.label, padL + 6, y + 3); }
    }
    y += h + gap;
  }
  // the fall: a vertical line at t = 0
  if (t0 < 0 && t1 > 0) {
    const x = Math.round(X(0)) + 0.5;
    ctx.strokeStyle = "#e0413f"; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, H - padB); ctx.stroke(); ctx.setLineDash([]);
  }
  if (big) {
    ctx.fillStyle = "#8d8c84"; ctx.textBaseline = "top"; ctx.textAlign = "center";
    for (let s = Math.ceil(t0); s <= t1; s++) {
      ctx.textAlign = s >= t1 - 0.3 ? "right" : "center";
      ctx.fillText(s === 0 ? "fall" : (s > 0 ? "+" : "") + s + "s", X(s), H - padB + 4);
    }
  }
}
function fmtAxis(v, span) {
  const d = span < 2 ? 2 : Math.abs(v) >= 10 || span >= 20 ? 0 : 1;
  return v.toFixed(d);
}

function legend(items) {
  return el("div", { class: "flegend" }, ...items.map(([label, color, dash]) =>
    el("span", {}, el("i", { style: `background:${color}${dash ? ";opacity:.6" : ""}` }), label)));
}

function sparkPanes(sp) {
  return [
    { weight: 3, minSpan: 20, series: [{ v: sp.target, color: S2, dash: [4, 3] }, { v: sp.pitch, color: S1 }] },
    { weight: 2, lo: -1, hi: 1, series: [{ v: sp.ml, color: S1 }, { v: sp.mr, color: S3 }] },
  ];
}

function chip(tag, cause) {
  const help = tagHelp[tag.tag]?.help || "";
  return el("span", { class: "fchip" + (cause ? " cause" : ""), title: help }, tag.label);
}

function row(f) {
  const isOpen = open.has(f.id);
  const canvas = el("canvas", { class: "fspark" });
  const head = el("div", { class: "fhead" },
    el("div", { class: "fwhen" }, when(f.time),
      el("small", {}, [KIND[f.trigger?.kind] || "fell", f.mode, f.battery_v ? f.battery_v + " V" : null].filter(Boolean).join(" · "))),
    el("div", { class: "fchips" }, ...f.tags.map((t, i) => chip(t, i === 0))));
  const body = el("div", { class: "fbody", hidden: !isOpen });
  const r = el("div", { class: "frow" + (isOpen ? " open" : "") }, head, canvas, body);
  head.onclick = canvas.onclick = () => {
    const now = body.hidden;
    body.hidden = !now;
    r.classList.toggle("open", now);
    now ? open.add(f.id) : open.delete(f.id);
    if (now) fill(f, body);
  };
  if (f.spark?.t?.length) {
    const draw = () => drawPanes(canvas, f.spark.t, sparkPanes(f.spark));
    requestAnimationFrame(draw);
    if (window.ResizeObserver) new ResizeObserver(draw).observe(canvas);   // (phone rotated, window resized)
  }
  if (isOpen) fill(f, body);
  return r;
}

async function fill(f, body) {
  if (body.dataset.done) { redraw(f, body); return; }
  body.dataset.done = "1";
  body.append(el("p", { class: "muted" }, "Loading…"));
  try {
    const rec = cache[f.id] || (cache[f.id] = await (await fetch("/api/falls/" + encodeURIComponent(f.id))).json());
    body.innerHTML = "";
    const why = el("ul", { class: "fwhy" }, ...f.tags.map((t) => el("li", {},
      el("b", {}, t.label), " " + t.detail)));
    const cv = el("canvas", { class: "fbig" });
    const plots = el("div", { class: "fplots" }, legend([["pitch", S1], ["target", S2, 1], ["motor L", S1], ["motor R", S3]]), cv);
    const events = rec.events.filter((e) => e[0] !== "bridge");
    const ev = events.length ? el("div", { class: "fevents" }, ...events.map((e) => el("div", {},
      el("span", { class: "t" }, (e[1] > 0 ? "+" : "") + e[1].toFixed(2) + "s"), e[0] === "cmd" ? "cmd " + JSON.stringify(e[2]) : e[2]))) : null;
    const pv = f.n_params ? paramList(rec.meta.params) : null;
    const actions = el("div", { class: "factions" },
      el("a", { href: "/api/falls/" + encodeURIComponent(f.id) + "?raw=1", download: "" }, "Download record"),
      el("button", { class: "ghost small warn", onclick: async () => {
        if (!confirm("Delete this fall record?")) return;
        await fetch("/api/falls/" + encodeURIComponent(f.id), { method: "DELETE" });
        open.delete(f.id); load();
      } }, "Delete"));
    body.append(why, plots, ev, pv, actions);
    redraw(f, body);
    if (window.ResizeObserver) new ResizeObserver(() => redraw(f, body)).observe(cv);
  } catch (e) {
    body.textContent = "Could not load the record";
    delete body.dataset.done;
  }
}

function redraw(f, body) {
  const rec = cache[f.id], cv = body.querySelector(".fbig");
  if (!rec || !cv) return;
  const c = rec.cols, rows = rec.rows, col = (name) => rows.map((r) => r[c.indexOf(name)]);
  const t = col("t");
  drawPanes(cv, t, [
    { weight: 3, minSpan: 20, label: "pitch °", series: [{ v: col("balance_target_deg"), color: S2, dash: [4, 3] }, { v: col("pitch_deg"), color: S1 }] },
    { weight: 2, lo: -1, hi: 1, label: "motors", series: [{ v: col("motor_left"), color: S1 }, { v: col("motor_right"), color: S3 }] },
    { weight: 2, minSpan: 200, label: "wheels mm/s", series: [{ v: col("wheel_left_mms"), color: S1 }, { v: col("wheel_right_mms"), color: S3 }] },
    { weight: 1.2, minSpan: 0.5, label: "battery V", series: [{ v: col("battery_v"), color: S1 }] },
  ], { big: true });
}

function paramList(params) {
  const keys = KEY_PARAMS.filter((k) => k in params);
  const all = Object.keys(params).sort();
  const dl = (names) => el("dl", { class: "kv fparams" }, ...names.flatMap((k) => [el("dt", {}, k), el("dd", {}, String(params[k]))]));
  return el("div", { class: "fparam-wrap" },
    el("h4", {}, "Params at the time"), dl(keys),
    all.length > keys.length ? el("details", {}, el("summary", {}, "All " + all.length), dl(all)) : null);
}

export async function loadFalls() {
  try {
    const d = await (await fetch("/api/falls")).json();
    tagHelp = d.tags || {};
    listEl.innerHTML = "";
    const st = d.stats || {};
    if (!d.falls.length) {
      listEl.append(el("p", { class: "muted" }, "No falls recorded. When the robot tips over or its motors cut out while balancing, " +
        "the seconds around it are saved here with the likely cause."));
    }
    for (const f of d.falls) listEl.append(row(f));
    infoEl.textContent = st.ignored ? st.ignored + " stop(s) from E-stop / disarm ignored" : "";
  } catch (e) {
    listEl.textContent = "Could not load falls";
  }
}
const load = loadFalls;

on("fall", (m) => {
  toast("Fall recorded: " + (m.fall?.cause_label || "?"), 4000);
  if (document.body.dataset.view === "logs") loadFalls();
});
