// Health view (issue #51): preflight checklist, battery minutes left, one tile per subsystem.
// Read-only: the bridge sends a `health` message once a second; this page never arms or commands anything.
import { state, on } from "./conn.js";
import { el } from "./ui.js";

const root = document.querySelector(".health");
const dot = document.getElementById("hl-dot"), text = document.getElementById("hl-text");
const items = document.getElementById("hl-items"), batt = document.getElementById("hl-batt"), tiles = document.getElementById("hl-tiles");
const PRE = { green: "ok", amber: "warn", red: "bad" };
const open = new Set();      // tiles the user tapped open

function battCard(h) {
  const b = h.battery || {}, t = state.tlm;
  batt.innerHTML = "";
  batt.className = "card hl-batt";
  const tile = (h.tiles || []).find((x) => x.id === "batt");
  if (tile) batt.classList.add("lv-" + tile.level);
  batt.append(el("h3", {}, "Battery"));
  if (!tile || tile.level === "na") { batt.append(el("div", { class: "hl-sub" }, "no robot data")); return; }
  if (t && t.battery_v < 3) {
    batt.append(el("div", { class: "hl-big" }, "Bench supply"), el("div", { class: "hl-sub" }, "no balance lead: cells not measured, no minutes estimate"));
    return;
  }
  const big = el("div", { class: "hl-big" }, b.v != null ? b.v.toFixed(1) + " V" : (t ? t.battery_v.toFixed(1) + " V" : "-"));
  if (b.pct != null) big.append(el("small", {}, b.pct + "%"));
  batt.append(big);
  let sub;
  if (b.minutes != null) sub = (b.state === "low" ? "empty" : "about " + b.minutes + " min left") + (b.load != null ? " at your usual load" : "");
  else if (b.state === "steady") sub = "voltage steady: no minutes estimate";
  else sub = "learning the discharge trend (about 2 min)";
  batt.append(el("div", { class: "hl-sub" }, sub), el("div", { class: "hl-sub" }, tile.why));
  if (b.pct != null) batt.append(el("div", { class: "hl-bar" }, el("i", { style: "width:" + b.pct + "%" })));
}

function render() {
  const h = state.health;
  const stale = !state.connected || !h || performance.now() - state.healthAt > 4000;
  root.classList.toggle("stale", stale && !!h);
  if (!h) { dot.className = "hl-dot"; text.textContent = state.connected ? "Waiting for the bridge..." : "Bridge offline"; return; }
  const pre = h.preflight;
  dot.className = "hl-dot lv-" + PRE[pre.level];
  text.textContent = stale ? "No data (bridge silent)" : pre.text;
  document.getElementById("hl-robot").textContent = state.status?.robot_name || "";   // robot profile (#97)
  items.replaceChildren(...pre.items.map((i) => el("div", { class: "hl-row" },
    el("span", { class: "hl-dot lv-" + i.level }), i.label, el("span", { class: "hl-why" }, i.why))));
  battCard(h);
  tiles.replaceChildren(...h.tiles.map((t) => {
    const b = el("button", { class: "hl-tile lv-" + t.level + (open.has(t.id) ? " open" : ""), title: t.why },
      el("div", { class: "hl-l" }, t.label), el("div", { class: "hl-v" }, t.value), el("div", { class: "hl-w" }, t.why));
    b.onclick = () => { open.has(t.id) ? open.delete(t.id) : open.add(t.id); b.classList.toggle("open"); };
    return b;
  }));
}
on("health", () => { if (document.body.dataset.view === "health") render(); });
on("conn", () => { if (document.body.dataset.view === "health") render(); });
export function healthShown(shown) { if (shown) render(); }