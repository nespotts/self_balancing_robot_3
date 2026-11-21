// Scrolling strip chart on a canvas, with its own ring buffer. Used by Plots (25 Hz telemetry) and
// Tune (full-rate "tune" rows, up to 400 Hz). Dense data is drawn as min/max per pixel column.
import { el } from "./ui.js";

export const S1 = "#3987e5", S2 = "#d95926", S3 = "#199e70", SUM = "#c3c2b7";   // categorical slots 1-3, total
// Range sensors, used everywhere (Sensors view, map): 0 = front (XSHUT 34) amber, 1 = rear (XSHUT 28) magenta.
// Checked with the dataviz palette validator on the dark surface (CVD dE 13). SENS_RGB for alpha fades.
export const SENS = ["#c98500", "#d55181"], SENS_RGB = [[201, 133, 0], [213, 81, 129]];
export const SENS_NEW = ["#ffc23d", "#ff86b4"];   // newest points (brighter)

export class Strip {
  // def: {title, series: [[key, label, color, fn?]], minSpan?, fixed?: [lo, hi]}; fn(m) computes the value
  constructor(def, n = 2000) {
    this.def = def; this.N = n;
    this.t = new Float64Array(n);
    this.v = def.series.map(() => new Float32Array(n));
    this.head = 0; this.count = 0; this.hoverX = null;
    this.canvas = el("canvas");
    this.ctx = this.canvas.getContext("2d");
    this.legend = el("div", { class: "legend" }, el("span", { class: "title" }, def.title));
    this.vals = def.series.map(([, label, color]) => {
      const b = el("b", {}, "—");
      this.legend.append(el("span", {}, el("i", { style: `background:${color}` }), label + " ", b));
      return b;
    });
    this.box = el("div", { class: "plot" }, this.canvas, this.legend);
    this.canvas.addEventListener("pointermove", (e) => { this.hoverX = e.offsetX; this.onhover?.(); });
    this.canvas.addEventListener("pointerleave", () => { this.hoverX = null; this.onhover?.(); });
  }

  push(t, m) {
    const h = this.head;
    this.t[h] = t;
    this.def.series.forEach(([k, , , fn], n) => { this.v[n][h] = fn ? fn(m) : m[k]; });
    this.head = (h + 1) % this.N;
    if (this.count < this.N) this.count++;
  }
  clear() { this.head = 0; this.count = 0; }
  idx(i) { return (this.head - this.count + i + this.N) % this.N; }   // i-th oldest sample
  last() { return this.count ? this.t[this.idx(this.count - 1)] : 0; }

  draw(tEnd, winS) {
    const { canvas, ctx, def } = this;
    const dpr = window.devicePixelRatio || 1, W = canvas.clientWidth, H = canvas.clientHeight;
    if (!W || !H) return;
    if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
      canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const padL = 46, padR = 12, padT = Math.max(38, this.legend.offsetHeight + 14), padB = 22;   // legend may wrap
    const pw = W - padL - padR, ph = H - padT - padB;
    const t0 = tEnd - winS * 1000, count = this.count, T = this.t;
    let first = 0;   // first sample in the window (binary search: times are increasing)
    { let lo = 0, hi = count; while (lo < hi) { const mid = (lo + hi) >> 1; if (T[this.idx(mid)] < t0) lo = mid + 1; else hi = mid; } first = lo; }
    let last = count - 1;
    while (last >= first && T[this.idx(last)] > tEnd) last--;
    let lo = Infinity, hi = -Infinity;
    if (def.fixed) [lo, hi] = def.fixed;
    else {
      for (const a of this.v) for (let i = first; i <= last; i++) { const x = a[this.idx(i)]; if (x < lo) lo = x; if (x > hi) hi = x; }
      if (!isFinite(lo)) { lo = -1; hi = 1; }
      const mid = (lo + hi) / 2, span = Math.max(hi - lo, def.minSpan || 1) * 1.15;
      lo = mid - span / 2; hi = mid + span / 2;
    }
    const step = niceStep(hi - lo, Math.max(2, Math.floor(ph / 40)));
    if (!def.fixed) { lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step; }
    const X = (t) => padL + ((t - t0) / (winS * 1000)) * pw;
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;
    // grid + labels
    ctx.font = "11px system-ui, sans-serif"; ctx.fillStyle = "#8d8c84"; ctx.textAlign = "right"; ctx.textBaseline = "middle";
    ctx.lineWidth = 1;
    for (let v = Math.ceil(lo / step) * step; v <= hi + step * 1e-6; v += step) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.strokeStyle = Math.abs(v) < step * 1e-6 ? "#45453f" : "#2a2a28";
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(W - padR, y); ctx.stroke();
      ctx.fillText(String(+v.toFixed(Math.max(0, -Math.floor(Math.log10(step) + 1e-9)))), padL - 6, y);
    }
    ctx.textBaseline = "top";
    const tStep = winS <= 2 ? 0.5 : winS <= 5 ? 1 : winS <= 10 ? 2 : winS <= 30 ? 5 : 10;
    for (let s = 0; s <= winS + 1e-9; s += tStep) {
      const x = padL + (s / winS) * pw;
      ctx.textAlign = s === 0 ? "left" : Math.abs(s - winS) < 1e-9 ? "right" : "center";
      ctx.fillText(Math.abs(s - winS) < 1e-9 ? "now" : "-" + +(winS - s).toFixed(1) + "s", x, H - padB + 6);
    }
    // series
    ctx.save();
    ctx.beginPath(); ctx.rect(padL, padT, pw, ph); ctx.clip();
    ctx.lineWidth = 2; ctx.lineJoin = "round";
    const from = Math.max(0, first - 1), dense = last - from > pw * 1.5;
    def.series.forEach(([, , color], n) => {
      const a = this.v[n];
      ctx.strokeStyle = color;
      ctx.beginPath();
      if (dense) {   // min/max per pixel column
        ctx.lineWidth = 1.5;
        let col = -1, mn = 0, mx = 0;
        const flush = () => { if (col >= 0) { ctx.moveTo(col + 0.5, Y(mn)); ctx.lineTo(col + 0.5, Y(mx) - 0.5); } };
        let prevY = null;
        for (let i = from; i <= last; i++) {
          const j = this.idx(i), c = Math.floor(X(T[j])), v = a[j];
          if (c !== col) {
            flush();
            if (prevY != null && col >= 0) { ctx.moveTo(col + 0.5, prevY); ctx.lineTo(c + 0.5, Y(v)); }
            prevY = Y(v); col = c; mn = mx = v;
          } else { if (v < mn) mn = v; if (v > mx) mx = v; prevY = Y(v); }
        }
        flush();
      } else {
        for (let i = from; i <= last; i++) {
          const j = this.idx(i), x = X(T[j]), y = Y(a[j]);
          i === from ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
      ctx.lineWidth = 2;
    });
    ctx.restore();
    // crosshair + values (hover) or latest values
    let at = last;
    if (this.hoverX != null && this.hoverX >= padL && last >= first) {
      const th = t0 + ((this.hoverX - padL) / pw) * winS * 1000;
      for (let i = last; i >= first; i--) { if (T[this.idx(i)] <= th) { at = i; break; } }
      const x = Math.round(X(T[this.idx(at)])) + 0.5;
      ctx.strokeStyle = "#8d8c84"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      def.series.forEach(([, , color], n) => {
        const y = Y(this.v[n][this.idx(at)]);
        ctx.fillStyle = color; ctx.strokeStyle = "#1a1a19"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill(); ctx.stroke();
      });
    }
    def.series.forEach((_, n) => { this.vals[n].textContent = at >= 0 && count ? fmtV(this.v[n][this.idx(at)]) : "—"; });
  }
}

function fmtV(v) {
  const a = Math.abs(v);
  return v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : 2);
}

function niceStep(span, n) {
  const raw = span / n, p = Math.pow(10, Math.floor(Math.log10(raw))), m = raw / p;
  return (m < 1.5 ? 1 : m < 3 ? 2 : m < 7 ? 5 : 10) * p;
}
