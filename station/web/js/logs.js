// Logs view: robot/bridge messages, recorded sessions (download), link details.
import { state, on, send } from "./conn.js";
import { el, sizeStr, toast } from "./ui.js";

const msgs = document.getElementById("msgs"), sessions = document.getElementById("sessions");
const linkInfo = document.getElementById("link-info"), relBtn = document.getElementById("serial-release");

function fmtT(s) {
  const m = Math.floor(s / 60), h = Math.floor(m / 60);
  return (h ? h + ":" + String(m % 60).padStart(2, "0") : m) + ":" + String(Math.floor(s % 60)).padStart(2, "0");
}
function addMsg(m) {
  const stick = msgs.scrollTop + msgs.clientHeight >= msgs.scrollHeight - 20;
  msgs.append(el("div", { class: m.src }, el("span", { class: "t" }, fmtT(m.ts || 0)), m.text));
  while (msgs.childElementCount > 400) msgs.firstChild.remove();
  if (stick) msgs.scrollTop = msgs.scrollHeight;
}
on("hello", () => { msgs.innerHTML = ""; });
on("log", addMsg);

function nameToDate(n) {
  const m = /sbr3-(\d{4})(\d\d)(\d\d)-(\d\d)(\d\d)(\d\d)/.exec(n);
  return m ? `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}` : n;
}
export async function loadSessions() {
  try {
    const r = await fetch("/api/logs");
    const d = await r.json();
    sessions.innerHTML = "";
    if (!d.sessions.length) sessions.append(el("p", { class: "muted" }, "No sessions yet. One is recorded every time the robot connects."));
    for (const s of d.sessions) {
      const base = "/api/logs/" + encodeURIComponent(s.name);
      sessions.append(el("div", { class: "srow" },
        el("div", { class: "sname" }, nameToDate(s.name), el("small", {}, sizeStr(s.size)), s.active ? el("span", { class: "rec" }, "● REC") : null),
        el("a", { href: base + "?csv=tlm", download: "" }, "Telemetry CSV"),
        el("a", { href: base + "?csv=rng", download: "" }, "Range CSV"),
        el("a", { href: base, download: "" }, "Full log (JSONL)")));
    }
  } catch (e) {
    sessions.textContent = "Could not load sessions";
  }
}
document.getElementById("logs-refresh").onclick = loadSessions;

function updateLink() {
  const s = state.status;
  linkInfo.innerHTML = "";
  const kv = (k, v) => { linkInfo.append(el("dt", {}, k), el("dd", {}, String(v))); };
  kv("Bridge", state.connected ? "connected" : "offline");
  if (!s) return;
  kv("Robot link", s.link + (s.link_error ? " (" + s.link_error + ")" : ""));
  kv("Port", s.port);
  kv("Telemetry", s.tlm_hz + " Hz" + (s.robot_alive ? "" : " (silent)"));
  kv("Frames / CRC errors", s.frames + " / " + s.crc_errors);
  if (state.tlm) kv("RC remote", state.tlm.radio_ok ? "ok" : "no signal");
  if (state.robot) kv("Firmware", state.robot.firmware);
  kv("Clients", s.clients);
  kv("Session", s.session || "—");
  kv("Bridge uptime", fmtT(s.uptime_s));
  relBtn.textContent = s.link === "released" ? "Re-attach serial port" : "Release serial port (for flashing)";
}
relBtn.onclick = async () => {
  const release = state.status?.link !== "released";
  if (release && !confirm("Release the Teensy serial port? Telemetry stops until it is re-attached.")) return;
  await fetch("/api/serial?release=" + (release ? 1 : 0), { method: "POST" });
  toast(release ? "Serial port released" : "Re-attaching…");
};
on("status", updateLink);
on("conn", updateLink);
updateLink();
