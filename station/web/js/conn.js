// WebSocket connection to the bridge + shared state store.
const listeners = {};
export function on(type, fn) { (listeners[type] ||= []).push(fn); }
function emit(type, m) { for (const fn of listeners[type] || []) { try { fn(m); } catch (e) { console.error(type, e); } } }

export const state = {
  connected: false,
  hello: null, robot: null, status: null, tlm: null, tlmAt: 0, nav: null,
  params: [], paramsFirst: {},     // id -> first value seen this page load (for "revert")
  waypoints: [],                   // sent to the robot (from the bridge)
  ranges: [],                      // [i, hx, hy, rx, ry, valid, tilt, t_ms, sensor_id, sensor_angle_deg, range_mm]
  track: [],                       // [x, y]
  logs: [],
  grid: null,                      // occupancy grid layer (issue #12), filled by a future "grid" message
};
// Telemetry older than this is not shown as live (the robot is silent: flashing, unplugged, reset).
export const STALE_MS = 1000;
const MAX_RANGES = 6000, MAX_TRACK = 6000, MAX_LOGS = 400;

let ws = null, retry = 0;

export function send(msg) {
  if (ws && ws.readyState === 1) { ws.send(JSON.stringify(msg)); return true; }
  return false;
}

export function modeName(m) { return state.hello?.modes?.[m] ?? (m == null ? "—" : "MODE " + m); }
export function flag(name) {
  const bit = state.hello?.tlm_bits?.[name];
  return !!(state.tlm && bit && (state.tlm.status & bit));
}
export function robotAlive() { return state.connected && state.status?.link === "connected" && state.status?.robot_alive; }
export function tlmAge() { return state.tlmAt ? performance.now() - state.tlmAt : Infinity; }
// true while the numbers on screen are current; false -> grey them out and say "no data"
export function tlmFresh() { return robotAlive() && tlmAge() < STALE_MS; }

function handle(m) {
  switch (m.t) {
    case "hello":
      // new connection: the bridge replays recent ranges and logs next
      state.hello = m; state.robot = m.robot; state.waypoints = m.waypoints || [];
      state.ranges = []; state.logs = [];
      emit("waypoints", m);
      break;
    case "robot": state.robot = m.robot; break;
    case "status": state.status = m; break;
    case "tlm": {
      state.tlm = m; state.tlmAt = performance.now();
      const tr = state.track, last = tr[tr.length - 1];
      if (!last || Math.hypot(m.x_mm - last[0], m.y_mm - last[1]) > 15) {
        tr.push([m.x_mm, m.y_mm]);
        if (tr.length > MAX_TRACK) tr.splice(0, tr.length - MAX_TRACK);
      }
      break;
    }
    case "ranges": {
      const r = state.ranges, last = r.length ? r[r.length - 1][0] : -1;
      for (const p of m.pts) if (p[0] > last) r.push(p);
      if (r.length > MAX_RANGES) r.splice(0, r.length - MAX_RANGES);
      break;
    }
    case "params":
      state.params = m.list;
      for (const p of m.list) if (!(p.id in state.paramsFirst)) state.paramsFirst[p.id] = p.value;
      break;
    case "nav": state.nav = m; break;
    case "track": state.track = m.pts; break;
    case "waypoints": state.waypoints = m.points; break;
    case "reset": state.ranges = []; state.track = []; state.waypoints = []; break;
    case "log":
      state.logs.push(m);
      if (state.logs.length > MAX_LOGS) state.logs.splice(0, state.logs.length - MAX_LOGS);
      break;
  }
  emit(m.t, m);
  emit("any", m);
}

export function connect() {
  const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws";
  ws = new WebSocket(url);
  ws.onopen = () => {
    retry = 0; state.connected = true;
    send({ t: "hi", name: /Android|iPhone/i.test(navigator.userAgent) ? "phone" : "pc" });
    emit("conn", true);
  };
  ws.onmessage = (e) => { let m; try { m = JSON.parse(e.data); } catch { return; } handle(m); };
  ws.onclose = () => {
    const was = state.connected;
    state.connected = false; ws = null;
    if (was) emit("conn", false);
    setTimeout(connect, Math.min(4000, 500 + 500 * retry++));
  };
  ws.onerror = () => { try { ws.close(); } catch {} };
}
