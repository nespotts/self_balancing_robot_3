# SBR3 ground station

Drive, map, plot and tune the robot from any browser (phone, PC, Kindle). Python 3.8+, stdlib only, no build step.

<p>
<img src="docs/screenshots/desktop-map.png" width="64%">
<img src="docs/screenshots/phone-tune.png" width="25%">
</p>
<p>
<img src="docs/screenshots/desktop-sensors.png" width="64%">
<img src="docs/screenshots/phone-sensors.png" width="25%">
</p>

```mermaid
flowchart LR
  T[Teensy 4.1<br>firmware v4] -- "USB serial<br>link protocol v2" --> B
  subgraph Pi[Orange Pi]
    B[bridge<br>python -m station.bridge] --> L[(~/sbr3-logs<br>sessions, 2 GB cap)]
  end
  B -- "WebSocket JSON<br>+ HTTP polling" --> W[Browsers<br>:8080]
  S[simulator<br>python -m station.sim] -. "TCP, same protocol" .-> B
```

## Try it on a PC (simulator)

```
python -m station.sim
```
Open http://localhost:8080/ . The fake Teensy runs the firmware's balance controller on a simulated robot
(CAD mass/inertia, 400 Hz), so gains behave like the real thing.
Options: `--remote` (RC remote on: its ARMED switch rules), `--remote-disarmed`, `--http-port 8090`,
`--head-error 8` (range head mounted 8° off: practise the trim). The range sweep follows the firmware
(`rng.*` params: sweep / park / trim / limits / timing).

`--tour` drives a loop around the room (mapping demo), `--drift 3` makes odometry drift (3°/min, 2 % long).

Tests: `python -m unittest discover -s station -t .` (protocol vs the C header, plus a full sim + bridge + browser-API run).
Screenshots: `python -m station.tools.screenshots --url http://localhost:8080/ --prep`

## Use it

| | |
|---|---|
| **STOP** (or Space / Esc) | motors off, robot disarms |
| **Arm & Balance** | two taps. With no RC remote, the station arms the robot; with the remote, its switch does |
| **Map** | tap to add waypoints, **Go** (asks before it arms the robot): with a map, the route goes around walls. **Explore**: the robot maps the room by itself |
| **Drive** | joystick / WASD in STATION mode |
| **Sensors** | both range sensors live: mm, valid, angle, readings/s; last sweep around the robot; range vs time. ⋯ → park / trim / sweep width / direction, **hand test** (cover one sensor: its card lights up). Colours: front amber, rear magenta (map too) |
| **Tune** | live P / I / D at the loop rate (**Full rate**), sliders apply at once, **Save to robot** writes EEPROM (when the motors are off); ⋯ → Restore defaults |
| **Logs** | messages, session downloads (CSV / JSONL), link details |

## Map (occupancy grid, route, explore)

<p>
<img src="docs/screenshots/desktop-route.png" width="64%">
<img src="docs/screenshots/phone-explore.png" width="25%">
</p>

| | |
|---|---|
| grey / white / black | unknown / free / wall (25 mm cells, built on the Pi from every range reading) |
| ⋯ → **Map** | on / off + opacity · **Save / Load map** (`~/sbr3-maps`) · **Clear map** |
| ⋯ → Map & route settings | distance kept off walls, robot radius, tilt limit, “no target” handling, drift correction (experimental) |
| **Go** with a map | the bridge plans a route (purple) around obstacles, re-plans when the robot is blocked |
| **Explore** | drives to the nearest unmapped edge until none is left; STOP ends it |

```mermaid
flowchart LR
  R[RangeFrame] --> F{tilted? no target?} -- ok --> G[grid: free wedge + hit arc] --> T[changed 1 m tiles<br>PNG, 2 Hz] --> W[browsers]
  G --> P[planner: A* / frontiers<br>worker thread] -- MSG_GOTO --> Robot
```

Pi cost: 0.2 ms per reading (< 1 % of a core at 50/s); a route or frontier search 0.1-0.3 s in a worker thread.
Offline: `python -m station.tools.mapbuild ~/sbr3-logs/<session>.jsonl -o map.png`.
Drift correction (scan-to-map matching) is off by default: in the sim it does not beat plain odometry yet.
Browser API: `{"t":"map","action":"reset|save|load|list|delete|set"}`, `{"t":"plan","action":"goto|explore|stop|set"}`;
receive `map` (tiles), `plan` (route, status). `GET /api/map`, `/api/map/current.png`.

## Install on the Orange Pi

One time, with sudo (ask the owner first):
```
sudo loginctl enable-linger orangepi        # user services start at boot
sudo usermod -aG dialout orangepi           # only if `id -nG` has no dialout
```
Then, no sudo, from the repo:
```
sh station/bridge/install/install.sh
```
It installs a **systemd user service**: `systemctl --user status|restart sbr3-bridge`, logs
`journalctl --user -u sbr3-bridge -f`. UI: `http://<pi>.local:8080/`.

## Flashing while the bridge runs

The bridge lets go of `/dev/ttyACM0` while **`/tmp/sbr3-bridge.release`** exists, and re-opens it by itself when the file is gone.

```
touch /tmp/sbr3-bridge.release; sleep 1      # bridge closes the port (checked every 0.25 s)
teensy_loader_cli --mcu=TEENSY41 -w -s -v firmware.hex
rm /tmp/sbr3-bridge.release                   # bridge reconnects
```
- No sudo; any process of the same machine can do it. `station/bridge/install/flash-teensy.sh` does exactly this.
- A file older than 15 min is ignored (a crashed flash script can't block the robot forever); touch it again to hold longer.
- Optional: `GET /api/serial` → `{"released": true}`; or `POST /api/serial?release=1` / `?release=0`.

## Browser API (WebSocket `/ws`, JSON)

| Send | |
|---|---|
| `{"t":"set_mode","mode":1}` | mode index from `hello.modes`; with no remote, non-IDLE arms |
| `{"t":"cmd_vel","speed":200,"turn":0}` | mm/s, °/s; forwarded only while sent (300 ms) |
| `{"t":"estop"}` | |
| `{"t":"set_param","id":0,"value":260}` / `{"t":"get_params"}` | |
| `{"t":"cfg","action":"save"}` / `"defaults"` | EEPROM save / firmware defaults |
| `{"t":"goto","points":[[x,y],...],"go":true}` / `{"t":"event","event":3}` | |
| `{"t":"sub","tune":true}` | receive every telemetry frame as `{"t":"tune","cols":[...],"rows":[...]}` |

Receive: `hello`, `status`, `tlm` (25 Hz), `params`, `ranges`, `nav`, `waypoints`, `log`, `tune`, `map`, `plan`.
`ranges.pts` rows (every reading, batched at 25 Hz): `[i, hit_x, hit_y, sensor_x, sensor_y, valid, tilt, pi_ms, sensor_id, sensor_angle_deg, range_mm]`
(sensor 0 = front / XSHUT 34, 1 = rear / XSHUT 28; angle in the robot frame, + = CCW; raw mm as sent). `status.rng_hz` = readings/s per sensor.
No WebSocket (old Kindle): `GET /api/state`, `GET|POST /api/cmd`.
