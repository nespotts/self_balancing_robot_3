# Bridge: Browser API, install, flashing contract

Visual tour of the app: [station/README.md](../station/README.md).

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
| `{"t":"set_mode","mode":1}` | mode index from `hello.modes`. With no remote it changes an armed robot's mode only; `"arm":true` (the UI's two-tap Arm step) arms with it (link v4: EVT_LINK_ARM first). Refused while E-stopped. `mode` IDLE = **Disarm**: no remote -> the robot disarms; with a remote present -> IDLE (motors off), the remote's ARMED switch still rules |
| `{"t":"cmd_vel","speed":200,"turn":0}` | mm/s, °/s; forwarded only while sent (300 ms) |
| `{"t":"estop"}` / `{"t":"estop_release"}` | latching E-stop (robot + bridge); release clears the latch only, the robot stays disarmed. `status.estop` = bridge latch, `status.robot_estop` = the robot's (`tlm.remote_flags` 0x08) |
| `{"t":"set_param","id":0,"value":260}` / `{"t":"get_params"}` | |
| `{"t":"cfg","action":"save"}` / `"defaults"` | EEPROM save / firmware defaults |
| `{"t":"goto","points":[[x,y],...],"go":true}` / `{"t":"event","event":3}` | `go` + `"arm":true` arms (no remote); same for `{"t":"plan","action":"goto\|explore","arm":true}` |
| `{"t":"places","action":"add","name":"Couch","x":1200,"y":300}` | saved place on the map (see **Saved places** below). `rename` (`name`, `new`), `delete` (`name`), `list`. `x`/`y` are odometry mm, like `goto` |
| `{"t":"plan","action":"home"}` / `{"t":"plan","action":"goto","place":"Couch"}` | plan a route to Home / a saved place (`arm` as for goto) |
| `{"t":"sub","tune":true}` | receive every telemetry frame as `{"t":"tune","cols":[...],"rows":[...]}` |
| `{"t":"preset"\|"score"\|"motor"\|"balpoint","action":...}` | Tune tools: see the docstring of `bridge/tuning_api.py` |

Receive: `hello`, `status`, `health` (1 Hz: preflight, tiles, battery minutes; also `GET /api/health`), `tlm` (25 Hz), `params`, `ranges`, `nav`, `waypoints`, `log`, `tune`, `map`, `plan`, `places`,
and for the Tune tools `presets`, `score_state`, `score_runs`, `motor_state`, `motor_fit`, `balpoint_state`.
`ranges.pts` rows (every reading, batched at 25 Hz): `[i, hit_x, hit_y, sensor_x, sensor_y, valid, tilt, pi_ms, sensor_id, sensor_angle_deg, range_mm]`
(sensor 0 = front / XSHUT 34, 1 = rear / XSHUT 28; angle in the robot frame, + = CCW; raw mm as sent). `status.rng_hz` = readings/s per sensor.
No WebSocket (old Kindle): `GET /api/state`, `GET|POST /api/cmd`.
The Android app uses `POST /api/cmd` when it is left: `cmd_vel 0`, `{"t":"plan","action":"stop"}`, and `set_mode` BALANCE if the
robot is armed + upright in GOTO / EXPLORE (`plan stop` alone leaves the robot in GOTO / EXPLORE with its waypoints).
That `set_mode` has no `arm`, so it can never arm a robot that was disarmed meanwhile.

### Saved places

Names on the map (Home, Couch, ...), stored with the map in `~/sbr3-maps/<map>.places.json` (map frame, mm).

| | |
|---|---|
| `places` `add` | `name` (1-30 characters), `x`, `y` in odometry mm. Max 50. A name that exists is refused, except **Home**: adding it moves Home |
| `places` `rename` / `delete` | `name`, `new`. Deleting Home falls back to the default Home (where the pose was last reset) |
| `places` `list` | asks for the list again |
| reply | `{"t":"places","map":"name","items":[{"name":"Home","x":0,"y":0,"home":true,"default":true},{"name":"Couch","x":2400,"y":-300}]}`, sent to every browser after any change and on connect. Home is always first. A bad request answers `{"t":"error","text":...}` |
| `plan` `home` | route (around walls when there is a map) to Home |
| `plan` `goto` + `place` | route to that saved place; unknown name: `error` "no place called ..."; needs a robot pose ("no robot pose yet") and, like any goto, a link and no E-stop latch |

`plan` `home` / `goto` `place` end like any plan: the Map banner shows the destination, **STOP** or **Clear** ends it.

**GO_HOME reroute.** When the robot itself starts GO_HOME (the remote's Actions > Go home, or event 3), the firmware logs `GO_HOME: to x, y` and drives a straight line. The bridge reads that line and, if it can, takes over with a planned route to the saved Home:

| Situation | Result |
|---|---|
| robot armed, no E-stop latch (bridge or robot), map has data | planned route to Home (or to the firmware's target when no Home is saved); never arms |
| no map yet | the straight line stays, bridge log says so |
| disarmed or latched (the robot ignored the request) | nothing |
| planning fails | the straight line stays, bridge log: "go-home route failed" |

### Who may command the robot

`/ws` and every `/api/*` request are checked (`httpws.check_origin`), so a web page you happen to open on the same Wi-Fi cannot drive the robot:

| Request | |
|---|---|
| the web app / Kindle page / Android WebView (same origin as the bridge) | allowed |
| curl, `tools/tune.py`, the app's native stop (no `Origin` header) | allowed |
| another site (`Origin` / `Referer` elsewhere, `Sec-Fetch-Site: cross-site`, `Origin: null`) | 403 |
| `Host` that is not an IP, `localhost`, a bare or `.local` / `.lan` name (DNS rebinding) | 403 |
| `POST /api/cmd` that is not `Content-Type: application/json` | 415 |

Relax it only if you must: `--allow-any-origin` (or `SBR3_ALLOW_ANY_ORIGIN=1`), `--allow-host pi.example.org`, `--allow-origin http://tablet.example:3000`.

Bad input never kills a client: any command that fails comes back as `{"t":"error","text":...}`. While the bridge has no link
to the robot, commands that need it (`set_mode`, `set_param`, `cfg`, `goto`, `event`, preset apply, motor test, plan goto / explore)
answer `{"t":"error","code":"no_link","text":"robot not connected ..."}` and change nothing; `estop` still latches but says
the STOP may not have arrived. The web app greys the Drive readouts and Sensors cards with a **no data** badge when
telemetry is older than 1 s:

<p>
<img src="../station/docs/screenshots/phone-live-drive.png" width="30%">
<img src="../station/docs/screenshots/phone-stale-drive.png" width="30%">
<img src="../station/docs/screenshots/phone-stale-sensors.png" width="30%">
</p>

GOTO and EXPLORE in the Drive mode grid need a second tap (like Arm & Balance). **Full rate** on the Tune page (`tlm.hz` 400)
is temporary: when the last Tune page closes (tab closed, phone asleep, app killed) the bridge puts `tlm.hz` back after 10 s,
unless you used **Save to robot** while it was raised (then that is the new default) or set it from `tools/tune.py`.
Half-open phones are dropped (WebSocket ping every 15 s, closed after 45 s of silence or a few MB of unsent data). Log CSV
exports and big downloads run in a worker thread, so they cannot stall the robot's 500 ms heartbeat; if the event loop is ever
blocked for more than 0.2 s the Logs tab says so.