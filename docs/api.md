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
- Optional: `GET /api/serial` â†’ `{"released": true}`; or `POST /api/serial?release=1` / `?release=0`.

## Browser API (WebSocket `/ws`, JSON)

| Send | |
|---|---|
| `{"t":"set_mode","mode":1}` | mode index from `hello.modes`. With no remote it changes an armed robot's mode only; `"arm":true` (the UI's two-tap Arm step) arms with it (link v4: EVT_LINK_ARM first). Refused while E-stopped. `mode` IDLE = **Disarm**: no remote -> the robot disarms; with a remote present -> IDLE (motors off), the remote's ARMED switch still rules |
| `{"t":"cmd_vel","speed":200,"turn":0}` | mm/s, Â°/s; forwarded only while sent (300 ms) |
| `{"t":"estop"}` / `{"t":"estop_release"}` | latching E-stop (robot + bridge); release clears the latch only, the robot stays disarmed. `status.estop` = bridge latch, `status.robot_estop` = the robot's (`tlm.remote_flags` 0x08) |
| `{"t":"set_param","id":0,"value":260}` / `{"t":"get_params"}` | |
| `{"t":"cfg","action":"save"}` / `"defaults"` | EEPROM save / firmware defaults |
| `{"t":"goto","points":[[x,y],...],"go":true}` / `{"t":"event","event":3}` | `go` + `"arm":true` arms (no remote); same for `{"t":"plan","action":"goto\|explore","arm":true}` |
| `{"t":"places","action":"add","name":"Couch","x":1200,"y":300}` | saved place on the map (see **Saved places** below). `rename` (`name`, `new`), `delete` (`name`), `list`. `x`/`y` are odometry mm, like `goto` |
| `{"t":"plan","action":"home"}` / `{"t":"plan","action":"goto","place":"Couch"}` | plan a route to Home / a saved place (`arm` as for goto) |
| `{"t":"zones","action":"add","kind":"nogo","pts":[[x,y],...]}` | virtual walls / no-go / fence / slow zones / mazes (see **Zones** below) |
| `{"t":"plan","action":"solve","maze":"m4"}` | drive through that maze to its exit cell, timed (`arm` as for goto) |
| `{"t":"map","action":"trust_pose"}` | LD19 matcher lost (#79): take the pose as it is and map from here (map menu **Trust the pose here**, shown only while lost). Error when it is not lost |
| `{"t":"map","action":"scan"}` | stop-and-scan pose correction now (#71): the next two sweeps taken standing still are matched to the map (also with the `stop_scan` map setting off). Result: a `scan` message |
| `{"t":"lidar","action":"set","settings":{"yaw_deg":90,...}}` | LD19 lidar (#76, see [ld19.md](ld19.md)): `get` / `set` / `defaults`; the reply (and every new client) is `{"t":"lidar","settings":{...},"defaults":{...},"status":{...}}`. Settings persist in `lidar.json` next to the saved maps. Error when the bridge has no lidar port |
| `{"t":"sub","tune":true}` | receive every telemetry frame as `{"t":"tune","cols":[...],"rows":[...]}` |
| `{"t":"ros","action":"subscribe"}` | ROS stream for the ROS 2 adapter (#96, [ros2.md](ros2.md#the-bridge-ros-stream)): `ros_hello` now, then `ros_scan` (10 Hz), `ros_odom` (25 batches/s, 50 rows/s), `ros_state` (2 Hz), only to this client, droppable. `unsubscribe` / `get` |
| `{"t":"preset"\|"score"\|"motor"\|"balpoint"\|"calib","action":...}` | Tune tools: see the docstring of `bridge/tuning_api.py` |

Receive: `hello`, `status`, `health` (1 Hz: preflight, tiles, battery minutes; also `GET /api/health`), `tlm` (25 Hz), `params`, `ranges`, `nav`, `waypoints`, `log`, `tune`, `map`, `plan`, `places`, `zones`, `scan`,
and for the Tune tools `presets`, `score_state`, `score_runs`, `motor_state`, `motor_fit`, `balpoint_state`, `calib_state`.
`ranges.pts` rows (every reading, batched at 25 Hz): `[i, hit_x, hit_y, sensor_x, sensor_y, valid, tilt, pi_ms, sensor_id, sensor_angle_deg, range_mm]`
(sensor 0 = front / XSHUT 34, 1 = rear / XSHUT 28; angle in the robot frame, + = CCW; raw mm as sent). `status.rng_hz` = readings/s per sensor.
`lidar_scan` (<= 3 Hz, LD19 only): `{"t":"lidar_scan","seq":812,"pts":[x0,y0,x1,y1,...]}`, the newest revolution in mm, odometry frame (what the Map draws as the "Lidar scan" layer).
`status.lidar` (null without a lidar port): `{"state":"ok|waiting|no data|junk|stale|off","port","error","pps","scan_hz","rpm","packets","crc_errors","bytes_dropped","scans","scan_age_s","flowing","cpu_pct","cost_ms","tilt_ref","dropped":{...}}`; Health has a "Lidar" tile from it.
`map.stats.lmatch` (LD19 scan matching, #78, [ld19.md](ld19.md#scan-matching-78)): `{"on":true,"lidar":true,"state":"matched|matched: pose fine|building the map|kept: <why>|wheel slip: ...|lost: searching ...|off","score":0.98,"fix":"xy th","applied":[dx,dy,dth],"ok_pct":97,"ms":3.4,"matched":812,"rejected":9,"jumps":0,"moved_mm":640,"moved_deg":6.2,"mode":"track|slip|lost","grip":0.97,"slip_s":null,"lost_s":null,"window":null,"slips":0,"lost":0,"found":0,"stuck":0,"checking":false,"checks":0}`
(`applied`: the last correction step, mm / deg; `ms`: cost per match). #79 wheel slip ([ld19.md](ld19.md#wheel-slip-79)): `mode` `slip` = the wheels turn but the
body does not follow (position held, nothing mapped), `lost` = the pose is being searched for (nothing mapped); `grip` = body / wheel travel at
the last check; `slip_s` / `lost_s` = seconds in that mode; `window` = the lost search's `[mm, deg]`; `checking` = `lost` only to check
the pose after a lidar gap (quiet, lost after 3 s); `slips` / `lost` / `found` / `stuck` / `checks` = counts. While slipping / lost a `map` message without tiles carries the new state and `corr` (<= 1 Hz). Its settings are map settings
(`{"t":"map","action":"set","settings":{"lidar_match":false}}`, `match_*`), in `settings.json`.
`status.robot_name` / `status.robot_source` (#97): the robot profile in use, `"balancer"`, `"shark"`, `"sim"` ... (`null` until the first HELLO + params are in), and how it was found: `"sys.platform"` (param 1 = balancer, 2 = shark), `"firmware"` (a simulator, `sim ...`), `"default"` (older firmware: balancer), `"override"` (`--robot NAME`). Maps, places, zones, `lidar.json`, `planner.json`, presets and fall records are kept per robot, see [the station README](../station/README.md#robot-profiles-one-pi-several-robots). The Pi can move between robots: a new connection is identified again from its own HELLO.
`status.lidar_nav` (null without a lidar port): `{"sent":812,"age_s":0.04,"skipped":{"off","no_pose","stale","tilt","few","no_link"}}`, the obstacle frames for the robot's navigator ([below](#link-messages-to-the-robot)).
`scan` (on connect and after each stop-and-scan): `{"t":"scan","corr":[dx,dy,dth],"applied":3,"last":{...},"history":[{"ok":true,"why":"","dx":-31,"dy":12,"dth":-1.2,"mm":33,"n":58,"inliers":51,"margin":0.11,"who":"auto","time":1795300000},...]}` (newest first, 20 kept; `why` says why a scan was not used).
No WebSocket (old Kindle): `GET /api/state`, `GET|POST /api/cmd`.
The Android app uses `POST /api/cmd` when it is left: `cmd_vel 0`, `{"t":"plan","action":"stop"}`, and `set_mode` BALANCE if the
robot is armed + upright in GOTO / EXPLORE (`plan stop` alone leaves the robot in GOTO / EXPLORE with its waypoints).
That `set_mode` has no `arm`, so it can never arm a robot that was disarmed meanwhile.

### Saved places

Names on the map (Home, Couch, ...), stored with the map in `~/sbr3-maps/<robot>/<map>.places.json` (map frame, mm).

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

### Zones

Virtual walls, no-go zones, the keep-in fence, slow zones and mazes, stored with the map in `~/sbr3-maps/<robot>/<map>.zones.json`
(map frame, mm; an unsaved map autosaves to `autosave.zones.json`). Points are sent in odometry mm, like `goto`.

| `zones` action | |
|---|---|
| `add` | `kind` `wall` (polyline, 2+ points) / `nogo` / `fence` / `slow` (polygons, 3+ points, at least 0.01 mÂ²), `pts`; `slow`: `speed` mm/s (30-500, default 120). A new fence replaces the old one. Max 100 items, 64 points each |
| `update` | `id`, `pts` and / or `speed` (mazes: no) |
| `delete` / `clear` / `undo` | `id` / optional `kind` (none = all) / the last change (30 kept) |
| `maze` | `x`, `y` centre, `w`, `h` mm (default 3000), `cell` mm (400-1500, default 700), `seed` (optional). Random perfect maze, 2-12 cells a side, entrance on the side facing the robot, exit opposite |
| `list` | send the layers again |
| reply | `{"t":"zones","map":"name","undo":3,"items":[{"id":"n2","kind":"nogo","pts":[[x,y],...]},{"id":"s3","kind":"slow","pts":[...],"speed":100},{"id":"m4","kind":"maze","x0":..,"y0":..,"cols":4,"rows":4,"cell":700,"seed":..,"walls":[[x1,y1,x2,y2],...],"entry":[x,y],"entry_in":[x,y],"exit":[x,y],"exit_out":[x,y],"exit_gap":[x1,y1,x2,y2],"best_s":51.1}]}` to every browser after a change and on connect; errors: `{"t":"error","text":"zones: ..."}` |

`plan` `solve` (`maze` id): a goto to the maze's exit cell with its exit gap counted as a wall, so the route goes in at the entrance.
`plan` messages carry `"solve":{"maze":"m4","elapsed":12.3}` meanwhile; at the end `status` = "maze solved in 51.1 s" and the best time is
kept in the maze (`best_s`). A `goto` / place inside a no-go zone or outside the fence is refused ("not sent: ..."). `status.zone_alert` =
the watchdog's last stop / joystick block. Planner setting `zones_block_manual` (default true): see [safety](safety.md#virtual-walls-and-zones).

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

## Link messages to the robot

The bridge <-> Teensy link itself (framed binary, CRC16) is defined in
[include/protocol/link_protocol.h](../include/protocol/link_protocol.h) (link v6), mirrored in `station/bridge/link_protocol.py`.
New in v6: **`EVT_LINK_STUCK`** (0x83, a link-only `MSG_EVENT` value, issue #79): the bridge's LD19 matcher saw the wheels turn
for `match_stuck_s` (2.5 s) in GOTO / EXPLORE while the robot did not move (wheel slip, a low obstacle the lidar does not see). The navigator's
next driving stuck check (within 500 ms) takes it like its own: BLOCKED, log `nav: stuck, wheels turn but the lidar sees no motion (waypoint 1/2)`,
back off `nav.stuck_backoff_mm`; the planner replans as after any `nav: stuck`. Not driving then (turning in place, stopped): dropped. v5 firmware
ignores it (unknown event); a v5 bridge never sends it. No new params, no layout change.

New in v5:

**`MSG_LIDAR_OBSTACLES` (0x87, Pi -> Teensy, ~10 Hz, issue #77)**: one LD19 return per sector (the nearest inside the robot's
driving corridor, else the nearest: #99), for the navigator
([ld19.md](ld19.md#obstacles-for-the-robot-issue-77)). Sent by `station/bridge/lidar_obstacles.py` once per good turn.

| Byte | Field | |
|---|---|---|
| 0 | `seq` u8 | + 1 per frame |
| 1 | `sectors` u8 | 36 (`LINK_LIDAR_SECTORS`); another count is dropped |
| 2-3 | `age_ms` u16 | age of the pose the points are relative to (the bridge's newest telemetry); > 300 (`LINK_LIDAR_MAX_AGE_MS`) is dropped |
| 4-147 | `xy_mm` 72 x i16 | x, y per sector: robot frame (x forward, y left, mm from the axle centre). Sector i = (i - 0.5) x 10 .. (i + 0.5) x 10 deg CCW from forward. `0, 0` = no return |

No frame for 500 ms (`LINK_LIDAR_FRESH_MS`) = no lidar: the robot uses the sweep again, or goes blind. v4 firmware ignores the
message; a v4 bridge never sends it.
