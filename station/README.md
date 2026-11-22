# SBR3 ground station

Drive, map, plot and tune the robot from any browser (phone, PC, Kindle). Python 3.8+, stdlib only, no build step.

```mermaid
flowchart LR
  T[Teensy 4.1<br>firmware v4] -- "USB serial<br>link protocol v4" --> B
  subgraph Pi[Orange Pi]
    B[bridge<br>python -m station.bridge] --> L[(logs, falls, maps)]
  end
  B -- "WebSocket JSON" --> W[Browsers :8080]
  S[simulator<br>python -m station.sim] -. "TCP" .-> B
```

Try it: `python -m station.sim`, open http://localhost:8080/ (`--host 0.0.0.0` shares it on the LAN, `--remote` adds the RC remote, `--shove 12` knocks the robot over every 12 s). Tests: `python -m unittest discover -s station -t .`

**STOP** (or Space / Esc) latches: **Release E-stop**, then arm again with the two-tap **Arm & Balance** (or the remote).

## Quick start: the phone is the remote

The RC remote is optional. Turn it off (or leave it home): the phone arms, drives and stops the robot.

```mermaid
flowchart LR
  A["Control tab<br>Arm & Balance<br>(tap, tap again)"] --> B["Stand the robot up<br>Disarm + STOP show"]
  B --> C["Pick a mode<br>Balance, Hold, Manual,<br>Go to, Explore"]
  C --> D["Disarm<br>(motors off)"]
  C --> E["STOP<br>(latches)"]
```

<p>
<img src="docs/screenshots/phone-controller.png" width="30%">
<img src="docs/screenshots/phone-controller-landscape.png" width="62%">
</p>
<p>Control screen: portrait, and landscape (the Android app's default)</p>

| Button | What it does |
|---|---|
| **Arm & Balance** | two taps, only when disarmed |
| **Disarm** | one tap, motors off, no latch (next to STOP while armed) |
| **STOP** | emergency stop, latches until **Release E-stop** |
| **Manual** | joystick (or **Tilt** in the app); **Go to** / **Explore** ask for a second tap |

"Controlled by" says who arms: **this phone** (no RC remote) or the **RC remote**. With no remote there is no "RC LOST"
alarm, and Health shows **RC remote: off** in grey. If the phone leaves the Wi-Fi or the app is closed, the robot stops
driving by itself within ~6 s and keeps balancing: [safety](../docs/safety.md).

## Tour

| Tab | |
|---|---|
| **Control** | Arm / Disarm, mode buttons, joystick / WASD, Tilt on the phone |
| **Map** | tap waypoints, **Go**, **Explore** |
| **Sensors** | both range sensors, last sweep |
| **Tune** | live PID, **Find balance point**, **Calibrate odometry**, presets, motor test |
| **Plots** | any signal vs time |
| **Health** | green / amber / red **preflight** before you arm, battery **minutes left**, one tile per part |
| **Logs** | messages, **Falls**, downloads |

<p>
<img src="docs/screenshots/phone-drive.png" width="30%">
<img src="docs/screenshots/phone-map.png" width="30%">
<img src="docs/screenshots/phone-sensors.png" width="30%">
</p>
<p>Drive, Map, Sensors</p>
<p>
<img src="docs/screenshots/phone-tune.png" width="30%">
<img src="docs/screenshots/phone-plots.png" width="30%">
<img src="docs/screenshots/phone-logs.png" width="30%">
</p>
<p>Tune, Plots, Logs</p>

Places (saved spots on the map, **Go here**, Home):

<p>
<img src="docs/screenshots/phone-places.png" width="30%">
<img src="docs/screenshots/phone-place-sheet.png" width="30%">
<img src="docs/screenshots/phone-places-menu.png" width="30%">
</p>
<p>Places on the map, tap a place, the menu. <a href="../docs/station-features.md">Details</a></p>

Falls (black-box recorder):

<p>
<img src="docs/screenshots/phone-falls.png" width="30%">
<img src="docs/screenshots/phone-balpoint-proposal.png" width="30%">
<img src="docs/screenshots/phone-stale-stop.png" width="30%">
</p>
<p>Falls list, Find balance point, no-data badge</p>

Health (preflight checklist, battery minutes, tiles; it only looks, it never arms):

<p>
<img src="docs/screenshots/phone-health.png" width="30%">
<img src="docs/screenshots/phone-health-fault.png" width="30%">
</p>
<p>All green, an encoder fault</p>

More: [Android app](android/README.md) | [features in detail](../docs/station-features.md) | [API, install, flashing contract](../docs/api.md)