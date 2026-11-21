# Range sweep and navigation

## Range sweep (issue #7)

```
            +X = forward (pitch + = leaning this way, wheels roll this way at + speed)
                 ^  sensor 0 at head angle 0
                 |
      +90 (left) |  head angle + = CCW (seen from above)
           <-----O----->  -90 (right)        O = servo axis, 26 mm ahead of the axle
                 |                             sensors 12.9 mm from it, 189 mm above the axle
                 v  sensor 1 = head angle + 180
```

| | |
|---|---|
| Servo command | `90 + trim + servo_dir × head_angle ÷ deg_per_cmd`, clamped to `servo_min_deg … servo_max_deg` |
| Sweep | head −85 … +85 and back (16 steps); sensor 1 covers the other half → 32 points / 360° |
| Hold | `rng.servo_hold_deg` = servo command (trim still added); head angle = `servo_dir × deg_per_cmd × (hold − 90)` |
| Idle off | disarmed / E-stop + `hold` −1 + `rng.idle_off` 1 → centre, settle 0.4 s, no pulses (quiet); still ranges at 0° / 180°; arm → pulses back at centre, sweep resumes (hold ≥ 0 always holds) |
| Reported angle | from the command actually sent (after clamping), so clipped positions are still reported truly |
| RangeFrame | every reading: pose + pitch when it finished, `sensor_angle_deg` = head angle (+180 for sensor 1), `valid` 0 = no target / > `rng.max_mm` / timeout |
| Radio telemetry | `range_front_mm` / `range_rear_mm` = latest reading within half a step of 0° / 180° (0 = none for 2 s) |
| Rate | ~80 ms per position (settle 40 + budget 33 + I2C) → 25 readings/s measured, 360° every ~1.3 s; 50/s when parked |

Never blocks the balance loop: after each balance step it spends at most a quarter of the step
period on I2C (single-shot ranging with raw register writes, ready from the INT pins, fallback polling).
The status line `rng: … readings/s … max slice … us` comes 5 s after start, then every 30 s.

### Aligning the head (motors off, robot on its stand)

**1. Travel** (`trim` 0): `rng.servo_hold_deg` = 5 (= `servo_min_deg`), mark where sensor 0 points;
then 175 (= `servo_max_deg`), mark again. Measure the angle between the marks (protractor / phone
compass app on top of the head). `rng.servo_deg_per_cmd` = angle ÷ 170.
Hold 175 must turn sensor 0 to the robot's **left**; if it goes right, `rng.servo_dir` = −1.
If the servo buzzes at either end, move `servo_min_deg` / `servo_max_deg` in a few degrees.

**2. Zero**: `rng.servo_hold_deg` = 90. Sensor 0 (XSHUT 34) must point straight ahead, along the wheels'
rolling direction. If it points X° left (CCW): `rng.servo_trim_deg` −= X ÷ deg_per_cmd (with `servo_dir` 1;
right → +=). Repeat until straight.

**Or by range, both at once**: robot square to a flat wall 30–60 cm ahead, `rng.sweep_deg` 90,
`rng.steps` 19, wait 20 s, then on the Pi `python3 tools/range_align.py --seconds 15` →
suggested `servo_trim_deg` (fit of the readings to a flat wall). Its `servo_deg_per_cmd` is only rough:
the sensor's ~25° cone flattens the curve, so measure the travel by eye (step 1).

**3.** `rng.servo_hold_deg` −1, `rng.sweep_deg` 170, `rng.steps` 16, `cfg.save`.

## Navigation (issue #8, `src/navigator.*`)

Waypoints (pose frame): remote MARK_WAYPOINT, or the Pi's `MSG_GOTO` (the planner sends `replace = 1`).
GOTO and EXPLORE follow the list the same way; HOLD holds the spot it started at.

```
           new list / mode GOTO, EXPLORE
                     │
                     v          |err| < align              dist < arrive_mm
 ┌────────────> TURNING ─────────────────> DRIVING ─────────────────────> (stop) ──┐
 │  next point   (turn in place)  <───────  speed = kp·dist                        │ stopped
 │                               |err| > 3·align  │  ▲                              │ dwell_ms
 │                                       obstacle │  │ GOTO: clear again            │
 │                              < rng.obstacle_mm v  │ EXPLORE: new MSG_GOTO list   v
 │                                             BLOCKED (+ backoff_mm)          more points?
 └──────────────────────────────────────────────────────────────── yes ───────────┤
                                                                                 no│
                          end_heading_deg ≥ 0: TURNING to it ──> ARRIVED <─────────┘

 HOLD:  HOLDING  speed = hold_kp × forward error (deadband hold_db_mm), BLOCKED if an obstacle is in the way
```

| | |
|---|---|
| Obstacles | every valid range reading → a world point (kept `nav.obst_age_ms`); points in a `nav.corridor_mm` strip ahead / behind give the front / rear distance as a sensor range |
| Floor | lean tips the head; a hit lower than `nav.floor_mm` above the floor is ignored |
| Slow-down | `nav.slow_mm` → `rng.obstacle_mm`: speed limit falls linearly (≥ `nav.min_speed`) |
| Side walls (#62) | points beside the body (body rear … 300 mm ahead) give left / right gaps from the body side. Gap < `nav.side_min_mm`: steer away (`nav.side_kp`), slow down (linear to `nav.min_speed` at 0). The corridor is then at most body width + 20 mm: a wall alongside or a door jamb steers, it does not stop. GOTO blocked with a wall alongside: turns away from it in place (≤ 30°) |
| Turning in place | corners sweep r 88 mm. Something ahead / behind within 105 mm → first step straight away (≤ `nav.stuck_backoff_mm`). Target behind and a wall close at one side → turn the nose away from that wall |
| Stuck (#62) | driving (not in the last 3 × `arrive_mm`) or turning, but < `nav.stuck_mm` / 15° progress for `nav.stuck_ms` → BLOCKED, back off `nav.stuck_backoff_mm` (at 2 × `nav.min_speed`, ≤ 5 s). GOTO then goes on; EXPLORE waits for the planner (it replans 5 s later with +100 mm margin) |
| Applies to | GOTO, EXPLORE, HOLD, STATION. MANUAL: `TLM_OBSTACLE` only |
| No sweep | `rng.enable 0` (no RangeFrame for 3 s) → no obstacle stop → **EXPLORE paused**, **GOTO ≤ `nav.blind_speed`** (log `nav: no range readings - obstacle stop is off: …`) |
| Pose reset | clears the list, points and hold point |
| GO_HOME | list = [first waypoint, or the origin], mode GOTO (never arms) |
| NavStatusFrame | at once on a change of state / index / count, else 2 Hz in GOTO/EXPLORE/HOLD; `obstacle_mm` = blocking range |
| Log | `nav: blocked, obstacle at 287 mm (waypoint 2/2)`, `nav: clear …`, `nav: reached 1/3, 12 mm off`, `nav: arrived`, `nav: stuck, no progress driving for 2.5 s (waypoint 1/2)`, `nav: too close to turn (92 mm), stepping back` |

Simulator: `station/sim/sim.py` `Navigator` is a line-by-line port (same params, states, logs);
tests `station/sim/tests/test_nav.py`.

Back: [firmware index](firmware.md).
