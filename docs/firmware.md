# Firmware v4 (Teensy 4.1)

## Modules

```
 BNO08x ──SPI──> imu ──400 Hz──> controller ──> motors ──> H-bridges
 (INT 30)         │  pitch,rate     ▲   ▲
                  │  heading        │   │ speed
 AS5600 x2 ─I2C─> odometry ─200 Hz──┘   │
                                        │ target speed/turn, enable
 nRF24 <──SPI──> remote_radio ──> mode_manager <── pi_link <──USB──> Orange Pi
   ▲  ack = TelemetryPacket          (arm, E-stop,      │   TelemetryFrame, params, logs
   └──────────────────────────────    failsafe)         └── params (EEPROM)
```

`loop()`: new IMU sample → balance step → encoders (if due) → range sweep slice (≤ ¼ step).
No sample → radio, Pi link, telemetry, battery, button. Nothing blocks the balance step.

## Rates

| What | Rate |
|---|---|
| Balance + heading loop | every IMU sample, `ctl.rate_hz` (400) |
| Velocity loop | 100 Hz |
| Encoders | 200 Hz (speed over the last 80 ms) |
| Remote | 50 Hz in, telemetry in each ACK |
| TelemetryFrame to Pi | `tlm.hz` (100; set ≥ 400 to get every balance step) |

## Modes and safety

| Condition | Result |
|---|---|
| ESTOP flag | motors off now, latched until a packet without it; then ARMED must go off → on |
| Not armed / IMU stale 25 ms | IDLE, motors off |
| Boot | IDLE; remote must show ARMED off → on |
| No remote, Pi sends SET_MODE ≠ IDLE | Pi arms (SET_MODE IDLE disarms) |
| Remote present | remote's ARMED/ESTOP rule over the Pi |
| Remote silent 250 ms | speed/turn ramp to 0, keeps balancing |
| Mode change | commanded motion → 0, heading target reset |
| Tilt > `bal.cutoff_deg` | motors off, integrators reset |
| Back within `bal.reengage_deg` of balance | balance again; steer + velocity after `ctl.settle_ms` |
| Encoder fault | velocity loop off (balance + heading only) |

| Mode | Drives from |
|---|---|
| IDLE | motors off |
| BALANCE, GOTO, EXPLORE, HOLD | stand still (nav = #8) |
| MANUAL | remote sticks, if packets < 250 ms old |
| STATION | `MSG_CMD_VEL`, if heartbeat < 500 ms and cmd < 300 ms |

Mode requests act when they change: a remote mode-switch change, or each Pi `SET_MODE`.
The remote's selection also applies when it (re)connects.

## Params (Pi link `MSG_SET_PARAM`, live)

Gains are br3ttb units: ki per second, kd seconds.

| Name | Default | Meaning |
|---|---|---|
| bal.kp / ki / kd | 250 / 2010 / 4.63 | balance PID (power per deg) |
| bal.offset_deg | 3.15 | balance point = pitch −3.15° |
| bal.cutoff_deg | 45 | fallen |
| bal.reengage_deg | 15 | re-engage window |
| bal.knob_trim_deg | 0 | ± trim from left knob (0 = off) |
| bal.d_src | 0 | D from: 0 d(pitch)/dt (v3), 1 gyro |
| str.kp / ki / kd | 20 / 5.9 / 0.075 | heading PID |
| str.max_err_deg | 45 | heading target leash |
| vel.kp / ki / kd | 0.006 / 0.00869 / 0.00007 | speed → lean |
| vel.limit_deg | 20 | max lean from velocity loop |
| drv.max_speed / max_turn | 450 mm/s / 180 °/s | full stick |
| drv.accel / turn_accel | 1500 / 1000 | ramp per second |
| drv.turn_deadband | 2 °/s | |
| mot.left_scale / right_scale | 0.95 / 1.0 | per-side |
| mot.max | 4095 | output limit |
| mot.curve_exp / knee_in / knee_out | 3.92 / 1250 / 2750 | dead-zone curve (v3) |
| ctl.rate_hz | 400 | IMU + balance rate (applies with motors off) |
| ctl.loops | 3 | 1 heading, 2 velocity, 3 both, 0 balance only |
| ctl.settle_ms | 500 | |
| batt.low_cell / adc_per_v | 3.4 V / 73.3 | |
| tlm.hz | 100 | |
| rng.enable | 0 | 1 = range sweep on (starts at boot, or live when motors are off); 0 = off, servo limp |
| cfg.save | — | set 1 → write EEPROM (when motors off) |
| cfg.defaults | — | set 1 → defaults (not saved) |
| rng.servo_trim_deg | 0 | ± added to every servo command: **horn fix** (see Range sweep) |
| rng.sweep_deg | 170 | head travel (inside the SG90's ~180°); two sensors → 360° less two 10° gaps |
| rng.steps | 16 | positions per pass (170/15 = 11.3° apart) |
| rng.settle_ms | 40 | servo wait per step (bigger moves wait in proportion) |
| rng.timing_budget_ms | 33 | per reading (20–200, longer = steadier) ¹ |
| rng.long_range | 1 | 1 = long-range mode (to ~2 m) ¹ |
| rng.max_mm | 2000 | farther → `valid` 0 |
| rng.servo_hold_deg | −1 | −1 = sweep; 0–180 = park the servo there (90 = centre) |
| rng.obstacle_mm | 300 | TLM_OBSTACLE when front < this while driving forward (0 = off) |
| rng.servo_dir | 1 | −1 if the scan comes out mirrored |
| rng.servo_min_deg / max_deg | 5 / 175 | servo command limits (keep off the end stops) |
| rng.servo_deg_per_cmd | 1.0 | real head degrees per commanded degree (measured travel ÷ command span) |

¹ applied between readings, only while the motors are off. Everything else applies at once.

EEPROM block has a name-layout CRC + value CRC; any mismatch → defaults (boot log says so).

## IMU (BNO08x on SPI) gotchas

| Problem | Fix (`src/imu.cpp`, `lib/sh2`) |
|---|---|
| nRF24 = SPI mode 0, BNO = mode 3; Teensy 4 flips SCK idle level on the first clocked byte → 1-bit shift | dummy byte with no CS before/after each IMU transfer |
| Advertisement is 276 B, but arrived as 512 B when shifted; SH-2 buffer was 384 | vendored SH-2 in `lib/sh2`, buffers 1024 |
| No WAKE pin: chip ignores writes unless it is asserting INT | writes ride on a chip transfer (full duplex); idle → reset chip, send on its boot transfer after the init response |
| `sh2_getProdIds` has no timeout | not used |

Boot: ~1 s until samples. Status line in the log 3 s after boot and on IMU health change.

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
suggested `servo_trim_deg` and `servo_deg_per_cmd` (fit of the readings to a flat wall).

**3.** `rng.servo_hold_deg` −1, `rng.sweep_deg` 170, `rng.steps` 16, `cfg.save`.

## Error codes (telemetry `error_code`)

0 ok · 1 IMU init · 2 IMU stale · 3 left encoder · 4 right encoder · 5 radio chip · 6 radio version

## Tuning: try first

1. `bal.d_src 0` (default): v3 maths, now on a fresh 400 Hz sample every step.
2. Then `bal.d_src 1` (gyro rate): same average D, less noise. If it feels damped differently, scale `bal.kd`.
3. Watch `bal_p / bal_i / bal_d` in TelemetryFrame with `tlm.hz 400`.
