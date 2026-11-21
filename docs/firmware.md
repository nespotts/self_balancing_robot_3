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
| Boot | IDLE; remote must show ARMED off → on; stays IDLE until the remote's mode selection **changes** (or Pi `SET_MODE`) |
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

```
 boot ─> first packet: remote shows MANUAL ─> robot learns it, stays IDLE   (remote: Idle>Manual)
         arm                               ─> still IDLE, motors off
         remote selection changes           ─> that mode (e.g. pick Idle, then Manual)
 dropout, reconnect, same selection         ─> mode kept (a Pi SET_MODE is not overwritten)
 dropout, selection changed meanwhile       ─> the new selection
```

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
| mot.curve_exp / knee_in / knee_out | 3.92 / 1250 / 2750 | dead-zone curve (v3), `mot.model 0` |
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
| mot.test | 0 | wheels-up motor test: 1 thresholds, 2 speed curve, 3 reversal, 4 all (see below) |
| mot.test_max | 4095 | highest duty the test uses |
| mot.model | 0 | 0 v3 curve, 1 volts model (per-wheel dead zone) |
| mot.vcomp | 0 | 1 = duty × v_ref / pack V (1.5 s filter, max 1.3×; off if the reading is bad) |
| mot.v_ref | 12 V | model 1: u = 1 → this many volts; vcomp reference |
| mot.dz_l_fwd / l_rev / r_fwd / r_rev | 2.0 V | model 1 dead zone per wheel + direction |
| mot.dz_band | 0.01 | model 1: below this \|u\| ramp 0 → dead zone (no kick on noise) |
| rng.idle_off | 1 | 1 = servo off while disarmed / E-stop (sweep only): head to centre, 0.4 s, then no pulses |
| mot.pwm_hz | 36621 | motor PWM frequency, 100–50000 Hz ² |
| mot.drive_mode | 0 | 0 PWM on EN (coast in the off-time), 1 PWM on IN (brake in the off-time) ² (see L298N drive) |

¹ applied between readings, only while the motors are off. ² applied only while both motors are off
(disarm / IDLE / fallen; log line `mot: drive_mode …, pwm … Hz`). Everything else applies at once.

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

## Error codes (telemetry `error_code`)

0 ok · 1 IMU init · 2 IMU stale · 3 left encoder · 4 right encoder · 5 radio chip · 6 radio version

## Tuning: try first

1. `bal.d_src 0` (default): v3 maths, now on a fresh 400 Hz sample every step.
2. Then `bal.d_src 1` (gyro rate): same average D, less noise. If it feels damped differently, scale `bal.kd`.
3. Watch `bal_p / bal_i / bal_d` in TelemetryFrame with `tlm.hz 400`.

## Motor test (wheels up) and motor correction

```
 u (controller, -1..1)
   mot.model 0:  duty = v3 curve(u)                       steep near 0, no real dead zone
   mot.model 1:  V = dz + (v_ref - dz)·|u|   duty = V / v_ref
                 dz per wheel + direction, in volts        ramp 0 → dz inside |u| < dz_band
   mot.vcomp 1:  duty × v_ref / V_pack                     same u = same motor volts at any charge
```

**Run it**

1. Robot on a stand, **both wheels off the ground**, upright (|pitch|, |roll| < 20°), not moving.
2. Mode **IDLE** (remote Mode menu), then **ARM** on the remote (the test needs the remote live). Balancing is off in IDLE.
3. Set `mot.test` (ground station: Tune → Motor test → Start test, or on the Pi `python3 tools/tune.py set mot.test=4`).
4. After 1 s still, it runs (test 4 ≈ 60–100 s). `mot.test` goes back to 0 when done.
5. The Motor test view shows the fit and an **Apply proposal** button; or on the Pi: `python3 tools/motor_fit.py` (newest session log); on the PC: `python tools/motor_fit.py --ssh robot`.

| mot.test | What | How |
|---|---|---|
| 1 | dead zone | each wheel: +10 duty / 50 ms until it moves (> 20 mm/s) = **start**, then −10 / 50 ms until it stops (< 8 mm/s) = **stop**; fwd, then rev |
| 2 | speed curve | both wheels: 16 duty steps to `mot.test_max`, 1.2 s each, mean speed over the last 0.5 s, step time constant; fwd, then rev |
| 3 | reversal | spin at (highest start + 300) duty (2000 if test 1 hasn't run), flip direction 6×; time to the turning point, time until 2° back, coast angle |
| 4 | all | 1, 2, 3 |

**Stops at once (motors 0, `mt,abort,<reason>`):** disarm, E-stop, mode ≠ IDLE, remote lost, IMU or encoder fault/stale,
|pitch| or |roll| > 20°, pitch rate > 150 °/s, `mot.test` changed, time limit. Not started within 60 s → dropped.

**Output** (Logs page / session log, `MSG_LOG`), volts = duty / 4095 × pack V at that moment:

```
mt,begin,<test>,max=<duty>,vb=<V>,pitch=<deg>,hz=<mot.pwm_hz>,drive=<mot.drive_mode>   (sum/end lines end with hz=,drive= too)
mt,a,<left|right>,<fwd|rev>,<start|stop>,<duty>,<volts>,<vbatt>,<speed mm/s>     (-1 = never moved)
mt,b,<wheel>,<dir>,<duty>,<volts>,<vbatt>,<speed mm/s>,<t63 ms>
mt,c,<wheel>,<new dir>,<rep>,<duty>,<volts>,<vbatt>,<to turning point ms>,<until 2° back ms>,<coast deg>
mt,sum,<a|b|c>,...      mt,end,<test>,ok,<s>,vb=<min>..<max>      mt,abort,<reason>,...
```

**Apply the fit:** motor_fit.py prints dead zones (fit V0 of speed vs volts, kept between stop and start), left/right
scale, how the v3 curve and the volts model map u → speed, and a `tune.py set mot.model=1 mot.vcomp=1 ...` line.
A/B live with `mot.model` 0 / 1; the volts model has a different gain near 0, so retune `bal.*` (the tool suggests a
starting factor). **Save to robot** keeps it. On a bench supply without the balance lead, vcomp does nothing (bad reading → 1×).

## L298N drive (issue #22)

```
 mot.drive_mode 0 (v3)                       mot.drive_mode 1
 EN  ┌┐ ┌┐ ┌┐  PWM                          EN  ────────── high
 IN1 ──────── high  (fwd)                    IN1 ┌┐ ┌┐ ┌┐  PWM  (fwd)
 IN2 ________ low                            IN2 ________ low
 off-time: bridge off = coast (fast decay)   off-time: IN1 = IN2 = low = brake (slow decay)
 command 0: coast                            command 0: brake (hold)
 disarmed / IDLE / E-stop / fault: EN low, IN1 = IN2 = low = free, in both modes
```

| | |
|---|---|
| Direction change | the IN pin going low switches to GPIO low at once, then the other IN starts its PWM: both INs never high together |
| Motor volts | ≈ duty × pack V in both modes; the linearity differs → run `mot.test=4` per config |
| Timers | pins 4 (L EN), 5, 6, 9 on FlexPWM2 SM0–2, pins 7 (R EN), 8 on FlexPWM1 SM3: all six set to `mot.pwm_hz`. Servo (pin 2) is FlexPWM4: not affected |
| Applies | only with both motors off; `motor_fit.py` groups runs by `drive:hz` (`--config latest` / `1:20000`, default all + comparison table) |

A/B, robot in IDLE on the stand (motors off, so it applies at once): `tune.py set mot.drive_mode=1 mot.pwm_hz=4000`,
armed, `mot.test=4`; repeat per config, then `motor_fit.py`.
