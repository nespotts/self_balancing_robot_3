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

`loop()`: new IMU sample → balance step → encoders (if due).
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
| cfg.save | — | set 1 → write EEPROM (when motors off) |
| cfg.defaults | — | set 1 → defaults (not saved) |

EEPROM block has a name-layout CRC + value CRC; any mismatch → defaults (boot log says so).

## Error codes (telemetry `error_code`)

0 ok · 1 IMU init · 2 IMU stale · 3 left encoder · 4 right encoder · 5 radio chip · 6 radio version

## Tuning: try first

1. `bal.d_src 0` (default): v3 maths, now on a fresh 400 Hz sample every step.
2. Then `bal.d_src 1` (gyro rate): same average D, less noise. If it feels damped differently, scale `bal.kd`.
3. Watch `bal_p / bal_i / bal_d` in TelemetryFrame with `tlm.hz 400`.
