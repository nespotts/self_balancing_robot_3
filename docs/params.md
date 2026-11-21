## Params (Pi link `MSG_SET_PARAM`, live)

Gains are br3ttb units: ki per second, kd seconds.

| Name | Default | Meaning |
|---|---|---|
| bal.kp / ki / kd | 250 / 2010 / 4.63 | balance PID (power per deg) |
| bal.offset_deg | 3.15 | balance point = pitch −3.15° (Station: Tune, Find balance point, measures it from the velocity loop) |
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
| cfg.defaults | — | set 1 → defaults, not saved (when motors off) |
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
| mot.test | 0 | wheels-up motor test: 1 thresholds, 2 speed curve, 3 reversal, 4 = 1+2+3, 5 tracking open loop, 6 tracking closed loop, 7 = 5+6 (see below) |
| mot.test_max | 4095 | highest duty the test uses |
| mot.model | 0 | 0 v3 curve, 1 volts model (per-wheel dead zone) |
| mot.vcomp | 0 | 1 = duty × v_ref / pack V (1.5 s filter, max 1.3×; off if the reading is bad) |
| mot.v_ref | 12 V | model 1: u = 1 → this many volts; vcomp reference |
| mot.dz_l_fwd / l_rev / r_fwd / r_rev | 2.0 V | model 1 dead zone per wheel + direction |
| mot.dz_band | 0.01 | model 1: below this \|u\| ramp 0 → dead zone (no kick on noise) |
| mot.bk_l_fwd / l_rev / r_fwd / r_rev | 2.46 / 2.11 / 2.73 / 2.49 V | model 1 stiction kick: breakaway volts of a stopped wheel (motor test **start**; see Stiction kick in motors.md) |
| mot.bk_gain | 1 | kick volts × this (0 = off, max 1.5) |
| mot.bk_ms | 80 | longest kick, ms (ends earlier once the wheel turns), then a 40 ms fade to the dead-zone model |
| mot.bk_band | 0.03 | no kick for \|u\| ≤ this (no buzz on noise) |
| mot.bk_mms | 15 | wheel speed ≥ this = turning (ends the kick); below it for 20 ms = stopped |
| rng.idle_off | 1 | 1 = servo off while disarmed / E-stop (sweep only): head to centre, 0.4 s, then no pulses |
| mot.pwm_hz | 36621 | motor PWM frequency, 100–50000 Hz ² |
| mot.drive_mode | 0 | 0 PWM on EN (coast in the off-time), 1 PWM on IN (brake in the off-time) ², **mode 1 only at pwm_hz ≤ brake_max_hz and with mot.model 1** (see L298N drive) |
| mot.test_wheels | 0 | motor test drives 0 both, 1 left only, 2 right only (the other: EN low, INs low) |
| mot.brake_max_hz | 2000 | drive_mode 1 refused above this mot.pwm_hz (mode 0 used, logged; the test aborts) |
| mot.brake_duty_max | 3000 | drive_mode 1 duty cap: motor test (with mot.test_max) **and balancing**; 4095 once the brake heat checks are done |
| mot.speed_loop | 0 | 1 = closed-loop wheel speed motor layer (experimental, see Wheel speed loop) |
| mot.sl_kp / sl_ki / sl_imax / sl_band | 0.006 / 0.08 / 3 / 0.3 | speed loop: V per mm/s, V per mm, integrator limit V, dead-zone taper V |
| mot.k_l / k_r | 200 | mm/s per V above the dead zone (motor_fit.py `k`); speed loop + tracking tests |
| nav.max_speed / max_turn | 250 mm/s / 90 °/s | GOTO / EXPLORE limits (also ≤ drv.*, × remote speed limit) |
| nav.speed_kp | 0.6 /s | speed = kp × distance … |
| nav.min_speed | 40 mm/s | … but at least this (dead zone), also the slowest obstacle slow-down |
| nav.turn_kp | 2 °/s per ° | turn = kp × bearing error |
| nav.align_deg | 10 | drive when the error < this; turn in place again above 3× |
| nav.arrive_mm | 50 | at the waypoint: stop |
| nav.stop_speed / stop_turn_dps | 50 mm/s / 20 °/s | "stopped" … |
| nav.dwell_ms | 500 | … this long → next waypoint |
| nav.end_heading_deg | −1 | ≥ 0: turn to this heading at the last waypoint (pose frame, 0 = +X) |
| nav.heading_tol_deg | 3 | end heading reached |
| nav.slow_mm | 800 | obstacle closer → slow down (linear to `rng.obstacle_mm`; 0 = off) |
| nav.clear_mm | 100 | blocked until nothing inside `rng.obstacle_mm` + this (+ backoff) for `nav.obst_age_ms` |
| nav.corridor_mm | 220 | width checked ahead / behind (with `nav.side_min_mm` on: at most the body width + 20, the rest is the side check) |
| nav.obst_age_ms | 1500 | range points kept this long |
| nav.floor_mm | 50 | ignore hits lower than this (floor, when leaning) |
| nav.backoff_mm | 0 | when blocked, back off this far (rear checked too) |
| nav.hold_kp / hold_max_speed / hold_db_mm | 2 /s / 150 mm/s / 15 mm | HOLD position loop |
| nav.blind_speed | 150 mm/s | GOTO speed cap with no range readings for 3 s (`rng.enable 0`); 0 = GOTO pauses too. EXPLORE always pauses |
| mot.left_dir | 1 | Left motor wiring: -1 if its wires are swapped (fixed in software) |
| mot.right_dir | 1 | Right motor wiring: -1 if its wires are swapped |
| nav.side_min_mm | 120 | wall closer than this to the body side (from the last ~1.5 s of range points): steer away + slow down; 0 = off (#62) |
| nav.side_kp | 0.5 °/s per mm | how hard: turn = kp × mm inside `side_min_mm` |
| nav.stuck_ms | 2500 | driving / turning but no progress (`stuck_mm`, or 15°) this long → BLOCKED, log `nav: stuck`, back off; 0 = off |
| nav.stuck_mm | 40 | progress needed within `stuck_ms` |
| nav.stuck_backoff_mm | 100 | stuck: back off this far (rear checked); also the most it steps away before turning in place next to a wall |

¹ applied between readings, only while the motors are off. ² applied only while both motors are off
(disarm / IDLE / fallen; log line `mot: drive_mode …, pwm … Hz`). Everything else applies at once.

EEPROM block has a name-layout CRC + value CRC; any mismatch → defaults (boot log says so).

Back: [firmware index](firmware.md).
