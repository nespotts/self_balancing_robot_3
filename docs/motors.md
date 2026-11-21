# Motors: test, L298N drive, wheel speed loop

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
| 5 | tracking, open loop | target wheel speed profile (below) with model 0 (`ol0`: u = target / top speed, v3 curve), then model 1 (`ol1`: V = dz + target / k, vcomp) |
| 6 | tracking, closed loop | same profile with the wheel speed loop (`cl`, `mot.sl_*`) |
| 7 | 5 + 6 | |

Tracking profile (per selected wheel, 1 s off between segments): `r+` 0→300→0 mm/s (3 s ramps, 1 s hold), `r-` the same
backwards, `s0` ±50 mm/s sine at 1 Hz, `s150` 150 ± 50 mm/s sine, `st` steps 150 → 250 → 150 mm/s. Uses `mot.k_l/k_r`,
`mot.dz_*`, `mot.v_ref`: set them from motor_fit.py first. Speed for scoring: 20 ms encoder window.

`mot.test_wheels 2` = right wheel only (left motor unplugged / suspect). With a wheel selected that can't turn the test
aborts `stall`. Bench supply without the balance lead (pack reading < 6 V): set `mot.v_ref` to the supply voltage, the
test uses it for volts (`vbsrc=v_ref` in the begin line, no sag check).

**Stops at once (motors 0, `mt,abort,<reason>`):** disarm, E-stop, mode ≠ IDLE, remote lost, IMU or encoder fault/stale,
|pitch| or |roll| > 20°, pitch rate > 150 °/s, `mot.test` changed, time limit. Not started within 60 s → dropped.
**Anomaly aborts** (binding / heat / supply, issue #26):

| reason | when |
|---|---|
| `stall,<wheel>` | wheel < 8 mm/s for 1 s while its volts are above start threshold (this run's test 1 start + 1 V, else max(8.5 V, dz + 2 V)) |
| `speed_drop,<wheel>` | at a fixed duty the speed falls > 30 % below the step's first value for 0.15 s (test 2: each level; test 3: each spin vs the first one that way) |
| `supply_sag` | pack reading > 1.5 V below its start value for 0.2 s |
| `drive_time` | one stretch of continuous drive > 60 s (drive mode 1: 25 s) |
| `brake_hz_above_max` | drive mode 1 asked for above `mot.brake_max_hz` (never drives) |

Drive mode 1 runs: duty capped at `mot.brake_duty_max`, motors off 60 s between test parts, 10 s between directions,
5 s between tracking segments (`mt,cool,...`).

**Output** (Logs page / session log, `MSG_LOG`), volts = duty / 4095 × pack V at that moment:

```
mt,begin,<test>,max=<duty>,vb=<V>,pitch=<deg>,hz=<mot.pwm_hz>,drive=<mot.drive_mode>   (sum/end lines end with hz=,drive= too)
mt,a,<left|right>,<fwd|rev>,<start|stop>,<duty>,<volts>,<vbatt>,<speed mm/s>     (-1 = never moved)
mt,ad,<wheel>,<dir>,<duty>,<volts>,<vbatt>,<speed mm/s>       test 1 ramp down while still turning (every 60 duty)
mt,b,<wheel>,<dir>,<duty>,<volts>,<vbatt>,<speed mm/s>,<t63 ms>
mt,c,<wheel>,<new dir>,<rep>,<duty>,<volts>,<vbatt>,<to turning point ms>,<until 2° back ms>,<coast deg>
mt,sum,<a|b|c>,...      mt,end,<test>,ok,<s>,vb=<min>..<max>      mt,abort,<reason>,...
mt,lcfg,<ol0|ol1|cl>,<wheel>,k=,dz=fwd/rev,vref=,vb=,kp=,ki=,imax=,band=,hz=,drive=
mt,l,<ctl>,<wheel>,<seg>,rms=,mean=,max=,start=,stop=,t63u=,t63d=,flips=<n>/s,dvrms=<V>,...   (mm/s, ms; -1 = n/a)
   start / stop = target speed when the wheel starts / stops on the ramps; flips = output sign changes; dvrms = output chatter
mt,tr,<ctl>,<l|r>,<seg>,<t ms>,<target>,<speed>,<volts>                                       (every 50 ms)
mt,sum,l,<ctl>,<wheel>,rms r+= r-= s0= s150= st=,t63 up= down=
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
| Applies | only with both motors off; `motor_fit.py` groups runs by `drive:hz` (`--config latest` / `1:2000`, default all + comparison table) |
| Brake limit | **mode 1 only at `mot.pwm_hz` ≤ `mot.brake_max_hz` (2000)**; above it mode 0 is used and the log says `drive_mode 1 refused`. The 36.6 kHz brake-mode smoke (#22/#23) was the binding left motor, but high-frequency brake decay also heats the L298N more |
| Brake + model | **mode 1 only with `mot.model 1`** (brake + v3 curve is far too hot near 0, motor-profile.md); else mode 0, log `drive_mode 1 refused: needs mot.model 1`. Set `mot.model=1` first. A live change to model 0 keeps the volts model until the next stop |
| Brake duty | balancing in mode 1: duty ≤ `mot.brake_duty_max` (3000) |

A/B, robot in IDLE on the stand (motors off, so it applies at once): `tune.py set mot.drive_mode=1 mot.pwm_hz=1000`,
armed, `mot.test=4`; repeat per config, then `motor_fit.py`. Results: [motor-profile.md](motor-profile.md).

## Wheel speed loop (mot.speed_loop, experimental, issue #26)

```
 balance u ──> target = u · k · (v_ref − dz) mm/s ──>(+)── e ──> PI ──(+)── x ──> dead-zone inverse ──> V ──> duty = V / V_pack
                                                     ↑−              target / k ─┘ (feed-forward)    x>0: dz_fwd + x
                                     encoder speed (20 ms window, 200 Hz)                             |x|<band: taper
```

Off by default. Anti-windup: integrator clamped to ±`mot.sl_imax` V and frozen while the output is at the limit.
Without fresh encoder reads (> 30 ms) it falls back to `mot.model`. Target 0 with the wheel stopped → 0 V (no hum).
Try it wheels-up first with `mot.test=6`, then retune `bal.*` (the loop changes the plant: u is a speed, not a voltage).
