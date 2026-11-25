# Internals: IMU, radio, errors

## IMU (BNO08x on SPI) gotchas

| Problem | Fix (`src/imu.cpp`, `lib/sh2`) |
|---|---|
| nRF24 = SPI mode 0, BNO = mode 3; Teensy 4 flips SCK idle level on the first clocked byte → 1-bit shift | dummy byte with no CS before/after each IMU transfer |
| Advertisement is 276 B, but arrived as 512 B when shifted; SH-2 buffer was 384 | vendored SH-2 in `lib/sh2`, buffers 1024 |
| No WAKE pin: chip ignores writes unless it is asserting INT | writes ride on a chip transfer (full duplex); idle → reset chip, send on its boot transfer after the init response |
| `sh2_getProdIds` has no timeout | not used |

Boot: ~1 s until samples. Status line in the log 3 s after boot and on IMU health change.

## Radio stats (remote issue #3)

Every 10 s while packets arrive (and once when they stop), `MSG_LOG`:
`radio: 50.0 pkt/s, gaps>100ms 0 (max 32 ms), bad 0, multi 0, ack fail 0, poll max 61 us`

| Field | Meaning |
|---|---|
| gaps>100ms | packets that came > 100 ms after the previous one (failsafe trips at 250) |
| bad | wrong size / version / corrupt |
| multi | polls that found > 1 packet waiting (background loop late) |
| ack fail | TX FIFO full when loading the ack (flushed + reloaded) |
| poll max | longest poll / ack load, µs (the balance loop is never blocked longer) |

The ack payload is reloaded after every poll that read anything (also invalid packets), stale acks flushed.

## Error codes (telemetry `error_code`)

0 ok · 1 IMU init · 2 IMU stale · 3 left encoder · 4 right encoder · 5 radio chip · 6 radio version

Only the first one shows. Link v9 `fault_flags` shows every encoder fault (+ recent I2C trouble, watchdog off): see
[safety.md](safety.md#encoder-fault).

## Tuning: try first

1. `bal.d_src 0` (default): v3 maths, now on a fresh 400 Hz sample every step.
2. Then `bal.d_src 1` (gyro rate): same average D, less noise. If it feels damped differently, scale `bal.kd`.
3. Watch `bal_p / bal_i / bal_d` in TelemetryFrame with `tlm.hz 400`.

Back: [firmware index](firmware.md).
