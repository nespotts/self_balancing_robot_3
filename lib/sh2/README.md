# sh2 (vendored)

CEVA/Hillcrest SH-2 + SHTP driver sources, copied from Adafruit BNO08x 1.2.5 (`src/sh2*.c/h`, `shtp*`),
Apache 2.0 (see NOTICE.txt and the file headers).

Only change: `sh2_hal.h` receive buffers 384 → 1024 bytes. Our BNO08x sends its advertisement as a
512-byte transfer after reset; with 384 it was dropped, no channels were mapped and nothing worked.

The SPI HAL (non-blocking, INT-driven) is in `src/imu.cpp`.
