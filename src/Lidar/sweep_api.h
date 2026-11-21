// Entry points for the VL53L0X + servo sweep (Lidar_scan.h / lidar.h, unchanged from v3).
// The sweep itself is issue #7; for now only setup runs at boot, as in v3.
#pragma once

void lidar_sweep_setup();
void lidar_sweep_run();
