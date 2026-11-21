// The v3 lidar headers define globals, so they are included in this one file only.
#include <Arduino.h>

#include "Lidar/Lidar_scan.h"
#include "Lidar/sweep_api.h"

void lidar_sweep_setup() { scan_setup(2, 16); }  // servo pin 2, 16 steps per 180 deg
void lidar_sweep_run() { scan_run(); }
