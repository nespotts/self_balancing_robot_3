#include "PWMServo.h"
#include "lidar.h"


typedef struct {
  PWMServo* pservo;  // pointer to servo object
  int servo_pin;      // servo signal pin
  float subdivisions;   // number of subdivisions of 180 deg travel
  float increment_deg;  // subdivision degrees
  long us_per_deg;     // servo rotate time per degree
  long travel_time;   // time allowed for servo travel in us
  int ranges[100];         // array containing all laser readings front and back in mm
  int positions[100];        // array containing servo positions in degrees
  int front_index;          // position to record front range
  int rear_index;       // position to record rear range
  int direction;       // direction of servo travel
  long servo_start_time;   // time servo start moving
  int state;        // 0 start ranging, 1 waiting on range, 2 servo move, 3 waiting on servo
} scanList;

PWMServo servo;

scanList scan{ &servo, 5, 32, 0, 2500, 0, {}, {}, 0, 0, 0, 0, 0 };

void scan_setup(int servo_pin = 5, int subdivisions = 16) {
  Lidar_setup(34, 35, 28, 29);
  scan.subdivisions = subdivisions;
  scan.servo_pin = servo_pin;
  servo.attach(scan.servo_pin);
  scan.increment_deg = 180.0 / scan.subdivisions;
  scan.travel_time = scan.us_per_deg * scan.increment_deg;

  for (int i = 0; i < (2 * (int)scan.subdivisions); i++) {
    scan.positions[i] = i * scan.increment_deg;
    // Serial.println(scan.positions[i]);
  }
  scan.state = 0;
}


void scan_run() {
  uint32_t currenttime = micros();
  lidar_update();
  // 0 start ranging
  if (scan.state == 0) {
    lidar.start_ranging = true;
    scan.state = 1;

    // waiting on range
  } else if (scan.state == 1) {
    if (lidar.ranging_complete) {
      // save scan results
      scan.ranges[scan.front_index] = sensors[0].range;
      scan.ranges[scan.rear_index] = sensors[1].range;
      scan.state = 2;

      // set index for back and forth movement
      if (scan.front_index == (scan.subdivisions - 1)) {
        scan.direction = -1;
      } else if (scan.front_index == 0) {
        scan.direction = 1;
      }
      scan.front_index += scan.direction;
      scan.rear_index = scan.front_index + scan.subdivisions;

      // Serial.println(scan.ranges[scan.front_index]);
    }

    // start servo move
  } else if (scan.state == 2) {
    scan.pservo->write(scan.positions[scan.front_index]);
    // scan.pservo->write(scan.positions[0]);
    scan.servo_start_time = currenttime;
    scan.state = 3;

    // waiting on servo travel
  } else if (scan.state == 3) {
    if ((currenttime - scan.servo_start_time) >= scan.travel_time) {
      scan.state = 0;
    }
  }
}