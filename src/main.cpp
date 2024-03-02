#include <Arduino.h>
#include "helper_functions.h"
#include "Wire.h"
#include <Adafruit_BNO08x.h>

#include "motor_controls.h"
Motor motor(3, 4, 5);

#include "encoder.h"
MagneticEncoder enc1(6, 500, false);

#include "mpu6050.h"
// IMU imu;

#include "Plotter.h"
Plotter p;


void setup() {
	Serial.begin(115200);
	Wire.begin();

	enc1.setup();

	motor.setup();
	pinMode(A7, INPUT);

	// imu.setup();
  int x = 1;
  int y = 2;

	p.Begin();
  p.AddTimeGraph("Test", 5000, "test", x, "test2", y);
	// p.AddTimeGraph("Encoder", 5000, "Angular Position", enc1.angle_deg, "Angular Velocity", enc1.angular_velocity_deg, "Angular Acceleration", enc1.angular_acceleration_deg);
	// p.AddTimeGraph("IMU Orientation", 10000, "Yaw", imu.ypr[0], "Pitch", imu.ypr[1], "Roll", imu.ypr[2]);
	// p.AddTimeGraph("IMU Linear Acceleration", 10000, "X", imu.aaReal.x, "Y", imu.aaReal.y, "Z", imu.aaReal.z);
	// p.AddTimeGraph("IMU Angular Velocity", 10000, "X", imu.angular_velocity.x, "Y", imu.angular_velocity.y, "Z", imu.angular_velocity.z);
	// p.AddTimeGraph("IMU Angular Acceleration", 10000, "X", imu.angular_acceleration.x, "Y", imu.angular_acceleration.y, "Z", imu.angular_acceleration.z);
	// p.AddTimeGraph("IMU Angular Velocity Comparison", 10000, "X gyro", imu.angular_velocity.x, "X calculated", imu.angular_velocity.x);
	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
}


void loop() {
	p.Plot();
	// enc1.run();
	// imu.run();

	// int pot_val = analogRead(A7);

	// int pwm_val = map(pot_val, 0, 1023, -255, 255);
	// motor.command_motor(pwm_val);
}


