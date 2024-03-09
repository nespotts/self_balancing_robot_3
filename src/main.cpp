#include <Arduino.h>
#include "helper_3dmath.h"
#include "helper_functions.h"
#include "Wire.h"
#include "IMU.h"
#include "Lidar_scan.h"
#include "encoder.h"
// #include "Odometry.h"
#include "motor_controls.h"
// #include "PID_controls.h"
// #include "battery_monitor.h"
#include "NRF_radio.h"
// #include "beeper_controller.h"
// #include "control_logic.h"
#include "functions.h"
#include "reboot.h"
#include "Plotter.h"

// #include <jled.h>
// // breathe LED for 5 times, LED is connected to pin 9 (PWM capable) gpio
// // can't use pin 13 because that is SCK for SPI 0
// // auto led = JLed(13).Breathe(2000).Forever();


Motor left_motor(4, 5, 6);
Motor right_motor(7, 8, 9);


MagneticEncoder enc1(38, 0, 500, false);
MagneticEncoder enc2(39, 1, 500, false);


IMU imu;

Plotter p;

int x = 1;
int y = 2;


Reset reset;




void setup() {
	Serial.begin(115200);
	Wire.begin();
	Wire1.begin();

	enc1.setup();
	enc2.setup();

	imu.setup();

	// servo pin, num subdivisions
	scan_setup(2, 16);

	Radio_Setup(3);

	// motor.setup();
	// pinMode(A7, INPUT);


	// p.Begin();
	// p.AddTimeGraph("Test", 5000, "test", x, "test2", y);
	// p.AddTimeGraph("Encoder", 5000, "Angular Position", enc1.angle_deg, "Angular Velocity", enc1.angular_velocity_deg, "Angular Acceleration", enc1.angular_acceleration_deg);
	// p.AddTimeGraph("IMU Orientation", 10000, "Yaw", imu.ypr[0], "Pitch", imu.ypr[1], "Roll", imu.ypr[2]);
	// p.AddTimeGraph("IMU Linear Acceleration", 10000, "X", imu.aaReal.x, "Y", imu.aaReal.y, "Z", imu.aaReal.z);
	// p.AddTimeGraph("IMU Angular Velocity", 10000, "X", imu.angular_velocity.x, "Y", imu.angular_velocity.y, "Z", imu.angular_velocity.z);
	// p.AddTimeGraph("IMU Angular Acceleration", 10000, "X", imu.angular_acceleration.x, "Y", imu.angular_acceleration.y, "Z", imu.angular_acceleration.z);
	// p.AddTimeGraph("IMU Angular Velocity Comparison", 10000, "X gyro", imu.angular_velocity.x, "X calculated", imu.angular_velocity.x);
	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
	// Serial.println("test1,test2");
}

uint32_t print_timer;
uint32_t interval = 50;


void loop() {
	reset.run();
	// p.Plot();

	enc1.run();
	enc2.run();
	imu.run();


	// scan_run();

	Receive_Data();

	if (millis() - print_timer >= interval) {
		print_timer = millis();

		// Serial.print(imu.data.Pitch());
		// Serial.println(imu.data.Pitch());
		// serial_printer("Pitch: ", String(imu.data.Pitch()), " Roll: ", String(imu.data.Roll()), "Yaw", String(imu.data.Yaw()));
		// serial_printer("angular velocity x: ", String(imu.data.ang_vel.x), "\ty: ", String(imu.data.ang_vel.y), "\t z: ", String(imu.data.ang_vel.z));
		serial_printer("linear acceleration x: ", String(imu.data.lin_acc.x), "\t y: ", String(imu.data.lin_acc.y), "\t z: ", String(imu.data.lin_acc.z));
	}

	// int left_command = (receive_data.throttle - 512 + receive_data.rudder - 512)*(2047.0/512.0);
	// if (left_command > 0) {
	// 	left_command += 2047;
	// } else {
	// 	left_command -= 2047;
	// }
	// int right_command = (receive_data.throttle - 512 - (receive_data.rudder - 512))*(2047.0/512.0);
	// if (right_command > 0) {
	// 	right_command += 2047;
	// } else {
	// 	right_command -= 2047;
	// }

	// left_motor.command_motor(left_command);
	// right_motor.command_motor(right_command);

	// Serial.println(imu.Pitch());

}