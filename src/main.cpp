#include <Arduino.h>
#include "Wire.h"
#include "battery_monitor.h"
BatteryMonitor bm;
#include "Helpers/reboot.h"
// #include "Beeper/beeper_controller.h"
// BalanceBeeper beeper;
#include "Helpers/helper_3dmath.h"
#include "Helpers/helper_functions.h"
#include "Lidar/Lidar_scan.h"
#include "Odometry/Odometry.h"
#include "IMU/IMU.h"
#include "Motors/motor_controls.h"
#include "Motors/PID_controls.h"
PID_Manager pids;
// #include "control_logic.h"
#include "NRF_radio.h"
#include "functions.h"
#include "Plotter.h"

// #include <jled.h>
// // breathe LED for 5 times, LED is connected to pin 9 (PWM capable) gpio
// // can't use pin 13 because that is SCK for SPI 0
// // auto led = JLed(13).Breathe(2000).Forever();

Plotter p;
Reset reset;


void setup() {
	Serial.begin(115200);
	Wire.begin();
	Wire1.begin();
	odom.setup();
	imu.setup();
	// servo pin, num subdivisions
	scan_setup(2, 16);
	Radio_Setup(3);
	pids.setup();
	bm.setup();
	// motor.setup();
	// beeper.setup();
	p.Begin();
	// p.AddXYGraph("Test", 5000, "X", odom.cent.x, "Y", odom.cent.y);
	// p.AddTimeGraph("Test", 5000, "test", x, "test2", y);
	// p.AddTimeGraph("IMU", 5000, "Absolute Yaw 1", imu.absolute_yaw, "yaw", imu.ypr.yaw);
	p.AddTimeGraph("PIDs", 5000, "left motor", pids.control.leftoutput, "right motor", pids.control.rightoutput);
	// p.AddTimeGraph("Angular Velocity", 5000, "Velocity", odom.cent.total_distance);
	// p.AddTimeGraph("Angular Velocity", 5000, "Velocity", odom.pose.inst_angle_rad);
	// p.AddTimeGraph("Left Encoder Deg", 2500, "Position", odom.left_wheel.encoder->angle_deg, "Velocity", odom.left_wheel.encoder->angular_velocity_deg, "Acceleration", odom.left_wheel.encoder->angular_acceleration_deg);
	// p.AddTimeGraph("Left Encoder", 5000, "Distance", odom.left_wheel.linear_distance, "Velocity", odom.left_wheel.linear_velocity, "Acceleration", odom.left_wheel.linear_acceleration);
	// p.AddTimeGraph("Left Encoder Rate", 5000, "Rate", odom.left_wheel.encoder->actual_rate);
	// p.AddTimeGraph("Left Encoder Rate", 5000, "Rate", odom.pose.theta_rad);
	// p.AddTimeGraph("Left Encoder", 5000, "Angular Position", odom.wheel.enc1.angle_deg, "Angular Velocity", odom.wheel.enc1.angular_velocity_deg, "Angular Acceleration", odom.wheel.enc1.angular_acceleration_deg);
	// p.AddTimeGraph("Right Encoder", 5000, "Angular Position", odom.right_encoder.angle_rad);
	// p.AddTimeGraph("IMU Orientation", 5000, "Yaw", imu.ypr.yaw, "Pitch", imu.ypr.pitch, "Roll", imu.ypr.roll);
	// p.AddTimeGraph("IMU Linear Acceleration", 5000, "z", imu.lin_acc.z, "y", imu.lin_acc.y, "x", imu.lin_acc.x);
	// p.AddTimeGraph("IMU Angular Velocity", 5000, "z", imu.ang_vel_deg.z, "y", imu.ang_vel_deg.y, "x", imu.ang_vel_deg.x);
	// p.AddTimeGraph("IMU Angular Acceleration", 5000, "z", imu.angular_acceleration_deg.z, "y", imu.angular_acceleration_deg.y, "x", imu.angular_acceleration_deg.x);
	// p.AddTimeGraph("IMU Rates", 5000, "Pose", imu.pose_rate, "Linear Acceleration", imu.lin_acc_rate, "Angular Velocity", imu.ang_vel_rate);
}

uint32_t print_timer;
uint32_t interval = 20;


void loop() {
	reset.run();
	// p.Plot();
	odom.run();
	imu.run();
	// scan_run();
	Receive_Data();
	pids.run();
	bm.run();
	// beeper.loop();

	if (millis() - print_timer >= interval) {
		print_timer = millis();

	}
}