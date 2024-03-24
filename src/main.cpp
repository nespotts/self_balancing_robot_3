#include <Arduino.h>
#include "Wire.h"
#include "battery_monitor.h"
BatteryMonitor battery;
#include "Helpers/reboot.h"
// #include "Beeper/beeper_controller.h"
// BalanceBeeper beeper;
#include "Helpers/helper_3dmath.h"
#include "Helpers/helper_functions.h"
#include "Lidar/Lidar_scan.h"
#include "NRF_radio.h"
NRF_Radio radio;
#include "Odometry/Odometry.h"
#include "IMU/IMU.h"
#include "Motors/motor_controls.h"
#include "Motors/PID_controls.h"
PID_Manager pids;
// #include "control_logic.h"
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
	SPI.begin();
	Wire.begin();
	Wire1.begin();
	odom.setup();
	imu.setup();
	// servo pin, num subdivisions
	scan_setup(2, 16);
	radio.setup(3);
	pids.setup();

	battery.setup();
	p.Begin();
	p.AddXYGraph("Test", 100000, "Right Motor Speed", left_motor.motor_speed_input, "Right Angular Velocity", odom.left_encoder.angular_velocity_deg);
	p.AddXYGraph("Test", 100000, "speed input", left_motor.motor_speed_input, "speed output", left_motor.motor_speed_output);
	// p.AddXYGraph("Test", 5000, "X", odom.cent.x, "Y", odom.cent.y);
	// p.AddTimeGraph("Test", 5000, "test", x, "test2", y);
	// p.AddTimeGraph("IMU", 5000, "Absolute Yaw 1", imu.absolute_yaw, "yaw", imu.ypr.yaw);
	// p.AddTimeGraph("PIDs", 5000, "left motor", pids.control.leftoutput, "right motor", pids.control.rightoutput);
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

uint32_t plot_timer;
uint32_t interval = 50;
uint32_t cycles_past = 0;
float loop_rate;

void loop() {
	cycles_past++;
	reset.run();
	// left_motor.exp_factor = (float)receive_data.left_knob / 50.0;
	// left_motor.interval = (float)receive_data.right_knob / 100.0;

	// left_motor.config_motor_run();
	odom.run();
	imu.run();
	// scan_run();
	radio.run();
	pids.run();
	battery.run();
	// beeper.loop();

	if (millis() - plot_timer >= interval) {
		loop_rate = (float)cycles_past / (((float)millis() - (float)plot_timer) / 1000.0);
		cycles_past = 0;
		Serial.println(loop_rate);

		// control plot interval to a manageable rate
		// p.Plot();
		// Serial.print(left_motor.motor_speed_input); 
		// Serial.print("\t");
		// Serial.print(left_motor.motor_speed_output);
		// Serial.print("\t");
		// Serial.print(left_motor.exp_factor);
		// Serial.println();
		plot_timer = millis();
	}
}















// TEST CODE TO SHOW IMU BLOCKING ISSUE

// #include <Arduino.h>
// #include <Adafruit_BNO08x.h>

// // For SPI mode, we need a CS pin
// #define BNO08X_CS 10
// #define BNO08X_INT 30
// #define BNO08X_RESET 33


// Adafruit_BNO08x  bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;


// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 5000;

// void setReports(sh2_SensorId_t reportType, long report_interval) {
// 	Serial.println("Setting desired reports");
// 	if (!bno08x.enableReport(reportType, report_interval)) {
// 		Serial.println("Could not enable stabilized remote vector");
// 	}
// }

// void setup(void) {

// 	Serial.begin(115200);
// 	while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

// 	Serial.println("Adafruit BNO08x test!");

// 	// Try to initialize!
// 	if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
// 		Serial.println("Failed to find BNO08x chip");
// 		while (1) { delay(10); }
// 	}
// 	Serial.println("BNO08x Found!");

// 	setReports(reportType, reportIntervalUs);

// 	Serial.println("Reading events");
// 	delay(100);
// }


// uint32_t timer;
// uint32_t interval = 50;
// uint32_t cycles_past = 0;
// double loop_rate;

// void loop() {
// 	cycles_past++;

// 	if (bno08x.wasReset()) {
// 		Serial.print("sensor was reset ");
// 		setReports(reportType, reportIntervalUs);
// 	}

// 	// comment this line out for inscreased loop_rate
// 	// if it is uncommented, the loop_rate exactly matches the event rate
// 	bno08x.getSensorEvent(&sensorValue);

// 	if (micros() - timer >= interval) {
// 		loop_rate = (double)cycles_past / (((double)micros() - (double)timer) / 1000000.0);
// 		cycles_past = 0;
// 		Serial.println(loop_rate);
// 		timer = micros();
// 	}
// }
