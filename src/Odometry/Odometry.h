#pragma once

#include "Odometry/wheel.h"

class Odometry {
private:
	bool update_flag = false;

	double delta_theta;
public:
	// should probably move these declaratioions into the Wheel class.  Works for now.
	MagneticEncoder left_encoder = MagneticEncoder(38, 0, false, 250, false);
	MagneticEncoder right_encoder = MagneticEncoder(39, 1, true, 250, false);

	Wheel left_wheel = Wheel(&left_encoder);
	Wheel right_wheel = Wheel(&right_encoder);

	// update rate
	uint16_t rate;


	Odometry(uint16_t p_rate = 500) {
		rate = p_rate;
	}

	double wheel_separation = 123.0; //mm

	typedef struct {
		double inst_distance;
		double total_distance;
		double velocity; // mm/s
		double acceleration;
		double x;       // x position delta
		double y;       // y position delta
	} centroid;

	centroid cent;

	typedef struct {
		double inst_angle_rad;
		double angle_rad; // angle about z axis in rads
		double angle_deg; // angle about z axis in degs
		double angular_velocity_rad;
		double angular_velocity_deg;
		double angular_acceleration_rad;
		double angular_acceleration_deg;
	} Pose;

	Pose pose;

	// double delta;      // last time differential


	void setup() {
		left_wheel.setup();
		right_wheel.setup();
	}

	void run() {
		left_wheel.run();
		right_wheel.run();

		calculate_odometry();
	}

	void calculate_odometry() {
		// pick one side as the basis to update from
		if (update_flag != left_wheel.update_flag) {
			update_flag = !update_flag;

			pose.inst_angle_rad = (right_wheel.inst_distance - left_wheel.inst_distance) / wheel_separation; // rad

			pose.angular_velocity_rad = (right_wheel.linear_velocity - left_wheel.linear_velocity) / wheel_separation;
			pose.angular_velocity_deg = pose.angular_velocity_rad * RAD_TO_DEG;
			
			pose.angular_acceleration_rad = (right_wheel.linear_acceleration - left_wheel.linear_acceleration) / wheel_separation;
			pose.angular_acceleration_deg = pose.angular_acceleration_rad * RAD_TO_DEG;

			cent.inst_distance = (left_wheel.inst_distance + right_wheel.inst_distance) / 2.0;
			cent.total_distance = (left_wheel.linear_distance + right_wheel.linear_distance) / 2.0;
			cent.velocity = (left_wheel.linear_velocity + right_wheel.linear_velocity) / 2.0;
			cent.acceleration = (left_wheel.linear_acceleration + right_wheel.linear_acceleration) / 2.0;

			// using odometry angle
			// cent.x += cent.inst_distance * cos(pose.angle_rad + pose.inst_angle_rad / 2.0);
			// cent.y += cent.inst_distance * sin(pose.angle_rad + pose.inst_angle_rad / 2.0);

			// using IMU yaw angle
			cent.x += cent.inst_distance * cos((imu.pose.z * DEG_TO_RAD) +  pose.inst_angle_rad / 2.0);
			cent.y += cent.inst_distance * sin((imu.pose.z * DEG_TO_RAD) +  pose.inst_angle_rad / 2.0);
			
			pose.angle_rad += pose.inst_angle_rad;
			pose.angle_deg = pose.angle_rad * RAD_TO_DEG;
		}
	}
};