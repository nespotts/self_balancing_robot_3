#pragma once

#include "Odometry/encoder.h"

class Wheel {
private:
	double wheel_diameter = 66.0; // mm
	double wheel_radius;
	// flag flips every time a measurement update is complete
	bool update_flag;
public:
	MagneticEncoder *encoder;

	double circumference;
	double linear_distance;
	double linear_velocity;
	double linear_acceleration;

	Wheel(MagneticEncoder *p_encoder) {
		encoder = p_encoder;
		wheel_radius = wheel_diameter / 2.0;
		circumference = PI * wheel_diameter;
	}


	void setup() {
		encoder->setup();
	}

	void run() {
		encoder->run();
		// use boolean flag change to indicate time to update
		if (update_flag != encoder->update_flag) {
			update_flag = !update_flag;
			linear_distance = encoder->angle_rad * wheel_radius;
			linear_velocity = encoder->angular_velocity_rad * wheel_radius;
			linear_acceleration = encoder->angular_acceleration_rad * wheel_radius;
		}
	}
};