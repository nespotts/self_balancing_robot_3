class Automation {
private:


public:
	bool reached_position, reached_theta;
	double deltaX, deltaY, target_theta, delta_theta, dist_remaining, absolute_target_theta;


	void setup() {

	}


	void run() {
		if (radio.receive_data.SWA == 2) {
			command_position(odom.cent.target_x, odom.cent.target_y, 180);

		} else if (radio.receive_data.SWA == 0) {
			// reset target position
			reached_position = false;
			reached_theta = false;
			odom.cent.target_x = odom.cent.x;
			odom.cent.target_y = odom.cent.y;
		}
	}


	// 
	void command_position(float x, float y, float ending_theta) {
		// determine path to get there
		// interpolate waypoints along the way to get there
		// use straight line for now

		// 1. determine angle and remaining distance
		deltaY = y - odom.cent.y;
		deltaX = x - odom.cent.x;
		target_theta = atan2f(deltaY, deltaX) * RAD_TO_DEG;  // -180 to 180, 0 east
		dist_remaining = pow(pow(deltaX, 2) + pow(deltaY, 2), (0.5));  // mm from pythagorean's theorem

		// 2. determine direction to rotate and move to angle - velocity proportional to error
		// get direction to go
		// get delta theta (error)
		if (!reached_position) {
			delta_theta = determine_delta_theta(target_theta);
			float ang_vel = map(delta_theta, -180, 180, -pids.cmd_vel.max_ang_vel * (pids.cmd_vel.duration / 1000000.0), pids.cmd_vel.max_ang_vel * (pids.cmd_vel.duration / 1000000.0));
			if (ang_vel > pids.cmd_vel.max_ang_vel) {
				ang_vel = pids.cmd_vel.max_ang_vel;
			} else if (ang_vel < -pids.cmd_vel.max_ang_vel) {
				ang_vel = -pids.cmd_vel.max_ang_vel;
			}
			pids.cmd_vel.ang_vel = ang_vel;
		}

		// 3. move ahead with speed proportional to remaining distance once close to target_theta
		if (!reached_position) {
			float lin_vel = map(fabs(dist_remaining), 0, 1500, 0, pids.cmd_vel.max_lin_vel);
			if (lin_vel > pids.cmd_vel.max_lin_vel) {
				lin_vel = pids.cmd_vel.max_lin_vel;
			}
			if (fabs(delta_theta) < 2.0) {
				pids.cmd_vel.lin_vel = lin_vel;
			} else {
				pids.cmd_vel.lin_vel = 0;
			}
		}

		// 4. stop moving
		if (!reached_position && fabs(dist_remaining) < 20 && fabs(odom.cent.velocity) < 2 && fabs(odom.cent.acceleration) < 25 && fabs(odom.pose.angular_velocity_deg) < 2) {
			reached_position = true;
			pids.cmd_vel.lin_vel = 0;
		}

		// 5. face in desired direction
		if (reached_position && !reached_theta) {
			// need to transalate ending_theta to nearest angle
			delta_theta = determine_delta_theta(ending_theta);
			float ang_vel = map(delta_theta, -180, 180, -pids.cmd_vel.max_ang_vel * (pids.cmd_vel.duration / 1000000.0), pids.cmd_vel.max_ang_vel * (pids.cmd_vel.duration / 1000000.0));
			if (ang_vel > pids.cmd_vel.max_ang_vel) {
				ang_vel = pids.cmd_vel.max_ang_vel;
			} else if (ang_vel < -pids.cmd_vel.max_ang_vel) {
				ang_vel = -pids.cmd_vel.max_ang_vel;
			}
			pids.cmd_vel.ang_vel = ang_vel;
			
			if (fabs(delta_theta) < 3 && fabs(odom.pose.angular_velocity_deg) < 10) {
				reached_theta = true;
				pids.cmd_vel.ang_vel = 0;
				pids.steer.setpoint = absolute_target_theta;
			}
		}

	}


	float determine_delta_theta(float target_theta) {
		// target theta -180 to 180, 0 east
		float output = 0;

		// get target theta on same revolution as absolute theta
		if (target_theta >= 0) {
			absolute_target_theta = (float)imu.revs * 360.0 + target_theta;
		} else {
			absolute_target_theta = (float)imu.revs * 360.0 + 360.0 + target_theta;
		}

		// figure out which direction is shortest
		float clockwise_degrees = absolute_target_theta - imu.absolute_yaw;
		float counterclockwise_degrees = (absolute_target_theta - 360.0) - imu.absolute_yaw;
		if (fabs(clockwise_degrees) < fabs(counterclockwise_degrees)) {
			output = clockwise_degrees;
		} else {
			output = counterclockwise_degrees;
		}
		return output;
	}


	// write method to command pose
	// this will be called to interpolate through intermediate angles
	void command_pose() {

	}


	// write method to command travel distance
	// this will be called to interpolate through intermediate distances
	void command_dist() {

	}




	// write a method that can set target angle (+/- 90 deg)
	// have property that indicates when the target is achieved (over time)
	// target_angle - relative to current_angle
	// 
	void turn(float target_angle, int radius) {

	}


	// write a method to set target linear distance
	// have a property that indicates when the target is achieved (over time)

};