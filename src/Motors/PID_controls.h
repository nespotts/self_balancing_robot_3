#pragma once

#include <PID_v1.h>

Motor left_motor(4, 5, 6);
Motor right_motor(7, 8, 9);

Odometry odom;
IMU imu;

class PID_Manager {

private:
	uint32_t currenttime;  // micros
public:
	typedef struct {
		double kp;
		double ki;
		double kd;
		double input;
		double output;
		double lower_output_limit;
		double upper_output_limit;
		double setpoint;
		float rate;
	} PID_vars;

	PID_vars velocity{};
	PID_vars bal{};
	PID_vars steer{};

	PID VelocityPID = PID(&velocity.input, &velocity.output, &velocity.setpoint, velocity.kp, velocity.ki, velocity.kd, DIRECT);
	PID BalancePID = PID(&bal.input, &bal.output, &bal.setpoint, bal.kp, bal.ki, bal.kd, DIRECT);
	PID SteerPID = PID(&steer.input, &steer.output, &steer.setpoint, steer.kp, steer.ki, steer.kd, DIRECT);

	typedef struct {
		double leftoutput;
		double rightoutput;
		double min_power;
		double max_power;
		double power_range;
		float balance_cutoff_angle;
		double balance_angle;
		long startbalancetimer;
		int16_t pid_mode; // 0-balance only;  1-balance + steer;  2-balance, steer, and velocity
		int16_t pid_enable; // 0-off; 1-pid enabled
	} PID_control_vars;

	PID_control_vars control{};

	struct cmd_vel_type {
		double lin_vel;
		double ang_vel;
		double rate;
		long timer;
		long duration;
		double max_lin_vel;   // m/s
		double max_ang_vel;  // deg/s
	};

	cmd_vel_type cmd_vel;

	PID_Manager() {
		// constructor
	}

	void setup() {
		left_motor.config_speed_output_params();
		right_motor.config_speed_output_params();
		control.min_power = 0;
		control.max_power = 4095;
		control.power_range = control.max_power - control.min_power;
		control.balance_angle = 3.15;
		control.balance_cutoff_angle = 40;

		cmd_vel.max_lin_vel = 450; // mm/s
		cmd_vel.max_ang_vel = 180; // deg/s
		cmd_vel.rate = 50;
		cmd_vel.duration = 1000.0 / cmd_vel.rate * 1000.0;

		velocity_setup();
		balance_setup();
		steer_setup();
	}


	void run() {
		currenttime = micros();
		setTunings();
		Command_Velocity();

		if (control.pid_enable == 0) {
			left_motor.command_motor(0);
			right_motor.command_motor(0);
			return;
		}

		// Serial.print(bal.input);
		// Serial.print("\t");
		// Serial.print(bal.output);
		// Serial.print("\t");
		// Serial.print(bal.setpoint);
		// Serial.print("\t");
		// Serial.println(steer.output);
		// Serial.println(control.balance_angle);


		// Serial.println(imu.absolute_yaw);

		if (millis() - control.startbalancetimer >= 500) {
			// Don't start steering until balanced first
			control.leftoutput = (bal.output - steer.output) * 0.95;
			control.rightoutput = bal.output + steer.output;

			// reset i terms
			// BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);
			// SteerPID.SetTunings(steer.kp, steer.ki, steer.kd);
			VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
		} else {
			control.leftoutput = bal.output;
			control.rightoutput = bal.output;
		}

		// Send Motor Command 
		if (fabs(imu.ypr.pitch) < control.balance_cutoff_angle && fabs(imu.ypr.roll) < control.balance_cutoff_angle) {


			balance_run();
			if (control.pid_mode == 1 || control.pid_mode == 2) {
				steer_run();
			}
			if (control.pid_mode == 2) {
				velocity_run();
			}

			left_motor.command_motor(control.leftoutput);
			right_motor.command_motor(control.rightoutput);
		} else {
			// anti-windup
			// BalancePID.SetTunings(bal.kp, 0, bal.kd);
			// SteerPID.SetTunings(steer.kp, 0, steer.kd);
			VelocityPID.SetTunings(0, 0, 0);

			left_motor.command_motor(0);
			right_motor.command_motor(0);
			imu.reset_absolute_yaw();
			steer.input = imu.absolute_yaw;
			steer.setpoint = steer.input;
			velocity.input = velocity.setpoint;

			// odom.pose.angle_rad = imu.absolute_yaw * DEG_TO_RAD;
			control.startbalancetimer = millis();
		}
	}


	void balance_setup() {
		bal.kp = 250;//202;//202;//175;//203;//214;// 262; //262//105; //110; //130
		bal.ki = 2010;//2010;//2010;//2000;//1118;//1119; //1067;// 1675; //2503; //2507; //2750
		bal.kd = 4.63;//5.70;//10.00;//10.20;//10.42;//11.34; //11.73;//10.6; //5.7; //6.8;  //6.5
		bal.rate = 400;
		bal.setpoint = control.balance_angle;
		bal.lower_output_limit = -control.power_range;
		bal.upper_output_limit = control.power_range;

		BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);
		BalancePID.SetMode(AUTOMATIC);
		BalancePID.SetOutputLimits(bal.lower_output_limit, bal.upper_output_limit);
		BalancePID.SetSampleTime(1000.0 / bal.rate);
	}

	void steer_setup() {
		// setup steering PID - balancing
		steer.kp = 20.0;
		steer.ki = 5.9;
		steer.kd = 0.075;
		steer.rate = 200;
		// steer.input = imu.absolute_yaw;
		steer.setpoint = imu.absolute_yaw;
		steer.lower_output_limit = -control.power_range;
		steer.upper_output_limit = control.power_range;

		SteerPID.SetTunings(steer.kp, steer.ki, steer.kd);
		SteerPID.SetMode(AUTOMATIC);
		SteerPID.SetOutputLimits(steer.lower_output_limit, steer.upper_output_limit);
		SteerPID.SetSampleTime(1000.0 / steer.rate);
	}


	void velocity_setup() {
		velocity.kp = 0.0060;//0.001;//0.002;//0.01;//10.0;
		velocity.ki = 0.00869;//0.0005;//0.019;//0.002;// 4.7;
		velocity.kd = 0.00007;//0.000023;//0.0006;//0.0002;//0.02;
		velocity.rate = 100;
		velocity.lower_output_limit = -20; // angle in degrees
		velocity.upper_output_limit = 20; // angle in degrees

		VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
		VelocityPID.SetMode(AUTOMATIC);
		VelocityPID.SetOutputLimits(velocity.lower_output_limit, velocity.upper_output_limit); 
		VelocityPID.SetSampleTime(1000.0 / velocity.rate);
	}



	// 1. command starts here
	void Command_Velocity() {
		// apply remote data
		control.balance_angle = map((float)radio.receive_data.left_knob, 0, 1023, -15, 15);
		control.pid_mode = radio.receive_data.SWC;
		control.pid_enable = radio.receive_data.SWA;
		cmd_vel.lin_vel = radio.receive_data.elevator;
		cmd_vel.ang_vel = radio.receive_data.aileron;

		if (currenttime - cmd_vel.timer >= cmd_vel.duration) {
			// balancing version
			velocity.setpoint = map(cmd_vel.lin_vel, 0, 1023, -cmd_vel.max_lin_vel, cmd_vel.max_lin_vel);


			if (fabs(imu.ypr.pitch) < control.balance_cutoff_angle && fabs(imu.ypr.roll) < control.balance_cutoff_angle) {
				// add deadband for ang_velocity
				if (cmd_vel.ang_vel <= 525 && cmd_vel.ang_vel >= 496) {
					steer.setpoint -= 0;
				} else {
					steer.setpoint -= map(cmd_vel.ang_vel, 0, 1023, -cmd_vel.max_ang_vel * (cmd_vel.duration / 1000000.0), cmd_vel.max_ang_vel * (cmd_vel.duration / 1000000.0));
				}
			}

			cmd_vel.timer = currenttime;
		}
	}

	// 2. velocity from odom is added
	void velocity_run() {
		velocity.input = odom.cent.velocity;
		VelocityPID.Compute();
	}

	void balance_run() {
		bal.setpoint = control.balance_angle - velocity.output;
		bal.input = -imu.ypr.pitch;
		BalancePID.Compute();
	}

	void steer_run() {
		steer.input = imu.absolute_yaw;
		SteerPID.Compute();
	}

	void setTunings() {
		char inByte[10];
		int index = 0;

		while (Serial.available() > 0) {
			inByte[0] = Serial.read();
			inByte[1] = '\0';
			// Serial.print(inByte);
			if (strcmp(inByte, "q") == 0) {
				bal.kp *= 1.01;
			} else if (strcmp(inByte, "a") == 0) {
				bal.kp *= 0.99;
			} else if (strcmp(inByte, "w") == 0) {
				bal.ki *= 1.01;
			} else if (strcmp(inByte, "s") == 0) {
				bal.ki *= 0.99;
			} else if (strcmp(inByte, "e") == 0) {
				bal.kd *= 1.01;
			} else if (strcmp(inByte, "d") == 0) {
				bal.kd *= 0.99;
			} else if (strcmp(inByte, "r") == 0) {
				bal.setpoint += 0.1;
			} else if (strcmp(inByte, "f") == 0) {
				bal.setpoint -= 0.1;
			} else if (strcmp(inByte, "g") == 0) {
				velocity.kp *= 0.99;
			} else if (strcmp(inByte, "t") == 0) {
				velocity.kp *= 1.01;
			} else if (strcmp(inByte, "h") == 0) {
				velocity.ki *= 0.99;
			} else if (strcmp(inByte, "y") == 0) {
				velocity.ki *= 1.01;
			} else if (strcmp(inByte, "j") == 0) {
				velocity.kd *= 0.99;
			} else if (strcmp(inByte, "u") == 0) {
				velocity.kd *= 1.01;
			}

			Serial.print(bal.kp, 8); Serial.print("\t"); Serial.print(bal.ki, 8);
			Serial.print("\t"); Serial.print(bal.kd, 8); Serial.print("\t"); Serial.print(bal.setpoint, 2);
			Serial.print("\t"); Serial.print(velocity.kp, 8); Serial.print("\t"); Serial.print(velocity.ki, 8);
			Serial.print("\t"); Serial.println(velocity.kd, 8);

			BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);
			SteerPID.SetTunings(steer.kp, steer.ki, steer.kd);
			VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
			index++;
		}
	}
};






