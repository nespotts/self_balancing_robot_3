#include "AS5600.h"


class MagneticEncoder {
private:
	// position variables
	double last_angle_deg;

	// velocity vars
	DoubleList ang_vel = DoubleList(20);
	double last_angular_vel;

	// acceleration vars
	DoubleList ang_acc = DoubleList(20);

	int16_t dir_pin;
	bool debug;

	uint16_t rate;  // Hz  (updates / second)

	uint32_t update_interval;  // (micros between each update)
	uint32_t update_timer;

	// uint32_t velocity_interval;  // (micros between each update)
	// uint32_t velocity_timer;

	// uint32_t acc_interval;  // (micros between each update)
	// uint32_t acc_timer;

	AS5600 as5600 = AS5600(&Wire);

	void calculate_update_interval() {
		update_interval = 1000000.0 / (float)rate;
		// velocity_interval = (double)update_interval * 1.0;
		// acc_interval = (double)update_interval * 1.0;

		// Serial.println(velocity_interval);
		// Serial.println(acc_interval);
	}

	void update_position() {
		// cum position assumes at least 4 readings per revolution
		counts = as5600.getCumulativePosition();
		angle_deg = counts * AS5600_RAW_TO_DEGREES;
		revolutions = angle_deg / 360.0;
	}

	void update_velocity(double seconds_past) {
		// calculate angular velocity myself
		// double current_ang_vel_deg = (angle_deg - last_angle_deg) / seconds_past;
		// last_angle_deg = angle_deg;

		// or use library to calculate angular velocity - may be more accurate because it does not rely o nthe cumulative position fx
		double current_ang_vel_deg = as5600.getAngularSpeed(0);

		// add current_ang_vel_deg to list
		if (ang_vel.length < ang_vel.max_length) {
			ang_vel.append(current_ang_vel_deg);
		} else {
			ang_vel.remove(0);
			ang_vel.append(current_ang_vel_deg);
		}

		// ang_vel.print();
		angular_velocity_deg = ang_vel.average();
		rpm = (angular_velocity_deg / 360.0) * 60.0;
	}

	void update_acceleration(double seconds_past) {
		// add current_ang_acc_deg to list
		double current_ang_acc_deg = (angular_velocity_deg - last_angular_vel) / seconds_past;
		last_angular_vel = angular_velocity_deg;

		if (ang_acc.length < ang_acc.max_length) {
			ang_acc.append(current_ang_acc_deg);
		} else {
			ang_acc.remove(0);
			ang_acc.append(current_ang_acc_deg);
		}
		angular_acceleration_deg = ang_acc.average();
	}


public:
	// position variables
	int32_t counts; // counts 4096 is one rev
	double angle_deg; // degrees
	double angle_rad; //
	double revolutions;

	double angular_velocity_deg;  // deg / sec
	double rpm;  // revolutions / minute

	double angular_acceleration_deg; // deg / sec2


	MagneticEncoder(int p_dir_pin, int p_rate = 100, bool p_debug = false) {
		dir_pin = p_dir_pin;
		rate = p_rate;
		debug = p_debug;
	}

	void resetPosition() {
		as5600.resetPosition();
	}

	void setup() {
		// calculate update interval
		calculate_update_interval();
		as5600.begin(dir_pin);  //  set direction pin.
		as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.

		if (debug) {
			Serial.print("AS5600 Address: ");
			Serial.println(as5600.getAddress());
		}

		int b = as5600.isConnected();
		if (debug) {
			Serial.print("Connect: ");
			Serial.println(b);
		}
	}

	void run() {
		uint32_t currentmicros = micros();

		if (currentmicros - update_timer >= update_interval) {
			double seconds_past = ((double)(currentmicros - update_timer) / 1000000.0);

			update_position();

			update_velocity(seconds_past);

			update_acceleration(seconds_past);

			update_timer = currentmicros;

			if (debug) {
				Serial.print(counts);
				Serial.print("\t");
				Serial.print(angle_deg);
				Serial.print("\t");
				Serial.print(angular_velocity_deg);
				Serial.print("\t");
				// Serial.print(rpm);
				// Serial.print("\t");
				Serial.print(angular_acceleration_deg);
				Serial.print("\t");
				Serial.println(revolutions);
			}
		}
	}
};



//  -- END OF FILE --