#include "Helpers/helper_functions.h"
#include "Helpers/helper_3dmath.h"
#include <Adafruit_BNO08x.h>

class IMU {
public:
	IMU() {}

	// orientation
	int orientation_status = 0;  // 0 - 3, 3 being most reliable
	Quaternion pose_quaternion;
	VectorFloat pose;  // in degrees
	struct euler_t {
		float yaw;
		float pitch;
		float roll;
	} ypr;

	// angular velocity
	VectorFloat ang_vel;   // rad/s
	VectorFloat ang_vel_deg;   // rad/s
	VectorFloat last_angular_velocity;
	// angular acceleration
	VectorFloat inst_ang_acc;
	ImuComponent angular_acceleration = ImuComponent(10, 10);  // rad/s^2
	VectorFloat angular_acceleration_deg;


	// linear acceleration
	VectorFloat lin_acc;

	// rates
	float pose_rate;
	float ang_vel_rate;
	float lin_acc_rate;


	void setup(void) {
		// Try to initialize!
		if (!bno08x.begin_SPI(CS_pin, INT_pin)) {
			if (debug) Serial.println("Failed to find BNO08x chip");
		}
		if (debug) Serial.println("BNO08x Found!");

		setReports();
		delay(10);
	}


	void run() {
		if (bno08x.wasReset()) {
			if (debug) Serial.print("sensor was reset ");
			setReports();
		}
		getSensorEvent();
	}


private:
	int CS_pin = 10;
	int INT_pin = 30;
	int RESET_pin = 33;
	bool fast_mode = true;
	bool debug = false;

	uint32_t last_pose_time;
	float seconds_past;
	uint32_t last_ang_time;
	uint32_t last_acc_time;

	Adafruit_BNO08x bno08x = Adafruit_BNO08x(RESET_pin);
	sh2_SensorValue_t sensorValue;

	void setReports() {
		if (debug) Serial.println("Setting desired reports");
		if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
			Serial.println("Could not enable rotation vector");
		}
		if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
			Serial.println("Could not enable gyroscope");
		}
		if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
			Serial.println("Could not enable linear acceleration");
		}
	}

	void getSensorEvent() {
		if (!bno08x.getSensorEvent(&sensorValue)) {
			return;
		}

		if (debug) { Serial.print(sensorValue.status); Serial.print("\t"); }

		switch (sensorValue.sensorId) {
		case SH2_GYROSCOPE_CALIBRATED:
			// angular velocity
			seconds_past = ((double)micros() - (double)last_ang_time) / 1000000.0;
			ang_vel.x = sensorValue.un.gyroscope.x;
			ang_vel.y = sensorValue.un.gyroscope.y;
			ang_vel.z = sensorValue.un.gyroscope.z;

			ang_vel_deg.x = ang_vel.x * RAD_TO_DEG;
			ang_vel_deg.y = ang_vel.y * RAD_TO_DEG;
			ang_vel_deg.y = ang_vel.y * RAD_TO_DEG;

			calculate_angular_acceleration();

			ang_vel_rate = 1000000.0 / (micros() - last_ang_time);
			last_ang_time = micros();
			break;
		case SH2_LINEAR_ACCELERATION:
			// linear acceleration
			lin_acc.x = sensorValue.un.linearAcceleration.x;
			lin_acc.y = sensorValue.un.linearAcceleration.y;
			lin_acc.z = sensorValue.un.linearAcceleration.z;

			lin_acc_rate = 1000000.0 / (micros() - last_acc_time);
			last_acc_time = micros();
			break;
		case SH2_ROTATION_VECTOR:
			// orientation
			orientation_status = sensorValue.status;
			quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);

			pose_quaternion.w = sensorValue.un.rotationVector.real;
			pose_quaternion.x = sensorValue.un.rotationVector.i;
			pose_quaternion.y = sensorValue.un.rotationVector.j;
			pose_quaternion.z = sensorValue.un.rotationVector.k;

			pose.x = ypr.roll;
			pose.y = ypr.pitch;
			pose.z = ypr.yaw;

			pose_rate = 1000000.0 / (micros() - last_pose_time);
			last_pose_time = micros();
			break;
		}
	}


	void calculate_angular_acceleration() {
		inst_ang_acc.x = (ang_vel.x - last_angular_velocity.x) / seconds_past;
		inst_ang_acc.y = (ang_vel.y - last_angular_velocity.y) / seconds_past;
		inst_ang_acc.z = (ang_vel.z - last_angular_velocity.z) / seconds_past;


		angular_acceleration.add_new_values(inst_ang_acc.x, inst_ang_acc.y, inst_ang_acc.z);

		angular_acceleration_deg.x = angular_acceleration.x * RAD_TO_DEG;
		angular_acceleration_deg.y = angular_acceleration.y * RAD_TO_DEG;
		angular_acceleration_deg.z = angular_acceleration.z * RAD_TO_DEG;

		last_angular_velocity.x = ang_vel.x;
		last_angular_velocity.y = ang_vel.y;
		last_angular_velocity.z = ang_vel.z;
	}


	void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
		float sqr = sq(qr);
		float sqi = sq(qi);
		float sqj = sq(qj);
		float sqk = sq(qk);

		ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
		ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
		ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

		if (degrees) {
			ypr->yaw *= RAD_TO_DEG;
			ypr->pitch *= RAD_TO_DEG;
			ypr->roll *= RAD_TO_DEG;
		}
	}

	void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
		quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
	}
};