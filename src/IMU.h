#include <Adafruit_BNO08x.h>

// this class should be a model for IMU data
class IMU_data {
public:
	IMU_data() {}

	// orientation
	Quaternion pose_quaternion;
	VectorFloat pose;  // in degrees

	// angular velocity
	VectorFloat ang_vel;
	// angular acceleration
	VectorFloat ang_acc;
	// linear acceleration
	VectorFloat lin_acc;

	float Yaw() {
		return pose.z;
	}

	float Pitch() {
		return pose.y;
	}

	float Roll() {
		return pose.x;
	}

private:
};


// this class should be focused on interfacing with the sensor
class IMU {
public:
	int orientation_status = 0;  // 0 - 3, 3 being most reliable
	IMU_data data;

	IMU() {
		// do anything?
		if (fast_mode) {
			reportType = SH2_GYRO_INTEGRATED_RV;
			reportIntervalUs = 2000;
		} else {
			reportType = SH2_ARVR_STABILIZED_RV;
			reportIntervalUs = 5000;
		}
	}


	void setup(void) {
		// Try to initialize!
		if (!bno08x.begin_SPI(CS_pin, INT_pin)) {
			if (debug) Serial.println("Failed to find BNO08x chip");
		}
		if (debug) Serial.println("BNO08x Found!");

		setReports();
		delay(100);
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

	sh2_SensorId_t reportType;
	long reportIntervalUs;

	struct euler_t {
		float yaw;
		float pitch;
		float roll;
	} ypr;

	Adafruit_BNO08x bno08x = Adafruit_BNO08x(RESET_pin);
	sh2_SensorValue_t sensorValue;

	void setReports() {
		if (debug) Serial.println("Setting desired reports");

		// ************************ Orientation Quaternion ****************************
		// slower 
		if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
			Serial.println("Could not enable stabilized remote vector");
		}
		// faster
		// if (!bno08x.enableReport(SH2_GYRO_INTEGRATED_RV, 2000)) {
		// 	Serial.println("Could not enable stabilized remote vector");
		// }

		// if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
		// 	Serial.println("Could not enable rotation vector");
		// }
		// if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
		// 	Serial.println("Could not enable geomagnetic rotation vector");
		// }
		// if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
		// 	Serial.println("Could not enable game rotation vector");
		// ****************************************************************************
		// if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
		// 	Serial.println("Could not enable accelerometer");
		// }
		if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
			Serial.println("Could not enable gyroscope");
		}
		// if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
		// 	Serial.println("Could not enable magnetic field calibrated");
		// }
		if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
			Serial.println("Could not enable linear acceleration");
		}
		// if (!bno08x.enableReport(SH2_GRAVITY)) {
		// 	Serial.println("Could not enable gravity vector");
		// }

		// }
		// if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
		// 	Serial.println("Could not enable step counter");
		// }
		// if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
		// 	Serial.println("Could not enable stability classifier");
		// }
		// if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
		// 	Serial.println("Could not enable raw accelerometer");
		// }
		// if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
		// 	Serial.println("Could not enable raw gyroscope");
		// }
		// if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
		// 	Serial.println("Could not enable raw magnetometer");
		// }
		// if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
		// 	Serial.println("Could not enable shake detector");
		// }
		// if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
		// 	Serial.println("Could not enable personal activity classifier");
		// }
	}

	void getSensorEvent() {
		if (!bno08x.getSensorEvent(&sensorValue)) {
			return;
		}

		static long last = 0;
		long now = micros();


		if (debug) { Serial.print(sensorValue.status); Serial.print("\t"); }
		switch (sensorValue.sensorId) {

		case SH2_ACCELEROMETER:
			// Serial.print("Accelerometer - x: ");
			// Serial.print(sensorValue.un.accelerometer.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.accelerometer.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.accelerometer.z);
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			// Serial.print("Gyro - x: ");
			// Serial.print(sensorValue.un.gyroscope.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.gyroscope.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.gyroscope.z);
			data.ang_vel.x = sensorValue.un.gyroscope.x;
			data.ang_vel.y = sensorValue.un.gyroscope.y;
			data.ang_vel.z = sensorValue.un.gyroscope.z;
			break;
		case SH2_MAGNETIC_FIELD_CALIBRATED:
			// Serial.print("Magnetic Field - x: ");
			// Serial.print(sensorValue.un.magneticField.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.magneticField.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.magneticField.z);
			break;
		case SH2_LINEAR_ACCELERATION:
			// Serial.print("Linear Acceration - x: ");
			// Serial.print(sensorValue.un.linearAcceleration.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.linearAcceleration.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.linearAcceleration.z);
			data.lin_acc.x = sensorValue.un.linearAcceleration.x;
			data.lin_acc.y = sensorValue.un.linearAcceleration.y;
			data.lin_acc.z = sensorValue.un.linearAcceleration.z;
			break;
		case SH2_GRAVITY:
			// Serial.print("Gravity - x: ");
			// Serial.print(sensorValue.un.gravity.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.gravity.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.gravity.z);
			break;
		case SH2_ROTATION_VECTOR:
			quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);

			// Serial.print(now - last);             Serial.print("\t");
			// last = now;
			// Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
			// Serial.print(ypr.yaw);                Serial.print("\t");
			// Serial.print(ypr.pitch);              Serial.print("\t");
			// Serial.print(ypr.roll); 							Serial.print("\t");

			// Serial.print("Rotation Vector - r: ");
			// Serial.print(sensorValue.un.rotationVector.real);
			// Serial.print(" i: ");
			// Serial.print(sensorValue.un.rotationVector.i);
			// Serial.print(" j: ");
			// Serial.print(sensorValue.un.rotationVector.j);
			// Serial.print(" k: ");
			// Serial.println(sensorValue.un.rotationVector.k);
			break;
		case SH2_ARVR_STABILIZED_RV:
			orientation_status = sensorValue.status;
			quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);

			data.pose_quaternion.w = sensorValue.un.arvrStabilizedRV.real;
			data.pose_quaternion.x = sensorValue.un.arvrStabilizedRV.i;
			data.pose_quaternion.y = sensorValue.un.arvrStabilizedRV.j;
			data.pose_quaternion.z = sensorValue.un.arvrStabilizedRV.k;

			data.pose.x = ypr.roll;
			data.pose.y = ypr.pitch;
			data.pose.z = ypr.yaw;

			// Serial.print(now - last);             Serial.print("\t");
			// last = now;
			// Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
			// Serial.print(ypr.yaw);                Serial.print("\t");
			// Serial.print(ypr.pitch);              Serial.print("\t");
			// Serial.print(ypr.roll); 							Serial.print("\t");

			// Serial.print("Rotation Vector - r: ");
			// Serial.print(sensorValue.un.arvrStabilizedRV.real);
			// Serial.print(" i: ");
			// Serial.print(sensorValue.un.arvrStabilizedRV.i);
			// Serial.print(" j: ");
			// Serial.print(sensorValue.un.arvrStabilizedRV.j);
			// Serial.print(" k: ");
			// Serial.println(sensorValue.un.arvrStabilizedRV.k);
			break;
		case SH2_GYRO_INTEGRATED_RV:
			quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);

			// Serial.print(now - last);             Serial.print("\t");
			// last = now;
			// Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
			// Serial.print(ypr.yaw);                Serial.print("\t");
			// Serial.print(ypr.pitch);              Serial.print("\t");
			// Serial.print(ypr.roll); 							Serial.print("\t");

			// Serial.print("Rotation Vector - r: ");
			// Serial.print(sensorValue.un.gyroIntegratedRV.real);
			// Serial.print(" i: ");
			// Serial.print(sensorValue.un.gyroIntegratedRV.i);
			// Serial.print(" j: ");
			// Serial.print(sensorValue.un.gyroIntegratedRV.j);
			// Serial.print(" k: ");
			// Serial.println(sensorValue.un.gyroIntegratedRV.k);
			break;

		case SH2_GEOMAGNETIC_ROTATION_VECTOR:
			// Serial.print("Geo-Magnetic Rotation Vector - r: ");
			// Serial.print(sensorValue.un.geoMagRotationVector.real);
			// Serial.print(" i: ");
			// Serial.print(sensorValue.un.geoMagRotationVector.i);
			// Serial.print(" j: ");
			// Serial.print(sensorValue.un.geoMagRotationVector.j);
			// Serial.print(" k: ");
			// Serial.println(sensorValue.un.geoMagRotationVector.k);
			break;

		case SH2_GAME_ROTATION_VECTOR:
			// Serial.print("Game Rotation Vector - r: ");
			// Serial.print(sensorValue.un.gameRotationVector.real);
			// Serial.print(" i: ");
			// Serial.print(sensorValue.un.gameRotationVector.i);
			// Serial.print(" j: ");
			// Serial.print(sensorValue.un.gameRotationVector.j);
			// Serial.print(" k: ");
			// Serial.println(sensorValue.un.gameRotationVector.k);
			break;

		case SH2_STEP_COUNTER:
			// Serial.print("Step Counter - steps: ");
			// Serial.print(sensorValue.un.stepCounter.steps);
			// Serial.print(" latency: ");
			// Serial.println(sensorValue.un.stepCounter.latency);
			break;

		case SH2_STABILITY_CLASSIFIER: {
			// Serial.print("Stability Classification: ");
			sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
			switch (stability.classification) {
			case STABILITY_CLASSIFIER_UNKNOWN:
				Serial.println("Unknown");
				break;
			case STABILITY_CLASSIFIER_ON_TABLE:
				Serial.println("On Table");
				break;
			case STABILITY_CLASSIFIER_STATIONARY:
				Serial.println("Stationary");
				break;
			case STABILITY_CLASSIFIER_STABLE:
				Serial.println("Stable");
				break;
			case STABILITY_CLASSIFIER_MOTION:
				Serial.println("In Motion");
				break;
			}
			break;
		}

		case SH2_RAW_ACCELEROMETER:
			// Serial.print("Raw Accelerometer - x: ");
			// Serial.print(sensorValue.un.rawAccelerometer.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.rawAccelerometer.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.rawAccelerometer.z);
			break;
		case SH2_RAW_GYROSCOPE:
			// Serial.print("Raw Gyro - x: ");
			// Serial.print(sensorValue.un.rawGyroscope.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.rawGyroscope.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.rawGyroscope.z);
			break;
		case SH2_RAW_MAGNETOMETER:
			// Serial.print("Raw Magnetic Field - x: ");
			// Serial.print(sensorValue.un.rawMagnetometer.x);
			// Serial.print(" y: ");
			// Serial.print(sensorValue.un.rawMagnetometer.y);
			// Serial.print(" z: ");
			// Serial.println(sensorValue.un.rawMagnetometer.z);
			break;
		}
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

	void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
		quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
	}
};