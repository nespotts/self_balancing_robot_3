// #include <Arduino.h>
// #include "helper_functions.h"
// #include "Wire.h"

// #include "motor_controls.h"
// Motor motor(3, 4, 5);

// #include "encoder.h"
// MagneticEncoder enc1(6, 500, false);

// #include "mpu6050.h"
// IMU imu;

// #include "Plotter.h"
// Plotter p;


// void setup() {
// 	Serial.begin(115200);
// 	Wire.begin();

// 	enc1.setup();

// 	motor.setup();
// 	pinMode(A7, INPUT);

// 	imu.setup();

// 	p.Begin();
// 	// p.AddTimeGraph("Encoder", 5000, "Angular Position", enc1.angle_deg, "Angular Velocity", enc1.angular_velocity_deg, "Angular Acceleration", enc1.angular_acceleration_deg);
// 	p.AddTimeGraph("IMU Orientation", 10000, "Yaw", imu.ypr[0], "Pitch", imu.ypr[1], "Roll", imu.ypr[2]);
// 	p.AddTimeGraph("IMU Linear Acceleration", 10000, "X", imu.aaReal.x, "Y", imu.aaReal.y, "Z", imu.aaReal.z);
// 	p.AddTimeGraph("IMU Angular Velocity", 10000, "X", imu.angular_velocity.x, "Y", imu.angular_velocity.y, "Z", imu.angular_velocity.z);
// 	p.AddTimeGraph("IMU Angular Acceleration", 10000, "X", imu.angular_acceleration.x, "Y", imu.angular_acceleration.y, "Z", imu.angular_acceleration.z);
// 	// p.AddTimeGraph("IMU Angular Velocity Comparison", 10000, "X gyro", imu.angular_velocity.x, "X calculated", imu.angular_velocity.x);
// 	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
// 	// p.AddTimeGraph("IMU Gravity Vector", 10000, "X", imu.gravity.x, "Y", imu.gravity.y, "Z", imu.gravity.z);
// }


// void loop() {
// 	p.Plot();
// 	// enc1.run();
// 	imu.run();

// 	// int pot_val = analogRead(A7);

// 	// int pwm_val = map(pot_val, 0, 1023, -255, 255);
// 	// motor.command_motor(pwm_val);
// }










// // Basic demo for readings from Adafruit BNO08x
// #include <Adafruit_BNO08x.h>

// // For SPI mode, we need a CS pin
// #define BNO08X_CS 10
// #define BNO08X_INT 9

// // For SPI mode, we also need a RESET
// #define BNO08X_RESET 8
// // but not for I2C or UART
// // #define BNO08X_RESET -1

// Adafruit_BNO08x bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;

// // Here is where you define the sensor outputs you want to receive
// void setReports(void) {
//   Serial.println("Setting desired reports");
//   // if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
//   //   Serial.println("Could not enable accelerometer");
//   // }
//   // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
//   //   Serial.println("Could not enable gyroscope");
//   // }
//   // if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
//   //   Serial.println("Could not enable magnetic field calibrated");
//   // }
//   // if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
//   //   Serial.println("Could not enable linear acceleration");
//   // }
//   // if (!bno08x.enableReport(SH2_GRAVITY)) {
//   //   Serial.println("Could not enable gravity vector");
//   // }
//   // if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
//   //   Serial.println("Could not enable rotation vector");
//   // }
//   // if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
//   //   Serial.println("Could not enable geomagnetic rotation vector");
//   // }
//   // if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
//   //   Serial.println("Could not enable game rotation vector");
//   // }
//   // if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
//   //   Serial.println("Could not enable step counter");
//   // }
//   // if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
//   //   Serial.println("Could not enable stability classifier");
//   // }
//   // if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
//   //   Serial.println("Could not enable raw accelerometer");
//   // }
//   // if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
//   //   Serial.println("Could not enable raw gyroscope");
//   // }
//   // if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
//   //   Serial.println("Could not enable raw magnetometer");
//   // }
//   // if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
//   //   Serial.println("Could not enable shake detector");
//   // }
//   // if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
//   //   Serial.println("Could not enable personal activity classifier");
//   // }
// }
// void printActivity(uint8_t activity_id) {
//   switch (activity_id) {
//   case PAC_UNKNOWN:
//     Serial.print("Unknown");
//     break;
//   case PAC_IN_VEHICLE:
//     Serial.print("In Vehicle");
//     break;
//   case PAC_ON_BICYCLE:
//     Serial.print("On Bicycle");
//     break;
//   case PAC_ON_FOOT:
//     Serial.print("On Foot");
//     break;
//   case PAC_STILL:
//     Serial.print("Still");
//     break;
//   case PAC_TILTING:
//     Serial.print("Tilting");
//     break;
//   case PAC_WALKING:
//     Serial.print("Walking");
//     break;
//   case PAC_RUNNING:
//     Serial.print("Running");
//     break;
//   case PAC_ON_STAIRS:
//     Serial.print("On Stairs");
//     break;
//   default:
//     Serial.print("NOT LISTED");
//   }
//   Serial.print(" (");
//   Serial.print(activity_id);
//   Serial.print(")");
// }

// void setup(void) {
//   Serial.begin(115200);
//   while (!Serial)
//     delay(10); // will pause Zero, Leonardo, etc until serial console opens

//   Serial.println("Adafruit BNO08x test!");

//   // Try to initialize!
//   // if (!bno08x.begin_I2C()) {
//     // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer! 
//     if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
//     Serial.println("Failed to find BNO08x chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("BNO08x Found!");

//   for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
//     Serial.print("Part ");
//     Serial.print(bno08x.prodIds.entry[n].swPartNumber);
//     Serial.print(": Version :");
//     Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
//     Serial.print(".");
//     Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
//     Serial.print(".");
//     Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
//     Serial.print(" Build ");
//     Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
//   }

//   setReports();

//   Serial.println("Reading events");
//   delay(100);
// }


// void loop() {
//   delay(10);

//   if (bno08x.wasReset()) {
//     Serial.print("sensor was reset ");
//     setReports();
//   }

//   if (!bno08x.getSensorEvent(&sensorValue)) {
//     return;
//   }

//   switch (sensorValue.sensorId) {

//   case SH2_ACCELEROMETER:
//     Serial.print("Accelerometer - x: ");
//     Serial.print(sensorValue.un.accelerometer.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.accelerometer.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.accelerometer.z);
//     break;
//   case SH2_GYROSCOPE_CALIBRATED:
//     Serial.print("Gyro - x: ");
//     Serial.print(sensorValue.un.gyroscope.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.gyroscope.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.gyroscope.z);
//     break;
//   case SH2_MAGNETIC_FIELD_CALIBRATED:
//     Serial.print("Magnetic Field - x: ");
//     Serial.print(sensorValue.un.magneticField.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.magneticField.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.magneticField.z);
//     break;
//   case SH2_LINEAR_ACCELERATION:
//     Serial.print("Linear Acceration - x: ");
//     Serial.print(sensorValue.un.linearAcceleration.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.linearAcceleration.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.linearAcceleration.z);
//     break;
//   case SH2_GRAVITY:
//     Serial.print("Gravity - x: ");
//     Serial.print(sensorValue.un.gravity.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.gravity.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.gravity.z);
//     break;
//   case SH2_ROTATION_VECTOR:
//     Serial.print("Rotation Vector - r: ");
//     Serial.print(sensorValue.un.rotationVector.real);
//     Serial.print(" i: ");
//     Serial.print(sensorValue.un.rotationVector.i);
//     Serial.print(" j: ");
//     Serial.print(sensorValue.un.rotationVector.j);
//     Serial.print(" k: ");
//     Serial.println(sensorValue.un.rotationVector.k);
//     break;
//   case SH2_GEOMAGNETIC_ROTATION_VECTOR:
//     Serial.print("Geo-Magnetic Rotation Vector - r: ");
//     Serial.print(sensorValue.un.geoMagRotationVector.real);
//     Serial.print(" i: ");
//     Serial.print(sensorValue.un.geoMagRotationVector.i);
//     Serial.print(" j: ");
//     Serial.print(sensorValue.un.geoMagRotationVector.j);
//     Serial.print(" k: ");
//     Serial.println(sensorValue.un.geoMagRotationVector.k);
//     break;

//   case SH2_GAME_ROTATION_VECTOR:
//     Serial.print("Game Rotation Vector - r: ");
//     Serial.print(sensorValue.un.gameRotationVector.real);
//     Serial.print(" i: ");
//     Serial.print(sensorValue.un.gameRotationVector.i);
//     Serial.print(" j: ");
//     Serial.print(sensorValue.un.gameRotationVector.j);
//     Serial.print(" k: ");
//     Serial.println(sensorValue.un.gameRotationVector.k);
//     break;

//   case SH2_STEP_COUNTER:
//     Serial.print("Step Counter - steps: ");
//     Serial.print(sensorValue.un.stepCounter.steps);
//     Serial.print(" latency: ");
//     Serial.println(sensorValue.un.stepCounter.latency);
//     break;

//   case SH2_STABILITY_CLASSIFIER: {
//     Serial.print("Stability Classification: ");
//     sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
//     switch (stability.classification) {
//     case STABILITY_CLASSIFIER_UNKNOWN:
//       Serial.println("Unknown");
//       break;
//     case STABILITY_CLASSIFIER_ON_TABLE:
//       Serial.println("On Table");
//       break;
//     case STABILITY_CLASSIFIER_STATIONARY:
//       Serial.println("Stationary");
//       break;
//     case STABILITY_CLASSIFIER_STABLE:
//       Serial.println("Stable");
//       break;
//     case STABILITY_CLASSIFIER_MOTION:
//       Serial.println("In Motion");
//       break;
//     }
//     break;
//   }

//   case SH2_RAW_ACCELEROMETER:
//     Serial.print("Raw Accelerometer - x: ");
//     Serial.print(sensorValue.un.rawAccelerometer.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.rawAccelerometer.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.rawAccelerometer.z);
//     break;
//   case SH2_RAW_GYROSCOPE:
//     Serial.print("Raw Gyro - x: ");
//     Serial.print(sensorValue.un.rawGyroscope.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.rawGyroscope.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.rawGyroscope.z);
//     break;
//   case SH2_RAW_MAGNETOMETER:
//     Serial.print("Raw Magnetic Field - x: ");
//     Serial.print(sensorValue.un.rawMagnetometer.x);
//     Serial.print(" y: ");
//     Serial.print(sensorValue.un.rawMagnetometer.y);
//     Serial.print(" z: ");
//     Serial.println(sensorValue.un.rawMagnetometer.z);
//     break;

//   case SH2_SHAKE_DETECTOR: {
//     Serial.print("Shake Detector - shake detected on axis: ");
//     sh2_ShakeDetector_t detection = sensorValue.un.shakeDetector;
//     switch (detection.shake) {
//     case SHAKE_X:
//       Serial.println("X");
//       break;
//     case SHAKE_Y:
//       Serial.println("Y");
//       break;
//     case SHAKE_Z:
//       Serial.println("Z");
//       break;
//     default:
//       Serial.println("None");
//       break;
//     }
//   }

//   case SH2_PERSONAL_ACTIVITY_CLASSIFIER: {

//     sh2_PersonalActivityClassifier_t activity =
//         sensorValue.un.personalActivityClassifier;
//     Serial.print("Activity classification - Most likely: ");
//     printActivity(activity.mostLikelyState);
//     Serial.println("");

//     Serial.println("Confidences:");
//     // if PAC_OPTION_COUNT is ever > 10, we'll need to
//     // care about page
//     for (uint8_t i = 0; i < PAC_OPTION_COUNT; i++) {
//       Serial.print("\t");
//       printActivity(i);
//       Serial.print(": ");
//       Serial.println(activity.confidence[i]);
//     }
//   }
//   }
// }












// #include <Arduino.h>
// // This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give 
// // quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.  
// // Note sensorValue.status gives calibration accuracy (which improves over time)
// #include <Adafruit_BNO08x.h>

// // For SPI mode, we need a CS pin
// #define BNO08X_CS 10
// #define BNO08X_INT 9


// #define FAST_MODE

// // For SPI mode, we also need a RESET 
// #define BNO08X_RESET 8
// // but not for I2C or UART
// // #define BNO08X_RESET -1

// struct euler_t {
// 	float yaw;
// 	float pitch;
// 	float roll;
// } ypr;

// Adafruit_BNO08x  bno08x(BNO08X_RESET);
// sh2_SensorValue_t sensorValue;

// #ifdef FAST_MODE
// // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
// sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
// long reportIntervalUs = 2000;
// #else
// // Top frequency is about 250Hz but this report is more accurate
// sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// long reportIntervalUs = 5000;
// #endif
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
// 	// if (!bno08x.begin_I2C()) {
// 	//if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
// 	if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
// 		Serial.println("Failed to find BNO08x chip");
// 		while (1) { delay(10); }
// 	}
// 	Serial.println("BNO08x Found!");


// 	setReports(reportType, reportIntervalUs);

// 	Serial.println("Reading events");
// 	delay(100);

// 	pinMode(23, INPUT_PULLUP);
// }

// void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

// 	float sqr = sq(qr);
// 	float sqi = sq(qi);
// 	float sqj = sq(qj);
// 	float sqk = sq(qk);

// 	ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
// 	ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
// 	ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

// 	if (degrees) {
// 		ypr->yaw *= RAD_TO_DEG;
// 		ypr->pitch *= RAD_TO_DEG;
// 		ypr->roll *= RAD_TO_DEG;
// 	}

// }

// void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
// 	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
// }

// void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
// 	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
// }

// void doReboot() {
// 	SCB_AIRCR = 0x05FA0004;
// }

// void loop() {

// 	if (bno08x.wasReset()) {
// 		Serial.print("sensor was reset ");
// 		setReports(reportType, reportIntervalUs);
// 	}

// 	if (bno08x.getSensorEvent(&sensorValue)) {
// 		// in this demo only one report type will be received depending on FAST_MODE define (above)
// 		switch (sensorValue.sensorId) {
// 		case SH2_ARVR_STABILIZED_RV:
// 			quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
// 		case SH2_GYRO_INTEGRATED_RV:
// 			// faster (more noise?)
// 			quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
// 			break;
// 		}
// 		static long last = 0;
// 		long now = micros();
// 		Serial.print(now - last);             Serial.print("\t");
// 		last = now;
// 		Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
// 		Serial.print(ypr.yaw);                Serial.print("\t");
// 		Serial.print(ypr.pitch);              Serial.print("\t");
// 		Serial.println(ypr.roll);
// 	}

// }




