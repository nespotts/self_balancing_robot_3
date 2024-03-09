#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

 /* =========================================================================
	NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
	when using Serial.write(buf, len). The Teapot output uses this method.
	The solution requires a modification to the Arduino USBAPI.h file, which
	is fortunately simple, but annoying. This will be fixed in the next IDE
	release. For more info, see these links:

	http://arduino.cc/forum/index.php/topic,109987.0.html
	http://code.google.com/p/arduino/issues/detail?id=958
  * ========================================================================= */


  // === INTERRUPT DETECTION ROUTINE ===

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


class IMU {
private:
	int INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards

	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
	// AD0 high = 0x69
	// MPU6050 mpu;
	//MPU6050 mpu(0x69); // <-- use for AD0 high
	MPU6050 mpu = MPU6050();

	void calculate_angular_velocity() {
		inst_ang_vel.x = ((ypr[2] - last_ypr[2]) * 180.0 / M_PI) / seconds_past; // deg/s
		inst_ang_vel.y = ((ypr[1] - last_ypr[1]) * 180.0 / M_PI) / seconds_past; // deg/s
		inst_ang_vel.z = ((ypr[0] - last_ypr[0]) * 180.0 / M_PI) / seconds_past; // deg/s

		angular_velocity.add_new_values(inst_ang_vel.x, inst_ang_vel.y, inst_ang_vel.z);

		last_ypr[0] = ypr[0];
		last_ypr[1] = ypr[1];
		last_ypr[2] = ypr[2];
	}


	void calculate_angular_acceleration() {
		inst_ang_acc.x = (angular_velocity.x - last_angular_velocity.x) / seconds_past; // deg/s
		inst_ang_acc.y = (angular_velocity.y - last_angular_velocity.y) / seconds_past; // deg/s
		inst_ang_acc.z = (angular_velocity.z - last_angular_velocity.z) / seconds_past; // deg/s


		angular_acceleration.add_new_values(inst_ang_acc.x, inst_ang_acc.y, inst_ang_acc.z);

		last_angular_velocity.x = angular_velocity.x;
		last_angular_velocity.y = angular_velocity.y;
		last_angular_velocity.z = angular_velocity.z;
	}

	void getDMPData() {
		// if programming failed, don't try to do anything
		if (!dmpReady) return;
		// read a packet from FIFO

		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
			seconds_past = ((double)micros() - (double)last_time) / 1000000.0;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			// mpu.dmpGetGyro(&angular_velocity, fifoBuffer);


			// check sample rate // about 167 Hz
			// Serial.println(1000000.0 / (micros() - last_time));

			calculate_angular_velocity();

			calculate_angular_acceleration();

			last_time = micros();
		}
	}

public:
	double seconds_past;
	u_int32_t last_time; // micros
	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	Quaternion q_orientation = Quaternion(0.67, 0.01, 0.74, 0.00); // [w, x, y, z]  quaternion describing orientation of mpu mounted in robot - used to transform all other results
	
	// angular position
	float last_ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	// angular velocity
	VectorFloat inst_ang_vel;
	ImuComponent angular_velocity = ImuComponent(5, 1);
	VectorFloat last_angular_velocity;
	// angular acceleration
	VectorFloat inst_ang_acc;
	// TODO - need to figure out way to scale outliar detection
	ImuComponent angular_acceleration = ImuComponent(20, 10);

	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaRealMax;
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorInt16 aaWorldMax;
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container

	// constructor
	IMU() {

	}


	void run() {
		getDMPData();



		// !!!! IMPORTANT to remember how to do !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// rotate output vector by the original orientation to transform the output coordinates to the installed orientation
		// aaReal = aaReal.getRotated(&q_orientation);

		// Serial.print("ypr\t");
		// Serial.print(ypr[0] * 180 / M_PI);
		// Serial.print("\t");
		// Serial.print(ypr[1] * 180 / M_PI);
		// Serial.print("\t");
		// Serial.println(ypr[2] * 180 / M_PI);


		// Serial.print("areal\t");
		// Serial.print(((float)aaReal.x / 16384.0) * 9.81);
		// Serial.print("\t");
		// Serial.print(((float)aaReal.y / 16384.0) * 9.81);
		// Serial.print("\t");
		// Serial.println(((float)aaReal.z / 16384.0) * 9.81);

		// Max acceleration values
		// if (aaReal.x > aaRealMax.x) {
		// 	aaRealMax.x = aaReal.x;
		// }
		// if (aaReal.y > aaRealMax.y) {
		// 	aaRealMax.y = aaReal.y;
		// }
		// if (aaReal.z > aaRealMax.z) {
		// 	aaRealMax.z = aaReal.z;
		// }

		// Serial.print("arealMax\t");
		// Serial.print(((float)aaRealMax.x / 16384.0) * 9.81);
		// Serial.print("\t");
		// Serial.print(((float)aaRealMax.y / 16384.0) * 9.81);
		// Serial.print("\t");
		// Serial.println(((float)aaRealMax.z / 16384.0) * 9.81);


	}


	void setup() {
		// join I2C bus (I2Cdev library doesn't do this automatically)

		Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


		// initialize device
		Serial.println(F("Initializing I2C devices..."));
		mpu.initialize();
		pinMode(INTERRUPT_PIN, INPUT);

		// verify connection
		Serial.println(F("Testing device connections..."));
		Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

		// wait for ready
		// Serial.println(F("\nSend any character to begin DMP programming and demo: "));
		// while (Serial.available() && Serial.read()); // empty buffer
		// while (!Serial.available());                 // wait for data
		// while (Serial.available() && Serial.read()); // empty buffer again

		// load and configure the DMP
		Serial.println(F("Initializing DMP..."));
		devStatus = mpu.dmpInitialize();

		// supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(147);
		mpu.setYGyroOffset(-64);
		mpu.setZGyroOffset(5);
		mpu.setXAccelOffset(-3464);
		mpu.setYAccelOffset(-2517);
		mpu.setZAccelOffset(1772); // 1688 factory default for my test chip

		Serial.print("DLPF Mode: ");
		Serial.println(mpu.getDLPFMode());

		mpu.setRate(0);


		Serial.print("Sample Rate: ");
		Serial.println(mpu.getRate());

		// make sure it worked (returns 0 if so)
		if (devStatus == 0) {
			// Calibration Time: generate offsets and calibrate our MPU6050
			// calculate automatically
			// mpu.CalibrateAccel(6);
			// mpu.CalibrateGyro(6);
			mpu.PrintActiveOffsets();
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			// enable Arduino interrupt detection
			Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
			Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
			Serial.println(F(")..."));
			attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
		} else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
		}
	}

};

