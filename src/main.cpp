#include <Arduino.h>
#include "helper_functions.h"
#include "Wire.h"
#include "motor_controls.h"
Motor motor(3, 4, 5);
#include "encoder.h"
MagneticEncoder enc1(6, 200, false);

#include "Plotter.h"
Plotter p;


void setup() {
	Serial.begin(115200);
	Wire.begin();

	enc1.setup();

	motor.setup();
	pinMode(A7, INPUT);

	p.Begin();
	p.AddTimeGraph("Encoder", 5000, "Angular Position", enc1.angle_deg, "Angular Velocity", enc1.angular_velocity_deg, "Angular Acceleration", enc1.angular_acceleration_deg);
}


void loop() {
	p.Plot();
	enc1.run();

	int pot_val = analogRead(A7);

	int pwm_val = map(pot_val, 0, 1023, -255, 255);
	motor.command_motor(pwm_val);
}


