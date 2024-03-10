#pragma once

#include "encoder.h"

class Wheel {
private:
public:
	MagneticEncoder *encoder;
	Wheel(MagneticEncoder *p_encoder) {
		encoder = p_encoder;
	}


	void setup() {
		encoder->setup();
	}

	void run() {
		encoder->run();
		// use boolean flag change to indicate time to update
	}
};