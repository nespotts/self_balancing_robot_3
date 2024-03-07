
#include <Bounce2.h>
Bounce bounce = Bounce();

class Reset {
private:
	int reset_pin;

public:
	Reset(int p_reset_pin = 31) {
		reset_pin = p_reset_pin;
		setup();
	}

	void setup() {
		bounce.attach(reset_pin, INPUT);
		bounce.interval(5); // interval in ms
	}

	void run() {
		bounce.update();

		// <Bounce>.changed() RETURNS true IF THE STATE CHANGED (FROM HIGH TO LOW OR LOW TO HIGH)
		if (bounce.changed()) {
			// THE STATE OF THE INPUT CHANGED
			// GET THE STATE
			int deboucedInput = bounce.read();
			// IF THE CHANGED VALUE IS LOW
			if (deboucedInput == LOW) {
				doReboot();
			}
		}
	}

	void doReboot() {
		USB1_USBCMD = 0;  // disconnect USB
		delay(50);        // enough time for USB hubs/ports to detect disconnect
		SCB_AIRCR = 0x05FA0004;  // restart Teensy
	}
};