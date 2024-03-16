class BatteryMonitor {
public:
	// constructor
	BatteryMonitor() {
		// pass
	}

	typedef struct {
		float cell1;
		float cell2;
		float cell3;
	} cellF;

	cellF volts;

	float total_voltage;
	float min_cell_voltage;
	bool warning = false;
	bool debug = true;

	void setup() {
		// pin assignments & scaling
		pinMode(pins.cell1, INPUT);
		pinMode(pins.cell2, INPUT);
		pinMode(pins.cell3, INPUT);
		pins.cell1 = 14;
		pins.cell2 = 40;
		pins.cell3 = 41;
	}

	void run() {
		if ((millis() - timer) >= interval) {
			adc.cell1 = analogRead(pins.cell1);
			adc.cell2 = analogRead(pins.cell2);
			adc.cell3 = analogRead(pins.cell3);
			volts.cell1 = adc.cell1 / adcPerV;
			volts.cell2 = (adc.cell2 - adc.cell1) / adcPerV;
			volts.cell3 = (adc.cell3 - adc.cell2) / adcPerV;
			total_voltage = volts.cell1 + volts.cell2 + volts.cell3;
			timer = millis();

			if (volts.cell1 < min_cell_voltage || volts.cell2 < min_cell_voltage || volts.cell3 < min_cell_voltage) {
				if (debug) Serial.println("Battery Too Low");
				warning = true;
			} else {
				warning = false;
			}

			if (debug) {
				Serial.print("Cell 1: "); Serial.print(volts.cell1);
				Serial.print("\tCell 2: "); Serial.print(volts.cell2);
				Serial.print("\tCell 3: "); Serial.print(volts.cell3);
				Serial.print("\tTotal Voltage: "); Serial.print(total_voltage);
				Serial.println();
			}
		}

	}

private:
	typedef struct {
		int cell1;
		int cell2;
		int cell3;
	} cellI;

	cellI adc;
	cellI pins;

	float adcPerV = 73.3182;
	uint16_t interval = 500;
	uint32_t timer;
};