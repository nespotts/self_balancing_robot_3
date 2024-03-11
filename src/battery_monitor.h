typedef struct {
  int cell1_pin;
  int cell2_pin;
  int cell3_pin;
  float cell1_volts;
  float cell2_volts;
  float cell3_volts;
  float total_voltage;
  float min_cell_voltage;
  int cell1_adc;
  int cell2_adc;
  int cell3_adc;
  long timer;
  long interval;
  float adcPerV;
  bool warning;
} battery_stats;

battery_stats battery{16,15,14,0,0,0,0,3.5,0,0,0,0,5000,55.6, false};

// setup and select run interval in ms
void battery_setup(long interval = 5000) {
  // pin assignments & scaling
  pinMode(battery.cell1_pin, INPUT);
  pinMode(battery.cell2_pin, INPUT);
  pinMode(battery.cell3_pin, INPUT);
  battery.interval = interval;
}

void battery_run() {
  if ((millis() - battery.timer) >= battery.interval) {
    battery.cell1_adc= analogRead(battery.cell1_pin);
    battery.cell2_adc = analogRead(battery.cell2_pin);
    battery.cell3_adc = analogRead(battery.cell3_pin);
    battery.cell1_volts = battery.cell1_adc / battery.adcPerV;
    battery.cell2_volts = (battery.cell2_adc-battery.cell1_adc) / battery.adcPerV;
    battery.cell3_volts = (battery.cell3_adc-battery.cell2_adc) / battery.adcPerV;
    battery.total_voltage = battery.cell1_volts+battery.cell2_volts+battery.cell3_volts;
    battery.timer = millis();

    if (battery.cell1_volts < battery.min_cell_voltage || battery.cell2_volts < battery.min_cell_voltage || battery.cell3_volts < battery.min_cell_voltage) {
      Serial.println("Battery Too Low");
      Serial.print("Cell 1: "); Serial.print(battery.cell1_volts);
      Serial.print("V\tCell 2: "); Serial.print(battery.cell2_volts);
      Serial.print("V\tCell 3: "); Serial.print(battery.cell3_volts);
      Serial.print("V\tTotal Voltage: "); Serial.print(battery.total_voltage); 
      Serial.println("V");
      battery.warning = true;
    } else {
      battery.warning = false;
    }
  }

}