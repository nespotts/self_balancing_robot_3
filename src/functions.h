// variables for looping frequency
long last_loop_time = 0;
long duration;
long freq;


void Calc_Loop_Speed() {
  uint32_t currenttime = micros();
  duration = currenttime - last_loop_time;
  freq = (double)1000000.0/(double)duration;

  // Serial.print("Loop Duration:  ");
  // Serial.print(duration); 
  // Serial.print("\tLoop Frequency:  ");
  // Serial.println(freq);
  last_loop_time = currenttime;
}

// create function that takes in any value

void serial_printer(String v1, String v2="", String v3="", String v4="", String v5="", String v6="") {
	Serial.print(v1);
	Serial.print(v2);
	Serial.print(v3);
	Serial.print(v4);
	Serial.print(v5);
	Serial.print(v6);
	Serial.println();
}