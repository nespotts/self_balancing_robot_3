#include <Adafruit_VL53L0X.h>

// Define which Wire objects to use, may depend on platform
// or on your configurations.
#define SENSOR1_WIRE Wire
#define SENSOR2_WIRE Wire

typedef struct {
  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;            // id for the sensor
  int shutdown_pin;  // which pin for shutdown;
  int interrupt_pin; // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t
      sensor_config;     // options for how to use the sensor
  uint16_t range;        // range value used in continuous mode stuff.
  uint16_t last_range;
  uint32_t timeout;
  uint32_t stopTimes;
  uint8_t sensor_status; // status from last ranging in continous.
  int max_range; //mm
  bool results_ready;   // ranging results ready
} sensorList_t;

// Actual object, could probalby include in structure above
Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;

// Setup for 2 sensors
sensorList_t sensors[] = {
    {&sensor1, &SENSOR1_WIRE, 0x30, 28, 29,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0, 0, 0, 0, 2500, false},
    {&sensor2, &SENSOR2_WIRE, 0x31, 34, 35,
     Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0, 0, 0, 0, 2500, false},
};


typedef struct {
  const int COUNT_SENSORS;   // number of lidar sensors
  bool start_ranging;        // flag to start ranging
  bool ranging_complete;     // flag to indicate all ranging is finished
  uint32_t timeout;              // sensor reading timeout
  uint32_t start_time;       // timer started at beg. of each range
} lidar_vars;

lidar_vars lidar = {sizeof(sensors) / sizeof(sensors[0]), false, false, 100, 0};




void callback1() {
  sensors[0].results_ready = true;
}

void callback2() {
  sensors[1].results_ready = true;
}


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then
   set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but
   0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its
   XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but
   0x29 and whatever you set the first sensor to
*/

// setup and assign i2c addresses to each of the lidar sensors
void Lidar_setup(int shutdown_pin1=1, int interrupt_pin1=2, int shutdown_pin2=3, int interrupt_pin2=4) {
  // Assign lidar pins
  sensors[0].shutdown_pin = shutdown_pin1;
  sensors[0].interrupt_pin = interrupt_pin1;
  sensors[1].shutdown_pin = shutdown_pin2;
  sensors[1].interrupt_pin = interrupt_pin2;

  // initialize all of the pins.
  Serial.println(F("VL53LOX_multi start, initialize IO pins"));
  for (int i = 0; i < lidar.COUNT_SENSORS; i++) {
    pinMode(sensors[i].shutdown_pin, OUTPUT);
    digitalWrite(sensors[i].shutdown_pin, LOW);
  }

  // Initialize interrupts and callbacks
  attachInterrupt(digitalPinToInterrupt(sensors[0].interrupt_pin), callback1, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensors[1].interrupt_pin), callback2, FALLING);

  Serial.println(F("Starting Lidar..."));

  // shutdown all sensors
  bool found_any_sensors = false;
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < lidar.COUNT_SENSORS; i++)
    digitalWrite(sensors[i].shutdown_pin, LOW);
  delay(10);

  // assign address to each sensor
  for (int i = 0; i < lidar.COUNT_SENSORS; i++) {
    // one by one enable sensors and set their ID
    digitalWrite(sensors[i].shutdown_pin, HIGH);
    delay(10); // give time to wake up.
    if (sensors[i].psensor->begin(sensors[i].id, false, sensors[i].pwire,
                                  sensors[i].sensor_config)) {
      found_any_sensors = true;
    } else {
      Serial.print(i, DEC);
      Serial.print(F(": failed to start\n"));
    }
  }
  if (!found_any_sensors) {
    Serial.println("No valid sensors found");
    // while (1)
      ;
  }
}



//====================================================================
// ASync read sensors.
//====================================================================
void lidar_update() {

  // Tell all sensors to start if start_ranging == true
  if (lidar.start_ranging) {
    lidar.start_time = millis();

    for (int i = 0; i < lidar.COUNT_SENSORS; i++) {
      sensors[i].psensor->startRange();
    }
    lidar.start_ranging = false;
    lidar.ranging_complete = false;
  }

  // record ranges if both sensors have completed measurement.  Set ranging_complete flag
  if ((sensors[0].results_ready && sensors[1].results_ready) || ((millis() - lidar.start_time) > lidar.timeout)) {

    for (int i = 0; i < lidar.COUNT_SENSORS; i++) {
        sensors[i].range = sensors[i].psensor->readRangeResult();
        if (sensors[i].range > sensors[i].max_range) {
          // sensors[i].range = sensors[i].max_range;
          sensors[i].range = sensors[i].last_range;
          
        }
        sensors[i].timeout = sensors[i].psensor->timeoutOccurred();
        sensors[i].stopTimes = millis();
        sensors[i].last_range = sensors[i].range;
    }

    // uint32_t delta_time = millis() - lidar.start_time;

    sensors[0].results_ready = false;
    sensors[1].results_ready = false;
    lidar.ranging_complete = true;
  }
}