#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(32,36); // CE, CSN
const byte receive_addr[6] = "00010";
const byte send_addr[6] = "00020";

struct RECEIVE_DATA_STRUCTURE {
  int16_t throttle;
  int16_t rudder;
  int16_t elevator;
  int16_t aileron;
  int16_t SWA;
  int16_t SWC;
  int16_t left_knob;
  int16_t right_knob;
  bool request_data;
};

RECEIVE_DATA_STRUCTURE receive_data;

struct SEND_DATA_STRUCTURE {
  float battery_voltage;
  uint16_t front_dist;
  uint16_t left_dist;
  uint16_t right_dist;
  uint16_t back_dist;
};

SEND_DATA_STRUCTURE send_data;

void Radio_Setup(int PA_level) {
  radio.begin();
  if (PA_level == 1) {
      radio.setPALevel(RF24_PA_MIN);
  } else if (PA_level == 2) {
      radio.setPALevel(RF24_PA_LOW);
  } else if (PA_level == 3) {
      radio.setPALevel(RF24_PA_HIGH);
  } else {
      radio.setPALevel(RF24_PA_MAX);
  }
}

void Send_Data() {
  // delay(10);
  // send_data.battery_voltage = battery.total_voltage;
  send_data.front_dist = scan.ranges[16];
  // send_data.back_dist = scan.ranges[((int)scan.subdivisions/2)];
  // send_data.left_dist = scan.ranges[((int)scan.subdivisions/4)];
  // send_data.right_dist = scan.ranges[((int)scan.subdivisions/4)*3];
  send_data.back_dist = scan.ranges[0];
  send_data.left_dist = scan.ranges[24];
  send_data.right_dist = scan.ranges[8];
  radio.openWritingPipe(send_addr);
  radio.stopListening();
  radio.write(&send_data, sizeof(SEND_DATA_STRUCTURE));
  // delay(5);
}

long setpoint_timer = 0;
float setpoint_duration = 20000; // us

void ApplyRemoteData() {
  // do this every time thru the loop - should be based on timed interval ****************

  // cmd_vel.lin_vel = receive_data.elevator;
  // cmd_vel.ang_vel = receive_data.aileron + map((float)receive_data.right_knob, 0, 1023, -250, 250);
}


void Receive_Data() {
  radio.startListening();
  radio.openReadingPipe(0, receive_addr);
  if (radio.available()) {
    radio.read(&receive_data, sizeof(receive_data));

    // Serial.println(sizeof(receive_data));

    pids.control.balance_angle = map((float)receive_data.left_knob, 0, 1023, -15, 15);
    // mpu.gyro_drift_compensation = map((float)receive_data.right_knob, 0, 1023, -0.15, 0.15); 

    // if (receive_data.SWC == 0) {
    //   velocity.kp = map((float)receive_data.left_knob, 0, 1023, 0, 30);
    // } else if (receive_data.SWC == 1) {
    //   velocity.ki = map((float)receive_data.left_knob, 0, 1023, 0, 30);
    // } else if (receive_data.SWC == 2) {
    //   velocity.kd = map((float)receive_data.left_knob, 0, 1023, 0, 3);
    // }
    // VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);

    // debug received data
    // Serial.println(control.balance_angle); 
    // Serial.print(receive_data.throttle);
    // Serial.print("\t");
    // Serial.print(receive_data.rudder);
    // Serial.print("\t");
    // Serial.print(receive_data.elevator);
    // Serial.print("\t");
    // Serial.print(receive_data.aileron);
    // Serial.print("\t");
    // Serial.println(receive_data.left_knob);
  }

    ApplyRemoteData();


  if (receive_data.request_data) {
    Send_Data();
    receive_data.request_data = false;
  }
}


