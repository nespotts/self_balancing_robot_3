#include "Encoder.h"
Encoder Left_Encoder(20, 6);
Encoder Right_Encoder(21, 7);

// long odom_rate = 20000; //default us
// long last_odom_time;
// double num_avg_points = 10;

typedef struct {
  long odom_rate;
  long last_odom_time;
  double num_avg_points;
  int index;
  double theta_rate;
  double cent_distance;
  double cent_velocity;
  double cent_accel;
  double theta;   // anuglar position in radians
  double theta_deg; // angular position in degrees
  double x;       // x position delta
  double y;       // y position delta
  double wheel_separation; //mm
  double delta;      // last time differential
  long transformation_rate;
  long transform_time;
} odometryList;

odometryList odom = {20000,0,10,0,0,0,0,0,0,0,0,0,138,0,0,0};

typedef struct {
  int pin;
  double wheel_diameter; //mm
  double circumference;
  double steps_rev;
  double count;
  double last_count;
  double inst_distance;
  double distance;
  double last_distance;
  double inst_velocity[50];
  double velocity;
  double last_velocity;
  double inst_accel[50];
  double accel;
  double last_accel;
  int direction; 
  long count_timer;
  int min_us_per_count;
} encoderList;

encoderList encoder[] = {
  {4, 66, 0, 60, 0, 0, 0, 0, 0, {}, 0, 0, {}, 0, 0, 1,0,7000},
  {7, 66, 0, 60, 0, 0, 0, 0, 0, {}, 0, 0, {}, 0, 0, 1,0,7000}  //250
};


// needs some work yet ###########################################
// Odometry Transformation
void Odometry_Transformation() {
  // if ((currenttime - odom.transform_time) >= odom.transformation_rate) {


    double delta_theta = (encoder[1].inst_distance - encoder[0].inst_distance) / (odom.wheel_separation / 1000.0);  //in radians
    odom.theta_rate = delta_theta / odom.delta;
    
    odom.cent_distance = (encoder[0].inst_distance + encoder[1].inst_distance) / 2.0;
    odom.cent_velocity = (encoder[0].velocity + encoder[1].velocity) / 2.0;
    odom.cent_accel = (encoder[0].accel + encoder[1].accel) / 2.0;
    encoder[0].min_us_per_count = map(fabs(odom.cent_velocity), 0, 0.4, 15000, 6666);
    encoder[1].min_us_per_count = encoder[0].min_us_per_count;

    odom.x += odom.cent_distance * cos(odom.theta + delta_theta/2.0);
    odom.y += odom.cent_distance * sin(odom.theta + delta_theta/2.0);
    odom.theta += delta_theta; // in radians
    odom.theta_deg = odom.theta * (180.0 / 3.1415926);

    // double delta_theta = mpu.Zangle - odom.theta;  //in radians
    // odom.theta_rate = delta_theta / odom.delta;
    
    // odom.cent_distance = (encoder[0].inst_distance + encoder[1].inst_distance) / 2.0;
    // odom.cent_velocity = (encoder[0].velocity + encoder[1].velocity) / 2.0;
    // odom.cent_accel = (encoder[0].accel + encoder[1].accel) / 2.0;

    // odom.x += odom.cent_distance * cos((odom.theta + delta_theta/2.0) * 3.1415/180.0);
    // odom.y += odom.cent_distance * sin((odom.theta + delta_theta/2.0) * 3.1415/180.0);
    // odom.theta += delta_theta; // in degrees

    // Serial.print(odom.cent_velocity);
    // Serial.print(encoder[1].distance,4);
    // Serial.print("\t");
    // Serial.print(odom.theta,4);
    // Serial.print("\t");
    // Serial.print(odom.wheel_separation / 1000.0, 4);
    // Serial.print("\t");
    // Serial.print(encoder[1].inst_distance,4);
    // Serial.print("\t");
    // Serial.print(encoder[0].inst_distance,4);
    // Serial.print("\t");
    // Serial.print(encoder[1].count);
    // Serial.print("\t");
    // Serial.println();
  // }
}


void Odometry_Update() {
  uint32_t currenttime = micros();
  encoder[0].count = Left_Encoder.read();
  encoder[1].count = Right_Encoder.read();

  if ((currenttime - odom.last_odom_time) >= odom.odom_rate) {
    int ind = odom.index;
    odom.delta = (currenttime- odom.last_odom_time) / 1000000.0; //micros

    for (int i = 0; i<2; i++) {
      double count_diff = (encoder[i].count - encoder[i].last_count);
      double revs = count_diff / encoder[i].steps_rev;
      encoder[i].inst_distance = (revs * encoder[i].circumference) / 1000.0;  // m
      // total distance traveled
      encoder[i].distance = encoder[i].last_distance + encoder[i].inst_distance;
      // instantaneous velocity
      encoder[i].inst_velocity[ind] = encoder[i].inst_distance / odom.delta;  // m/s

      double sum_vel = 0;
      for (int a = 0; a < (odom.num_avg_points - 1); a++) {
        sum_vel += encoder[i].inst_velocity[a];
      }
      encoder[i].velocity = sum_vel / odom.num_avg_points;

      // instantaneous acceleration
      encoder[i].inst_accel[ind] = (encoder[i].velocity - encoder[i].last_velocity) / odom.delta;
      double sum_accel = 0;
      for (int a = 0; a < (odom.num_avg_points - 1); a++) {
        sum_accel += encoder[i].inst_accel[a];
      }
      encoder[i].accel = sum_accel / odom.num_avg_points;

        // for debugging purposes
      // if (i==1) {
      //   Serial.print(encoder[i].count);
      //   Serial.print("\t");
      //   Serial.print(encoder[i].last_count);
      //   Serial.print("\t");
      //   Serial.print(odom.delta, 4);
      //   Serial.print("\t");
      //   Serial.print(count_diff);
      //   Serial.print("\t");
      //   Serial.print(revs, 4);
      //   Serial.print("\t");
      //   Serial.print(encoder[i].inst_distance, 4);
      //   Serial.print("\t");
      //   Serial.print(encoder[i].distance, 4);
      //   Serial.print("\t");
      //   Serial.print(encoder[i].velocity, 4);
      //   Serial.print("\t");
      //   Serial.print(encoder[i].accel, 4);
      //   Serial.println();
      // }

      encoder[i].last_count = encoder[i].count;
      encoder[i].last_distance = encoder[i].distance;
      encoder[i].last_velocity = encoder[i].velocity;
      encoder[i].last_accel = encoder[i].accel;
    }

    odom.last_odom_time = currenttime;
    if (odom.index < 9) {
        odom.index ++;
    } else {
        odom.index = 0;
    }
  Odometry_Transformation();
  }
}


// void left_encoder_CB() {
//   if (micros() - encoder[0].count_timer > encoder[0].min_us_per_count) {
//     if (encoder[0].direction == 1) {
//       encoder[0].count++;
//     } else {
//       encoder[0].count--;
//     }
//     encoder[0].count_timer = micros();
//   }
// }

// void right_encoder_CB() {
//   if (micros() - encoder[1].count_timer > encoder[0].min_us_per_count) {
//     if (encoder[1].direction == 1) {
//       encoder[1].count++;
//     } else {
//       encoder[1].count--;
//     }
//     encoder[1].count_timer = micros();
//   }
// }

void Odometry_setup(int left_pin, int right_pin, double freq, int number_avg_pts = 10) {
  // encoder[0].pin = left_pin;
  // encoder[1].pin = right_pin;
  
  // attachInterrupt(digitalPinToInterrupt(encoder[0].pin), left_encoder_CB, RISING);
  // attachInterrupt(digitalPinToInterrupt(encoder[1].pin), right_encoder_CB, RISING);

  encoder[0].circumference = 3.1415 * encoder[0].wheel_diameter;
  encoder[1].circumference = 3.1415 * encoder[1].wheel_diameter;
  odom.odom_rate = 1000000.0/freq; // in us
  odom.transformation_rate = odom.odom_rate * 2.0;
  odom.num_avg_points = number_avg_pts;
}

