#pragma once

#include <PID_v1.h>


Motor left_motor(4, 5, 6);
Motor right_motor(7, 8, 9);
Odometry odom;
IMU imu;


uint32_t currenttime;

typedef struct{
  double kp;
  double ki;
  double kd;
  double input;
  double output;
  double setpoint;
  float rate;
} PID_vars;

PID_vars bal{};
PID_vars steer{};
PID_vars velocity{};

typedef struct{
  double leftoutput;
  double rightoutput;
  double min_power;
  double max_power;
  double power_range;
  float balance_cutoff_angle;
  double balance_angle;
  long startbalancetimer;
  // double steer_ang_vel;
} PID_control_vars;

PID_control_vars control{};

struct cmd_vel_type{
  double lin_vel;
  double ang_vel;
  double rate;
  long timer;
  long duration;
  double max_lin_vel;   // m/s
  double max_ang_vel;  // deg/s
};

cmd_vel_type cmd_vel;


PID BalancePID(&bal.input, &bal.output, &bal.setpoint, bal.kp, bal.ki, bal.kd, DIRECT);
PID SteerStraightPID(&steer.input, &steer.output, &steer.setpoint, steer.kp, steer.ki, steer.kd, DIRECT);
PID VelocityPID(&velocity.input, &velocity.output, &velocity.setpoint, velocity.kp, velocity.ki, velocity.kd, DIRECT);


void controls_setup() {
  // setup balancing PID
  bal.kp = 105; //110; //130
  bal.ki = 2503; //2507; //2750
  bal.kd = 5.7; //6.8;  //6.5
  bal.rate = 100;
  BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);

  // setup steering PID - balancing
  steer.kp = 10.0;
  steer.ki = 3.0;
  steer.kd = 0.03;

  // for non-balancing version
  // steer.kp = 50.0;
  // steer.ki = 3.0;
  // steer.kd = 0.03;

  steer.rate = 100;
  SteerStraightPID.SetTunings(steer.kp, steer.ki, steer.kd);

  // setup commanded velocity PID - output is setpoint for balance PID
  // balance
  velocity.kp = 10.0;
  velocity.ki = 4.7;
  velocity.kd = 0.02;
  // for non-balancing
  // velocity.kp = 4000.0;
  // velocity.ki = 500;
  // velocity.kd = 60;

  velocity.rate = 100;
  VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);

  control.min_power = 0;
  control.max_power = 4095;
  control.power_range = control.max_power - control.min_power;
  control.balance_angle = -1.0;
  control.balance_cutoff_angle = 40;
  bal.setpoint = control.balance_angle;

  BalancePID.SetMode(AUTOMATIC);
  BalancePID.SetOutputLimits(-control.power_range, control.power_range);
  BalancePID.SetSampleTime(1000.0/bal.rate);
  // BalancePID.SetMode(AUTOMATIC);


  SteerStraightPID.SetMode(AUTOMATIC);
  SteerStraightPID.SetOutputLimits(-control.power_range, control.power_range);
  SteerStraightPID.SetSampleTime(1000.0/steer.rate);
  // SteerStraightPID.SetMode(AUTOMATIC);

  VelocityPID.SetMode(AUTOMATIC);
  // VelocityPID.SetOutputLimits(-10, 10);
  VelocityPID.SetOutputLimits(-control.power_range, control.power_range);
  VelocityPID.SetSampleTime(1000.0/velocity.rate);

  cmd_vel.max_lin_vel = 0.35; // m/s
  cmd_vel.max_ang_vel = 360; // deg/s
  cmd_vel.rate = 50;
  cmd_vel.duration = 1000.0 / cmd_vel.rate * 1000.0;

  // TODO update with pitch angle
  steer.setpoint = imu.ypr.yaw;
}


void setTunings() {
  char inByte[10];
  int index = 0;

  while(Serial.available() > 0) {
    inByte[0] = Serial.read();
    inByte[1] = '\0';
    // Serial.print(inByte);
    if (strcmp(inByte,"q") == 0) {
      bal.kp *= 1.01;
    } else if (strcmp(inByte,"a") == 0) {
      bal.kp *= 0.99;
    } else if (strcmp(inByte,"w") == 0) {
      bal.ki *= 1.01;
    } else if (strcmp(inByte,"s") == 0) {
      bal.ki *= 0.99;
    } else if (strcmp(inByte,"e") == 0) {
      bal.kd *= 1.01;
    } else if (strcmp(inByte,"d") == 0) {
      bal.kd *= 0.99;
    } else if (strcmp(inByte, "r") == 0) {
      bal.setpoint += 0.1;
    } else if (strcmp(inByte, "f") == 0) {
      bal.setpoint -= 0.1;
    } else if (strcmp(inByte,"g") == 0) {
      velocity.kp *= 0.99;
    } else if (strcmp(inByte,"t") == 0) {
      velocity.kp *= 1.01;
    } else if (strcmp(inByte,"h") == 0) {
      velocity.ki *= 0.99;
    } else if (strcmp(inByte,"y") == 0) {
      velocity.ki *= 1.01;
    } else if (strcmp(inByte,"j") == 0) {
      velocity.kd *= 0.99;
    } else if (strcmp(inByte, "u") == 0) {
      velocity.kd *= 1.01;
    } 

    Serial.print(bal.kp,8); Serial.print("\t"); Serial.print(bal.ki,8); 
    Serial.print("\t"); Serial.print(bal.kd,8); Serial.print("\t"); Serial.print(bal.setpoint,2);
    Serial.print("\t"); Serial.print(velocity.kp,8); Serial.print("\t"); Serial.print(velocity.ki,8);
    Serial.print("\t"); Serial.println(velocity.kd,8);

    BalancePID.SetTunings(bal.kp,bal.ki,bal.kd);
    // SteerStraightPID.SetTunings(steer.kp, steer.ki, steer.kd);
    VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
    index++;
  }
}

// 1. command starts here
void Command_Velocity(float lin_vel, float ang_vel){
  if (currenttime - cmd_vel.timer >= cmd_vel.duration) {

    // balancing version
    velocity.setpoint = map(lin_vel, 0, 1023, -0.35,0.35);
    // for non-balancing version
    // velocity.setpoint = map(lin_vel, 0, 1023, -0.75,0.75);

    steer.setpoint -= map(ang_vel, 0,1023, -cmd_vel.max_ang_vel*(cmd_vel.duration/1000000.0), cmd_vel.max_ang_vel*(cmd_vel.duration/1000000.0));
    cmd_vel.timer = currenttime;
  }
}

// 2. velocity from odom is added
void velocity_run() {
  velocity.input = odom.cent_velocity;
  VelocityPID.Compute();
  // Serial.print(velocity.setpoint); Serial.print("\t");
  // Serial.print(velocity.input); Serial.print("\t"); Serial.print(velocity.output);
  // Serial.println();

}

// 3. bal setpoint is combination of balance angle and velocity output
// bal input is actual mpu Y angle
void balance_run() {
  bal.setpoint = control.balance_angle + velocity.output;
  bal.input = imu.ypr.pitch;
  BalancePID.Compute();
  // yangle += pitch_correction; // correct for linear acceleration

  // set balance power deadband
  if (bal.output > 0) {
    bal.output += control.min_power;
  } else if (bal.output < 0) {
    bal.output -= control.min_power;
  }
}

// independent pid loop to maintain z angle
void steer_run() {
  // apply gyro drift compensation - NOW DONE IN MPU_FUNCTIONS.H
  // if ((millis() - control.gyro_timer) >= control.gyro_duration) {
  //   steer.input += (mpu.Zangle - control.last_z_angle) + control.gyro_drift_compensation;  // correct for gyro drift about z axis
  //   // Serial.print(mpu.Zangle); Serial.print("\t"); Serial.print(steer.input); Serial.print("\t"); Serial.println(control.gyro_drift_compensation);
  //   control.last_z_angle = mpu.Zangle;
  //   control.gyro_timer = millis();
  // }
  steer.input = imu.ypr.yaw;
  SteerStraightPID.Compute();
  
  // set steer power deadband
  if (steer.output > 0) {
    steer.output += control.min_power;
  } else if (steer.output < 0) {
    steer.output -= control.min_power;
  }
}


void controls_run() {
  currenttime = micros();
  setTunings();
  Command_Velocity(cmd_vel.lin_vel, cmd_vel.ang_vel);

  // Don't start steering until balanced first
  if (millis() - control.startbalancetimer >= 500) {
      control.leftoutput = bal.output + steer.output*1.0;
      control.rightoutput = bal.output - steer.output;
  } else {
      control.leftoutput = bal.output;
      control.rightoutput = bal.output;
  }

  //   // assigning power for non-balancing version
  // if (millis() - control.startbalancetimer >= 500) {
  //     control.leftoutput = -velocity.output + steer.output*1.0;
  //     control.rightoutput = -velocity.output - steer.output;
  // } else {
  //     control.leftoutput = -velocity.output;
  //     control.rightoutput = -velocity.output;
  // }


  // Send Motor Command 
  if (fabs(imu.ypr.pitch) < control.balance_cutoff_angle && fabs(imu.ypr.roll) < control.balance_cutoff_angle) {
    // velocity_run();
    balance_run();
    // steer_run();
    left_motor.command_motor(-control.leftoutput);
    right_motor.command_motor(-control.rightoutput);

  } else {
    left_motor.command_motor(0);
    right_motor.command_motor(0);
    steer.input = imu.ypr.yaw;
    steer.setpoint = steer.input;
    // odom.theta = imu.ypr.yaw;
    control.startbalancetimer = millis();
  }


}




