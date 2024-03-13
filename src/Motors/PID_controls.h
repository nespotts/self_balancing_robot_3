#pragma once

#include <PID_v1.h>

Motor left_motor(4, 5, 6);
Motor right_motor(7, 8, 9);

Odometry odom;
IMU imu;

class PID_Manager {

private:
  uint32_t currenttime;  // micros
public:
  typedef struct {
    double kp;
    double ki;
    double kd;
    double input;
    double output;
    double lower_output_limit;
    double outer_output_limit;
    double setpoint;
    float rate;
  } PID_vars;

  PID_vars velocity{};
  PID_vars bal{};
  PID_vars steer{};

  PID VelocityPID = PID(&velocity.input, &velocity.output, &velocity.setpoint, velocity.kp, velocity.ki, velocity.kd, DIRECT);
  PID BalancePID = PID(&bal.input, &bal.output, &bal.setpoint, bal.kp, bal.ki, bal.kd, DIRECT);
  PID SteerPID = PID(&steer.input, &steer.output, &steer.setpoint, steer.kp, steer.ki, steer.kd, DIRECT);

  typedef struct {
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

  struct cmd_vel_type {
    double lin_vel;
    double ang_vel;
    double rate;
    long timer;
    long duration;
    double max_lin_vel;   // m/s
    double max_ang_vel;  // deg/s
  };

  cmd_vel_type cmd_vel;

  PID_Manager() {

  }

  void setup() {
    control.min_power = 500;
    control.max_power = 4095;
    control.power_range = control.max_power - control.min_power;
    control.balance_angle = -1.0;
    control.balance_cutoff_angle = 40;

    cmd_vel.max_lin_vel = 0.35; // m/s
    cmd_vel.max_ang_vel = 360; // deg/s
    cmd_vel.rate = 50;
    cmd_vel.duration = 1000.0 / cmd_vel.rate * 1000.0;

    // velocity_setup();
    balance_setup();
    // steer_setup();
  }

  void run() {
    currenttime = micros();
    setTunings();


    // Don't start steering until balanced first
    if (millis() - control.startbalancetimer >= 500) {
      control.leftoutput = bal.output;// + steer.output * 1.0;
      control.rightoutput = bal.output;// - steer.output;
    } else {
      control.leftoutput = bal.output;
      control.rightoutput = bal.output;
    }

    // Send Motor Command 
    if (fabs(imu.ypr.pitch) < control.balance_cutoff_angle && fabs(imu.ypr.roll) < control.balance_cutoff_angle) {
      // velocity_run();
      balance_run();
      // steer_run();
      left_motor.command_motor(control.leftoutput);
      right_motor.command_motor(control.rightoutput);
    } else {
      left_motor.command_motor(0);
      right_motor.command_motor(0);
      // steer.input = mpu.Zangle;
      // steer.setpoint = steer.input;
      odom.pose.angle_rad = imu.ypr.yaw * DEG_TO_RAD;
      control.startbalancetimer = millis();
    }
  }

  void velocity_setup() {
    velocity.kp = 10.0;
    velocity.ki = 4.7;
    velocity.kd = 0.02;
    velocity.rate = 100;

    VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
    VelocityPID.SetMode(AUTOMATIC);
    VelocityPID.SetOutputLimits(-control.power_range, control.power_range);
    VelocityPID.SetSampleTime(1000.0 / velocity.rate);
  }

  void balance_setup() {
    bal.kp = 262; //262//105; //110; //130
    bal.ki = 1067;// 1675; //2503; //2507; //2750
    bal.kd = 11.73;//10.6; //5.7; //6.8;  //6.5
    bal.rate = 100;
    bal.setpoint = control.balance_angle;

    BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);
    BalancePID.SetMode(AUTOMATIC);
    BalancePID.SetOutputLimits(-control.power_range, control.power_range);
    BalancePID.SetSampleTime(1000.0 / bal.rate);
  }

  void steer_setup() {
    // setup steering PID - balancing
    steer.kp = 10.0;
    steer.ki = 3.0;
    steer.kd = 0.03;
    steer.rate = 100;
    // steer.setpoint = mpu6050.getAngleZ();  
    SteerPID.SetTunings(steer.kp, steer.ki, steer.kd);
    SteerPID.SetMode(AUTOMATIC);
    SteerPID.SetOutputLimits(-control.power_range, control.power_range);
    SteerPID.SetSampleTime(1000.0 / steer.rate);
  }

  // 1. command starts here
  void Command_Velocity(float lin_vel, float ang_vel) {
    if (currenttime - cmd_vel.timer >= cmd_vel.duration) {

      // balancing version
      velocity.setpoint = map(lin_vel, 0, 1023, -0.35, 0.35);
      // for non-balancing version
      // velocity.setpoint = map(lin_vel, 0, 1023, -0.75,0.75);

      steer.setpoint -= map(ang_vel, 0, 1023, -cmd_vel.max_ang_vel * (cmd_vel.duration / 1000000.0), cmd_vel.max_ang_vel * (cmd_vel.duration / 1000000.0));
      cmd_vel.timer = currenttime;
    }
  }

  // 2. velocity from odom is added
  void velocity_run() {

  }

  void balance_run() {
    bal.setpoint = control.balance_angle;
    bal.input = -imu.ypr.pitch;
    BalancePID.Compute();

    // set balance power deadband
    if (bal.output > 0) {
      bal.output += control.min_power;
    } else if (bal.output < 0) {
      bal.output -= control.min_power;
    }
  }

  void steer_run() {

  }

  void setTunings() {
    char inByte[10];
    int index = 0;

    while (Serial.available() > 0) {
      inByte[0] = Serial.read();
      inByte[1] = '\0';
      // Serial.print(inByte);
      if (strcmp(inByte, "q") == 0) {
        bal.kp *= 1.01;
      } else if (strcmp(inByte, "a") == 0) {
        bal.kp *= 0.99;
      } else if (strcmp(inByte, "w") == 0) {
        bal.ki *= 1.01;
      } else if (strcmp(inByte, "s") == 0) {
        bal.ki *= 0.99;
      } else if (strcmp(inByte, "e") == 0) {
        bal.kd *= 1.01;
      } else if (strcmp(inByte, "d") == 0) {
        bal.kd *= 0.99;
      } else if (strcmp(inByte, "r") == 0) {
        bal.setpoint += 0.1;
      } else if (strcmp(inByte, "f") == 0) {
        bal.setpoint -= 0.1;
      } else if (strcmp(inByte, "g") == 0) {
        velocity.kp *= 0.99;
      } else if (strcmp(inByte, "t") == 0) {
        velocity.kp *= 1.01;
      } else if (strcmp(inByte, "h") == 0) {
        velocity.ki *= 0.99;
      } else if (strcmp(inByte, "y") == 0) {
        velocity.ki *= 1.01;
      } else if (strcmp(inByte, "j") == 0) {
        velocity.kd *= 0.99;
      } else if (strcmp(inByte, "u") == 0) {
        velocity.kd *= 1.01;
      }

      Serial.print(bal.kp, 8); Serial.print("\t"); Serial.print(bal.ki, 8);
      Serial.print("\t"); Serial.print(bal.kd, 8); Serial.print("\t"); Serial.print(bal.setpoint, 2);
      Serial.print("\t"); Serial.print(velocity.kp, 8); Serial.print("\t"); Serial.print(velocity.ki, 8);
      Serial.print("\t"); Serial.println(velocity.kd, 8);

      BalancePID.SetTunings(bal.kp, bal.ki, bal.kd);
      // SteerStraightPID.SetTunings(steer.kp, steer.ki, steer.kd);
      VelocityPID.SetTunings(velocity.kp, velocity.ki, velocity.kd);
      index++;
    }
  }
};






