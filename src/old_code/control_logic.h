// Anti Tip Over Parameters *************************************************
// float MaxSpeedForwardTilt = 150; //200
// float MaxSpeedTiltAngle = 5.0;     //4.0
// int num_avg_points = 60;
// float maxPower[100];
// int maxMotorPower = 0;
// double avgPower[100];
// double avgMotorPower = 0;
// int anti_tip_index = 0;

// void AntiTipOver() {
//     float max_sum = 0;
//     double avg_sum = 0;
//     avgPower[anti_tip_index] = (control.leftoutput + control.rightoutput)/2.0;
//     maxPower[anti_tip_index] = max(fabs(control.leftoutput), fabs(control.rightoutput));

//     for(int i=0; i<num_avg_points; i++) {
//         max_sum += maxPower[i];
//         avg_sum += avgPower[i];
//     }

//     if (anti_tip_index == num_avg_points-1){
//         anti_tip_index = 0;
//     } else {
//         anti_tip_index++;
//     }
//     // avgMotorPower = constrain(sum/(float)num_avg_points, 0, MaxSpeedForwardTilt);
//     maxMotorPower = max_sum/(float)num_avg_points;
//     avgMotorPower = avg_sum/(double)num_avg_points;
// }


// long startbalancetimer = 0;

// double last_avg_power = 0;
// long last_acc_time = 0;
// double acceleration = 0;
// double pitch_correction = 0;
// double correction_constant = -8;


// void calc_vel_acc(){
//   acceleration = (avgMotorPower - last_avg_power) / ((double)currenttime - (double)last_acc_time);
//   pitch_correction = atan2(acceleration, 9.81) * correction_constant;
//   last_avg_power = avgMotorPower;
//   last_acc_time = currenttime;
// }


// int autostate[2];
// double autoforwardvel;
// double autotargetangle;
// float autoangulardirection;
// int lastmode;

// void MakeDecision() {
//     float max_dist = 0;
//     float min_dist = 500;
//     float max_dist_index;
//     float min_dist_index;
//     float min_dist_angle;

//     double anglechange;
//     // determine best direction to go and command EVery 5th echo
//     for (int i=0; i<5; i++) {
//         // Account for position of sonar sensor on robot and reading direction
//         if (i == 0 || i == 4) { 
//             sonar_distance[i] -= 8.0;
//         } else if (i == 1 || i ==3) {
//             sonar_distance[i] -= 5.0;
//         } else {
//             sonar_distance[i] -= 2.5;
//         }

//         if (sonar_distance[i] > max_dist) {
//             max_dist = sonar_distance[i];
//             max_dist_index = i;
//         }
//         if (sonar_distance[i] < min_dist) {
//             min_dist = sonar_distance[i];
//             min_dist_index = i;
//         }
//     }

//     if (min_dist <= 30) {
//         min_dist_angle = ((4-min_dist_index)*45.0) - 90.0;
//         if (min_dist_angle == -90) {
//             anglechange = 25;
//             autoforwardvel = 4;
//         } else if (min_dist_angle == -45) {
//             anglechange = 45;
//             autoforwardvel = -3;
//         } else if (min_dist_angle == 0) {
//             anglechange = 90;
//             autoforwardvel = -4;
//         } else if (min_dist_angle == 45) {
//             anglechange = -45;
//             autoforwardvel = -3;
//         } else if (min_dist_angle == 90) {
//             anglechange = -25;
//             autoforwardvel = 4;
//         }
//     } else {
//         anglechange = ((4-max_dist_index)*45.0)-90.0;
//         autoforwardvel = map(max_dist , 0.0, 500.0, 4.3, 5.0);
//     }

//     if (anglechange < 0) {
//         autoangulardirection = -1;
//     } else {
//         autoangulardirection = 1;
//     }

//     autotargetangle = steerangle + anglechange;
//     autostate[0] = 2;
// }


// void AutoMode() {

//     double forward_vel;
//     double angular_vel;

//     if (autostate[0] == 0) {
//         // measure all 5 distances
//         MeasureDistances();
//         angular_vel = 0;
//     } else if (autostate[0] == 1) {
//         // Make Decision
//         MakeDecision();
//         angular_vel = 0;
//     } else if (autostate[0] == 2) {
//         // Change Direction
//         if (autotargetangle == steerangle && autostate[1] == 0) {
//             autostate[0] = 3;
//             autostate[1] = 0;
//             angular_vel = 0;
//             forward_vel = autoforwardvel;
//         } else if (autostate[1] == 0) {
//             // forward_vel = 0;
//             angular_vel = ang_vel*autoangulardirection;
//             forward_vel = autoforwardvel;
//             autostate[1] = 1;
//         }
//         if (fabs(autotargetangle - steerangle) <= 1 && autostate[1] == 1) {
//             angular_vel = 0;
//             autostate[1] = 2;
//         }
//         if (fabs(steerangle - zangle) <= 10 && autostate[1] == 2) {
//             angular_vel = 0;
//             autostate[0] = 3;
//             autostate[1] = 0;
//         }
        
//     } else if (autostate[0] == 3) {
//         forward_vel = autoforwardvel;
//         angular_vel = 0;
//         autostate[0] = 0;
//         servopos = 0; // initialize servo position (will get changed to 180)
//         distance_index = 0;
//     }

//     // apply values to PID controller setpoints all the time
//     steerangle += angular_vel;
//     balance_angle_PID = balance_angle + map(forward_vel,-5.12,5.12,-MaxSpeedTiltAngle,MaxSpeedTiltAngle)*((MaxSpeedForwardTilt-maxMotorPower)/MaxSpeedForwardTilt);

// }