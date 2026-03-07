/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * SPDX-License-Identifier: MPL-2.0
 */


#include "main.h"

float w_KP = .05;
float w_KI = 0;
float w_KD = 0.5;

float wa_kP = .4;
float wa_kI = 0;
float wa_kD = 0.5;



bool stop_task = false; 
float targer_distance = 0;

void chassis_drive_wall(float distance, float DRIVE_SPEED, bool chain, bool back_sensor) {

  if (back_sensor){
      float distance_for_chassis_ml = -(distance_back.get_distance() - distance)/24.4;
      master.print(0, 0, "%d", distance_back.get_distance() );
      master.print(0, 0, "%.1f", distance_for_chassis_ml);
      chassis.pid_drive_set(distance_for_chassis_ml, DRIVE_SPEED, true);
      if (chain){
        chassis.pid_wait_quick_chain();
      } else {
        chassis.pid_wait();
      }
    
  }

  if (!back_sensor){
      float distance_for_chassis = (distance_front_l.get_distance() - distance)/24.4;
      master.print(0, 0, "%d", distance_front_l.get_distance() );
      master.print(0, 0, "%.1f", distance_for_chassis);
      chassis.pid_drive_set(distance_for_chassis, DRIVE_SPEED, true);
      if (chain){
        chassis.pid_wait_quick_chain();
      } else {
        chassis.pid_wait();
      }
}
}


// void drive_wall(float distance){
//   targer_distance = distance;
//   pros::Task drive_wall_task_running (drive_wall_task);
//   while (stop_task){
//     pros::delay(10);
//   }
//   drive_wall_task_running.remove();
// }

float d_KP = 0.3;
float d_KI = 0;
float d_KD = 0.0021;

void drive_wall(float distance, float DRIVE_SPEED) {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  stop_task = true;
  float error;
  float new_error;
  float prev_error;
  float prev_output;
  float integral;
  float derivative;
  float arrival_time_B = 0;
  float arrival_time_S = 0;
  float arrival_distance_S = 5;
  float arrival_distance_B = 15;
  float time_out_S = 10;
  float time_out_B = 100;
  //float slue_value = 10;

  while (true) {

    //Big error timeout
    if (distance_front_l.get_distance() < distance + arrival_distance_B && distance_front_l.get_distance() > distance - arrival_distance_B){
      if (arrival_time_B == 0){
        arrival_time_B = pros::millis();
      }
    }else {
      arrival_time_B = 0;
    }
    if (arrival_time_B != 0 && (pros::millis() - arrival_time_B) > time_out_B){
      break;
    }
    //Small error timeout
    if (distance_front_l.get_distance() < distance + arrival_distance_S && distance_front_l.get_distance() > distance - arrival_distance_S ){
      if (arrival_time_S == 0){
        arrival_time_S = pros::millis();
      }
    }else {
      arrival_time_S = 0;
    }
    if (arrival_time_S != 0 && (pros::millis() - arrival_time_S) > time_out_S){
      break;
    }

    error = distance_front_l.get_distance() - distance;
    derivative = error - prev_error;
    // if (error == 0){
    //   error = 300;
    // }
    //imu_error = imu_sensor_value - inertial.get_heading();
    //float turn_output = imu_error*0.1;
    float output = (error * d_KP + error * derivative * d_KD + error * integral * d_KI);
    // if ((output -  prev_output) > slue_value){
    //   output = prev_output + slue_value;
    // }
    chassis.drive_set(output,output);

    prev_error = error;
    //if (error < 600){
      integral += error;
    //}

    pros::delay(50);
  }
  chassis.drive_set(0,0);
  // stop_task = false;
  chassis_brake();
}


void wall_alignment_R(float timeout) {
  float error;
  float prev_error;
  float integral;
  float derivative;

  float currentTime = float(pros::millis());
  float kp = 0.35;
  float kd = 2;

  while ((pros::millis() - currentTime) < timeout) {
    error = distance_front_r.get_distance()+5 - distance_back_r.get_distance();
    derivative = error - prev_error;
    float output = error * kp + kd * derivative;

    L1.move_velocity(output);
    L2.move_velocity(output);
    L3.move_velocity(output);
    R1.move_velocity(-output);
    R2.move_velocity(-output);
    R3.move_velocity(-output);

    prev_error = error;
  }

}

void wall_alignment(float timeout) {
  float error;
  float prev_error;
  float integral;
  float derivative;

  float currentTime = float(pros::millis());
  float kp = 0.3;
  float kd = 1000000000000;

  while ((pros::millis() - currentTime) < timeout) {
    error = distance_front_l.get_distance() - distance_back_l.get_distance();
    derivative = error - prev_error;
    float output = error * kp + kd * derivative;

    L1.move_velocity(-output);
    L2.move_velocity(-output);
    L3.move_velocity(-output);
    R1.move_velocity(output);
    R2.move_velocity(output);
    R3.move_velocity(output);

    prev_error = error;
  }

}

void wall_tracking_with_alignment_B(float target_distance, float DRIVE_SPEED, float drive_distance) {
  float error_tracking;
  float error_turn;
  float prev_error_tr;
  float prev_error_tu;
  float integral;
  float derivative_tracking;
  float derivative_turn;
  float output_tracking;
  float output_turn;
  float error_tw = 0.1;

  float currentTime = float(pros::millis());

  while (distance_front.get_distance() < drive_distance) {
    // Track wall
    error_tracking = (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
    derivative_tracking = error_tracking - prev_error_tr;
    output_tracking = error_tracking * w_KP + w_KD * derivative_tracking;
    
    // Calculate turns 
    error_turn = (distance_front_l.get_distance() - distance_back_l.get_distance());
    derivative_turn = error_turn - prev_error_tu;
    output_turn = error_turn * wa_kP + wa_kD * derivative_turn;

    float left = DRIVE_SPEED - output_tracking - output_turn;
    float right  = DRIVE_SPEED + output_tracking + output_turn;

    L1.move_velocity(left);
    L2.move_velocity(left);
    L3.move_velocity(left);
    R1.move_velocity(right);
    R2.move_velocity(right);
    R3.move_velocity(right);

    prev_error_tr = error_tracking;
    prev_error_tu = error_turn;
  }
}

void wall_tracking_with_alignment(float target_distance, float DRIVE_SPEED, float drive_distance) {
  float error_tracking;
  float error_turn;
  float prev_error_tr;
  float prev_error_tu;
  float integral_tr;
  float integral_tu;
  float derivative_tracking;
  float derivative_turn;
  float output_tracking;
  float output_turn;

  while (distance_front.get_distance() > drive_distance) {
    // Track wall
    error_tracking = (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
    derivative_tracking = error_tracking - prev_error_tr;
    output_tracking = (error_tracking * w_KP) + (w_KD * derivative_tracking);
    
    // Calculate turns 
    error_turn = (distance_front_l.get_distance() - distance_back_l.get_distance());
    derivative_turn = error_turn - prev_error_tu;
    output_turn = error_turn * wa_kP + wa_kD * derivative_turn;

    float left = DRIVE_SPEED + output_tracking - output_turn;
    float right  = DRIVE_SPEED - output_tracking + output_turn;

    L1.move_velocity(left);
    L2.move_velocity(left);
    L3.move_velocity(left);
    R1.move_velocity(right);
    R2.move_velocity(right);
    R3.move_velocity(right);

    prev_error_tr = error_tracking;
    prev_error_tu = error_turn;
  }

}

void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance) {
  float error;
  float prev_error;
  float integral;
  float derivative;

  while (1) {
    error = (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
    derivative = error - prev_error;
    float output = error * w_KP + integral * w_KI + w_KD * derivative;

    float left = DRIVE_SPEED + output;
    float right  = DRIVE_SPEED - output;

    L1.move_velocity(left);
    L2.move_velocity(left);
    L3.move_velocity(left);
    R1.move_velocity(right);
    R2.move_velocity(right);
    R3.move_velocity(right);

    prev_error = error;
    integral += error;
  }
}

void wall_riding(float target_distance, float DRIVE_SPEED, float drive_distance) {
  float error;
  float prev_error;
  float integral;
  float derivative;
  float inetial_heading = inertial.get_heading();

  while (1) {
    error = target_distance - distance_front_l.get_distance() * sin(inetial_heading + inertial.get_heading());
    derivative = error - prev_error;
    float output = error * w_KP + integral * w_KI + w_KD * derivative;

    float left = DRIVE_SPEED + output;
    float right = DRIVE_SPEED - output;

    L1.move_velocity(left);
    L2.move_velocity(left);
    L3.move_velocity(left);
    R1.move_velocity(right);
    R2.move_velocity(right);
    R3.move_velocity(right);

    prev_error = error;
    integral += error;
  }
}

void chassis_brake() {
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();
}
