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

float w_KP = 0.1;
float w_KI = 0;
float w_KD = 0;

float wa_kP = .1;
float wa_kI = 0;
float wa_kD = 0;

float d_KP = 0.15;
float d_KI = 0.00007;
float d_KD = 0;

void drive_wall(float distance, float speed){
  float error;
  float prev_error;
  float integral;
  float derivative;
  while (distance_front.get_distance() > distance){
    error = distance_front.get_distance() - distance ;
    derivative = error - prev_error;
    float output = (error * d_KP + error * derivative * d_KD + error * integral * d_KI);

    L1.move_velocity(output);
    L2.move_velocity(output);
    L3.move_velocity(output);
    R1.move_velocity(output);
    R2.move_velocity(output);
    R3.move_velocity(output);

    prev_error = error;
    if (error < 600){
      integral += error;
    }
    

    pros::delay(50);
  }
}

void wall_alignment(float timeout) {
  float error;
  float prev_error;
  float integral;
  float derivative;

  float currentTime = float(pros::millis());

  while ((pros::millis() - currentTime) < timeout) {
    error = distance_front_l.get_distance() - distance_back_l.get_distance();
    derivative = error - prev_error;
    float output = error * wa_kP + wa_kD * derivative;

    L1.move_velocity(-output);
    L2.move_velocity(-output);
    L3.move_velocity(-output);
    R1.move_velocity(output);
    R2.move_velocity(output);
    R3.move_velocity(output);
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
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();
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
