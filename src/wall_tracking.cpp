#include "main.h"

float w_KP = 0.1;
float w_KI = 0;
float w_KD = 3;

float wa_kP = .17;
float wa_kI = 0.0;
float wa_kD = .01;

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

void wall_tracking_with_alignment(float target_distance, float DRIVE_SPEED, float drive_distance) {
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

  while (1) {
    // Track wall
    error_tracking = (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
    derivative_tracking = error_tracking - prev_error_tr;
    output_tracking = error_tracking * w_KP + w_KD * derivative_tracking;
    
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
