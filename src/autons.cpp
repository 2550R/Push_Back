/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at 
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) 2021-2025, EZ Robotics. The original Source Code was
 * made available under the Mozilla Public License, v. 2.0, at
 * https://github.com/EZ-Robotics/EZ-Template.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include "main.h"

/* Below is a copy-pasted reminder from EZ Template:
 * Odometry and Pure Pursuit ARE NOT MAGIC!
 *
 * It is possible to get perfectly consistent results with tracking wheels,
 * but it is also possible to have extremely inconsistent results with
 * tracking wheels. You can do cool curved motions, but you have to give your
 * robot the best chance to be consistent:
 * 1. avoid wheel slip
 * 2. avoid wheelies
 * 3. avoid throwing momentum around (super harsh turns, like in the example 
 *    below)
 */

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
float p_x;
float p_y;

int screen = 0;

void controller_update() {
  while (true) {
    if (screen == 0) {
      master.print(0, 0, "%d          ", (vertical_tracker.get_position()/100 * 2 * 3.14 * 3.25)/*, intake_bottom.get_current_draw(),distance_front.get_distance()*/);
      pros::delay(100);
    } else {
      master.print(0, 0, "%f/%f      ", p_x, p_y);
      pros::delay(100);
    }
    pros::delay(100);
  }

  std::cout << "Controller output running\n";
}
bool intake_auto_reverse_enabled = true;
void intake_counter_spin(){
  while (true){
      if (intake_distance.get_distance() < 150){
        intake_bottom.move(0);
			  intake_top.move(0);
        break;
      } else {
		  	intake_bottom.move(-90);
			  intake_top.move(-90);
      }
  }

}

int top_speed_intake = 0;
int top_speed_score_intake = 0;
int bottom_speed_intake = 0;
bool change = false;

void anti_jam_auton(){
  float spin_time = 200;
  int v_threshold_top;
  int velocity_top;
  int current_top;
  int velocity_bottom;
  int current_bottom;
  int current_threshold = 1600;
  bool is_jammed_top = false;
  bool is_jammed_bottom = false;
  while(true){
    velocity_bottom = intake_bottom.get_actual_velocity();
    velocity_top = intake_top.get_actual_velocity();
    if (change){pros::delay(300); change = false;}
    current_bottom = intake_bottom.get_current_draw();
    current_top = intake_top.get_current_draw();

    if (current_bottom > current_threshold && abs(velocity_bottom) < 10){
      is_jammed_bottom = true;
    }

    if (current_top > current_threshold && abs(velocity_top) < 10){
      is_jammed_top = true;
    }

    //This one chacks the top motor
    if (is_jammed_top){
      float start_time = pros::millis();
      while (intake_distance.get_distance() > 150 && ((float)pros::millis() - start_time) < spin_time){
        intake_top.move(-top_speed_intake*127);
      }
      is_jammed_top = false;
      intake_top.move(bottom_speed_intake);
      pros::delay(300);
    }

    // This one checks the bottom motor
    if (is_jammed_bottom){
      float start_time = pros::millis();
      while ( ((float)pros::millis() - start_time) < spin_time){
        intake_bottom.move(-bottom_speed_intake*127);
      }
      is_jammed_bottom = false;
      intake_bottom.move(top_speed_intake);
      pros::delay(300);
    }

  }
}

void color_sort_top_auton() {
  color_sort.set_integration_time(10);
  while (true) {
    int hue_lower;
    int hue_higher;
    color_sort.set_led_pwm(100);
    if (color == "B") {
      hue_lower = 210;
      hue_higher = 240;
    } else if (color == "R") {
      hue_lower = 0;
      hue_higher = 10;
    } else {
      continue;
    }

    bool in_proximity = color_sort.get_proximity() > 220;

    if (in_proximity && (hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher)) {
      intake_top_score.move(-127);
      intake_top.move(30);
      pros::delay(300);
      intake_top.move(top_speed_intake);
      intake_top_score.move(top_speed_score_intake);
    }
  }
}

void timer(int timeout){
  if (timeout > pros::millis()/100){
    L1.brake();
    L2.brake();
    L3.brake();
    R1.brake();
    R2.brake();
    R3.brake();

    intake_bottom.brake();
    intake_top.brake();
  }
}

void top_intake(int speed_top){
  change = true;
  intake_top.move(speed_top);
  top_speed_intake = speed_top;
}

void top_intake_score(int speed_top){
  change = true;
  intake_top_score.move(speed_top);
  top_speed_score_intake = speed_top;
}

void bottom_intake(int speed_bt){
  change = true;
  intake_bottom.move(speed_bt);
  bottom_speed_intake = speed_bt;
}



void default_constants() {
  chassis.pid_drive_constants_set(22, 0, 150);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  // large distance turn pid: chassis.pid_turn_constants_set(4.3, 0, 37, 15.0);
  chassis.pid_turn_constants_set(3.2, 0, 22, 12.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(2.4,0.0,34);
  chassis.pid_odom_boomerang_constants_set(1.5, 0.0, 35);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 3_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_turn_constants_set(6_deg, 40);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.3);

  chassis.odom_look_ahead_set(10_in);
  chassis.odom_boomerang_distance_set(16_in); // Max distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.65); // How aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);
  chassis.drive_imu_scaler_set(1);
}

void right_elims_quick(){
  chassis.odom_xyt_set(0_in, 0_in, 30_deg);

  bottom_intake(127);
  trapdoor.set(true);
  chassis.pid_drive_set(30_in, 100);
  pros::delay(450);
  Little_Mech_Mac.set(true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, 60);
  chassis.pid_wait();
  
  chassis.pid_drive_set(30, 100);
  chassis.pid_wait();
  Little_Mech_Mac.set(true);
  chassis.pid_turn_set(180, 60);
  chassis.pid_wait();
  top_intake(127);
  chassis.pid_drive_set(24, 60);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(-32, 100);
  chassis.pid_wait();
  trapdoor.set(false);
}

void intake_test(){
  pros::Task anti_jam_auton1 (anti_jam_auton);


  bottom_intake(127);
  intake_top.move(127);
  intake_top_score.move(127);
}

// FINISHED
void left_middle_top(){
  //Colorsort color set
  color = "B";
  pros::Task color_sor(color_sort_top_auton);


   chassis.odom_xyt_set(0_in, 0_in, -30_deg);

   trapdoor.set(1);

  intake_bottom.move(127);
  intake_top.move(127);

  chassis.pid_drive_set(25.5, 80, true); //24.8 with little bill activation
  pros::delay(500);
  //temp test Little_Mech_Mac.set(true);
  chassis.pid_wait_quick();
  Little_Mech_Mac.set(0);

  pros::delay(400);

  // chassis.pid_drive_set(-2, 80, true);
  // chassis.pid_wait();

  chassis.pid_turn_set(-135, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-16, 80, true);
  chassis.pid_wait();

    intake_top_score.move(-100);

  pros::delay(1200);

  chassis.pid_drive_set(38.5, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(179, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(12.5, 70, true);
    Little_Mech_Mac.set(1);
    intake_bottom.move(127);
    top_intake(127);
    top_intake_score(127);
    //intake_top.move(127);
    //intake_top_score.move(127);
  chassis.pid_wait();

  chassis.pid_drive_set(6.8, 50, true);

  pros::delay(850);

  chassis.pid_drive_set(-27.8, 75, true);
  intake_bottom.move(-20);
    pros::delay(300);
    // pros::Task  color_sort_safe(color_sort_top_auton);
    intake_bottom.move(127);
  chassis.pid_wait();

  trapdoor.set(0);
  Little_Mech_Mac.set(0);

  pros::delay(1400);

  chassis.pid_swing_set(RIGHT_SWING, 68, 90, 2.5, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  right_rush_mech.set(1);

  chassis.pid_drive_set(-28, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-145, 60, true);

}



void left_elims_7ball(){
  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, -30_deg);

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(24.8, 100, true);
  pros::delay(500);
  Little_Mech_Mac.set(true);
  chassis.pid_wait_quick();
  Little_Mech_Mac.set(0);

  pros::delay(150);

  chassis.pid_turn_set(-130, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(18.5, 80, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait();
  pros::delay(150);

  chassis.pid_drive_set(-35, 100, true);


  // int hue_lower = 210;
  // int hue_higher = 250;
  // int current_time = pros::millis();
  // bool in_proximity = color_sort.get_proximity() > 50;
  // while ((pros::millis() - current_time < 1800) || (in_proximity && !(hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher))){
  //   intake_bottom.move(127);
  //   intake_top.move(127);
  //   intake_top_score.move(127);
  // } 

  intake_bottom.move(-20);
  top_intake(127);
  top_intake_score(127);
  //intake_top.move(127);
  //intake_top_score.move(127);
  pros::delay(450);
  intake_bottom.move(127);
  trapdoor.set(0);
  // pros::Task color_sort_left(color_sort_top_auton);
  pros::delay(2200);
  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(3, 100, true);
  chassis.pid_wait_quick_chain();

  trapdoor.set(1);

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 180, 80, 0, true);
  chassis.pid_wait_quick_chain();

  right_rush_mech.set(1);

  chassis.pid_drive_set(-23, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-160, 10, false);
}

// FINISHED
void left_elims_quick(){

  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, -30_deg);

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(24.8, 100, true);
  pros::delay(500);
  Little_Mech_Mac.set(true);
  chassis.pid_wait_quick();
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-130, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-15, 100, true);


  // int hue_lower = 210;
  // int hue_higher = 250;
  // int current_time = pros::millis();
  // bool in_proximity = color_sort.get_proximity() > 50;
  // while ((pros::millis() - current_time < 1800) || (in_proximity && !(hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher))){
  //   intake_bottom.move(127);
  //   intake_top.move(127);
  //   intake_top_score.move(127);
  // } 
  
  intake_bottom.move(-20);
  top_intake(127);
  top_intake_score(127);
  //intake_top.move(127);
  //intake_top_score.move(127);
  pros::delay(200);
  intake_bottom.move(127);
  trapdoor.set(0);
  pros::delay(1600);
  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(3, 100, true);
  chassis.pid_wait_quick_chain();

  trapdoor.set(1);

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 180, 80, 0, true);
  chassis.pid_wait_quick_chain();

  right_rush_mech.set(1);

  chassis.pid_drive_set(-23, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-160, 10, false);



  
}


/*  old
-  // get 3 middle balls
  //pros::Task controller (controller_update); 

  chassis.pid_odom_set({{0_in, 14_in}, fwd, 100}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45.9, 100, true);
  chassis.pid_wait();

  bottom_intake(127);

  chassis.pid_drive_set(20, 30, true);
  chassis.pid_wait();

  //score 3 in middle goal
  chassis.pid_drive_set(-5.6, 60, true);
  chassis.pid_wait();

  // chassis.pid_odom_set({{-9.9_in, 26_in}, fwd, 60}, true);
  // pros::delay(100);
  // chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45, 80);
  intake_bottom.move(-55);
  chassis.pid_wait();


  chassis.pid_drive_set(15, 80);
  chassis.pid_wait();

  intake_bottom.move(-70);
  pros::delay(900);
  intake_top.move(0);
  intake_bottom.move(0);

  // line up for match loader

  chassis.pid_drive_set(-10, 80, true);
  chassis.pid_wait();

  middle_stage.set(0);
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait();

  while (distance_front.get_distance() > 655){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  pros::delay(100);

  chassis.pid_turn_set(91, 60, true);
  chassis.pid_wait();

  while (distance_front.get_distance() > 690){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  pros::delay(50);

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(-179, 80, true);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_drive_set(13.5, 60, true);
  chassis.pid_wait();

  pros::delay(150);

  chassis.pid_drive_set(-33, 60, true);
  chassis.pid_wait();
  intake_top.move(127);
  pros::delay(900);

  // chassis.pid_drive_set(8, 80, true);
  // chassis.pid_wait_quick();

  // chassis.pid_drive_set(-15, 127, false);
}
*/


void right_safe(){

  color = "R";
  pros::Task color_sor(color_sort_top_auton);
  trapdoor.set(1);
  chassis.pid_drive_set(22.6, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(12.3, 70, true);
    Little_Mech_Mac.set(1);
    bottom_intake(127);
    intake_top.move(127);
    intake_top_score.move(127);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(-26.8, 75, true);
  chassis.pid_wait();
  trapdoor.set(0);

  pros::delay(1100);
  trapdoor.set(1);
  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(9, 80, true);
  chassis.pid_wait_quick_chain();
    trapdoor.set(1);

  chassis.pid_turn_set(-138.5, 80, true);

  chassis.pid_wait_quick_chain();

  //Intaking the balls
  chassis.pid_drive_set(30, 60, true);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(20, 80, true);
    intake_piston.set(1);

    pros::delay(100);
    intake_bottom.move(-70);
    intake_top.move(-100);
    intake_top_score.move(-100);
  //Little_Mech_Mac.set(1);
  chassis.pid_wait();

  //Scoring in the middle goal

  
  pros::delay(900);

  intake_piston.set(0);

  chassis.pid_drive_set(-34, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-92, 80, true);
  chassis.pid_wait();
  
  chassis.pid_drive_set(22, 127, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(-135, 80, true);
  chassis.pid_wait();
  
}


// needs updated
void solo_left() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  // trapdoor.set(1);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 400_ms);

  intake_bottom.move(127);
  chassis.pid_drive_set(36.5, 80, true);
  chassis.pid_wait();
  Little_Mech_Mac.set(1);
  pros::delay(50);
  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait_quick();



  chassis.pid_drive_set(13, 90, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);

  intake_bottom.move(127);
  chassis.pid_drive_set(13, 60, true);
  pros::delay(140);

  // chassis.pid_turn_set(180, 80, true);
  // chassis.pid_wait_quick();
  
  chassis.pid_drive_set(-32, 70, true);
  chassis.pid_wait_quick();
  //chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.pid_drive_set(1, 5, true);
  trapdoor.set(1);
  intake_top.move(127);
  pros::delay(1600);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  Little_Mech_Mac.set(0);

  //intake_top.brake();


  chassis.pid_drive_set(4, 100, true);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 46.2, 74, -23.5, true); 
  chassis.pid_wait();
  trapdoor.set(0);

  chassis.pid_drive_set(35, 70, true);
  //pros::delay(700);
  chassis.pid_wait();
  intake_top.move(127);
  chassis.pid_turn_set(-130, 100, true);
  chassis.pid_wait_quick();
  
  chassis.pid_drive_set(-8, 100, true);  
  pros::delay(300);
  trapdoor.set(1);
  chassis.pid_wait_quick();
  


  intake_bottom.move(127);
  intake_top.move(127);
  pros::delay(150);

  trapdoor.set(0);
  /*
  chassis.pid_drive_set(15, 80, true);
  chassis.pid_wait_quick();

  intake_top.move(-127);

  middle_stage.set(0);
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(95, 100, true);
  chassis.pid_wait();
  */
  
  chassis.pid_swing_set(RIGHT_SWING, 92, 110, -24, true); 
  chassis.pid_wait();
  intake_top.brake();

  chassis.pid_drive_set(39, 90, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-41,80, false);
  chassis.pid_wait_quick();
  intake_bottom.move(-127);
  chassis.pid_drive_set(11.5, 90, true);
  chassis.pid_wait_quick();



  // intake_bottom.set_brake_mode(MOTOR_BRAKE_COAST);

  // chassis.pid_swing_set(LEFT_SWING, -37, 70*1.45, 52*1.15, false);
  // chassis.pid_wait_quick();

  // chassis.pid_drive_set(-3, 60, true);
  
  pros::delay(100);

}
void solo_left1() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  // trapdoor.set(1);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 400_ms);

  // while (distance_front.get_distance() > 712){
  //   chassis.pid_drive_set(1000000, 50);
  // }
  // L1.brake();
  // L2.brake();
  // L3.brake();
  // R1.brake();
  // R2.brake();
  // R3.brake();
  intake_bottom.move(127);
  chassis.pid_drive_set(37.5, 80, true);
  chassis.pid_wait();
  Little_Mech_Mac.set(1);
  pros::delay(50);
  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait_quick();



  chassis.pid_drive_set(13, 90, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);

  intake_bottom.move(127);
  chassis.pid_drive_set(13, 60, true);
  pros::delay(140);

  // chassis.pid_turn_set(180, 80, true);
  // chassis.pid_wait_quick();
  
  chassis.pid_drive_set(-32, 100, true);
  // pros::delay(400);
  // intake_top.move(127);
  chassis.pid_wait_quick();
  //chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.pid_drive_set(1, 5, true);
  trapdoor.set(1);
  intake_top.move(127);
  pros::delay(1600);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  Little_Mech_Mac.set(0);

  //intake_top.brake();


  chassis.pid_drive_set(4, 100, true);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 46.2, 74, -18.5, true); 
  chassis.pid_wait();
  trapdoor.set(0);

  chassis.pid_drive_set(35, 70, true);
  //pros::delay(700);
  chassis.pid_wait();
  intake_top.move(127);
  chassis.pid_turn_set(-130, 100, true);
  chassis.pid_wait_quick();
  
  chassis.pid_drive_set(-8, 100, true);  
  pros::delay(200);
  trapdoor.set(1);
  chassis.pid_wait_quick();
  


  intake_bottom.move(127);
  intake_top.move(127);
  pros::delay(150);

  trapdoor.set(0);
  /*
  chassis.pid_drive_set(15, 80, true);
  chassis.pid_wait_quick();

  intake_top.move(-127);

  middle_stage.set(0);
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(95, 100, true);
  chassis.pid_wait();
  */
  
  chassis.pid_swing_set(RIGHT_SWING, 92, 110, -24, true); 
  chassis.pid_wait();
  intake_top.brake();

  chassis.pid_drive_set(33, 90, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-43,80, false);
  chassis.pid_wait_quick();
  intake_bottom.move(-127);
  chassis.pid_drive_set(8.5, 90, true);
  chassis.pid_wait_quick();



  // intake_bottom.set_brake_mode(MOTOR_BRAKE_COAST);

  // chassis.pid_swing_set(LEFT_SWING, -37, 70*1.45, 52*1.15, false);
  // chassis.pid_wait_quick();

  // chassis.pid_drive_set(-3, 60, true);
  
  pros::delay(100);

}

void elims_mid_control (){

  color = "B";
  pros::Task color_sor(color_sort_top_auton);

  trapdoor.set(1);

  chassis.pid_drive_set(21, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  //Going into the machloader
  Little_Mech_Mac.set(1);
  chassis.pid_drive_set(14.5, 80, true);
    intake_bottom.move(80);
    intake_top.move(127);
    intake_top_score.move(127);
  chassis.pid_wait();

  pros::delay(200);

  //intaking the balls from the machloader
  
  chassis.pid_drive_set(-28.5, 75, true);
  chassis.pid_wait();
  Little_Mech_Mac.set(0);

  //Scoring the balls
  trapdoor.set(0);
  pros::delay(1100);

  chassis.pid_drive_set(9, 80, true);
  chassis.pid_wait_quick_chain();
    trapdoor.set(1);

  chassis.pid_turn_set(-138.5, 80, true);

  chassis.pid_wait_quick_chain();

  //Intaking the balls
  chassis.pid_drive_set(30, 80, true);
    pros::delay(650);
  chassis.pid_drive_set(12, 60, true);
  chassis.pid_wait();

  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(20, 60, true);
  chassis.pid_wait();
      pros::delay(100);
    intake_bottom.move(-100);
    intake_top.move(-127);
    intake_top_score.move(0);
  //Scoring in the middle goal

  
  pros::delay(1000);

  intake_piston.set(0);

  chassis.pid_drive_set(-18, 127, true);
  chassis.pid_wait();

  intake_bottom.move(127);
  intake_top.move(100);
  intake_top_score.move(100);

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(43, 127, true);
  pros::delay(700);
  chassis.pid_drive_set(21, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(132, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-15, 80, true);
  chassis.pid_wait(); 
  intake_top_score.move(-127);
  pros::delay(1000);

  chassis.pid_drive_set(8, 80, true);
  chassis.pid_wait();
  pros::delay(100);
  mid_descore.set(1);

  chassis.pid_drive_set(-15, 40, true);
  pros::delay(800);
  chassis.pid_drive_set(0, 0, true);



}

//done
void solo_right (){
  //pros::Task anti_jam_auton1 (anti_jam_auton);
  color = "R";
  pros::Task color_sor(color_sort_top_auton);
  trapdoor.set(1);
  chassis.pid_drive_set(22.6, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(12.3, 70, true);
    Little_Mech_Mac.set(1);
    bottom_intake(127);
    intake_top.move(127);
    intake_top_score.move(127);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_drive_set(-26.8, 75, true);
  chassis.pid_wait();
  trapdoor.set(0);

  pros::delay(1000);
  trapdoor.set(1);

  // chassis.pid_swing_set(LEFT_SWING, -135, 127, -20, true);
  // chassis.pid_wait_quick();

  chassis.pid_drive_set(5, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-143, 80, true);
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23.7, 80, true);
  pros::delay(650);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(179, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(47, 65, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-6.5, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(135, 80, true);
  pros::delay(200);
    intake_top.move(-30);
    intake_top_score.move(-30);
    bottom_intake(-30);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-14.8, 60, true);
    pros::delay(400);
    intake_top.move(127);
    intake_top_score.move(-127);
    bottom_intake(127);
  chassis.pid_wait();

    pros::delay(500);

    intake_top.move(0);
    bottom_intake(0);

  chassis.pid_drive_set(40, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

    intake_top_score.move(127);

  chassis.pid_drive_set(14, 60, true);
    intake_top.move(127);
    intake_top_score.move(127);
    bottom_intake(127);
    Little_Mech_Mac.set(1);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_drive_set(-27, 75, true);
  chassis.pid_wait();
    trapdoor.set(0);

  pros::delay(1100);



}

void skills_without_odom(){ 
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);
  color = "x";

  bottom_intake(127);
  top_intake(127);
  //Instead of while 735 
  // chassis.pid_turn_set(0,60);
  // chassis.pid_wait();

  // chassis.pid_turn_set(-90,60);
  // chassis.pid_wait();

  chassis.pid_drive_set(-10,60);
  chassis.pid_wait();

  drive_wall(500);

  /*
  Empty first match loader
  */
  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(11.5, 90, true);
  chassis.pid_wait();

  //intake_bottom.move(127);
  //intake_top.move(127);

  pros::delay(1350);

  /*
  Back up from match loader
  */

  Little_Mech_Mac.set(0);
  pros::delay(100);

  chassis.pid_drive_set(-9.8, 90, true);
  chassis.pid_wait_quick();

  bottom_intake(0);
  top_intake(0);

  chassis.pid_turn_set(-90, 60, true);
  chassis.pid_wait();

  bottom_intake(0);
  top_intake(0);
  drive_wall(200);

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  


}
void skills_before_changing_the_wall() {  
  pros::Task task1(controller_update);
  //pros::Task color_sort_task_running(color_sort_S);
  drive_wall(510);
  //discore_mech.set(1);
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(0);
  color = "x";

  /*
  Setup for first matchload
  */
  
  // chassis.pid_odom_set({{-34_in, 0_in}, fwd, 127}, true);
  // chassis.pid_wait();
  
  bottom_intake(127);
  top_intake(127);
  // while (distance_front.get_distance() > 787){
  //   chassis.pid_drive_set(1000000, 50);
  // }
  // L1.brake();
  // L2.brake();
  // L3.brake();
  // R1.brake();
  // R2.brake();
  // R3.brake();
  chassis.odom_xy_set(-33, 0);
  pros::delay(100);
  
  /*
  Empty first match loader
  */
  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(11.5, 90, true);
  chassis.pid_wait();

  //intake_bottom.move(127);
  //intake_top.move(127);

  pros::delay(1350);

  /*
  Back up from match loader
  */

  Little_Mech_Mac.set(0);
  pros::delay(100);

  chassis.pid_drive_set(-9.8, 90, true);
  chassis.pid_wait_quick();

  bottom_intake(0);
  top_intake(0);

  chassis.pid_turn_set(-90, 60, true);
  chassis.pid_wait();

  /*
  Relocate to blue side
  */

  chassis.pid_odom_set({{-45_in, 0_in}, fwd, 100}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  /*
  Reset location to 0, 0
  */

  chassis.pid_odom_set({{-46_in, 90_in}, fwd, 110}, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();
  pros::delay(100);

  // Reset odom position with distance sensors
  float position_y = (distance_front.get_distance() - 600) / 25.4;
  float position_x = ((distance_front_l.get_distance() + distance_back_l.get_distance()) / 2 - 160) / 25.4;

  p_x = position_x;
  p_y = position_y;

  chassis.odom_xy_set(position_x, position_y);
  pros::delay(150);

  /*
  Setup for match loader
  */
  
  chassis.pid_turn_set(90, 60, true);
  chassis.pid_wait();

  // chassis.pid_odom_set({{6.7_in, 0_in}, fwd, 110}, true);
  // chassis.pid_wait();
  // screen = 1;
  chassis.pid_drive_set(12_in, 100);
  chassis.pid_wait();
  /*
  Score first match loader
  */

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  top_intake(-20);
  bottom_intake(-20);
  //intake_top.move(127);
  //intake_bottom.move(127);

  chassis.pid_drive_set(-16, 90, true);
  chassis.pid_wait_quick();

  top_intake(127);
  bottom_intake(127);

  chassis.pid_drive_set(-100, 10, true);
  trapdoor.set(1);
  pros::delay(100);
  trapdoor.set(1);
  pros::delay(1700);

  /*
  empty second match loader
  */

  chassis.pid_drive_set(31.5, 100, true);

  pros::delay(1800);

  /*
  Score second 6 on long goal
  */
  trapdoor.set(0);

  Little_Mech_Mac.set(0);

  pros::delay(100);

  chassis.pid_drive_set(-29.5, 60, true);
  chassis.pid_wait();

  top_intake(-15);
  bottom_intake(-15);

  chassis.pid_drive_set(-5, 20, true);
  pros::delay(150);

  top_intake(127);
  bottom_intake(127);

  trapdoor.set(1);

  pros::delay(1000);


  /*
  Set up for first 2 middle balls
  */

  // Activate color sort
  
  
  chassis.pid_drive_set(10, 110, true);
  chassis.pid_wait();

  chassis.pid_turn_set(88, 80, true);
  chassis.pid_wait();
  trapdoor.set(0);

  // chassis.pid_odom_set({{26.8, -10.5}, fwd, 80}, true);
  // chassis.pid_wait_quick();

  chassis.pid_drive_set(80, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-1.5, 60);
  chassis.pid_wait();

  while (distance_front.get_distance() > 730){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();
  pros::delay(200);

  
  //drive_wall(450);

  chassis.pid_turn_set(88.5, 60);
  chassis.pid_wait();

  while (distance_front.get_distance() > 756){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();
  pros::delay(200);

  //drive_wall(450);

  chassis.pid_turn_set(-3, 60);
  chassis.pid_wait();

  pros::delay(150);

  /*
  reset position
  */

  float position_y_1 = (distance_front.get_distance() - 525) / 25.4;
  float position_x_1 = ((distance_front_r.get_distance() + distance_back_r.get_distance()) / 2 - 435) / 25.4;

  p_x = position_x_1;
  p_y = position_y_1;

  pros::delay(150);

  chassis.odom_xy_set(position_x_1, position_y_1);
  chassis.pid_wait();

  /*
  grab third match load
  */

  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(24, 60, true);
  chassis.pid_wait();
  
  top_intake(127);
  bottom_intake(127);

  //intake_top.move(127);
  //intake_bottom.move(127);

  pros::delay(1300);

  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(-20, 60, true);
  chassis.pid_wait();

  top_intake(0);
  bottom_intake(0);

  chassis.pid_turn_set(-88, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-11.9, 60, true);
  chassis.pid_wait();

  /*
  go to blue side
  */

  chassis.pid_turn_set(-3, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-85, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_set(86, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-11, 80, true);
  chassis.pid_wait();

  /*
  score third set of match load
  */

  chassis.pid_turn_set(176, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-19.5, 60, true);
  chassis.pid_wait();

  top_intake(127);
  bottom_intake(127);

  trapdoor.set(1);

  pros::delay(2000);

  /*
  empty second match loader
  */

  
  
  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(35, 80, true);
  trapdoor.set(0);

  pros::delay(2200);

  /*
  Score second 6 on long goal
  */

  Little_Mech_Mac.set(0);

  pros::delay(100);

  chassis.pid_turn_set(179, 60, true);

  chassis.pid_drive_set(-33.5, 60, true);
  chassis.pid_wait();

  top_intake(127);
  bottom_intake(127);

  trapdoor.set(1);

  pros::delay(2000);

  chassis.odom_xyt_set(0, 0, 180);

  chassis.pid_drive_set(5, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-95, 127, true);
  chassis.pid_wait();

  chassis.pid_drive_set(48, 127, true);
  chassis.pid_wait();

  chassis.pid_turn_set(178, 127, true);
  chassis.pid_wait();

  chassis.pid_drive_set(100, 127, false);
  chassis.pid_wait_quick();

}

void new_skills(){
  discore_mech.set(0);
  trapdoor.set(1);
  intake_piston.set(0);

  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(127);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 100_ms);

  //start wiggle

  chassis.pid_turn_set(5, 30, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-5 , 30, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait_quick_chain();

  //drive in and back out and back in

  chassis.pid_drive_set(5, 100, true);
  chassis.pid_wait_quick_chain();

  //wiggle part 2

  chassis.pid_turn_set(5, 30, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-5 , 30, true);
  chassis.pid_wait_quick_chain();
  pros::delay(200);
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait_quick_chain();

  //back out

  chassis.pid_drive_set(-30, 80, true);
  chassis.pid_wait_quick_chain();

  //line up on the wall

  chassis.pid_turn_set(0, 80, true);
  chassis.pid_wait();

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);

  
  while (distance_front_l.get_distance() < 800){
    chassis.pid_drive_set(-1000000, -40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  pros::delay(200);

  //line up on mid goal

  chassis.pid_turn_set(-25, 80, true);
  chassis.pid_wait();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 200_ms, 300_ms);


  chassis.pid_drive_set(-16, 80, true);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);


  chassis.pid_swing_set(RIGHT_SWING, 45, -80, 0, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(45, 80, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-5, 80, true);
  chassis.pid_wait_quick_chain();

  /*7 ball score*/
  intake_top_score.move(-80);
  chassis.pid_drive_set(1, 60, true);
  chassis.pid_wait();
  pros::delay(2000);

  //grab 7th ball

  chassis.pid_drive_set(5, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-4.5, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1, 60, true);
  chassis.pid_wait();

}






void skills() {
  discore_mech.set(0);
  //grab middle balls
  trapdoor.set(1);
  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(100);

  chassis.pid_drive_set(2, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-46, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(28, 80, true);
  pros::delay(400);
  Little_Mech_Mac.set(1);
  chassis.pid_wait();
  // TO make shure that we are grabbing all 4 balls
  pros::delay(100);

  chassis.pid_drive_set(-2, 80, true);
  chassis.pid_wait_quick_chain();

  //score middle balls
  Little_Mech_Mac.set(0);
  chassis.pid_turn_set(-135, 80, true);
  chassis.pid_wait_quick_chain();
  //Moving the intake backwards to prevent jamming
  intake_top.move(-10);
  intake_top_score.move(-10);

  chassis.pid_drive_set(-14, 60, true);
  chassis.pid_wait_quick_chain();

  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(-100);

  //Little_Mech_Mac.set(0);

  pros::delay(1500);

  //line up with first match loader

  chassis.pid_drive_set(25, 80, true);
  chassis.pid_wait_quick_chain();

  intake_top_score.move(127);

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait_quick_chain();

  while (distance_front_l.get_distance() > 600){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  chassis.pid_turn_set(-90, 60, true);
  chassis.pid_wait_quick_chain();

  while (distance_front_l.get_distance() > 600){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(1);

  trapdoor.set(1);

  chassis.pid_drive_set(13, 60, true);
  chassis.pid_wait();
  pros::delay(750); 

  //cross to other side

  chassis.pid_drive_set(-10, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.5, 80, true);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_set(1, 80, true);
  chassis.pid_wait_quick_chain();

  intake_top.move(0);
  intake_bottom.move(0);
  intake_top_score.move(0);

  chassis.pid_drive_set(78, 80, true);
  chassis.pid_wait();

  //score first match loader

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(4.2, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-13, 80, true);  
  
  pros::delay(100);
  intake_top.move(-20);
  intake_bottom.move(-20);
  intake_top_score.move(30);
  chassis.pid_wait();

  trapdoor.set(0);

  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(127);

  pros::delay(1700);

  //grab second match loader

  Little_Mech_Mac.set(1);

  trapdoor.set(1);

  chassis.pid_drive_set(28, 60, true);
  chassis.pid_wait();
  pros::delay(750); 

  //score second match loader

  chassis.pid_turn_set(2, 80, true);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(-35.5, 80, true);
  pros::delay(800);
  intake_top.move(-30);
  intake_bottom.move(-30);
  intake_top_score.move(30);
  chassis.pid_wait();

  trapdoor.set(0);
  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(127);
  pros::delay(2000);
  Little_Mech_Mac.set(0);

  //The part were we line up for clearing

  chassis.pid_drive_set(8, 127, true);
  chassis.pid_wait_quick_chain();

  trapdoor.set(1);

  chassis.pid_turn_set(50, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 87, 85, 0, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(86, 80, true);
  chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(5, 127, true);
  // chassis.pid_wait_quick_chain();

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(70, 127, true);
  pros::delay(300);
  Little_Mech_Mac.set(1);
  pros::delay(800);
  Little_Mech_Mac.set(0);
  chassis.pid_wait();

  Little_Mech_Mac.set(0);
  chassis.pid_turn_set(95, 127, true);
  pros::delay(200);
  Little_Mech_Mac.set(0);
  pros::delay(100);


  chassis.pid_turn_set(0, 80, true);
  chassis.pid_wait_quick_chain();


  while (distance_front_l.get_distance() < 600){
    chassis.pid_drive_set(-1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  chassis.pid_turn_set(90, 60, true);
  chassis.pid_wait_quick_chain();

  while (distance_front_l.get_distance() > 600){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  chassis.pid_turn_set(0, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(1);
  trapdoor.set(1);

  chassis.pid_drive_set(20.5, 60, true);
  chassis.pid_wait();
  pros::delay(1050); 

  //cross to other side


  chassis.pid_drive_set(-10, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(5.5, 80, true);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_set(179, 80, true);
  chassis.pid_wait_quick_chain();

  intake_top.move(0);
  intake_bottom.move(0);
  intake_top_score.move(0);

  chassis.pid_drive_set(78, 80, true);
  chassis.pid_wait();

  //score first match loader

  chassis.pid_turn_set(-90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(5, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-12, 80, true);    
  pros::delay(200);
  intake_top.move(-40);
  intake_bottom.move(-40);
  intake_top_score.move(40);
  chassis.pid_wait();

  trapdoor.set(0);

  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(127);

  pros::delay(1700);

  //grab second match loader

  Little_Mech_Mac.set(1);

  trapdoor.set(1);

  chassis.pid_drive_set(27.5, 80, true);
  chassis.pid_wait();
  pros::delay(1450);

  //score second match loader
  
  chassis.pid_drive_set(-35.8, 60, true);
  pros::delay(800);
  intake_top.move(-20);
  intake_bottom.move(-20);
  intake_top_score.move(20);
  chassis.pid_wait();

  trapdoor.set(0);

  intake_top.move(127);
  intake_bottom.move(127);
  intake_top_score.move(127);

  trapdoor.set(1);
  Little_Mech_Mac.set(0);
  pros::delay(2000);
  

  //The part were we line up for parking
  
  chassis.pid_drive_set(8, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-130, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19.5, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, -93, 85, 0, true);
  chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(5, 127, true);
  // chassis.pid_wait_quick_chain();



  intake_bottom.move(127);
  chassis.pid_drive_set(45, 127, true);
  pros::delay(250);
  Little_Mech_Mac.set(1);
  pros::delay(1000);

  chassis.pid_wait();

  Little_Mech_Mac.set(0);



}


/*   old
void new_elim_auton(){
  // pros::Task anti (anti_jam_auton);
  trapdoor.set(1);
  chassis.pid_drive_set(14.1, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-45.9, 100, true);
  chassis.pid_wait();

  bottom_intake(127);
  top_intake(127);

  chassis.pid_drive_set(20, 30, true);
  chassis.pid_wait();

  //score 3 in middle goal
  chassis.pid_drive_set(-4, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait();

  while (distance_front.get_distance() > 655){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  pros::delay(50);

  chassis.pid_turn_set(-91, 60, true);
  chassis.pid_wait();

  while (distance_front.get_distance() > 725){
    chassis.pid_drive_set(1000000, 40);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

  pros::delay(100);

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(179, 80, true);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_drive_set(15.5, 60, true);
  chassis.pid_wait();

  pros::delay(90);

  chassis.pid_drive_set(-31, 80, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);
  trapdoor.set(0);
  
  chassis.pid_drive_set(-2, 80, true);
  chassis.pid_wait_quick();
}
*/


/*   old
void blue_top_quals() {
  // pros::Task anti (anti_jam_auton);
  trapdoor.set(1);
  intake_top.move(127);
  chassis.odom_xyt_set(0_in, 0_in, 152.5_deg);
  chassis.pid_drive_constants_set(27, 0, 135);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 100_ms, 3_in, 200_ms, 200_ms);
  chassis.pid_odom_set({{{-10.6_in, 18_in}, rev, 127},
                      {{-12_in, 22_in}, rev, 127},
                      {{-11.2_in, 27_in}, rev, 127},
                      {{-15_in, 42.3_in}, rev, 127}},
                      true);

  pros::delay(950);
  right_rush_mech.set(1);
  chassis.pid_wait_quick();

  chassis.pid_drive_constants_set(22, 0, 130);

  intake_bottom.move(127);

  chassis.pid_swing_set(RIGHT_SWING, 80, 60, 17);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(10, 60, true);
  chassis.pid_wait_quick();

  pros::delay(350);
  //Turn that on when we have the pump
  right_rush_mech.set(0);
  pros::delay(250);

  chassis.pid_drive_set(-14, -80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(125, 60, true);
  intake_bottom.move(80);
  intake_top.move(0);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, 90, 80, 40, true);
  chassis.pid_wait_quick();

  intake_bottom.move(0);

  chassis.pid_odom_set({{14.6, 36.8}, fwd, 100}, true);
  chassis.pid_wait_quick();

  middle_stage.set(1);
  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(45, 60, true);
  chassis.pid_wait();
  
  chassis.pid_drive_set(8.5, 60, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  pros::delay(550);

  chassis.pid_odom_set({{-11_in, 8_in}, rev, 100}, true);
  chassis.pid_wait_quick();
  
  middle_stage.set(0);

  chassis.pid_turn_set(173, 60, true);
  chassis.pid_wait_quick();

  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(13, 60, true);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_drive_set(-31, 80, true);
  chassis.pid_wait();

  trapdoor.set(0);
}
*/


/*   old
void blue_bottom_elims() {
  trapdoor.set(1);
  intake_top.move(127);
  chassis.odom_xyt_set(0_in, 0_in, -152.5_deg);
  chassis.pid_drive_constants_set(30, 0, 129);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 100_ms, 3_in, 200_ms, 200_ms);
  chassis.pid_odom_set({{{9_in, 18_in}, rev, 127},
                      {{10_in, 22_in}, rev, 127},
                      {{11_in, 28_in}, rev, 127},
                      {{12.5_in, 43_in}, rev, 127}},
                      true);
  chassis.pid_wait_quick();
  // right_rush_mech.set(1);
  chassis.pid_drive_constants_set(22, 0, 130);
  pros::delay(200);

  chassis.pid_swing_set(LEFT_SWING, -105, 60);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_swing_set(RIGHT_SWING, -150, 60, 30);
  chassis.pid_wait();

  chassis.pid_swing_set(LEFT_SWING, -100, 60, 20);
  chassis.pid_wait();
  //Turn that on when we have the pump
  // right_rush_mech.set(0);
  pros::delay(200);

  chassis.pid_drive_set(-15, -40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-140, -60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(LEFT_SWING, -90, 60, 30, true);
  chassis.pid_wait_quick();

  chassis.pid_odom_set({{13.5_in, 14_in}, fwd, 80}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-172, 60, true);
  chassis.pid_wait_quick();
  //wall_alignment_R(1000);

  // blooper
  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(18, 50, true);
  chassis.pid_wait();

  pros::delay(500);

  intake_top.move(127);

  chassis.pid_drive_set(-32, 70, true);
  chassis.pid_wait();

  trapdoor.set(0);
}
*/


/*   old
void blue_bottom_quals() {
  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, -166_deg);

  chassis.pid_odom_set({{{14_in, 18_in}, rev, 127},
                      {{11_in, 22_in}, rev, 127},
                      {{10.9_in, 30_in}, rev, 110},
                      {{8.3_in, 46.3_in}, rev, 110}},
                      true);
  chassis.pid_wait();

  right_rush_mech.set(1);
  pros::delay(300);

  chassis.pid_swing_set(RIGHT_SWING, -90, 60);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, -160, 60);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, -100, 60, 20);
  chassis.pid_wait();

  right_rush_mech.set(0);
  pros::delay(350);

  chassis.pid_drive_set(-20, 40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-150, 60, true);
  chassis.pid_wait_quick();

  intake_top.move(127);
  intake_bottom.move(50);

  chassis.pid_swing_set(RIGHT_SWING, -80, 60, 20, true);
  chassis.pid_wait();

  intake_bottom.move(0);

  chassis.pid_odom_set({{-16, 41}, fwd, 60}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-45, 60, true);
  chassis.pid_wait();

  middle_stage.set(1);

  pros::delay(1000);
  
  chassis.pid_drive_set(8.5, 60, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  pros::delay(1500);

  chassis.pid_odom_set({{17.5_in, 13_in}, rev, 80}, true);
  chassis.pid_wait();
  
  middle_stage.set(0);

  pros::delay(250);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait();

  // blooper

  chassis.pid_drive_set(12, 60, true);
  chassis.pid_wait();

  pros::delay(1500);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-28, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);
}
*/

/* Odom TESTING Functions */

void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  chassis.pid_wait();
}

void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

void square_odom_test() {
  chassis.pid_turn_set(3600_deg, 40, ez::raw);
  chassis.pid_wait();
  // chassis.pid_odom_set({{ 0_in, 24_in }, fwd, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{ 24_in, 24_in }, rev, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{ 24_in, 0_in }, rev, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_odom_set({{ 0_in, 0_in }, fwd, DRIVE_SPEED});
  // chassis.pid_wait();
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
}

void auton_setup_left() {
  chassis.pid_drive_set(-1.8_in, 30, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-24.9_deg, 30, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-11.7, 30, true);
  chassis.pid_wait();

}

void auton_setup_right(){
  chassis.pid_drive_set(-1.8_in, 30, true);
  chassis.pid_wait();

  chassis.pid_turn_set(24.9_deg, 30, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-11.7, 30, true);
  chassis.pid_wait();
}

/* TESTS */
void pid_test(){

  chassis.pid_drive_set(24, 127, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12, 127, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12, 127, true);
  chassis.pid_wait();
}
void wall_tracking_test() {
  drive_wall(450);
}
void wall_alignment_test() {
  pros::Task task1(controller_update);
  wall_tracking_with_alignment(145, 100, 500);

  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();
}
void pid_tune(){
  //pros::Task controller (color_sort_top_auton);
  //pros::Task controller1 (anti_jam_auton);
  int intake1 = 60;
  trapdoor.set(1);
  bottom_intake(127);
  top_intake(127);
  top_intake_score(127);
  
  pros::delay(3000);  

  top_intake(-50);
  top_intake_score(-50);
  pros::delay(600);
  top_intake_score(0);
  top_intake(-80);
  pros::delay(300);
  //0 pauses
  top_intake(intake1);
  top_intake_score(-40);


  // //1 pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //second pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //third pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //fourth pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //fifth pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // // sixth pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //seventh pause
  // top_intake(0);
  // top_intake_score(0);
  // pros::delay(200);
  // //Continue
  // top_intake(intake1);
  // top_intake_score(-40);
  // pros::delay(300);
  // //final stop
  // top_intake(0);
  // top_intake_score(0);
}
void color_sort_test(){
  color = "B";
  pros::Task color_sor(color_sort_S);
  pros::Task antij(anti_jam_auton);
  top_intake(120);
  bottom_intake(120);
  color = "B";
  trapdoor.set(1);
  intake_top.move(127);  
  intake_bottom.move(127);  
}
void PIDvsODOM(){
  /*
  chassis.pid_drive_set(48, 80, true);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_turn_set(90, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(48, 80, true);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(48, 80, true);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_turn_set(-90, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(48, 80, true);
  chassis.pid_wait();

  pros::delay(250);

  chassis.pid_turn_set(0, 60, true);
  */

  chassis.pid_odom_set({{0_in, 48_in}, fwd, 100}, true);
  chassis.pid_wait();

    chassis.pid_turn_set(90, 60);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 48_in}, fwd, 100}, true);
  chassis.pid_wait();

    chassis.pid_turn_set(180, 60);
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 0_in}, fwd, 100}, true);
  chassis.pid_wait();

    chassis.pid_turn_set(-90, 60);
  chassis.pid_wait();
  
  chassis.pid_odom_set({{0_in, 0_in}, fwd, 100}, true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_turn_set(0, 60);
  chassis.pid_wait();
}
