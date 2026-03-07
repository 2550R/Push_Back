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
      master.print(0, 0, "%f          ", (inertial.get_heading())/*, intake_bottom.get_current_draw(),distance_front.get_distance()*/);
      pros::delay(100);
    } else {
      master.print(0, 0, "%f/%f      ", p_x, p_y);
      pros::delay(100);
    }
  }

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


int auton_count_color = 0;
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
    
    if (in_proximity && ((hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher) || (color == "R" && color_sort.get_hue() > 340))){
      auton_count_color ++;
    } else {
      auton_count_color = 0;
    }

    if (auton_count_color >= 6) {
      pros::delay(50);
      intake_top_score.move(-127);
      intake_top.move(20);
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
  chassis.pid_drive_constants_set(22, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.2, 0, 22, 12.0); // 3.2 0 22 12.0
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  // large distance turn pid: chassis.pid_turn_constants_set(4.3, 0, 37, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(2.4,0.0,34);
  chassis.pid_odom_boomerang_constants_set(1.5, 0.0, 35);

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
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


/////////////////////////////////////////////////////////
//                      AUTONS                        //
////////////////////////////////////////////////////////

// unfinished
void top_middle_bottom(){
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);
  

    pros::Task color_sor(color_sort_top_auton);
  trapdoor.set(1);
  chassis.pid_drive_set(21, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 80, true);
  pros::delay(500);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(12.3, 70, true);
    Little_Mech_Mac.set(1);
    bottom_intake(127);
    intake_top.move(127);
    intake_top_score.move(127);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 60);
  pros::delay(200);

  chassis.pid_drive_set(-28.8, 75, true);
  chassis.pid_wait();
  trapdoor.set(0);

  pros::delay(1000);
  trapdoor.set(1);

  //Scoring on a high goal
  pros::delay(1100);
  trapdoor.set(1);
  Little_Mech_Mac.set(0);

  //Getting the 3 balls in the middle 
  // chassis.pid_drive_set(13.5, 127, true);
  // chassis.pid_wait_quick_chain();
  //   trapdoor.set(1);

  // chassis.pid_turn_set(45, 80, true);

  // chassis.pid_wait_quick_chain();

  
  chassis.pid_turn_set(-88, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(16, 100, true);
  chassis.pid_wait_quick_chain();

  //Intaking the balls
  // chassis.pid_drive_set(35, 100, true);
  // chassis.pid_wait();
  // pros::delay(300);

  chassis.pid_turn_set(-135, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-18, 80, true);
  chassis.pid_wait_quick(); 
  trapdoor.set(0);

  //Scoring the balls
  intake_top.move(127);
  intake_top_score.move(-60);
  pros::delay(600);
  intake_top_score.move(0);
  intake_top.move(-40);

  //Grabbing 3 more balls
  chassis.pid_drive_set(16, 100, true);
  chassis.pid_wait_quick(); 
  trapdoor.set(0);

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(42, 100, true);
  chassis.pid_wait();
  pros::delay(200);

  chassis.pid_turn_set(-45, 80, true);
  chassis.pid_wait_quick_chain();

  //Scoring the 3 balls
  intake_bottom.move(-70);
  chassis.pid_drive_set(15, 80, true);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-1, 30, true);
  pros::delay(3000);

}

// finished & checked
void left_4_3_push(){
  pros::Task color_sor(color_sort_top_auton);
  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(RIGHT_SWING, -12, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-138, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.7, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(-180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_drive_set(11.8, 70, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(680);

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-27, 80, true);
  pros::delay(450);
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  trapdoor.set(0);
  pros::delay(800);
  trapdoor.set(1);

  chassis.pid_drive_set(4.5, 80, true);
  chassis.pid_wait_quick_chain();

  discore_mech.set(1);

  chassis.pid_turn_set(-131, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_exit_condition_set(0_ms, 4_deg, 250_ms, 7_deg, 500_ms, 100_ms);
  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_swing_set(LEFT_SWING, 180, 127, 22, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-14, 80, true);
  chassis.pid_wait();

  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(10, 80, true);
  discore_mech.set(0);  
  chassis.pid_wait_quick_chain();

  chassis_drive_wall(815, 80, false, false);

  chassis.pid_turn_set(-135, 60, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-35, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(2, 1, true);

  intake_bottom.move(-80);
  intake_top.move(-80);
  intake_top_score.move(-80);

  pros::delay(200);

  intake_bottom.move(127);
  intake_top.move(50);
  intake_top_score.move(-35);
  trapdoor.set(1);

  int waittime = pros::millis();

  while (pros::millis() - waittime  <= 1200) { 
    if(color == "R"){
      if (color_sort.get_hue() > 0 && color_sort.get_hue() < 10){
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
    if(color == "B"){  
      if (color_sort.get_hue() > 210 && color_sort.get_hue() < 240){    
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
  }

    chassis.pid_drive_set(4.1, 80, true);
    chassis.pid_wait();

    mid_descore.set(1);

    pros::delay(400);

    chassis.pid_drive_set(-10, 50, false);
}

// finished & checked
void left_7_mid(){
  pros::Task color_sor(color_sort_top_auton);

  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(RIGHT_SWING, -12, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-138, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.7, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(-180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_drive_set(11.8, 70, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(660);
  chassis.pid_drive_set(-13, 40, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-130, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 175, 90, 20, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-17, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-180, 60, true);
  chassis.pid_wait();

  pros::delay(1000);

  discore_mech.set(1);

  chassis.pid_drive_set(19, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-135, 60, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-35, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(2, 1, true);

  intake_bottom.move(-80);
  intake_top.move(-80);
  intake_top_score.move(-80);

  pros::delay(250);

  intake_bottom.move(127);
  intake_top.move(50);
  intake_top_score.move(-30);

  int waittime = pros::millis();

  while (pros::millis() - waittime  <= 2800) { 
    if(color == "R"){
      if (color_sort.get_hue() > 0 && color_sort.get_hue() < 10){
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
    if(color == "B"){  
      if (color_sort.get_hue() > 210 && color_sort.get_hue() < 240){    
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
  }

    mid_descore.set(1);

    chassis.pid_drive_set(4.3, 80, true);
    chassis.pid_wait_quick();


}

// finished & checked
void left_elims_7ball(){
  pros::Task color_sor(color_sort_top_auton);
  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(RIGHT_SWING, -12, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-138, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24.5, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.7, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(-180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_drive_set(11.8, 70, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(660);

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-27, 80, true);
  pros::delay(450);
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  trapdoor.set(0);

  int waittime = pros::millis();
  
  while (pros::millis() - waittime  <= 2000) { 
    if(color == "R"){
      if (color_sort.get_hue() > 0 && color_sort.get_hue() < 10){
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
    if(color == "B"){  
      if (color_sort.get_hue() > 210 && color_sort.get_hue() < 240){    
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
  }
  trapdoor.set(1);
}

// unfinished
void left_elims_quick_ml(){

  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  pros::Task color_sor(color_sort_top_auton);
  trapdoor.set(1);
  chassis.pid_drive_set(22.6, 90, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(12.4, 70, true);
    Little_Mech_Mac.set(1);
    bottom_intake(127);
    intake_top.move(127);
    intake_top_score.move(127);
  chassis.pid_wait();

  pros::delay(200);

  chassis.pid_drive_set(-27.3, 75, true);
  chassis.pid_wait();
  trapdoor.set(0);

  pros::delay(1200);
  trapdoor.set(1);

  chassis.pid_drive_set(4, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-130, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 180, 90, 20, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-20, 80, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-145, 60, true);
  chassis.pid_wait();


  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
}

// finished & checked
void left_elims_quick(){
  pros::Task color_sor(color_sort_top_auton);

  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0
  chassis.pid_swing_exit_condition_set(0_ms, 0.1_deg, 250_ms, 7_deg, 500_ms, 200_ms);

  chassis.pid_swing_set(RIGHT_SWING, -12, 127, 0, false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);

  chassis.pid_drive_set(23.67, 127, false); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(59, 127, true);
  pros::delay(200);  
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 50_ms, 50_ms);
  chassis.pid_drive_constants_set(25, 0, 150); // 22 0 150

  chassis.pid_drive_set(-12.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);  
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150

  chassis.pid_swing_constants_set(12, 0.0, 90);
  chassis.pid_swing_set(RIGHT_SWING, 180, -127, 0, true);
  pros::delay(500);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-2, 127, true);
  pros::delay(320);

  chassis.pid_drive_set(4, 80, true);
  chassis.pid_wait_quick_chain();

  discore_mech.set(1);

  chassis.pid_turn_set(-131, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_exit_condition_set(0_ms, 4_deg, 250_ms, 7_deg, 500_ms, 100_ms);
  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_swing_set(LEFT_SWING, 180, 127, 22, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-20, 80, true);


  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
/*
working but a little slow
  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(RIGHT_SWING, -12, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(59, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 50_ms, 50_ms);

  chassis.pid_drive_set(-10.5, 127, true);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis.pid_swing_set(RIGHT_SWING, 180, -127, 0, true);
  pros::delay(800);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-2, 127, true);
  pros::delay(190);

  chassis.pid_drive_set(4, 80, true);
  chassis.pid_wait_quick_chain();

  discore_mech.set(1);

  chassis.pid_turn_set(-130, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, 180, 90, 20, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-20, 90, true);


  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
  */
}

// unfinished
void right_4_3_push(){
  pros::Task color_sor(color_sort_top_auton);

discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(LEFT_SWING, 13, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(138, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.7, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(-180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_drive_set(11.8, 70, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(680);

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-25, 80, true);
  pros::delay(700);
  trapdoor.set(0);
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  Little_Mech_Mac.set(0);
  pros::delay(250);
  trapdoor.set(1);

  chassis.pid_drive_set(4, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-52, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(30, 90, true);
  chassis.pid_wait_quick();

  intake_piston.set(1);

  chassis.pid_drive_set(10, 60, true);
  pros::delay(400);
  intake_top.move(-60);
  intake_top_score.move(-127);
  intake_bottom.move(-60);
  chassis.pid_wait_quick_chain();

  pros::delay(2000);

    ///////////////////////////////

  chassis.pid_drive_set(-30, 127, true);
  chassis.pid_wait();

  intake_piston.set(0);

  chassis.pid_turn_set(0, 80, true);
  discore_mech.set(1);
  chassis.pid_wait();

  pros::delay(100);
  
  chassis.pid_drive_set(19, 127, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(-45, 80, true);
  chassis.pid_wait();
}

// finished & checked
void right_elims_7ball(){
  pros::Task color_sor(color_sort_top_auton);

  discore_mech.set(0);
  trapdoor.set(1);

  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_constants_set(9, 0.0, 90.5);
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_swing_set(LEFT_SWING, 13, 127, 0, false);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(21.67, 127, true); //24.8 with little bill activation
  pros::delay(500);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(138, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24.5, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.7, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(-180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_drive_set(11.8, 70, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(660);

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-27, 80, true);
  pros::delay(450);
  Little_Mech_Mac.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  trapdoor.set(0);

  int waittime = pros::millis();
  
  while (pros::millis() - waittime  <= 2000) { 
    if(color == "R"){
      if (color_sort.get_hue() > 0 && color_sort.get_hue() < 10){
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
    if(color == "B"){  
      if (color_sort.get_hue() > 210 && color_sort.get_hue() < 240){    
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
        pros::delay(100);
        break;
      }
    }
  }
  trapdoor.set(1);
}

// finished & tested
void push_solo(){
  pros::Task color_sor(color_sort_top_auton);

  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);
  discore_mech.set(0);

  top_intake(127);
  top_intake_score(127);
  bottom_intake(127);

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);
  chassis.pid_drive_set(3, 127, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  
  //chassis_drive_wall(460, 100, false, true);
  
  chassis.pid_drive_set(-37.5, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 90, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(9, 100);
  trapdoor.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 65, true);
  pros::delay(400);

  chassis.pid_drive_set(-30, 75, true);
    pros::delay(880);
    trapdoor.set(0);
  chassis.pid_wait();
  Little_Mech_Mac.set(0);

  pros::delay(900);

  chassis.pid_turn_set(-88, 100, true);
  chassis.pid_wait_quick_chain();
  trapdoor.set(1);
  intake_top.move(127);
  intake_top_score.move(-80);

  chassis.pid_drive_set(56, 100, true);
  pros::delay(400);
  intake_top_score.move(127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-135, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(16.5, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-10, 100, true);
  pros::delay(500);
  trapdoor.set(0);
  chassis.pid_wait();
  trapdoor.set(0);
  pros::delay(1000);
  trapdoor.set(1);

  //Intaking the 2rd machloader

  chassis.pid_drive_set(20, 100, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 50);
  trapdoor.set(1);
  pros::delay(500); 

  chassis.pid_drive_set(-3, 100, true);
  chassis.pid_wait_quick_chain();
  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-135, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-50, 100, true);
  top_intake(-50);
  top_intake_score(-50);
  bottom_intake(-50);
  pros::delay(200);
  top_intake(127);
  top_intake_score(127);
  bottom_intake(127);
  chassis.pid_wait_quick_chain();

  top_intake_score(-127);
  top_intake(80);
  bottom_intake(127);
  while (true) { 
    if(color == "R"){
      if (color_sort.get_hue() > 0 && color_sort.get_hue() < 10){
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
      }
    }
    if(color == "B"){  
      if (color_sort.get_hue() > 210 && color_sort.get_hue() < 240){    
        top_intake_score(127);
        top_intake(127);
        bottom_intake(127);
      }
    }
  }


 
}

// finished & tested
void safe_skills(){
  chassis.pid_drive_constants_set(24, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0


  //grab ball 2
  discore_mech.set(0);
  trapdoor.set(1); 

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(1, 90, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-45, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(26, 40.67, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-0.1, 80, true);
  chassis.pid_wait_quick_chain();

  //score 2 into mid
  chassis.pid_turn_set(-135, 80, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16, 60, true);
  chassis.pid_wait_quick_chain();
  intake_top_score.move(-32.67);
  intake_top.move(-65);
  pros::delay(300);
  intake_top.move(60); 

  chassis.pid_drive_set(1, 60, true);

  while (!(color_sort.get_hue() < 250 && color_sort.get_hue() > 160)){
    pros::delay(10);
  }
  pros::delay(300);

  intake_top_score.move(0);
  intake_top.move(0);
  intake_bottom.move(0);
  
  //grab match loader
  chassis.pid_drive_set(40.5, 80, true);
  pros::delay(300);
  intake_bottom.move(80);
  intake_top.move(-80);
  intake_top_score.move(80);
  chassis.pid_wait_quick_chain();

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_turn_set(180, 80, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(1000, 70, true);
  pros::delay(300);
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(850);
  chassis.pid_drive_set(-1, 40, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(1300);

  //rotate to other side

  chassis.pid_drive_set(-8, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-39, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(1, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(59, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  //score match loader
  chassis.pid_turn_set(60, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.2, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_drive_set(-10.2, 100, true);
  pros::delay(500);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();  
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  pros::delay(1700);
  trapdoor.set(1);
  
  // grab second match loader
 
  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(6, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(20, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(RIGHT_SWING, 0, 127, 30, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 40, true);
  pros::delay(1000);
  chassis.pid_drive_set(-1, 40, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(1500);

  //score long goal
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-30, 80, true);
  pros::delay(1500);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  trapdoor.set(0);
  Little_Mech_Mac.set(0);
  pros::delay(1700);
  //line up for clear
  
  discore_mech.set(0);

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_drive_set(7.3, 127, true);
  chassis.pid_wait_quick_chain();
  trapdoor.set(1);
  intake_top_score.move(-127);

  trapdoor.set(1);

  chassis.pid_swing_set(LEFT_SWING, 87, 84, 31, true);
  chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(5, 127, true);
  // chassis.pid_wait_quick_chain();

  //clear the 6 ball from the park zone
  
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 30000000_ms);
  chassis.pid_drive_set(80, 95, true);
  intake_top_score.move(127);
  chassis.pid_wait();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  chassis_drive_wall(1000, 80, false, false);

  //grab ball number 7
  
  chassis.pid_turn_set(0, 80, true);
  chassis.pid_wait_quick_chain();

  chassis_drive_wall(920, 80, false, false);

  //score 8 in mid

  chassis.pid_turn_set(45, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-20.5, 65, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(2, 1, true);

  intake_bottom.move(-80);
  intake_top.move(-80);
  intake_top_score.move(-80);

  pros::delay(200);

  intake_bottom.move(127);
  intake_top.move(50);
  intake_top_score.move(-31);

  pros::delay(2600);

  //grab 3rd match loader

  chassis.pid_drive_set(39, 80, true);
  pros::delay(300);
  intake_bottom.move(80);
  intake_top.move(-80);
  intake_top_score.move(80);
  chassis.pid_wait_quick_chain();

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.pid_turn_set(0, 80, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(1000, 70, true);
  pros::delay(300);
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(900);
  chassis.pid_drive_set(-1, 40, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(1300);

  //rotate to other side

  chassis.pid_drive_set(-8, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(141, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(59, 100, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);

  //score match loader
  chassis.pid_turn_set(-120, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(6.2, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 100_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_drive_set(-10.2, 100, true);
  pros::delay(500);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  pros::delay(1950);
  trapdoor.set(1);
 
  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);


  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

  chassis.odom_xyt_set(0_in, 0_in, 180_deg);

  chassis.pid_drive_set(6, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-165, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(RIGHT_SWING, 180, 127, 30, true);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(1000, 70, true);
  pros::delay(300);
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(700);
  chassis.pid_drive_set(-1, 40, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(1000, 40, true);
  pros::delay(1300);

  //score long goal
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_drive_constants_set(11, 0, 175, 10.0);
  chassis.pid_drive_set(-29, 80, true);
  pros::delay(1500);
  trapdoor.set(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_constants_set(22, 0, 150);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);


  trapdoor.set(0);
  Little_Mech_Mac.set(0);
  pros::delay(1700);
  //line up for clear
  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);

 chassis.pid_drive_set(7.3, 127, true);
  chassis.pid_wait_quick_chain();
  trapdoor.set(1);
  intake_top_score.move(-127);

  trapdoor.set(1);

  chassis.odom_xyt_set(0_in, 0_in, 180_deg);
  discore_mech.set(0);

  chassis.pid_swing_set(LEFT_SWING, -93, 84, 31, true);
  chassis.pid_wait_quick_chain();

  // chassis.pid_drive_set(5, 127, true);
  // chassis.pid_wait_quick_chain();

  //clear the 6 ball from the park zone
  
  intake_top_score.move(127);
  chassis.pid_drive_set(40, 85, true);
  chassis.pid_wait();

  while (distance_front.get_distance() < 1660){
    chassis.pid_drive_set(1000000, 60);
  }
  L1.brake();
  L2.brake();
  L3.brake();
  R1.brake();
  R2.brake();
  R3.brake();

}

//old but worth keeping
void norcal_skills(){
  
  // pros::Task anti_jam_auton1(anti_jam_auton);
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);

  discore_mech.set(0);
  trapdoor.set(1);
  intake_piston.set(0);

  intake_bottom.move(127);
  intake_top.move(127);
  intake_top_score.move(127);
  // Taking the working 6 balls
  chassis.pid_drive_set(2, 20, true);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 1000_ms);

  chassis.pid_drive_set(50, 65, true);
  pros::delay(200);
  Little_Mech_Mac.set(1);
  pros::delay(300);
  Little_Mech_Mac.set(0);
  chassis.pid_wait();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 100_ms);

  chassis.pid_drive_set(-10, 100, true);
  chassis.pid_wait();


  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);

  //line up on the wall

  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait();

  chassis_drive_wall(600, 127, false, false);
  
  chassis.pid_turn_set(-88, 80, true);
  chassis.pid_wait();

  chassis_drive_wall(1450, 127, false, false);

  chassis.pid_turn_set(180, 80, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-26, 80, true);
  chassis.pid_wait();

  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 150_ms, 150_ms);


  chassis.pid_swing_set(RIGHT_SWING, -127, 127, -30, true);
  pros::delay(500);
  top_intake(-80);
  top_intake_score(-70);
  bottom_intake(-80);
  pros::delay(300);
  bottom_intake(60);
  top_intake(60);
  top_intake_score(-40);
  chassis.pid_wait_quick();

  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);

  //7 ball score
  chassis.pid_drive_set(-1, 1, false);

  pros::delay(1200);
  top_intake(-80);
  top_intake_score(-70);
  bottom_intake(-80);
  //grab 7th ball
  bottom_intake(127);
  top_intake(127);
  top_intake_score(0);
  chassis.pid_turn_set(-135, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(7, 80, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-5, 30, true);
  chassis.pid_wait_quick_chain();
  top_intake(70);
  top_intake_score(-30);
  pros::delay(1600);

  //line up for first match loader

  chassis.pid_drive_set(50, 80, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(180, 80, true);
  top_intake(127);
  bottom_intake(127);
  top_intake_score(127);
  Little_Mech_Mac.set(1);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(10, 80, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1000, 60, true);
  pros::delay(1150);

  //cross to other side
  chassis.pid_drive_set(-8, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(-39, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(8, 100, true);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_set(1, 100, true);
  chassis.pid_wait_quick_chain();

  top_intake(0);
  bottom_intake(0);
  top_intake_score(0);

  chassis.pid_drive_set(52, 100, true);
  chassis.pid_wait_quick_chain();

  //score first match loader

  chassis.pid_turn_set(60, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(7.8, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-9.8, 100, true); 
  chassis.pid_wait();

  chassis.pid_drive_set(-1, 10, true);
  // intake_top.move(-100);
  // intake_top_score.move(127);
  // intake_bottom.move(127);
  // pros::delay(200);
  // intake_top_score.move(-60);
  // pros::delay(300);
  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);
  trapdoor.set(0);

  pros::delay(2000);
  chassis.pid_wait();

  //grab second match loader

  Little_Mech_Mac.set(1);

  trapdoor.set(1);

  chassis.pid_drive_set(27.5, 80, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1000, 60, true);
  pros::delay(1300); 

  //score second match loader
  
  chassis.pid_drive_set(-29.5, 70, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-1, 10, true);
  // intake_top.move(-100);
  // intake_top_score.move(127);
  // pros::delay(200);
  // intake_top_score.move(-60);
  // pros::delay(300);
  intake_top.move(127);
  intake_top_score.move(127);
  trapdoor.set(0);

  pros::delay(1500);
  chassis.pid_wait();
  Little_Mech_Mac.set(0);

  // line up for clear
  
  top_intake(127);
  bottom_intake(127);
  top_intake_score(127);
  discore_mech.set(0);

  chassis.pid_turn_set(90, 80, true);
  chassis.pid_wait_quick_chain();
  trapdoor.set(1);

  chassis.pid_drive_set(60, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(35, 60, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(16.5, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-13, 60, true);
  chassis.pid_wait_quick_chain();
  trapdoor.set(0);
  //Scroing on the 
  chassis.pid_drive_set(-1, 10, true);
  pros::delay(700);


  //Intaking the 3rd machloader
  Little_Mech_Mac.set(1);

  
  ///new stuff

  Little_Mech_Mac.set(1);
  chassis.pid_drive_set(28.3, 60, true);
  chassis.pid_wait();
  trapdoor.set(1);
  chassis.pid_drive_set(1000, 60, true);
  pros::delay(1650); 

  chassis.pid_drive_set(-8, 80, true);
  chassis.pid_wait_quick_chain();

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(133, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(7, 80, true);
  chassis.pid_wait_quick_chain(); 

  chassis.pid_turn_set(-180, 80, true);
  chassis.pid_wait_quick_chain();

  top_intake(0);
  bottom_intake(0);
  top_intake_score(0);

  chassis.pid_drive_set(56, 80, true);
  chassis.pid_wait_quick_chain();

  //score first match loader

  chassis.pid_turn_set(-120, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(8, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-9, 100, true); 
  chassis.pid_wait();

  chassis.pid_drive_set(1, 60, true);
  // intake_top.move(-100);
  // intake_top_score.move(127);
  // intake_bottom.move(127);
  // pros::delay(200);
  // intake_top_score.move(-60);
  // pros::delay(300);
  intake_top.move(127);
  intake_top_score.move(127);
  intake_bottom.move(127);
  trapdoor.set(0);

  pros::delay(1500);
  chassis.pid_wait();

  //grab second match loader
  Little_Mech_Mac.set(1);

  trapdoor.set(1);

  chassis.pid_drive_set(27.4, 80, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1000, 60, true);
  pros::delay(1650); 

  //score second match loader
  
  chassis.pid_drive_set(-32.5, 70, true);
  chassis.pid_wait();
  top_intake_score(127);
  chassis.pid_drive_set(-1, 60, true);
  // intake_top.move(-100);
  // intake_top_score.move(127);
  // pros::delay(200);
  // intake_top_score.move(-60);
  // pros::delay(300);
  intake_top.move(127);
  intake_top_score.move(127);
  trapdoor.set(0);


  pros::delay(1300);
  chassis.pid_wait();
  Little_Mech_Mac.set(0);
  
  chassis.pid_drive_set(8.3, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(LEFT_SWING, -93, 85, 31, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20, 127, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(23, 127, true);
  chassis.pid_wait();
}

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
  chassis.pid_drive_constants_set(23, 0, 150); // 22 0 150
  chassis.pid_turn_constants_set(3.4, 0, 22, 12.0); // 3.2 0 22 12.0

  chassis.pid_turn_set(90, 100, true);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_turn_set(180, 100, true);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_turn_set(-90, 100, true);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_turn_set(0, 100, true);
  chassis.pid_wait();

}
void wall_tracking_test() {
  drive_wall(450,127);
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
  pros::Task Task (controller_update);
  chassis.drive_imu_scaler_set(1);
  chassis.pid_turn_exit_condition_set(90_ms, 1_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  //pros::delay(5000);
  chassis.pid_turn_set(90_deg, 40);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(180_deg, 40);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_turn_set(270_deg, 40);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, 40);
  chassis.pid_wait();
  pros::delay(1999999999);

  chassis.pid_turn_set(360_deg, 40, ez::raw);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_turn_set(360*2_deg, 40, ez::raw);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_turn_set(360*3_deg, 80, ez::raw);
  chassis.pid_wait();
  pros::delay(1000);
}

void intake_test(){
  pros::Task anti_jam_auton1 (anti_jam_auton);


  bottom_intake(127);
  intake_top.move(127);
  intake_top_score.move(127);
}

void color_sort_test(){

  pros::Task color_sor(color_sort_S);
  pros::Task antij(anti_jam_auton);
  top_intake(120);
  bottom_intake(120);

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
