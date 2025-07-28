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
      master.print(0, 0, "%d/%d/%d       ", intake_top.get_current_draw(), intake_bottom.get_current_draw(),distance_front.get_distance());
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

int top_stage_intake = 0;
int bottom_stage_intake = 0;
bool change = false;
void anti_jam_auton(){
  float spin_time = 200;
  int v_threshold_top;
  int velocity_top;
  int current_top;
  int velocity_bottom;
  int current_bottom;
  int current_threshold = 2300;
  bool is_jammed_fwd = false;
  bool is_jammed_bcwd = false;
  while(true){
    if (change){pros::delay(300);}
    velocity_top = intake_top.get_actual_velocity();
    velocity_bottom = intake_bottom.get_actual_velocity();
    current_top = intake_top.get_current_draw();
    current_bottom = intake_bottom.get_current_draw();
    if (top_stage_intake <= 0 || bottom_stage_intake <= 0){ 
      if ((current_top > current_threshold && velocity_top > -10) || (current_bottom > current_threshold && velocity_top > -10) ){
        is_jammed_fwd = true;
      }
    }
    if (top_stage_intake > 0 || bottom_stage_intake > 0){ 
      if ((current_top > current_threshold && velocity_top < 10) || (current_bottom > current_threshold && velocity_bottom < 10) ){
        is_jammed_bcwd = true;
      }
    }

    if (is_jammed_fwd){
      float start_time = pros::millis();
      while (intake_distance.get_distance() > 150 && ((float)pros::millis() - start_time) < spin_time){
        intake_top.move(127);
        intake_bottom.move(127);
      }
      is_jammed_fwd = false;
      intake_top.move(top_stage_intake);
      intake_bottom.move(bottom_stage_intake);
      pros::delay(300);
    }
    if (is_jammed_bcwd){
      float start_time = pros::millis();
      while (intake_distance.get_distance() > 150 && ((float)pros::millis() - start_time) < spin_time){
        intake_top.move(-127);
        intake_bottom.move(-127);
      }
      is_jammed_bcwd = false;
      intake_top.move(top_stage_intake);
      intake_bottom.move(bottom_stage_intake);
      pros::delay(300);
    }
    pros::delay(40);
  }
}

void top_intake(int speed){
  change = true;
  intake_top.move(speed);
  top_stage_intake = speed;
}
void bottom_intake(int speed){
  change = true;
  intake_bottom.move(speed);
  bottom_stage_intake = speed;
}


void default_constants() {
  chassis.pid_drive_constants_set(22, 0, 130);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(4, 0.05, 30, 15.0);
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
}
void pid_tune(){
  pros::Task anti (anti_jam_auton);
  top_intake(127);
  bottom_intake(127);
  pros::delay(6000);
  top_intake(-127);
  bottom_intake(-127);
}

void blue_top_elims() {
  trapdoor.set(1);
  intake_top.move(127);
  chassis.odom_xyt_set(0_in, 0_in, 152.5_deg);
  chassis.pid_drive_constants_set(27, 0, 135);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 100_ms, 3_in, 200_ms, 200_ms);
  chassis.pid_odom_set({{{-7.6_in, 18_in}, rev, 127},
                      {{-9_in, 22_in}, rev, 127},
                      {{-12_in, 28_in}, rev, 127},
                      {{-13.5_in, 43_in}, rev, 127}},
                      true);

  pros::delay(1100);
  right_rush_mech.set(1);
  chassis.pid_wait_quick();

  chassis.pid_drive_constants_set(22, 0, 130);
  pros::delay(200);

  chassis.pid_swing_set(RIGHT_SWING, 100, 60,-5);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, 150, 60, 30);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 100, 60, 20);
  chassis.pid_wait();
  pros::delay(400);
  //Turn that on when we have the pump
  right_rush_mech.set(0);
  pros::delay(200);

  chassis.pid_drive_set(-15, -40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(140, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, 90, 60, 30, true);
  chassis.pid_wait_quick();


  chassis.pid_odom_set({{-14.5_in, 17_in}, rev, 80}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(172, 60, true);
  chassis.pid_wait_quick();
  //wall_alignment_R(1000);

  // blooper
  Little_Mech_Mac.set(1);
  pros::delay(300);

  chassis.pid_drive_set(22, 50, true);
  chassis.pid_wait();
  
  pros::delay(500);

  intake_top.move(127);

  chassis.pid_drive_set(-32, 70, true);
  chassis.pid_wait();

  trapdoor.set(0);
}


void skills() {  
  pros::Task anti (anti_jam_auton);
  pros::Task task1(controller_update);
  pros::Task color_sort_task_running(color_sort_task);

  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);
  color = "x";

  /*
  Setup for first matchload
  */

  chassis.pid_odom_set({{-33_in, 0_in}, fwd, 127}, true);
  chassis.pid_wait();

  /*
  Empty first match loader
  */
  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(10.5, 90, true);
  chassis.pid_wait();

  bottom_intake(127);
  top_intake(127);

  //intake_bottom.move(127);
  //intake_top.move(127);

  pros::delay(1000);

  /*
  Back up from match loader
  */

  Little_Mech_Mac.set(0);
  pros::delay(100);

  chassis.pid_drive_set(-9.8, 90, true);
  chassis.pid_wait_quick();

  //intake_counter_spin();

  chassis.pid_turn_set(-90, 60, true);
  chassis.pid_wait();

  /*
  Relocate to blue side
  */

  chassis.pid_odom_set({{-46_in, 0_in}, fwd, 100}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  /*
  Reset location to 0, 0
  */

  chassis.pid_odom_set({{-47_in, 90_in}, fwd, 110}, true);
  chassis.pid_wait();
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();
  pros::delay(100);

  // Reset odom position with distance sensors
  float position_y = (distance_front.get_distance() - 600) / 25.4;
  float position_x = ((distance_front_l.get_distance() + distance_back_l.get_distance()) / 2 - 160) / 25.4;

  p_x = position_x;
  p_y = position_y;

  chassis.odom_xyt_set(position_x, position_y, 0);
  pros::delay(150);

  /*
  Setup for match loader
  */
  
  chassis.pid_turn_set(90, 60, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{9.5_in, 0_in}, fwd, 110}, true);
  chassis.pid_wait();
  screen = 1;
  
  /*
  Score first match loader
  */

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  top_intake(127);
  bottom_intake(127);
  //intake_top.move(127);
  //intake_bottom.move(127);

  chassis.pid_drive_set(-12, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);

  pros::delay(1450);

  /*
  empty second match loader
  */

  trapdoor.set(1);

  chassis.pid_drive_set(29, 80, true);

  pros::delay(1800);

  /*
  Score second 6 on long goal
  */

  Little_Mech_Mac.set(0);

  pros::delay(100);

  chassis.pid_drive_set(-27, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);

  pros::delay(1500);


  /*
  Set up for first 2 middle balls
  */

  // Activate color sort
  trapdoor.set(1);
  
  chassis.pid_drive_set(17, 110, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, 60, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{25, -9.2}, fwd, 80}, true);
  chassis.pid_wait_quick();

  /*
  Grab first 2 from mid
  */

  bottom_intake(127);
  top_intake(0);
  //intake_bottom.move(127);
  //intake_top.move(0);

  chassis.pid_drive_set(15, 40, true);
  chassis.pid_wait();

  /*
  Grab second 2 from mid
  */

  chassis.pid_odom_set({{64, -2}, fwd, 100}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(25, 30, true);
  chassis.pid_wait();

  pros::delay(200);

  /*
  score on middle
  */

  // chassis.pid_drive_set(-2, 90, true);
  // chassis.pid_wait();

  top_intake(-50);
  bottom_intake(-50);
  //intake_top.move(-50);
  //intake_bottom.move(-50);

  chassis.pid_drive_set(-2, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-137, 60, true);
  chassis.pid_wait();
  
  middle_stage.set(1);
  Little_Mech_Mac.set(1);

  pros::delay(250);

  chassis.pid_drive_set(13.8, 60, true);
  chassis.pid_wait();

  top_intake(100);
  bottom_intake(100);
  //intake_top.move(100);
  //intake_bottom.move(100);

  pros::delay(1500);

  Little_Mech_Mac.set(0);

  /*
  second lineup
  */

  chassis.pid_drive_set(-20, 80);
  chassis.pid_wait();

  middle_stage.set(0);

  pros::delay(400);

  chassis.pid_turn_set(0, 60);
  chassis.pid_wait();

  while (distance_front.get_distance() > 750){
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

  chassis.pid_turn_set(89, 60);
  chassis.pid_wait();

  while (distance_front.get_distance() > 740){
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

  chassis.pid_drive_set(15, 60, true);
  chassis.pid_wait();
  
  top_intake(127);
  bottom_intake(127);

  //intake_top.move(127);
  //intake_bottom.move(127);

  pros::delay(1200);

  Little_Mech_Mac.set(0);

  chassis.pid_drive_set(-13, 60, true);
  chassis.pid_wait();

  //intake_counter_spin();

  chassis.pid_odom_set({{12_in, 0_in}, rev, 100}, true);
  chassis.pid_wait();

  /*
  go to blue side
  */

  chassis.pid_odom_set({{15_in, -85_in}, rev, 110}, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{6.5_in, -85_in}, rev, 110}, true);
  chassis.pid_wait();

  /*
  score third set of match load
  */

  chassis.pid_turn_set(178, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-8.5, 60, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  trapdoor.set(0);

  pros::delay(2000);

  /*
  empty second match loader
  */

  trapdoor.set(1);
  
  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(29, 80, true);

  pros::delay(2000);

  /*
  Score second 6 on long goal
  */

  Little_Mech_Mac.set(0);

  pros::delay(100);

  chassis.pid_drive_set(-27.5, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);

  pros::delay(2000);

  chassis.pid_drive_set(20, 60, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{-60, -100}, rev, 100}, true);
  chassis.pid_wait();




}


void red_top_elims() {
  blue_top_elims();
}

void blue_top_quals() {
  trapdoor.set(1);
  intake_top.move(127);
  chassis.odom_xyt_set(0_in, 0_in, 152.5_deg);
  chassis.pid_drive_constants_set(27, 0, 135);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 100_ms, 3_in, 200_ms, 200_ms);
  chassis.pid_odom_set({{{-7.6_in, 18_in}, rev, 127},
                      {{-10_in, 22_in}, rev, 127},
                      {{-12.5_in, 28_in}, rev, 127},
                      {{-13.5_in, 43_in}, rev, 127}},
                      true);

  pros::delay(1100);
  right_rush_mech.set(1);
  chassis.pid_wait_quick();

  chassis.pid_drive_constants_set(22, 0, 130);
  pros::delay(200);

  chassis.pid_swing_set(RIGHT_SWING, 100, 60,-5);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, 150, 60, 30);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 100, 60, 20);
  chassis.pid_wait();
  pros::delay(400);
  //Turn that on when we have the pump
  right_rush_mech.set(0);
  pros::delay(200);

  chassis.pid_drive_set(-15, -40, true);
  chassis.pid_wait();

  intake_bottom.move(70);
  intake_top.move(0);

  chassis.pid_turn_set(140, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, 90, 60, 30, true);
  chassis.pid_wait_quick();

  intake_bottom.move(0);

  chassis.pid_odom_set({{18.2, 39}, fwd, 60}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(38, 60, true);
  chassis.pid_wait();

  middle_stage.set(1);
  Little_Mech_Mac.set(1);

  pros::delay(1000);
  
  chassis.pid_drive_set(6.5, 60, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  pros::delay(1500);

  chassis.pid_odom_set({{-12.5_in, 13_in}, rev, 80}, true);
  chassis.pid_wait();
  
  middle_stage.set(0);
  Little_Mech_Mac.set(0);

  pros::delay(250);

  chassis.pid_turn_set(172, 60, true);
  chassis.pid_wait();

  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(16, 60, true);
  chassis.pid_wait();

  pros::delay(1500);

  chassis.pid_turn_set(172, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-29, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);
}

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

void red_bottom_elims() {
  blue_bottom_elims();
}

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

void red_bottom_quals() {
  blue_bottom_quals();
}

void solo_winpoint_left() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 400_ms);

  chassis.pid_odom_set({{-33_in, 0_in}, fwd, 127}, true);
  chassis.pid_wait();

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(12, 90, true);
  chassis.pid_wait();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);

  intake_bottom.move(127);
  intake_top.move(127);


  pros::delay(390);

  Little_Mech_Mac.set(0);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-28, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);

  pros::delay(1600);

  intake_top.brake();
  intake_bottom.move(127);

  chassis.pid_swing_set(RIGHT_SWING, 45, 80, -19.5, true);
  chassis.pid_wait();

  chassis.pid_drive_set(30, 45, true);
  chassis.pid_wait();

  trapdoor.set(1);

  intake_top.move(-20);
  intake_bottom.move(-20);

  intake_top.set_brake_mode(MOTOR_BRAKE_HOLD);
  intake_bottom.set_brake_mode(MOTOR_BRAKE_HOLD);

  middle_stage.set(1);
  pros::delay(400);
  
  Little_Mech_Mac.set(1);
  intake_top.move(0);
  intake_bottom.brake();

  chassis.pid_drive_set(9.7, 60, true);  
  chassis.pid_wait();
  
  intake_bottom.move(100);
  intake_top.move(100);

  pros::delay(1200);

  chassis.pid_drive_set(-12, 50, true);
  chassis.pid_wait();

  intake_top.brake();

  middle_stage.set(0);
  Little_Mech_Mac.set(0);

  chassis.pid_odom_set({{26.3_in, 34.9_in}, fwd, 100}, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(RIGHT_SWING, 20, 60, 10, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-75, 60, false);
  chassis.pid_wait_quick();

  intake_top.move(-100);
  intake_bottom.move(-80);

  chassis.pid_swing_set(LEFT_SWING, -45, 70, 45);
  chassis.pid_wait_quick();

  pros::delay(100);

  chassis.pid_drive_set(1.5, 127, false);
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
  chassis.pid_odom_set({{ 0_in, 24_in }, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{ 24_in, 24_in }, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{ 24_in, 0_in }, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({{ 0_in, 0_in }, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
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

/* Wall Tracking TEST */

void wall_tracking_test() {
  wall_tracking_with_alignment(150, 50, 1);
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
