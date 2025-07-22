#include "main.h"

/*
  Odometry and Pure Pursuit ARE NOT MAGIC

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

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

  chassis.odom_look_ahead_set(8_in);
  chassis.odom_boomerang_distance_set(16_in); // Max distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.65); // How aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);
}
void skills() {
  
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 400_ms);

  chassis.pid_odom_set({{-33_in, 0_in}, fwd, 127}, true);
  chassis.pid_wait();
  //drive_wall(460, 30);
  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();
  //wall_tracking_with_alignment_R(407,110,390);

  chassis.pid_drive_set(9.8, 90, true);
  chassis.pid_wait();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);


  //intake_bottom.move(127);

  pros::delay(390);
  chassis.pid_drive_set(-9.8, 90, true);
  chassis.pid_wait_quick();
  //wall_tracking_with_alignment_R(407,-110,500);
  chassis.pid_odom_set({{{-45_in, 10_in}, rev, 127},
                    {{-37_in, 36_in}, rev, 127}},
                    false);
  chassis.pid_wait();

  Little_Mech_Mac.set(0);
  trapdoor.set(0);
  pros::delay(200);
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();
  pros::delay(900);

  wall_tracking_with_alignment(150,50,1000);
  wall_tracking_with_alignment(150,60,500);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  pros::delay(10000);
  
  chassis.pid_turn_set(90, 60, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{8.7_in, 0_in}, fwd, 127});
  chassis.pid_wait();
  Little_Mech_Mac.set(1);
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();
  chassis.pid_drive_set(11, 90, true);
  chassis.pid_wait();

  pros::delay(1600);
  chassis.pid_turn_set(0, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-27, 110, true);
  chassis.pid_wait();
  Little_Mech_Mac.set(0);

 

  








}
void blue_qual() { }

void blue_top_elims() {
  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, 166_deg);
  
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 100_ms, 3_in, 200_ms, 200_ms);

  chassis.pid_odom_set({{{-13_in, 18_in}, rev, 127},
                      {{-10.8_in, 22_in}, rev, 127},
                      {{-11.4_in, 30_in}, rev, 127},
                      {{-8.15_in, 47.2_in}, rev, 127}},
                      false);
  chassis.pid_wait_quick();

  right_rush_mech.set(1);
  pros::delay(200);

  chassis.pid_swing_set(RIGHT_SWING, 90, 60,-5);
  chassis.pid_wait();

  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, 170, 60,3);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 100, 60, 20);
  chassis.pid_wait();

  pros::delay(300);

  right_rush_mech.set(0);

  pros::delay(200);

  chassis.pid_drive_set(-20, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(150, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, 90, 60, 30, true);
  chassis.pid_wait_quick();

  chassis.pid_odom_set({{-5.8_in, 17_in}, fwd, 80}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait_quick();

  // blooper
  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(12, 50, true);
  chassis.pid_wait();

  //pros::delay(500);

  intake_top.move(127);

  chassis.pid_drive_set(-29.5, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);
}

void red_top_elims() {
  blue_top_elims();
}

void blue_top_quals() {
  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, 166_deg);

  chassis.pid_odom_set({{{-14_in, 18_in}, rev, 127},
                      {{-11_in, 22_in}, rev, 127},
                      {{-10.9_in, 30_in}, rev, 110},
                      {{-8.3_in, 46.3_in}, rev, 110}},
                      true);
  chassis.pid_wait();

  right_rush_mech.set(1);
  pros::delay(300);

  chassis.pid_swing_set(RIGHT_SWING, 90, 60);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, 160, 60);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, 100, 60, 20);
  chassis.pid_wait();

  right_rush_mech.set(0);
  pros::delay(350);

  chassis.pid_drive_set(-20, 40, true);
  chassis.pid_wait();

  chassis.pid_turn_set(150, 60, true);
  chassis.pid_wait_quick();

  intake_top.move(127);
  intake_bottom.move(50);

  chassis.pid_swing_set(RIGHT_SWING, 80, 60, 20, true);
  chassis.pid_wait();

  intake_bottom.move(0);

  chassis.pid_odom_set({{16, 41}, fwd, 60}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45, 60, true);
  chassis.pid_wait();

  middle_stage.set(1);

  pros::delay(1000);
  
  chassis.pid_drive_set(8.5, 60, true);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  pros::delay(1500);

  chassis.pid_odom_set({{-17.5_in, 13_in}, rev, 80}, true);
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

void blue_bottom_elims() {
  trapdoor.set(1);
  chassis.odom_xyt_set(0_in, 0_in, -166_deg);

  chassis.pid_odom_set({{{13_in, 18_in}, rev, 127},
                      {{10.8_in, 22_in}, rev, 127},
                      {{11.4_in, 30_in}, rev, 127},
                      {{8.4_in, 46.8_in}, rev, 127}},
                      true);
  chassis.pid_wait();

  right_rush_mech.set(1);
  pros::delay(300);

  chassis.pid_swing_set(RIGHT_SWING, -90, 60,-5);
  chassis.pid_wait();

  intake_top.move(127);
  intake_bottom.move(127);

  chassis.pid_swing_set(LEFT_SWING, -170, 60,3);
  chassis.pid_wait();

  chassis.pid_swing_set(RIGHT_SWING, -100, 60, 20);
  chassis.pid_wait();

  pros::delay(500);

  right_rush_mech.set(0);

  pros::delay(600);

  chassis.pid_drive_set(-20, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-150, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, -90, 60, 30, true);
  chassis.pid_wait();

  chassis.pid_drive_set(7, 60, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{6.2_in, 17_in}, fwd, 80}, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait();

  // blooper
  Little_Mech_Mac.set(1);

  chassis.pid_drive_set(12, 50, true);
  chassis.pid_wait();

  pros::delay(1500);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-29.5, 60, true);
  chassis.pid_wait();

  trapdoor.set(0);
}

void red_bottom_elims(){
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

void red_bottom_quals(){
  blue_bottom_quals();
}



void solo_winpoint_left() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
  trapdoor.set(1);

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 100_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 100_ms, 400_ms);

  //chassis.pid_odom_set({{-33_in, 0_in}, fwd, 127}, true);
  drive_wall(450, 30);
  chassis.odom_xyt_set(-33_in, 0_in, -90_deg);

  Little_Mech_Mac.set(1);

  chassis.pid_turn_set(180, 110, true);
  chassis.pid_wait();

  chassis.pid_drive_set(9.8, 90, true);
  chassis.pid_wait();

  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 200_ms, 3_in, 400_ms, 300_ms);
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms,500_ms);


  intake_bottom.move(127);

  pros::delay(390);

  chassis.pid_turn_set(180, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(-28, 60, true);
  chassis.pid_wait();

  Little_Mech_Mac.set(0);

  trapdoor.set(0);

  intake_top.move(127);

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

  chassis.pid_odom_set({{26.3_in, 35.9_in}, fwd, 100}, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(RIGHT_SWING, 15, 60, 10, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-57, 60, false);
  chassis.pid_wait_quick();

  intake_top.move(-100);
  intake_bottom.move(-60);

  chassis.pid_drive_set(17.5, 60, false);
  chassis.pid_wait_quick();

  pros::delay(100);

  chassis.pid_drive_set(-1.5, 127, false);
  chassis.pid_wait_quick();
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

void auton_setup() {
  chassis.pid_drive_set(6_in, 30);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, 30);
  chassis.pid_wait();

  chassis.pid_drive_set(15_in, 30);
  chassis.pid_wait();

  chassis.pid_turn_set(164_deg, 30);
  chassis.pid_wait();
}

/* Wall Tracking TEST */

void wall_tracking_test() {
  wall_tracking_with_alignment(150, 50, 1);
}
