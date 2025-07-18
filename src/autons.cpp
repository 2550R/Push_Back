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

void blue_qual() { }

void blue_top_elims() {
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

  chassis.pid_drive_set(-20, 60, true);
  chassis.pid_wait();

  chassis.pid_turn_set(150, 60, true);
  chassis.pid_wait_quick();

  chassis.pid_swing_set(RIGHT_SWING, 80, 60, 20, true);
  chassis.pid_wait();

  chassis.pid_odom_set({{-8_in, 15_in}, fwd, 80}, true);
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

  chassis.pid_drive_set(-29, 60, true);
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
