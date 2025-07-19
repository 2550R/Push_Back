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

/* Order of function calls:
 * initialize() -> competition_initialize() -> autonomous() -> disabled()
 * -> opcontrol() -> disabled()
 */

ez::Drive chassis(
  {10, -7, -4},
  {-3, 6, 1},
  2,
  3.25,
  450
);

ez::tracking_wheel horiz_tracker(9, 2, 0);
ez::tracking_wheel vert_tracker(12, 2, 0);

void initialize() {
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

	chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  ez::as::auton_selector.autons_add({
    {"Solo AWP Left", solo_winpoint_left},
    {"Blue Top Elims", blue_top_elims},
    {"Red Top Elims", red_top_elims},
    {"Blue Quals", blue_top_quals},
    {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
    {"Injected Boomerang Example", odom_boomerang_injected_pure_pursuit_example}
  });
  
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "-");
}

void disabled() { }

void competition_initialize() { }

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void screen_print_temp(pros::v5::Motor *motor, std::string name, int line) {
  if (motor != nullptr) {
    double temperature = motor->get_temperature();
    double fahrenheit = temperature * 9 / 5 + 32;
    std::string formatted = util::to_string_with_precision(fahrenheit);
    ez::screen_print(name + " Temperature: " + formatted, line);
  }
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected() && ez::as::page_blank_is_on(0)) {
      ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                       "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                       "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                       1
			);

      screen_print_tracker(chassis.odom_tracker_left, "l", 4);
      screen_print_tracker(chassis.odom_tracker_right, "r", 5);
      screen_print_tracker(chassis.odom_tracker_back, "b", 6);
      screen_print_tracker(chassis.odom_tracker_front, "f", 7);
    }

    if (ez::as::page_blank_is_on(1)) {
      screen_print_temp(&L1, "L1", 1);
      screen_print_temp(&L2, "L2", 2);
      screen_print_temp(&L3, "L3", 3);
      screen_print_temp(&R1, "R1", 4);
      screen_print_temp(&R2, "R2", 5);
      screen_print_temp(&R3, "R3", 6);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

double avg_motor_temps() {
  double sum = (
    L1.get_temperature() + L2.get_temperature() + L3.get_temperature() + R1.get_temperature()
    + R2.get_temperature() + R3.get_temperature()
  );

  double mean = sum / 6;
  double fahrenheit = mean * 9 / 5 + 32;

  return fahrenheit;
}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
	int count = 0;

  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);

		if (master.get_digital(DIGITAL_L1)) {
			intake_bottom.move(-127);
			intake_top.move(-127);
		} else if (master.get_digital(DIGITAL_L2)) {
			intake_bottom.move(127);
			intake_top.move(127);
		} else if (master.get_digital(DIGITAL_R1)) {
			intake_bottom.move(-65);
			intake_top.move(-65);
		} else if (master.get_digital(DIGITAL_R2)) {
			intake_bottom.move(65);
			intake_top.move(65);
		} else {
			intake_bottom.move(0);
			intake_top.move(0);
		}

		if (count == 20) {
			// only update controller screen every 20 cycles
			count = 0;

      double motor_temp = avg_motor_temps();
      master.print(0, 0, "%f", motor_temp);
		}

    middle_stage.button_toggle(master.get_digital_new_press(DIGITAL_Y));
    trapdoor.button_toggle(master.get_digital_new_press(DIGITAL_RIGHT));
    Little_Mech_Mac.button_toggle(master.get_digital_new_press(DIGITAL_B));

		count++;
		pros::delay(ez::util::DELAY_TIME);
  }
}
