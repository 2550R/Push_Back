#include "main.h"

ez::Drive chassis(
    {-7, 8, -5},
    {9, -4, 1},
    2,
    3.25,
    450
);

ez::tracking_wheel horiz_tracker(6, 2, 0);
ez::tracking_wheel vert_tracker(11, 2, 0);

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
      {"Blue Elims", blue_top_elims},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
			{"Injected Boomerang Example", odom_boomerang_injected_pure_pursuit_example}
  });
  
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "-");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() { }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() { }

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 */
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
      ez::screen_print("Motor Temps L1 "+ util::to_string_with_precision(L1.get_temperature()*9/5 +32), 1);
      ez::screen_print("Motor Temps L2 "+ util::to_string_with_precision(L2.get_temperature()*9/5 +32), 2); 
      ez::screen_print("Motor Temps L3 "+ util::to_string_with_precision(L3.get_temperature()*9/5 +32), 3); 
      ez::screen_print("Motor Temps R1 "+ util::to_string_with_precision(R1.get_temperature()*9/5 +32), 4); 
      ez::screen_print("Motor Temps R2 "+ util::to_string_with_precision(R2.get_temperature()*9/5 +32), 5); 
      ez::screen_print("Motor Temps R3 "+ util::to_string_with_precision(R3.get_temperature()*9/5 +32), 6); 
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
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

    string temps1 = util::to_string_with_precision(intake_bottom.get_temperature()*9/5 +32);
    string temps2 = util::to_string_with_precision(intake_top.get_temperature()*9/5 +32);
    master.print(0,0,"m",util::to_string_with_precision(L1.get_temperature()*9/5 +32)); 

    middle_stage.button_toggle(master.get_digital_new_press(DIGITAL_Y));
    trapdoor.button_toggle(master.get_digital_new_press(DIGITAL_RIGHT));
    Little_Mech_Mac.button_toggle(master.get_digital_new_press(DIGITAL_B));

		pros::delay(ez::util::DELAY_TIME);
  }
}
