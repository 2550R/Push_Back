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
  {-11, -15, -16}, //left
  {5, 8, 21}, //right
  1,
  3.25,
  450
);

//ez::tracking_wheel horiz_tracker(9, 2, 0);
// ez::tracking_wheel vert_tracker(-12, 2, 0);

bool anti_jam_w = false;
int anti_jam_is_working = 0;
int color_count = 0;
void anti_jam(){
  while(true && anti_jam_is_working==1){
    float currentTime = float(pros::millis());
    double current_top = intake_top.get_current_draw();
    double velocity_top = intake_top.get_actual_velocity();
    double current_bottom = intake_bottom.get_current_draw();
    double velocity_bottom = intake_bottom.get_actual_velocity();
    double current_threshold = 2000;
    double spin_time = 200;
    if (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_R1)) {
      pros::delay(400);
      double v_threshold_top = -127;
      double v_threshold_bottom = -127;
      if (((v_threshold_top+110)<velocity_top) && current_top > current_threshold){
        int start_time = pros::millis();
        anti_jam_w = true;
        while (pros::millis() - start_time < spin_time && intake_distance.get_distance() > 150){
          intake_top.move(127);
        }
        
        anti_jam_w = false;
        
      }
      if (((v_threshold_bottom+110)<velocity_bottom) && current_bottom > current_threshold){
        
        int start_time = pros::millis();
        anti_jam_w = true;
        while (pros::millis() - start_time < spin_time && intake_distance.get_distance() > 150){
          intake_bottom.move(127);
        }
        
        anti_jam_w = false;
      }
		} 
    else if (master.get_digital(DIGITAL_L2) || master.get_digital(DIGITAL_R2)) {
      pros::delay(400);
      double v_threshold_top = 127;
      double v_threshold_bottom = 127;
      if (((v_threshold_top-110)>velocity_top) && current_top > current_threshold){
        int start_time = pros::millis();
        anti_jam_w = true;
        while (pros::millis() - start_time < spin_time && intake_distance.get_distance() > 150){
          intake_top.move(-127);
        }
        
        anti_jam_w = false;
      }
      if (((v_threshold_bottom-110)>velocity_bottom) && current_bottom > current_threshold){
        int start_time = pros::millis();
        anti_jam_w = true;
        while (pros::millis() - start_time < spin_time && intake_distance.get_distance() > 150){
          intake_bottom.move(-127);
        }
        
        anti_jam_w = false;
      }
		} 
    pros::delay(ez::util::DELAY_TIME);
  }
}

std::string color = "x"; // against R or B; press UP+X to change; x for disabled
bool control_to_controller = true;

void color_sort_top() {
  color_sort.set_integration_time(3);
  while (true) {
    int hue_lower;
    int hue_higher;
    color_sort.set_led_pwm(100);
    if (color == "B") {
      hue_lower = 210;
      hue_higher = 250;
    } else if (color == "R") {
      hue_lower = 0;
      hue_higher = 10;
    } else {
      continue;
    }

    bool in_proximity = color_sort.get_proximity() > 220;

    if (in_proximity && (hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher)) {
      control_to_controller = false;
      intake_top_score.move(-127);
      intake_top.move(30);
      pros::delay(300);
      control_to_controller = true;

    }
  }
}


void color_sort_bottom() {
  color_sort.set_integration_time(3);
  while (true) {
    int hue_lower;
    int hue_higher;
    color_sort.set_led_pwm(100);
    if (color == "B") {
      hue_lower = 180;
      hue_higher = 260;
    } else if (color == "R") {
      hue_lower = 0;
      hue_higher = 30;
    } else {
      continue;
    }

    bool in_proximity = color_sort.get_proximity() > 50;

    if (in_proximity && ((hue_lower < color_sort.get_hue() && color_sort.get_hue() < hue_higher)||(color == "R" && color_sort.get_hue() > 300)) && color_count < 17) {
      color_sort_piston.set(1);
      pros::delay(200);
      color_sort_piston.set(0);
      color_count += 1;
    }
  }
}

void initialize() {
  discore_mech.set(1);
  ez::ez_template_print();
  color_sort.set_led_pwm(100);
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
	//chassis.odom_tracker_back_set(&horiz_tracker);
  // chassis.odom_tracker_right_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 1);

  default_constants();
  pros::Task task1(anti_jam);

  ez::as::auton_selector.autons_add({
    {"solo_right", skills},
    {"Right Safe", skills_before_changing_the_wall},
    {"Skills", left_safe},
    {"Left Side Solo", left_elims_quick},
    {"Left Elims Quick", left_elims_quick},
    {"Right Elims Quick", right_elims_quick},
    {"Testing PID VS Odom", wall_alignment_test},
    {"Skills", skills},
    {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
    {"Injected Boomerang Example", odom_boomerang_injected_pure_pursuit_example}
  });
  
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "-");
}

void controller_update_main() {
  while (true) {
    master.print(0, 0, "%f       ", (vertical_tracker.get_position()/36000.0)* 2.0/*The diameter of the wheel*/ * 3.14 );
    pros::delay(100);
  }

}

double odom_distace;
double odom_scaling;
double distance_desired = 45;
void odom_reset(){
  pros::Task controller (controller_update_main);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  while (!master.get_digital(DIGITAL_UP)){
    controller_update_main;
  }
  odom_distace = (vertical_tracker.get_position()/36000.0)* 2.0/*The diameter of the wheel*/ * 3.14 ;
  odom_scaling = odom_distace / distance_desired;
  master.print(0,0, "%f%f         ", odom_scaling, odom_distace);
  

}

void disabled() { }

void competition_initialize() {

  discore_mech.set(1);
 }

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  //odom_reset();

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
    if (ez::as::page_blank_is_on(2)) {
      ez::screen_print("test_variable: " + util::to_string_with_precision(color_sort.get_hue()), 1);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

double to_fahrenheit(double celsius) {
	return celsius * 9 / 5 + 32;
}

double avg_motor_temps() {
  double sum = (
    L1.get_temperature() + L2.get_temperature() + L3.get_temperature() + R1.get_temperature()
    + R2.get_temperature() + R3.get_temperature()
  );

  double mean = sum / 6;

  return mean;
} 


int Digital_X;
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
	int count = 0;
  bool intake_auto_reverse_enabled = false;
  // pros::Task anti_jam_T(anti_jam);
  pros::Task color_sort_task_running (color_sort_top);
  color = "x";
  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);
      
    
    if (master.get_digital(DIGITAL_L1)) {
			intake_bottom.move(-127);
			intake_top.move(-127);
      intake_top_score.move(-127);

		} 
    else if (master.get_digital(DIGITAL_L2)) {
			intake_bottom.move(127);
      if  (control_to_controller)(intake_top.move(127));
      if  (control_to_controller)(intake_top_score.move(127));

		} 
    else if (master.get_digital(DIGITAL_R1)) {
			intake_bottom.move(127);
			intake_top.move(127);
      intake_top_score.move(-75);
		} 
    else if (master.get_digital(DIGITAL_R2)) {
			intake_bottom.move(-127);
			intake_top.move(-127);
      intake_top_score.move(-127);
      intake_piston.set(1);
		} 

    else if (control_to_controller){
      intake_bottom.move(0);
		  intake_top.move(0);
      intake_top_score.move(0);
      intake_piston.set(0);
    }
    // } 
    //   else {
		//   	intake_bottom.move(-40);
		// 	  intake_top.move(-60);
    //   }
    // }


    if (master.get_digital(DIGITAL_RIGHT)) {
      trapdoor.set(0);
    }
    else {
      trapdoor.set(1);
    }

    if (master.get_digital(DIGITAL_Y)) {
      middle_stage.set(0);
    }
    else {
      middle_stage.set(1);
    }

    if (master.get_digital(DIGITAL_B)) {
      Little_Mech_Mac.set(1);
    }
    else {
      Little_Mech_Mac.set(0);
    }

    if (master.get_digital(DIGITAL_DOWN)) {
      discore_mech.set(1);
    }
    else {
      discore_mech.set(0);
    }

    if (master.get_digital_new_press(DIGITAL_X)) {
      Digital_X += 1;
      if (Digital_X == 4){Digital_X = 1;}
      if (Digital_X == 1){color = "B";}
      if (Digital_X == 2){color = "x";}
      if (Digital_X == 3){color = "R";}
    }
    if (master.get_digital_new_press(DIGITAL_UP)){
      if (anti_jam_is_working == 1){anti_jam_is_working = 0;}
      else {anti_jam_is_working = 1;}
    }
    
    if (
      master.get_digital_new_press(DIGITAL_LEFT)
      && master.get_digital(DIGITAL_UP)
    ) {
      intake_auto_reverse_enabled = !intake_auto_reverse_enabled;
    }

    // trapdoor.button_toggle(master.get_digital_new_press(DIGITAL_RIGHT));
    // middle_stage.button_toggle(master.get_digital_new_press(DIGITAL_Y));
    // Little_Mech_Mac.button_toggle(master.get_digital_new_press(DIGITAL_B));
    // color_sort_piston.button_toggle(master.get_digital_new_press(DIGITAL_X));
    // left_rush_mech.button_toggle(master.get_digital_new_press(DIGITAL_RIGHT));


    if (count == 80) {
      // only update controller screen every 80 cycles
      count = 0;

      int dt_temps = (int) to_fahrenheit(avg_motor_temps());
      int top_temp = (int) to_fahrenheit(intake_top.get_temperature());
      int bottom_temp = (int) to_fahrenheit(intake_bottom.get_temperature());
      std::string intake_back = "";
      if (intake_auto_reverse_enabled){
        intake_back = "N";
      }

      master.print(0, 0, "%d/%d/%d/%s            ", /*L1.get_temperature(int)color_sort.get_hue()*//*anti_jam_is_working(int)vertical_tracker.get_position()/100 */dt_temps , color_sort.get_proximity()/*top_temp*/, bottom_temp, color, intake_back);
    }

		count++;

    // if (!pros::competition::is_connected() && master.get_digital(DIGITAL_LEFT) && master.get_digital(DIGITAL_DOWN)) {
    //   pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
    //   autonomous();
    //   chassis.drive_brake_set(preference);
    // }

		pros::delay(ez::util::DELAY_TIME);
  }
}
