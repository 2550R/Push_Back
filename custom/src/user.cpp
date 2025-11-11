#include "vex.h"
#include "motor-control.h"
#include "../custom/include/autonomous.h"

using namespace vex;

bool anti_jam_running = false;
bool anti_jam_wait = false;
int anti_jam_spin_time = 200; // msec
bool anti_jam_work = true;
void reverse_shuffle_motor(motor device) {
	anti_jam_running = true;

	device.spin(fwd, 12, voltageUnits::volt);
	wait(anti_jam_spin_time, msec);

	device.spin(reverse, 12, voltageUnits::volt);
	wait(anti_jam_spin_time, msec);

	anti_jam_running = false;
}

void anti_jam() {
  while(true && anti_jam_work){
    float currentTime = Brain.timer(msec);

    double top_stage1_torque = top_stage1.torque();
    double top_stage2_torque = top_stage2.torque();
    double low_stage_torque = low_stage.torque();

    double top_stage1_max_torque = 0.4;
		double top_stage2_max_torque = 0.4;
		double low_stage_max_torque = 0.4;

    if (anti_jam_wait) {
      wait(anti_jam_spin_time, msec);
      anti_jam_wait = false;

			continue;
    }

		if (top_stage1_torque > top_stage1_max_torque){
			reverse_shuffle_motor(top_stage1);
    }

    if (top_stage2_torque > top_stage2_max_torque){
			reverse_shuffle_motor(top_stage2);
    }

    if (low_stage_torque > low_stage_max_torque){
			reverse_shuffle_motor(low_stage);
    }

  }
}

void runAutonomous() {
  int auton_selected = 1;
  switch(auton_selected) {
    case 1:
      pid_tune_auton_drive();
      break;
    case 2:
      break;  
    case 3:
      break;
    case 4:
      break; 
    case 5:
      break;
    case 6:
      break;
    case 7:
      break;
    case 8:
      break;
    case 9:
      break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

char color_sorted = 'x';
int Digital_X = 0;
bool was_pressed = true;
bool was_pressed_x = true;
bool was_pressed_a = true;
bool anti_wait = false;
bool control_to_controller = true;
void runDriver() {
  thread anti_jam_task(anti_jam);
  stopChassis(coast);
  heading_correction = false;
  while (true) {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();


    //Mach loader controll
    if (button_b){matchloader.set(false);}
    else{matchloader.set(true);}

    //Anti jam stops if buttons are pressed
    if ((l1 || l2 || r1 || r2) && was_pressed) {
      anti_jam_wait = true;
      was_pressed = false;
    }
    if (!(l1 && l2 && r1 && r2)) {
      was_pressed = true;
    }

    //Intake controll
    if(l2 && !anti_jam_running){
      top_stage1.spin(fwd,12,voltageUnits::volt);
      if (control_to_controller) top_stage2.spin(fwd,12,voltageUnits::volt);
      low_stage.spin(fwd,12,voltageUnits::volt);
    } else if(l1 && !anti_jam_running){
      top_stage1.spin(reverse,12,voltageUnits::volt);
      if (control_to_controller) top_stage2.spin(reverse,12,voltageUnits::volt);
      low_stage.spin(reverse,12,voltageUnits::volt);
    } else if (r2 && !anti_jam_running){
      top_stage1.spin(fwd,12,voltageUnits::volt);
      if (control_to_controller) top_stage2.spin(reverse,12,voltageUnits::volt);
      low_stage.spin(fwd,12,voltageUnits::volt);
    } else if (control_to_controller && !anti_jam_running){
      top_stage1.stop();
      if (control_to_controller) top_stage2.stop();
      low_stage.stop();
    }

    controller_1.Screen.setCursor(1, 1);
    controller_1.Screen.print("%c",color_sorted);
    controller_1.Screen.setCursor(1, 4);
    controller_1.Screen.print("%d", (int)anti_jam_work);

    if (controller_1.ButtonX.pressing() && was_pressed_x) {
      Digital_X ++;
      was_pressed_x = false;
    }
    if (!controller_1.ButtonX.pressing()) {
      was_pressed_x = true;
    }

    if (controller_1.ButtonA.pressing() && was_pressed_a) {
      if (anti_jam_work == false){anti_jam_work = true;}
      else {anti_jam_work = false;}


      was_pressed_a = false;
    }
    if (!controller_1.ButtonA.pressing()) {
      was_pressed_a = true;
    }

    if (Digital_X == 4) Digital_X = 1;
    if (Digital_X == 1) color_sorted = 'B';
    if (Digital_X == 2) color_sorted = 'x';
    if (Digital_X == 3) color_sorted = 'R';

    driveChassis(ch3 * 0.12 + ch1 * 0.12 , ch3 * 0.12 - ch1 * 0.12);

    wait(10, msec); 
  }
}

void runPreAutonomous() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating()) {
    wait(10, msec);
  }

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);
  
  // odom tracking
  resetChassis();
  if(using_horizontal_tracker && using_vertical_tracker) {
    thread odom = thread(trackXYOdomWheel);
  } else if (using_horizontal_tracker) {
    thread odom = thread(trackXOdomWheel);
  } else if (using_vertical_tracker) {
    thread odom = thread(trackYOdomWheel);
  } else {
    thread odom = thread(trackNoOdomWheel);
  }
}
