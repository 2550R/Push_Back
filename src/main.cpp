/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Richard Wang (1698V)                                      */
/*    Created:      July 9, 2023                                              */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "vex.h"
#include "motor-control.h"
#include "math.h"
#include "../custom/include/autonomous.h"
#include "../custom/include/user.h"

using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
// against R or B; press UP+X to change; x for disabled

int color_count = 0;
void color_sort() {
  color_sorted = 'B';
  bool blue_color_sort = true;
  top_colorsort.integrationTime(30);
  while (true) {
    int hue_lower;
    int hue_higher;
    top_colorsort.setLightPower(100);
    if (color_sorted == 'B') {
      hue_lower = 210;
      hue_higher = 250;
    } else if (color_sorted == 'R') {
      hue_lower = 0;
      hue_higher = 10;
    } else {
      continue;
    }
    bool in_proximity = top_colorsort.isNearObject();;

    if (in_proximity && ((hue_lower < top_colorsort.hue() && top_colorsort.hue() < hue_higher)||(color_sorted == 'R' && top_colorsort.hue() > 300)) && color_count < 17) {
      control_to_controller = false;
      top_stage2.spin(reverse,12,voltageUnits::volt);
      wait(200, msec);
      control_to_controller = true;
    }
  }
}
void pre_auton(void) {
  runPreAutonomous();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*-------------------------------
--------------------------------------------*/

void autonomous(void) {
  runAutonomous();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  thread color_sort_thread(color_sort);
  runDriver();
}

//
// Main will set up the competition functions and callbacks.
//
int main() {


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
