#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::Distance distance_back_l(13);
inline pros::Distance distance_front_l(19);
inline pros::Distance distance_back_r(17);
inline pros::Distance distance_front_r(18);
inline pros::Imu inertial(2);

inline pros::Motor intake_bottom(20);
inline pros::Motor intake_top(11);

// these dt ports have been changed to the numbers on the new bot - 11/6/2025
inline pros::Motor L1(10);
inline pros::Motor L2(-7);
inline pros::Motor L3(-4);

inline pros::Motor R1(-3);
inline pros::Motor R2(6);
inline pros::Motor R3(1);

inline ez::Piston trapdoor('A');
inline ez::Piston middle_stage('C');
inline ez::Piston Little_Mech_Mac('B');


inline ez::Piston right_rush_mech('F');
inline ez::Piston left_rush_mech('F');

inline pros::Controller master(pros::E_CONTROLLER_MASTER); 

