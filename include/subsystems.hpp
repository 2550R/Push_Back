#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::Distance distance_back_l(12);
inline pros::Distance distance_front_l(11);
inline pros::Distance distance_back_r(13);
inline pros::Distance distance_front_r(14);
inline pros::Imu inertial(21);

inline pros::Motor intake_bottom(20);
inline pros::Motor intake_top(12);

// these dt ports have been changed to the numbers on the new bot - 11/6/2025
inline pros::Motor L1(-7);
inline pros::Motor L2(8);
inline pros::Motor L3(-5);

inline pros::Motor R1(9);
inline pros::Motor R2(-4);
inline pros::Motor R3(1);

inline ez::Piston trapdoor('C');
inline ez::Piston middle_stage('D');

inline ez::Piston right_rush_mech('D');
inline ez::Piston left_rush_mech('D');
