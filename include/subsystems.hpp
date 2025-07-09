#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Distance distance_back_l(12);
inline pros::Distance distance_front_l(11);
inline pros::Distance distance_back_r(13);
inline pros::Distance distance_frontR_r(14);

// inline pros::Motor intake(1);
inline pros::Motor L1(1);
inline pros::Motor L2(-2);
inline pros::Motor L3(-3);

inline pros::Motor R1(-4);
inline pros::Motor R2(5);
inline pros::Motor R3(6);


// inline pros::adi::DigitalIn limit_switch('A');