#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Distance distance_back(20);
inline pros::Distance distance_front(19);

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');