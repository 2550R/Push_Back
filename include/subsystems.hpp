/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

extern Drive chassis;

inline pros::Motor L1(-1);
inline pros::Motor L2(17);
inline pros::Motor L3(-18);

inline pros::Motor R1(-5);
inline pros::Motor R2(4);
inline pros::Motor R3(-3);

inline pros::Imu inertial(8);

inline pros::Distance distance_back_l(99); // removed sensor
inline pros::Distance distance_front_l(11);
inline pros::Distance distance_back(3);
inline pros::Distance distance_match_loader(21);
inline pros::Distance distance_back_r(99); // removed sensor
inline pros::Distance distance_front_r(14); 
inline pros::Distance distance_front(99); // removed sensor

inline pros::Optical color_sort(12);
inline pros::Optical matchloader_color(9);

inline pros::Motor intake_bottom(10);
inline pros::Motor intake_top(-16);
inline pros::Motor intake_top_score( -13);
inline pros::Motor vertical_tracker('Z');

inline ez::Piston trapdoor('B');
inline ez::Piston Little_Mech_Mac('A'); 
inline ez::Piston intake_piston('D');

inline ez::Piston mid_descore('H');

inline ez::Piston discore_mech('F');

inline pros::Distance intake_distance(16);

//inline pros::Rotation vertical_tracker(12);
