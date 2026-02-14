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

inline pros::Motor L1(-6);
inline pros::Motor L2(-5);
inline pros::Motor L3(-8);

inline pros::Motor R1(14);
inline pros::Motor R2(19);
inline pros::Motor R3(18);

inline pros::Imu inertial(11);

inline pros::Distance distance_back_l(13); // removed sensor
inline pros::Distance distance_front_l(9);
inline pros::Distance distance_match_loader(21);
inline pros::Distance distance_back_r(99); // removed sensor
inline pros::Distance distance_front_r(99); // removed sensor
inline pros::Distance distance_front(99); // removed sensor

inline pros::Optical color_sort(16);

inline pros::Motor intake_bottom(17);
inline pros::Motor intake_top(-2);
inline pros::Motor intake_top_score(-10);
inline pros::Motor vertical_tracker('Z');

inline ez::Piston trapdoor('B');
inline ez::Piston middle_stage('C'); // removed mech
inline ez::Piston Little_Mech_Mac('H'); 
inline ez::Piston color_sort_piston('D'); // removed mech
inline ez::Piston intake_piston('C');

inline ez::Piston mid_descore('G');

inline ez::Piston right_rush_mech('F'); // removed mech
inline ez::Piston left_rush_mech('B'); // removed mech

inline ez::Piston discore_mech('A');

inline pros::Distance intake_distance(16);

//inline pros::Rotation vertical_tracker(12);
