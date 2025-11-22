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

inline pros::Motor L1(-11);
inline pros::Motor L2(-15);
inline pros::Motor L3(-16);

inline pros::Motor R1(5);
inline pros::Motor R2(8);
inline pros::Motor R3(21);

inline pros::Imu inertial(1);

inline pros::Distance distance_back_l(13);
inline pros::Distance distance_front_l(19);
inline pros::Distance distance_back_r(17);
inline pros::Distance distance_front_r(18);
inline pros::Distance distance_front(13);

inline pros::Optical color_sort(9);

inline pros::Motor intake_bottom(7);
inline pros::Motor intake_top(-17);
inline pros::Motor intake_top_score(-20);

inline ez::Piston trapdoor('A');
inline ez::Piston middle_stage('C');
inline ez::Piston Little_Mech_Mac('C');
inline ez::Piston color_sort_piston('D');
inline ez::Piston intake_piston('H');

inline ez::Piston right_rush_mech('F');
inline ez::Piston left_rush_mech('B');

inline ez::Piston discore_mech('B');

inline pros::Distance intake_distance(16);

inline pros::Rotation vertical_tracker(12);
