/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2017-2021, Purdue University ACM SIGBots.
 * All rights reserved. The original source code was made
 * available under the terms of the Mozilla Public License, v. 2.0. The
 * unmodified source code for this file created by Purdue University
 * can be obtained at https://github.com/purduesigbots/pros.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

#include "api.h"

#include "EZ-Template/api.hpp"

#include "autons.hpp"
#include "subsystems.hpp"
#include "wall_tracking.hpp"
#include "color_sort.hpp"

using namespace okapi::literals;

#ifdef __cplusplus
extern "C" {
#endif
extern std::string color;
void autonomous(void);
//void color_sort_task(void);
void color_sort_S(void);
void color_sort_bottom(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
void odom_scale(void);
void controller_update_main(void);
extern double odom_scaling;
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#endif

#endif  // _PROS_MAIN_H_
