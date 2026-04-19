/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at 
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 * 
 * Copyright (c) 2021-2025, EZ Robotics. The original Source Code was
 * made available under the Mozilla Public License, v. 2.0, at
 * https://github.com/EZ-Robotics/EZ-Template.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

void default_constants();
void empty();
void norcal_skills();
void safe_skills();

void left_9ball();
void left_9split();
void right_9ball();
void right_9split();

void left_4_3_push();
void top_middle_bottom();
void left_elims_quick_ml();
void left_elims();
void red_top_elims();
void blue_top_quals();

void left_7_mid();
void push_solo();
void left_elims_7ball();

void intake_test();

void blue_bottom_elims();
void red_bottom_elims();
void blue_bottom_quals();
void red_bottom_quals();

void new_elim_auton();

void solo_left();

void pid_test();

/* Odom TESTING Functions */

void odom_pure_pursuit_wait_until_example();
void odom_boomerang_injected_pure_pursuit_example();
void square_odom_test();
void auton_setup_left();
void auton_setup_right();
void solo_right();
void elims_mid_control();

/* Wall Tracking TEST */

void wall_tracking_test();
void wall_alignment_test();
void pid_tune();

void elims_auto();

/* safe routes */
void left_middle_top();
void right_4_3_push();

/* old */
void right_safe1();
void right_auton_new();

void left_elims_quick();
void right_elims_7ball();

/* Color Sort tests */
void color_sort_test();
void PIDvsODOM();

/* New Skills*/
void full_skills_auton();