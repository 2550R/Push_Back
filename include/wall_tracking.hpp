/*
 * Copyright (c) 2025, Grizzly Robotics Team 2550R. This Source Code
 * Form is subject to the terms of the Mozilla Public License, v. 2.0
 * and made available under the terms of the License at 
 * https://github.com/2550R/Push_Back.
 * A copy of the MPL can be obtained at http://mozilla.org/MPL/2.0/.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

void wall_alignment(float timeout);
void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_riding(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_tracking_with_alignment(float target_distance, float DRIVE_SPEED, float drive_distance);
