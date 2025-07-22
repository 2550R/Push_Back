#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

void wall_alignment(float timeout);
void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_riding(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_tracking_with_alignment(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_tracking_with_alignment_B(float target_distance, float DRIVE_SPEED, float drive_distance);
void drive_wall(float distance, float speed);