#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

void wall_allignment(float timeout);
void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_Riding(float target_distance, float DRIVE_SPEED, float drive_distance);
void wall_tracking_with_allingment(float target_distance, float DRIVE_SPEED, float drive_distance);
