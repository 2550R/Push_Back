#include "main.h"

float W_kp = 1;
float W_ki = 0;
float W_kd = 0;

void wall_ride(float target_distance_mm, char right_left,float DRIVE_SPEED){
    float error = target_distance_mm - (distance_back_l.get_distance() - distance_front_l.get_distance())/2;
    chassis



}