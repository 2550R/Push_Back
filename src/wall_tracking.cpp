#include "main.h"

float w_KP = 1;
float w_KI = 0;
float w_KD = 0;


void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance){
    while (1){
        float error ;
        float prev_error;
        float integral;
        float derivitive;

        derivitive = error - prev_error;
        error =  target_distance - (distance_back_l.get_distance() + distance_back_r.get_distance())/2;
        float output = error * w_KP + integral * w_KI + w_KD * derivitive;
    
        L1.move_velocity(DRIVE_SPEED + error);
        L2.move_velocity(DRIVE_SPEED + error);
        L3.move_velocity(DRIVE_SPEED + error);
        R1.move_velocity(DRIVE_SPEED - error);
        R2.move_velocity(DRIVE_SPEED - error);
        R3.move_velocity(DRIVE_SPEED - error);

        prev_error = error;
        integral += error;

    }

}