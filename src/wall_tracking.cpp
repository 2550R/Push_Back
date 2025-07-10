#include "main.h"

float w_KP = 0.05;
float w_KI = 0;
float w_KD = 3;

float wa_kP = .17;
float wa_kI = 0.0;
float wa_kD = .01;

void wall_allignment(float timeout){

    float error ;
    float prev_error;
    float integral;
    float derivitive;

    float currentTime = float(pros::millis());

    while((pros::millis() - currentTime) < timeout){
        error = distance_front_l.get_distance() - distance_back_l.get_distance();
        derivitive = error - prev_error;
        float output = error * wa_kP + wa_kD * derivitive;

        L1.move_velocity(-output);
        L2.move_velocity(-output);
        L3.move_velocity(-output);
        R1.move_velocity(output);
        R2.move_velocity(output);
        R3.move_velocity(output);
    }
}

void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance){

        float error ;
        float prev_error;
        float integral;
        float derivitive;

    while (1){
        error =  (target_distance -(distance_front_l.get_distance() + distance_back_l.get_distance())/2)*.5;
        derivitive = error - prev_error;
        float output = error * w_KP + integral * w_KI + w_KD * derivitive;

        float drivePluserror = DRIVE_SPEED + output;
        float driveMinuserror  = DRIVE_SPEED - output;

        L1.move_velocity(drivePluserror);
        L2.move_velocity(drivePluserror);
        L3.move_velocity(drivePluserror);
        R1.move_velocity(driveMinuserror);
        R2.move_velocity(driveMinuserror);
        R3.move_velocity(driveMinuserror);

        prev_error = error;
        integral += error;

    }
}

void wall_Riding(float target_distance, float DRIVE_SPEED, float drive_distance){
    float error ;
    float prev_error;
    float integral;
    float derivitive;
    float inetial_heading = inertial.get_heading();
    while(1){
        error =  target_distance - distance_front_l.get_distance() * sin(inetial_heading + inertial.get_heading());
        derivitive = error - prev_error ;
        float output = error * w_KP + integral * w_KI + w_KD * derivitive;

        float drivePluserror = DRIVE_SPEED + output;
        float driveMinuserror  = DRIVE_SPEED - output;

        L1.move_velocity(drivePluserror);
        L2.move_velocity(drivePluserror);
        L3.move_velocity(drivePluserror);
        R1.move_velocity(driveMinuserror);
        R2.move_velocity(driveMinuserror);
        R3.move_velocity(driveMinuserror);

        prev_error = error;
        integral += error;
    }

}