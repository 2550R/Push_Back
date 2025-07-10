#include "main.h"

float w_KP = 0.1;
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


void wall_tracking_with_allingment(float target_distance, float DRIVE_SPEED, float drive_distance){

        float error_tracking;
        float error_turn;
        float prev_error_tr;
        float prev_error_tu;
        float integral;
        float derivitive_tracking;
        float derivitive_turn;
        float output_tracking;
        float output_turn;
        float error_tw = 0.1;

    while (1){
        // This is tracking
        error_tracking =  (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
        derivitive_tracking = error_tracking - prev_error_tr;
        output_tracking = error_tracking * w_KP + w_KD * derivitive_tracking;
        
        if (error_tracking == 0){error_tracking = 0.001;} 
        //This is turning 
        error_turn = (distance_front_l.get_distance() - distance_back_l.get_distance());
        derivitive_turn = error_turn - prev_error_tu;
        output_turn = error_turn * wa_kP + wa_kD * derivitive_turn;


        float drivePluserror = DRIVE_SPEED + output_tracking - output_turn;
        float driveMinuserror  = DRIVE_SPEED - output_tracking + output_turn;

        L1.move_velocity(drivePluserror);
        L2.move_velocity(drivePluserror);
        L3.move_velocity(drivePluserror);
        R1.move_velocity(driveMinuserror);
        R2.move_velocity(driveMinuserror);
        R3.move_velocity(driveMinuserror);

        prev_error_tr = error_tracking;
        prev_error_tu = error_turn;

    }
}



void wall_tracking(float target_distance, float DRIVE_SPEED, float drive_distance){

        float error ;
        float prev_error;
        float integral;
        float derivitive;

    while (1){
        error =  (target_distance - (distance_front_l.get_distance() + distance_back_l.get_distance())/2);
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