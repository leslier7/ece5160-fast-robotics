#include "mapping.h"
#include <Arduino.h>
#include "data_collection.h"
#include "distance_functions.h"
#include "globals.h"
#include "pid.h"

bool mapping_running = false;
int points_collected = 0;
float start_yaw;

MappingStates MappingState;

PIDController mapping_pid;

void startMapping(){
    MappingState = START;
    points_collected = 0;
    mapping_pid.readSensor = readYaw;
    mapping_pid.integral = 0.0f;
    mapping_pid.prev_error = 0.0f;
    mapping_pid.prev_deriv_filt = 0.0f;
    mapping_pid.prev_time_ms = millis();
    mapping_pid.prev_meas = readYaw();
    updateYaw(&start_yaw, myICM); // Make sure to collect up to date yaw
    mapping_running = true;
}

void mappingStateTick(){
    switch(MappingState){
        case START:
            if(mapping_running){
                MappingState = COLLECT;
            } else {
                MappingState = START;
            }
            break;

        case TURN:


            break;

        case COLLECT:

            int distance = getSensorDistance(distanceSensorFront);    
            if (distance != -1){ // Valid measurement collected
                updateYaw(&map_data[points_collected].yaw, myICM); // Make sure to collect up to date yaw
                map_data[points_collected].distance = distance;
                setSetpoint(pid_controller, map_data[points_collected].yaw + 25);
                points_collected++;
                MappingState = TURN;
            } else { // distance was bad
                MappingState = COLLECT;
            }
            break;

        case END:
            mapping_running = false;
            MappingState = START;
            break;
    }
}