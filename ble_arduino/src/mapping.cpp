#include "mapping.h"
#include <Arduino.h>
#include "data_collection.h"
#include "distance_functions.h"
#include "globals.h"
#include "pid.h"

bool mapping_running = false;
int points_collected = 0;
float start_yaw;
static float accumulated_rotation = 0.0f;
static float prev_collect_yaw = 0.0f;

static const float STEADY_STATE_THRESH_DEG = 5.0f;   // degrees
static const unsigned long STEADY_STATE_DURATION_MS = 200; // ms below threshold
static unsigned long steady_state_start_ms = 0;

MappingStates MappingState;

PIDController mapping_pid;

void startMapping(){
    MappingState = MAPPING_START;
    points_collected = 0;
    mapping_pid.readSensor = readYaw;
    changePIDValues(mapping_pid, 9.0, 0.35, 0.03, 0.5, 90); //7.0 worked for non tapped wheels

    mapping_pid.integral = 0.0f;
    mapping_pid.prev_error = 0.0f;
    mapping_pid.prev_deriv_filt = 0.0f;
    mapping_pid.prev_time_ms = millis();
    mapping_pid.prev_meas = readYaw();
    mapping_pid.running = true;
    updateYaw(&start_yaw, myICM); // Make sure to collect up to date yaw
    accumulated_rotation = 0.0f;
    prev_collect_yaw = start_yaw;
    mapping_running = true;
}

void mappingStateTick(){
    switch(MappingState){
        case MAPPING_START:
            if(mapping_running){
                MappingState = MAPPING_COLLECT;
            } else {
                MappingState = MAPPING_START;
            }
            break;

        case MAPPING_TURN: {
            float pid_percent = updatePID(mapping_pid);
            setBothMotors(pid_percent, -pid_percent); //Spin motors in oposite directions

            float yaw_error = mapping_pid.setpoint - yaw;
            if (yaw_error > 180.0f)  yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;
            if (fabs(yaw_error) < STEADY_STATE_THRESH_DEG) {
                if (steady_state_start_ms == 0) {
                    steady_state_start_ms = millis();
                } else if (millis() - steady_state_start_ms >= STEADY_STATE_DURATION_MS) {
                    steady_state_start_ms = 0;
                    brakeBothMotors(); // Stop both motors and make sure that robot has come to a complete stop
                    delay(50);
                    MappingState = MAPPING_COLLECT;
                }
            } else {
                steady_state_start_ms = 0; // reset if error spikes back up
                MappingState = MAPPING_TURN;
            }
            break;
        }

        case MAPPING_COLLECT: {
            int distance = getSensorDistance(distanceSensorFront);
            if (distance != -1){ // Valid measurement collected
                map_data[points_collected].yaw = yaw; // use global yaw, already fresh from blocking loop
                map_data[points_collected].distance = distance;

                // Accumulate actual rotation with wrap handling
                float delta = yaw - prev_collect_yaw;
                if (delta > 180.0f)  delta -= 360.0f;
                if (delta < -180.0f) delta += 360.0f;
                accumulated_rotation += fabs(delta);
                prev_collect_yaw = yaw;

                setSetpoint(mapping_pid, yaw + 15); // Move in smaller increments to get more points
                startPID(mapping_pid);
                points_collected++;
                if (accumulated_rotation >= 360.0f || points_collected >= MAX_MAP_POINTS) {
                    MappingState = MAPPING_END;
                    return;
                }
                MappingState = MAPPING_TURN;
            } else { // distance was bad
                MappingState = MAPPING_COLLECT;
            }
            break;
        }

        case MAPPING_END:
            setBothMotors(0, 0);
            mapping_running = false;
            MappingState = MAPPING_START;
            break;
    }
}