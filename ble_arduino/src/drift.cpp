#include "drift.h"
#include "pid.h"
#include "data_collection.h"

bool drift_running = false;

DriftStates DriftState;

PIDController imu_pid;

float prev_yaw_error = 0.0f;

void startDrift(){
    DriftState = START;
    drift_running = true;
    imu_pid.readSensor = readYaw;
    imu_pid.integral = 0.0f;
    imu_pid.prev_error = 0.0f;
    imu_pid.prev_deriv_filt = 0.0f;
    imu_pid.prev_time_ms = millis();
    imu_pid.prev_meas = readYaw();
    //setSetpoint(imu_pid, yaw); // Set the ideal yaw to the current yaw
    //TODO tune this
}

int drive_time = 1700;
int break_time = 150;
int turn_angle = 135;
int angle_zone = 10;

void driftStateTick(){
    switch(DriftState){
        case START:
            if(drift_running){
                //queueMotorJob(100, 100, 850);
                queueMotorJob(100, 100, drive_time);
                //startMotorJob(100, 100, 850);
                startMotorQueue();
                DriftState = TOWARD_WALL;
            } else {
                DriftState = START;
            }
            break;
        case TOWARD_WALL:
            //this assumes that serviceMotorQueue is called in main
            if(isMotorQueueBusy()) {
                DriftState = TOWARD_WALL;
            } else if (isMotorQueueIdle()) {
                if(yaw < 180){
                    setSetpoint(imu_pid, yaw+turn_angle);
                } else if (yaw > 180) {
                    setSetpoint(imu_pid, yaw-turn_angle);
                } else {
                    setSetpoint(imu_pid,0);
                }
                startPID(imu_pid);

                prev_yaw_error = imu_pid.setpoint - yaw;
                if (prev_yaw_error > 180.0f) prev_yaw_error -= 360.0f;
                if (prev_yaw_error < -180.0f) prev_yaw_error += 360.0f;
                DriftState = DRIFT;
            }
            break;
        case DRIFT: {

            float pid_percent = updatePID(imu_pid);
            setBothMotors(pid_percent, -pid_percent); //Spin motors in oposite directions

            // Within +- 10 degrees of setpoint
            float yaw_error = imu_pid.setpoint - yaw;
            if (yaw_error > 180.0f) yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;

            bool crossed_target =
                (prev_yaw_error > 0.0f && yaw_error < 0.0f) ||
                (prev_yaw_error < 0.0f && yaw_error > 0.0f);

            if (fabs(yaw_error) <= (float)angle_zone || crossed_target) {
                stopPID(imu_pid);
                stopBothMotors();
                queueMotorJob(-pid_percent, pid_percent, break_time);   // brief brake to shed angular momentum
                queueMotorJob(100, 100, drive_time);
                startMotorQueue();
                DriftState = RETURN;
            } else {
                DriftState = DRIFT;
            }
            prev_yaw_error = yaw_error;
            break;
        }
        case RETURN:
            // Wait for queue to empty
            if(isMotorQueueBusy()){
                DriftState = RETURN;
            } else if (isMotorQueueIdle()){
                DriftState = END;
            }
            break;
        case END:
            drift_running = false;
            DriftState = START;
            break;
    }

}