#include <Arduino.h>
#include "pid.h"
#include "distance_functions.h"
#include "imu_functions.h"


void initPID(PIDController& pid, float kp, float ki, float kd, SensorReadFn sensor_fn, float windup_max) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.windup_max = windup_max;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = 0;
    pid.running = false;
    pid.readSensor = sensor_fn;
}

void startPID(PIDController& pid, float setpoint) {
    pid.setpoint = setpoint;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = millis();
    pid.running = true;
}

void stopPID(PIDController& pid) {
    pid.running = false;
}

//Sets a new setpoint without restarting the controller
void setSetpoint(PIDController& pid, float value){
    pid.setpoint = value;
}

float updatePID(PIDController& pid){
    if (!pid.running || pid.readSensor == nullptr) return 0.0f;

    uint32_t now = millis();
    float dt = (now - pid.prev_time_ms) / 1000.0f;
    if (dt <= 0.0f) return 0.0f;

    float measured = pid.readSensor();
    float error = measured - pid.setpoint;

    pid.integral += error * dt;

    //Solving for windup
    if(pid.integral >= pid.windup_max){
        pid.integral = pid.windup_max;
    }

    float derivative = (error - pid.prev_error) / dt;

    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    pid.prev_error = error;
    pid.prev_time_ms = now;

    // Clamp output to motor range
    //TODO make this full range later, but start slow
    return constrain(output, -30.0f, 30.0f);
}

float readFrontDist() { return (float)cur_dists.front; }
float readSideDist()  { return (float)cur_dists.side; }