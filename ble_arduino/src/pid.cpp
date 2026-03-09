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

void changePIDValues(PIDController& pid, float kp, float ki, float kd){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}

void changePIDValues(PIDController& pid, float kp, float ki, float kd, float windup_max){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.windup_max = windup_max;
}

void startPID(PIDController& pid, float setpoint) {
    pid.setpoint = setpoint;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = millis();
    pid.running = true;
}

void startPID(PIDController& pid) {
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
    float derivative = (error - pid.prev_error) / dt;


    float p_term = pid.kp * error;
    float d_term = pid.kd * derivative;
    float i_term = pid.ki * pid.integral;

    float output = p_term + i_term + d_term;

    bool pos_sat = (output >= 40.0f && error > 0.0f);
    bool neg_sat = (output <= -40.0f && error < 0.0f);

    //Solving for windup. Only update the integral term if the output isnt saturated
    if(!(pos_sat || neg_sat)){
        pid.integral += error * dt;

        if(pid.integral >= pid.windup_max){
            pid.integral = pid.windup_max;
        } else if (pid.integral < -pid.windup_max){
            pid.integral = -pid.windup_max;
        }

        // Update i_term with new value
        i_term = pid.ki * pid.integral;
        output = p_term + i_term + d_term;
    }

    pid.prev_error = error;
    pid.prev_time_ms = now;

    // Clamp output to motor range
    //TODO make this full range later, but start slow
    return constrain(output, -40.0f, 40.0f);
}

float readFrontDist() { return (float)cur_dists.front; }
float readSideDist()  { return (float)cur_dists.side; }