//
// Created by leslier on 3/4/26.
//

#ifndef PID_FUNCTIONS_H
#define PID_FUNCTIONS_H

#include <Arduino.h>
#include "motor_functions.h"

typedef float (*SensorReadFn)();

struct PIDController {
    float kp, ki, kd;
    float setpoint;
    float integral;
    float windup_max;
    float prev_error;
    float prev_meas;
    float prev_deriv_filt;
    float alpha;
    uint32_t prev_time_ms;
    bool running;
    SensorReadFn readSensor; // plug in any sensor
};

void initPID(PIDController& pid, float kp, float ki, float kd, float alpha, SensorReadFn sensor_fn, float windup_max);
void changePIDValues(PIDController& pid, float kp, float ki, float kd);
void changePIDValues(PIDController& pid, float kp, float ki, float kd, float alpha);
void changePIDValues(PIDController& pid, float kp, float ki, float kd, float alpha, float windup_max);
void setSetpoint(PIDController& pid, float value);
void startPID(PIDController& pid, float setpoint);
void startPID(PIDController& pid); // Uses existing setpoint, but resets rest of controller
void stopPID(PIDController& pid);
float updatePID(PIDController& pid);

float readFrontDist();
float readKfDist();
float readSideDist();
float readYaw();

#endif //PID_FUNCTIONS_H