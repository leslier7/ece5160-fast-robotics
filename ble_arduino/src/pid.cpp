#include <Arduino.h>
#include "pid.h"
#include "distance_functions.h"
#include "imu_functions.h"
#include "data_collection.h"

void initPID(PIDController& pid, float kp, float ki, float kd, float alpha, SensorReadFn sensor_fn, float windup_max) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.alpha = alpha;
    pid.windup_max = windup_max;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = 0;
    pid.prev_deriv_filt = 0;
    pid.running = false;
    pid.readSensor = sensor_fn;
    pid.prev_meas = (pid.readSensor != nullptr) ? pid.readSensor() : 0.0f;
}

void changePIDValues(PIDController& pid, float kp, float ki, float kd){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}

void changePIDValues(PIDController& pid, float kp, float ki, float kd, float alpha){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.alpha = alpha;
}

void changePIDValues(PIDController& pid, float kp, float ki, float kd, float alpha, float windup_max){
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.alpha = alpha;
    pid.windup_max = windup_max;
}

void startPID(PIDController& pid, float setpoint) {
    pid.setpoint = setpoint;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = millis();
    pid.prev_meas = (pid.readSensor != nullptr) ? pid.readSensor() : 0.0f;
    pid.prev_deriv_filt = 0;
    pid.running = true;
}

void startPID(PIDController& pid) {
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_time_ms = millis();
    pid.prev_meas = (pid.readSensor != nullptr) ? pid.readSensor() : 0.0f;
    pid.prev_deriv_filt = 0;
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
    if (measured == -1){ //TODO this might not work for all sensor
        return 0.0f;
    }
    float error = measured - pid.setpoint;

    

    #ifdef TOF
    float derivative = pid.prev_deriv_filt;

    // Only update D on a fresh front measurement
    if (cur_dists.front_updated && cur_dists.front_dt > 0) {
        float real_measured = (float)cur_dists.front;
        float dt_real = cur_dists.front_dt / 1000.0f;

        float derivative_raw = -(real_measured - pid.prev_meas) / dt_real;
        derivative = pid.alpha * pid.prev_deriv_filt + (1.0f - pid.alpha) * derivative_raw;

        pid.prev_meas = real_measured;
        pid.prev_deriv_filt = derivative;
    }
    #elif defined(IMU)
    float derivative_raw = -(measured - pid.prev_meas) / dt;
    float derivative = pid.alpha * pid.prev_deriv_filt + (1.0f - pid.alpha) * derivative_raw;

    pid.prev_meas = measured;
    pid.prev_deriv_filt = derivative;
    #endif

    // float derivative_raw = -(measured - pid.prev_meas) / dt; // Calculated this way to eliminate derivative kick

    // float derivative = pid.alpha * pid.prev_deriv_filt + (1 - pid.alpha) * derivative_raw;

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
    pid.prev_meas = measured;
    pid.prev_deriv_filt = derivative;


    // Clamp output to motor range
    //TODO make this full range later, but start slow
    return constrain(output, -60.0f, 60.0f);
}

// Return the current if it has been updated this cycle, otherwise return the previous
float readFrontDist() { 
    if(cur_dists.front_updated){
        return (float)cur_dists.front;
    } else if (pred_dists.front_updated){
        return (float)pred_dists.front;
    }
    return (float)prev_dists.front; 
}

float readSideDist()  { 
    if(cur_dists.side_updated){
        return (float)cur_dists.side;
    } else if (pred_dists.side_updated){
        return (float)pred_dists.side;
    }
    return (float)prev_dists.side; 
}

float readYaw(){
    return yaw;
}