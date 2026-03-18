//
// Created by Robbie Leslie on 2/8/26
//

#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

#include "imu_functions.h"
#include "distance_functions.h"
#include "motor_functions.h"

#define DATA_ARR_SIZE 1500

struct TimeData {
    unsigned long values[DATA_ARR_SIZE];
    unsigned long index;
};

struct TemperatureData {
    int values[3]; //I made this smaller since temp doesnt really matter, so save some ram. But removing it outright is a pain
    int index;
};

struct IMUData {
    Attitude values[DATA_ARR_SIZE];
    int index;
};

struct YawData {
    float value[DATA_ARR_SIZE];
    int index;
};

struct DistanceData {
    DistanceLog values[DATA_ARR_SIZE];
    int index;
};

struct MotorData {
    MotorSpeeds values[DATA_ARR_SIZE];
    int index;
};

extern TimeData time_data;

extern TemperatureData temp_data;

extern IMUData imu_data;

extern DistanceData dist_data;

extern CompFilter comp_filter;

extern YawData yaw_data;

extern float yaw;

extern MotorData motor_data;

extern bool recording;

void send_time();

void collect_time(TimeData &time_values);

void collect_temps(TemperatureData &temp_values);

bool updateIMU();

void collect_imu(IMUData &imu_values);

void collect_distance(DistanceData &dist_values);

void collect_motor(MotorData &motor_values);

void collectAllData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_data, MotorData &motor_values);

void collectDriveData(TimeData &time_values, YawData &yaw_values, DistanceData &dist_values, MotorData &motor_values);

void clearData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_values, MotorData &motor_values);

void clearDriveData(TimeData &time_values, YawData &yaw_values, DistanceData &dist_values, MotorData &motor_values);

// Generic version
template<typename T, typename V>
void collectData(T &data, const V &value) {
    data.values[data.index] = value;
    data.index = (data.index + 1) % DATA_ARR_SIZE;
}

template<typename T>
void clearData(T &data) {
    data = {};
}

#endif