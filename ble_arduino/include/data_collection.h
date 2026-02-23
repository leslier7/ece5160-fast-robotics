//
// Created by Robbie Leslie on 2/8/26
//

#ifndef DATA_COLLECTION_H
#define DATA_COLLECTION_H

#include "imu_functions.h"
#include "distance_functions.h"

#define DATA_ARR_SIZE 2000

struct TimeData {
    unsigned long values[DATA_ARR_SIZE];
    unsigned long index;
};

struct TemperatureData {
    int values[DATA_ARR_SIZE];
    int index;
};

struct IMUData {
    Attitude values[DATA_ARR_SIZE];
    int index;
};

struct DistanceData {
    Distances values[DATA_ARR_SIZE];
    int index;
};

extern TimeData time_data;

extern TemperatureData temp_data;

extern IMUData imu_data;

extern DistanceData dist_data;

extern CompFilter comp_filter;

extern bool recording;

void send_time();

void collect_time(TimeData &time_values);

void collect_temps(TemperatureData &temp_values);

bool updateIMU();

void collect_imu(IMUData &imu_values);

void collect_distance(DistanceData &dist_values);

void collectAllData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_data);

void clearData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_values);

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