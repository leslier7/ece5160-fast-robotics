#include "data_collection.h"
#include "ble_config.h"
#include "imu_functions.h"
#include "debug.h"

// unsigned long time_values[DATA_ARR_SIZE];
// unsigned long time_index = 0;

// int temp_values[DATA_ARR_SIZE];
// int temp_index = 0;

TimeData time_data = { {}, 0};

TemperatureData temp_data = { {}, 0};

IMUData imu_data = { {}, 0};


LowPass lp_theta = {0, 0, 0.4};
LowPass lp_phi = {0, 0, 0.4};

Attitude gyro_attitude = {0, 0, 0};
Attitude accel_attitude = {0, 0, 0};

CompFilter comp_filter = {{0,0,0}, 0.1, 0.03f}; //dt is dynamically updated later

unsigned long last_time = 0;

bool recording = false;

void send_time(){
    char char_arr[MAX_MSG_SIZE];

    snprintf(char_arr, MAX_MSG_SIZE, "T:%lu", millis());

    tx_estring_value.clear();
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void collect_time(TimeData &time_values){
    time_values.values[time_values.index] = millis();
    time_values.index = (time_values.index + 1) % DATA_ARR_SIZE;
}

void collect_temps(TemperatureData &temp_values){
    temp_values.values[temp_values.index] = getTempDegC();
    temp_values.index = (temp_values.index + 1) % DATA_ARR_SIZE;
}


bool updateIMU(){
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0f; // Convert ms to seconds
    last_time = current_time;

    //IMU processing
    if (myICM.dataReady()) {
        comp_filter.dt = dt; // Update dt dynamically

        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        // const float theta = calculatePitch(&myICM);
        // const float phi = calculateRoll(&myICM);
        // updateLowPass(&lp_theta, theta);
        // updateLowPass(&lp_phi, phi);

        updateGyroAttitude(&gyro_attitude, myICM, dt);
        updateAccelAttitude(&accel_attitude, myICM);

        updateCompFilter(&comp_filter, myICM, accel_attitude);
        return true;
    }
    return false;

}

void collectIMUData(IMUData &imu_values){
    imu_values.values[imu_values.index] = comp_filter.comp_attitude;
    imu_values.index = (imu_values.index + 1) % DATA_ARR_SIZE;
}

void collectAllData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values){
    collect_time(time_values);
    collect_temps(temp_values);
    collectIMUData(imu_values);
}

void clearData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values){
    clearData(time_values);
    clearData(temp_values);
    clearData(imu_values);
}