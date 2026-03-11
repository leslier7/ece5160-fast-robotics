#include "data_collection.h"
#include "ble_config.h"
#include "imu_functions.h"
#include "distance_functions.h"
#include "motor_functions.h"
#include "debug.h"
#include "globals.h"

// unsigned long time_values[DATA_ARR_SIZE];
// unsigned long time_index = 0;

// int temp_values[DATA_ARR_SIZE];
// int temp_index = 0;

TimeData time_data = { {}, 0};

TemperatureData temp_data = { {}, 0};

IMUData imu_data = { {}, 0};

DistanceData dist_data = { {}, 0};

MotorData motor_data = {{}, 0};

LowPass lp_theta = {0, 0, 0.4};
LowPass lp_phi = {0, 0, 0.4};

Attitude gyro_attitude = {0, 0, 0};
Attitude accel_attitude = {0, 0, 0};

CompFilter comp_filter = {{0,0,0}, 0.1, 0.03f}; //dt is dynamically updated later

Distances dists = {-1, -1};

unsigned long last_time = 0;

bool recording = true; // record at startup

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

void collect_dist(DistanceData &dist_data){
    int front_to_log;

    if (cur_dists.front_updated) {
        if (cur_dists.front_status == 0) {
            front_to_log = cur_dists.front;
        } else if (cur_dists.front_status == 4 && cur_dists.front == 0) {
            // out of measurable range in short mode
            front_to_log = 1200;
        } else {
            // current update exists but is not usable
            // fall back to prediction if available, otherwise previous
            if (pred_dists.front_updated) {
                front_to_log = pred_dists.front;
            } else {
                front_to_log = prev_dists.front;
            }
        }
    } else if (pred_dists.front_updated) {
        front_to_log = pred_dists.front;
    } else {
        // hold the previous logged value rather than injecting -1 into the distance trace
        front_to_log = dist_data.values[(dist_data.index + DATA_ARR_SIZE - 1) % DATA_ARR_SIZE].front;
    }

    dist_data.values[dist_data.index].front = front_to_log;

    // if(cur_dists.front_updated){
    //     if (cur_dists.front_status == 4 && cur_dists.front == 0){ //Target is out of range
    //         dist_data.values[dist_data.index].front = 1200;
    //     } else {
    //         dist_data.values[dist_data.index].front = cur_dists.front;
    //     }
    // } else if(pred_dists.front_updated) {
    //     dist_data.values[dist_data.index].front = pred_dists.front;
    // } else {
    //     dist_data.values[dist_data.index].front = -1; //no data updated at this time
    // }

    if(cur_dists.side_updated){
        dist_data.values[dist_data.index].side = cur_dists.side;
    } else if(pred_dists.side_updated) {
        dist_data.values[dist_data.index].side = pred_dists.side;
    } else {
        dist_data.values[dist_data.index].side = -1;
    }

    //dist_data.values[dist_data.index] = cur_dists;
    dist_data.index = (dist_data.index + 1) % DATA_ARR_SIZE;
}

void collect_motor(MotorData &motor_values){
    motor_values.values[motor_values.index] = getCurSpeeds();
    motor_values.index = (motor_values.index + 1) % DATA_ARR_SIZE;
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

void collect_imu(IMUData &imu_values){
    imu_values.values[imu_values.index] = comp_filter.comp_attitude;
    imu_values.index = (imu_values.index + 1) % DATA_ARR_SIZE;
}

void collectIMUTempData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values){
    collect_time(time_values);
    collect_temps(temp_values);
    collect_imu(imu_values);
}

void collectAllData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_values, MotorData &motor_values){
    collect_time(time_values);
    collect_temps(temp_values);
    collect_imu(imu_values);
    collect_dist(dist_values);
    collect_motor(motor_values);
}

void clearData(TimeData &time_values, TemperatureData &temp_values, IMUData &imu_values, DistanceData &dist_values, MotorData &motor_values){
    clearData(time_values);
    clearData(temp_values);
    clearData(imu_values);
    clearData(dist_values);
    clearData(motor_values);
}