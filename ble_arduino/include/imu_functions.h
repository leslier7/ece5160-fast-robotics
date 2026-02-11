//
// Created by leslier on 1/29/26.
//

#ifndef LAB2_IMU_FUNCTIONS_H
#define LAB2_IMU_FUNCTIONS_H

#include <Arduino.h>
#include <ICM_20948.h>

typedef struct LowPass {
    float value_n;
    float value_n_1;
    float alpha;
} LowPass;

typedef struct Attitude {
    float pitch;
    float roll;
    float yaw;
} Attitude;

typedef struct CompFilter {
    Attitude comp_attitude;
    float alpha;
    float dt;
} CompFilter;

extern ICM_20948_I2C myICM;

bool initIMU(ICM_20948_I2C &imu);

void printPaddedInt16b(int16_t val);

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor);
#else
void printScaledAGMT(ICM_20948_I2C *sensor);
#endif

void printSerialPlot(ICM_20948_I2C *sensor);

void printAccelPitchRoll(ICM_20948_I2C *sensor);

void printAccelPitchRoll(float theta, float phi);

void printAccelPitchRollLPF(LowPass *lp_theta, LowPass *lp_phi, float theta, float phi);

void updateLowPass(LowPass *filter, float new_value);

void updateGyroAttitude(Attitude *attitude, ICM_20948_I2C &sensor, float dt);

void updateAccelAttitude(Attitude *attitude, ICM_20948_I2C &sensor);

void updateCompFilter(CompFilter *filter, ICM_20948_I2C &sensor, const Attitude& accel_attitude);

void printAttitude(const Attitude& attitude);

void printManyAttitudes(Attitude* attitudes, int length);

inline float calculatePitch(ICM_20948_I2C *sensor) {
    return atan2(sensor->accX(), sensor->accZ()) * 57.295779513f; // pitch (in deg)
}

inline float calculateRoll(ICM_20948_I2C *sensor) {
    return atan2(sensor->accY(), sensor->accZ()) * 57.295779513f; // roll (in deg)
}

#endif //LAB2_IMU_FUNCTIONS_H