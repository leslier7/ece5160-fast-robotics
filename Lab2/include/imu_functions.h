//
// Created by leslier on 1/29/26.
//

#ifndef LAB2_IMU_FUNCTIONS_H
#define LAB2_IMU_FUNCTIONS_H

#include <Arduino.h>
#include <ICM_20948.h>

void printPaddedInt16b(int16_t val);

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor);
#else
void printScaledAGMT(ICM_20948_I2C *sensor);
#endif

void printSerialPlot(ICM_20948_I2C *sensor);

void printAccelPitchRoll(ICM_20948_I2C *sensor);

#endif //LAB2_IMU_FUNCTIONS_H