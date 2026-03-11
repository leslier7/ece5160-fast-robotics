#ifndef GLOBALS_H
#define GLOBALS_H

#include "SparkFun_VL53L1X.h"
#include "pid.h"

extern SFEVL53L1X distanceSensorFront;
extern SFEVL53L1X distanceSensorSide;

extern PIDController pid_controller;
extern float pid_percent;
extern float commanded_percent;

#endif