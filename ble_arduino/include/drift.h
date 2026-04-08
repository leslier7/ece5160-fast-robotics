#ifndef DRIFT_H
#define DRIFT_H

#include "pid.h"

extern bool drift_running;

extern int drive_time;
extern int break_time;
extern int turn_angle;
extern int angle_zone;

extern PIDController imu_pid;

enum DriftStates{
    START,
    TOWARD_WALL,
    DRIFT,
    RETURN,
    END
};

void startDrift();

void driftStateTick();

#endif