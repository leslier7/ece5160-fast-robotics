//
// Created by Robbie Leslie on 2/8/26
//

#ifndef COMMANDS_H
#define COMMANDS_H

#include <ArduinoBLE.h>
#include "BLECStringCharacteristic.h"
#include "RobotCommand.h"
#include "EString.h"

// Command variables
enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    DATA_RATE,
    GET_IMU_READINGS,
    START_RECORDING,
    STOP_RECORDING,
    GET_DIST_READINGS,
    GET_ALL_READINGS,
    SET_MOTOR_JOB,
    SET_MOTOR_SEQUENCE,
    SET_PID_VALUES,
    SET_SETPOINT,
    START_PID,
    STOP_PID,
    GET_DRIVE_DATA
};

void handle_command();

#endif