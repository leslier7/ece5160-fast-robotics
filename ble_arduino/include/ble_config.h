//
// Created by Robbie Leslie on 2/8/26
//

#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#include <ArduinoBLE.h>
#include "BLECStringCharacteristic.h"
#include "RobotCommand.h"
#include "EString.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "4843fd62-068c-4a37-af34-e724c6a05681"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables (declarations) ////////////
extern BLEService testService;
extern BLECStringCharacteristic rx_characteristic_string;
extern BLEFloatCharacteristic tx_characteristic_float;
extern BLECStringCharacteristic tx_characteristic_string;
extern RobotCommand robot_cmd;
extern EString tx_estring_value;
extern float tx_float_value;

void bleSetup();

void read_data();

void write_data();

#endif