#include "SparkFun_VL53L1X.h"
#include <Arduino.h>
#include "distance_functions.h"
#include "debug.h"

Distances cur_dists = {-1, -1};

bool setupSensor(SFEVL53L1X &sensor, bool alternate){

    if(alternate){
        pinMode(A0, OUTPUT); //Used to control xshut on the side ToF
        digitalWrite(A0, LOW);  // Xshut low to set I2C address

        sensor.setI2CAddress(0x30);

        int tries = 0;
        while((sensor.getI2CAddress() != 0x30) && (tries < SETUP_TRIES)){
            DEBUG_PRINTLN("Error setting I2C address");
            tries++;
        }

        if (tries >= SETUP_TRIES){
            return false;
        }

        digitalWrite(A0, HIGH);
    }

    if (sensor.begin() != 0) { //Begin returns 0 on a good init
        int tries = 0;
        if(alternate) DEBUG_PRINTLN("Alternate Sensor failed to begin. Please check wiring. Freezing...");
        else DEBUG_PRINTLN("Main Sensor failed to begin. Please check wiring. Freezing...");
        while (tries < SETUP_TRIES){
            if (sensor.begin() == 0) break;
            tries++;
        }

        if (tries >= SETUP_TRIES){
            return false;
        }
    } 

    sensor.startRanging(); // Start ranging

    return true;
}