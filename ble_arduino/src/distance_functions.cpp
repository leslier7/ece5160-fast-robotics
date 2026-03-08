#include "SparkFun_VL53L1X.h"
#include <Arduino.h>
#include "distance_functions.h"
#include "debug.h"
#include <Wire.h>

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
        if(alternate) DEBUG_PRINTLN("Alternate Sensor failed to begin. Please check wiring. Trying again");
        else DEBUG_PRINTLN("Main Sensor failed to begin. Please check wiring. Trying again");
        while (tries < SETUP_TRIES){
            if (sensor.begin() == 0) break;
            tries++;
        }

        if (tries >= SETUP_TRIES){
            DEBUG_PRINTLN("Hit max number of tries, exiting");
            return false;
        }
    } 

    sensor.startRanging(); // Start ranging

    return true;
}

bool setupBothSensors(SFEVL53L1X &front, SFEVL53L1X &side){

    //Disable front sensor
    pinMode(A0, OUTPUT); //Used to control xshut on the front ToF

    digitalWrite(A0, LOW);  // Xshut low to set I2C address
    delay(10);

    if(side.begin() != 0){
        DEBUG_PRINTLN("Side sensor failed to start");
        return false;
    }
    side.setI2CAddress(0x60); // 7-bit 0x30 expressed in the 8-bit form
    delay(10);
    

    digitalWrite(A0, HIGH);  // restart front sensor
    delay(20);
    
    uint32_t t0 = millis();
    while (!front.checkBootState()) {
        if (millis() - t0 > 100) {
            DEBUG_PRINTLN("Front sensor did not boot");
            return false;
        }
    }

    if(front.begin() != 0){
        DEBUG_PRINTLN("Front sensor failed to start");
        return false;
    }

    side.startRanging();
    front.startRanging();

    return true;
}