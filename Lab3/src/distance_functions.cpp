#include "SparkFun_VL53L1X.h"
#include <Arduino.h>
#include "distance_functions.h"

bool setupSensor(SFEVL53L1X &sensor, bool alternate){

    if(alternate){
        pinMode(A0, OUTPUT); //Used to control xshut on the side ToF
        digitalWrite(A0, LOW);  // Xshut low to set I2C address

        sensor.setI2CAddress(0x30);

        int tries = 0;
        while((sensor.getI2CAddress() != 0x30) && (tries < SETUP_TRIES)){
            Serial.println("Error setting I2C address");
            tries++;
        }

        if (tries >= SETUP_TRIES){
            return false;
        }

        digitalWrite(A0, HIGH);
    }

    if (sensor.begin() != 0) { //Begin returns 0 on a good init
        int tries = 0;
        Serial.println("Front Sensor failed to begin. Please check wiring. Freezing...");
        while (tries < SETUP_TRIES){
            tries++;
        }

        if (tries >= SETUP_TRIES){
            return false;
        }
    } 

    sensor.startRanging(); // Start ranging

    return true;
}