// Lab 2 code
// By Robbie Leslie 2026
// Based on ICM 20948 Arduino Library Demo

#include <Arduino.h>
#include <math.h>
#include "ICM_20948.h"
#include "imu_functions.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Create an ICM_20948_I2C object

void setup() {
    SERIAL_PORT.begin(115200);
    while (!SERIAL_PORT) {
    };

    // Set up I2C
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    bool initialized = false;
    while (!initialized) {
        myICM.begin(WIRE_PORT, AD0_VAL);

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());

        if (myICM.status != ICM_20948_Stat_Ok) {
            SERIAL_PORT.println("Trying again...");
            delay(500);
        }
        else {
            initialized = true;
        }
    }
}

void loop() {
    if (myICM.dataReady())
    {
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
        //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        //printSerialPlot(&myICM);
        printAccelPitchRoll(&myICM);
        delay(30);
    }
    else
    {
        SERIAL_PORT.println("Waiting for data");
        delay(500);
    }
}
