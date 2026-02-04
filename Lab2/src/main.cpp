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
LowPass lp_theta = {0, 0, 0.4};
LowPass lp_phi = {0, 0, 0.4};

Attitude gyro_attitude = {0, 0, 0};
Attitude accel_attitude = {0, 0, 0};

CompFilter comp_filter = {{0,0,0}, 0.5, 0.03f};

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
        const float theta = calculateTheta(&myICM);
        const float phi = calculatePhi(&myICM);
        updateLowPass(&lp_theta, theta);
        updateLowPass(&lp_phi, phi);

        //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
        //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        //printSerialPlot(&myICM);
        //printAccelPitchRoll(&myICM);
        //printAccelPitchRoll(lp_theta.value_n, lp_phi.value_n);
        //printAccelPitchRollLPF(&lp_theta, &lp_phi, theta, phi);

        // gyro_attitude.pitch = gyro_attitude.pitch + myICM.gyrY() * 0.03f; // Integrate gyro Y to get pitch (deg)
        // gyro_attitude.roll = gyro_attitude.roll + myICM.gyrX() * 0.03f;  // Integrate gyro X to get roll (deg)
        // gyro_attitude.yaw = gyro_attitude.yaw + myICM.gyrZ() * 0.03f;   // Integrate gyro Z to get yaw (deg)

        updateGyroAttitude(&gyro_attitude, myICM, 0.03f);
        updateAccelAttitude(&accel_attitude, myICM);

        updateCompFilter(&comp_filter, gyro_attitude, accel_attitude);
        printAttitude(comp_filter.comp_attitude);
        //TODO print all three attitudes to compare

        // printFormattedFloat(gyro_attitude.pitch, 5, 2);
        // SERIAL_PORT.print(",");
        // printFormattedFloat(gyro_attitude.roll, 5, 2);
        // SERIAL_PORT.print(",");
        // printFormattedFloat(gyro_attitude.yaw, 5, 2);
        // SERIAL_PORT.print(",");
        // SERIAL_PORT.print(millis());
        // SERIAL_PORT.println("");

        //TODO micros() is millis() for uS
        //delay(30);
    }
    else
    {
        SERIAL_PORT.println("Waiting for data");
        delay(500);
    }
}
