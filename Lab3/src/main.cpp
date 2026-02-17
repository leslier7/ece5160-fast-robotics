// Original code by: Nathan Seidle
// Modified by: Robbie Leslie

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensorFront;
SFEVL53L1X distanceSensorSide;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensorFront(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensorFront.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Front Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  } 
  // else if (distanceSensorSide.begin() != 0)
  // {
  //   Serial.println("Side Sensor failed to begin. Please check wiring. Freezing...");
  //   while (1)
  //     ;
  // }
  
  Serial.println("Both Sensors online!");
  distanceSensorFront.setDistanceModeShort(); //Setting it to short mode
  
  Serial.print("Sensor I2C address: ");
  Serial.println(distanceSensorFront.getI2CAddress());
}

void loop(void)
{
  // distanceSensorFront.startRanging(); //Write configuration bytes to initiate measurement
  // while (!distanceSensorFront.checkForDataReady())
  // {
  //   delay(1);
  // }
  // int distance = distanceSensorFront.getDistance(); //Get the result of the measurement from the sensor
  // distanceSensorFront.clearInterrupt();
  // distanceSensorFront.stopRanging();

  // Serial.print("Distance(mm): ");
  // Serial.print(distance);

  // float distanceInches = distance * 0.0393701;
  // float distanceFeet = distanceInches / 12.0;

  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);

  // Serial.println();
}
