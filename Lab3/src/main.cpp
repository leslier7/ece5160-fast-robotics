// Original code by: Nathan Seidle
// Modified by: Robbie Leslie

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "distance_functions.h"

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensorFront;
SFEVL53L1X distanceSensorSide;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensorFront(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

int prev_time = 0;

inline void printFrontSensor(){
  distanceSensorFront.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensorFront.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensorFront.getDistance(); //Get the result of the measurement from the sensor
  distanceSensorFront.clearInterrupt();
  distanceSensorFront.stopRanging();

  Serial.print("Distance(cm): ");
  Serial.print(float(distance) / 10);

  Serial.print("\tDistance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();
}

void setup(void)
{ 
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  bool front_setup = setupSensor(distanceSensorFront, true);
  bool side_setup = setupSensor(distanceSensorSide, false);

  if(!front_setup || !side_setup){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Failed to start both distance sensors");
      while(1); // TODO maybe change this and set some kind of bluetooth status checker
  }

  // pinMode(A0, OUTPUT); //Used to control xshut on the side ToF
  // digitalWrite(A0, LOW);  // Xshut low to set I2C address

  // distanceSensorFront.setI2CAddress(0x30);

  // while(distanceSensorFront.getI2CAddress() != 0x30){
  //   Serial.println("Error setting I2C address");
  // }

  // digitalWrite(A0, HIGH);

  // if (distanceSensorFront.begin() != 0) //Begin returns 0 on a good init
  // {
  //   Serial.println("Front Sensor failed to begin. Please check wiring. Freezing...");
  //   while (1)
  //     ;
  // } 

  // if (distanceSensorSide.begin() != 0)
  // {
  //   Serial.println("Side Sensor failed to begin. Please check wiring. Freezing...");
  //   while (1)
  //     ;
  // }
  
  Serial.println("Both Sensors online!");
  distanceSensorFront.setDistanceModeShort(); //Setting it to short mode
  
}

void loop(void)
{ 
  // int front_dist = getSensorDistance(distanceSensorFront);
  // int side_dist = getSensorDistance(distanceSensorSide);

  if(distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()){
    Serial.println(micros() - prev_time); // printing clock
    prev_time = micros();
  }

  //Serial.println(front_dist);
  //Serial.println(side_dist);
  //Serial.println(""); //just for formatting, does make it slower

  //printFrontSensor();
}

