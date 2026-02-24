// Original code by: Nathan Seidle
// Modified by: Robbie Leslie

#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#include "distance_functions.h"

#ifdef IMU
#include <ICM_20948.h>
#include "imu_functions.h"
#endif

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensorFront;
SFEVL53L1X distanceSensorSide;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensorFront(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

Distances dists = {-1, -1};

#ifdef IMU
ICM_20948_I2C myICM; // Create an ICM_20948_I2C object
#endif

int prev_time = 0;

uint32_t last_report_us = 0;
uint32_t loop_count = 0;

uint16_t front_mm = 0, side_mm = 0;
uint32_t front_updates = 0, side_updates = 0;

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

  #ifdef IMU
  initIMU(myICM);
  #endif

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

  // if(distanceSensorFront.checkForDataReady() && distanceSensorSide.checkForDataReady()){
  //   distanceSensorFront.clearInterrupt();
  //   distanceSensorSide.clearInterrupt();
  //   uint32_t now = micros();
  //   Serial.println(now - prev_time); // printing clock
  //   prev_time = now;
  // }

  // loop_count++;

  // // --- Poll sensors, non-blocking ---
  // if (distanceSensorFront.checkForDataReady()) {
  //   front_mm = distanceSensorFront.getDistance();   // consume new sample
  //   distanceSensorFront.clearInterrupt();           // ack/clear ready
  //   // If your library requires it: distanceSensorFront.startRanging();
  //   front_updates++;
  // }

  // if (distanceSensorSide.checkForDataReady()) {
  //   side_mm = distanceSensorSide.getDistance();
  //   distanceSensorSide.clearInterrupt();
  //   // If your library requires it: distanceSensorSide.startRanging();
  //   side_updates++;
  // }

  // // --- Periodic reporting (DO NOT print every iteration) ---
  // uint32_t now = micros();
  // if (now - last_report_us >= 100000) { // 100 ms
  //   float seconds = (now - last_report_us) / 1e6f;
  //   float loop_hz = loop_count / seconds;

  //   Serial.print("loop_hz=");
  //   Serial.print(loop_hz, 1);
  //   Serial.print(" front_hz=");
  //   Serial.print(front_updates / seconds, 1);
  //   Serial.print(" side_hz=");
  //   Serial.println(side_updates / seconds, 1);

  //   last_report_us = now;
  //   loop_count = 0;
  //   front_updates = 0;
  //   side_updates = 0;
  // }

  
  //Serial.println(""); //just for formatting, does make it slower

  //printFrontSensor();

  updateDistance(dists, distanceSensorFront, distanceSensorSide);

  Serial.print("Front: ");
  Serial.print(dists.front);
  Serial.print("    Side: ");
  Serial.print(dists.side);
  Serial.println("");

  #ifdef IMU
  Serial.print(dists.front);
  Serial.print(",");
  Serial.print(dists.side);
  Serial.print(",");

  myICM.getAGMT();
  printSerialPlot(&myICM);
  #endif
}

