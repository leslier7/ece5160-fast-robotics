#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include "SparkFun_VL53L1X.h"
#include <Arduino.h>

#define SETUP_TRIES 5

inline int getSensorDistance(SFEVL53L1X &sensor){

  if(sensor.checkForDataReady()){
    int distance = sensor.getDistance(); //Get the result of the measurement from the sensor
    sensor.clearInterrupt();
    return distance;
  } else {
    return -1; //-1 is data not ready error
  }
  
}

bool setupSensor(SFEVL53L1X &sensor, bool alternate);

#endif