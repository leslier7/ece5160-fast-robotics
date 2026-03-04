#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include "SparkFun_VL53L1X.h"
#include <Arduino.h>

#define SETUP_TRIES 5

struct Distances{
  int front;
  int side;
};

inline int getSensorDistance(SFEVL53L1X &sensor){

  if(sensor.checkForDataReady()){
    int distance = sensor.getDistance(); //Get the result of the measurement from the sensor
    sensor.clearInterrupt();
    return distance;
  } else {
    return -1; //-1 is data not ready error
  }
  
}

inline bool bothDistancesIfReady(SFEVL53L1X &front, SFEVL53L1X &side)
{
  bool fr = front.checkForDataReady();
  bool sr = side.checkForDataReady();
  if (!(fr && sr)) return false;

  return true;
}

inline bool readBothDistancesIfReady(SFEVL53L1X &front, SFEVL53L1X &side, Distances &out)
{
  bool fr = front.checkForDataReady();
  bool sr = side.checkForDataReady();
  if (!(fr && sr)) return false;

  out.front = front.getDistance();
  out.side  = side.getDistance();

  front.clearInterrupt();
  side.clearInterrupt();
  return true;
}

inline Distances getDistances(SFEVL53L1X &frontSensor, SFEVL53L1X &sideSensor){
  Distances dists = {-1, -1};
  
  int front_dist = getSensorDistance(frontSensor);
  int side_dist = getSensorDistance(sideSensor);

  if(front_dist != -1){
    dists.front = front_dist;
  }

  if(side_dist != -1){
    dists.side = side_dist;
  }

  return dists;
}

inline void updateDistance(Distances &out, SFEVL53L1X &frontSensor, SFEVL53L1X &sideSensor){

  if(frontSensor.checkForDataReady()){
    out.front = getSensorDistance(frontSensor);
  }

  if(sideSensor.checkForDataReady()){
    out.side = getSensorDistance(sideSensor);
  }
}

bool setupSensor(SFEVL53L1X &sensor, bool alternate);

#endif