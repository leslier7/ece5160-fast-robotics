#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include "SparkFun_VL53L1X.h"
#include <Arduino.h>

#define SETUP_TRIES 5

struct Distances{
  int front;
  int side;
  int front_status;
  int side_status;
  bool front_updated;
  bool side_updated;
  int front_dt;
  unsigned long front_prev_time;
  int side_dt;
  unsigned long side_prev_time;
};

struct DistanceLog {
    int16_t front;
    int16_t side;
};

extern Distances cur_dists;
extern Distances prev_dists;
extern Distances pred_dists;

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
  out.front_status = front.getRangeStatus();
  out.side  = side.getDistance();
  out.side_status = side.getRangeStatus();

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
  out.front_updated = false;
  out.side_updated = false;

  if(frontSensor.checkForDataReady()){
    prev_dists.front = out.front;
    out.front_updated = true;
    unsigned long now = millis();
    out.front_dt = now - out.front_prev_time;
    out.front_prev_time = now;
    //out.front = getSensorDistance(frontSensor);
    out.front = frontSensor.getDistance();
    out.front_status = frontSensor.getRangeStatus();
    frontSensor.clearInterrupt();
    // if (out.front == -1) {
    //   digitalWrite(LED_BUILTIN, HIGH);
    // }
  }

  if(sideSensor.checkForDataReady()){
    prev_dists.side = out.side;
    out.side_updated = true;
    unsigned long now = millis();
    out.side_dt = now - out.side_prev_time;
    out.side_prev_time = now;
    out.side = sideSensor.getDistance();
    out.side_status = sideSensor.getRangeStatus();
    sideSensor.clearInterrupt();
    //out.side = getSensorDistance(sideSensor);
  }
}

bool setupSensor(SFEVL53L1X &sensor, bool alternate);

bool setupBothSensors(SFEVL53L1X &front, SFEVL53L1X &side);

Distances predictDistances(Distances &cur_dists, Distances &prev_dists);

#endif