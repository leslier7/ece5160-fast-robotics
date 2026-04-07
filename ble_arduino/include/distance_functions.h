#ifndef DISTANCE_FUNCTIONS_H
#define DISTANCE_FUNCTIONS_H

#include "SparkFun_VL53L1X.h"
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

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

extern Matrix<2,1> kf_mu;
extern Matrix<2,2> kf_sigma;


// KF values
constexpr float d = 15.881127 / 1000; //converting to mm
constexpr float m = 5.534828 / 1000;
constexpr float proc_noise = 100.0f; //(10mm^2)
//constexpr float sigma_1 = (10 * 10^-3) * math.sqrt(ave_dt / 0.13);
//constexpr float sigma_2 = (10 * 10^-3) * math.sqrt(ave_dt / 0.13);
//constexpr float sigma_3 = 20*(10^-3); // 20mm measurement noise

// Matrices for KF
const Matrix<2,2> A = {0, 1,
                      0, -d/m};

const Matrix<2,1> B = {0, 1/m};

const Matrix<1, 2> C = {1.0, 0.0};

const Matrix<2, 2> I = {1.0, 0.0,
                        0.0, 1.0};

// Global constant for measurement noise (14mm)^2 (the other two sigs depend on dt and have to be recalculated)
const Matrix<1, 1> sig_z = {196.0f};                     


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

  static uint32_t last_tof_check = 0;
  if (millis() - last_tof_check < 10) return; // poll at 100Hz max
  last_tof_check = millis();

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

inline void kf(Matrix<2, 1> &mu, Matrix<2, 2> &sigma, Matrix<1, 1> u, Matrix<1, 1> y) {
    
    // Time delta calculation
    static unsigned long kf_prev_time = micros();
    unsigned long current_time = micros();
    float dt = (current_time - kf_prev_time) / 1000000.0f; 
    dt = min(dt, 0.5f); // cap at 500ms to prevent blowup on first call
    kf_prev_time = current_time;

    // Dynamically calculate process noise variance
    float variance_step = (proc_noise / 0.13f) * dt; 
    Matrix<2, 2> sig_u = {variance_step, 0.0f,
                          0.0f, variance_step};

    // Dynamically construct discrete matrices
    Matrix<2, 2> Ad = {1.0f, dt,
                       0.0f, 1.0f - (dt * d / m)};

    Matrix<2, 1> Bd = {0.0f,
                       dt / m};

    // Prediction Step
    Matrix<2, 1> mu_p = Ad * mu + Bd * u;
    Matrix<2, 2> sigma_p = Ad * sigma * ~Ad + sig_u;
    
    // Measurement Update Step
    Matrix<1, 1> sigma_m = C * sigma_p * ~C + sig_z;
    Matrix<2, 1> kkf_gain = sigma_p * ~C * Inverse(sigma_m);
    
    Matrix<1, 1> y_m = y - C * mu_p;
    mu = mu_p + kkf_gain * y_m;
    sigma = (I - kkf_gain * C) * sigma_p;
    
}


#endif