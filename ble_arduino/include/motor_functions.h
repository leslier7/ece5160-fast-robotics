//
// Created by leslier on 2/28/26.
//

#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include <Arduino.h>

#define L1 2
#define L2 3
#define R1 15
#define R2 14

typedef enum channel{
    RIGHT,
    LEFT
} Channel;

struct MotorJob {
  float right_percent;
  float left_percent;
  uint32_t duration_ms;
};

struct MotorSpeeds {
    float left_percent;
    float right_percent;
};

extern float calibration_factor;

struct MotorJobQueue {
  static constexpr uint8_t CAP = 16; // pick size
  MotorJob buf[CAP];
  uint8_t head = 0;  // pop index
  uint8_t tail = 0;  // push index
  uint8_t count = 0;

  bool push(const MotorJob& j) {
    if (count >= CAP) return false;         // full
    buf[tail] = j;
    tail = (uint8_t)((tail + 1) % CAP);
    count++;
    return true;
  }

  bool pop(MotorJob& out) {
    if (count == 0) return false;           // empty
    out = buf[head];
    head = (uint8_t)((head + 1) % CAP);
    count--;
    return true;
  }

  void clear() { head = tail = count = 0; }
  bool empty() const { return count == 0; }
  bool full()  const { return count >= CAP; }
};

extern MotorJobQueue motor_q;

struct {
  bool active;
  uint32_t stop_at_ms;
  int16_t right_percent;
  int16_t left_percent;
} motor_job;

// Percent from -100 to 100 to set direction as well
inline bool setMotor(channel chan, float percent){

    if (percent > 100 || percent < -100) return false;
    if (percent == 100) percent = 99; //Making sure to always have a bit of duty cycle
    if (percent == -100) percent = -99;

    if (chan == LEFT) percent = -percent;   // Inverting one motor to keep direction the same

    bool forward = true;
    if(percent < 0) forward = false;

    int val = (int)((abs(percent) / 100.0) * 255);
    //DEBUG_PRINTF("Val: %d\n", val);

    int pin1, pin2;

    if (chan == RIGHT) {
        pin1 = R1; pin2 = R2;
    } else {
        pin1 = L1; pin2 = L2;
    }

    if (forward) {
        analogWrite(pin1, val);
        analogWrite(pin2, 0);
    } else {
        analogWrite(pin1, 0);
        analogWrite(pin2, val);
    }

    return true;
}

inline float applyMotorBand(float m) {
    const float LOWER_BAND = 10.0f;
    const float UPPER_BAND = 20.0f;

    if (fabs(m) < LOWER_BAND) return 0;
    if (m > 0 && m < UPPER_BAND) return UPPER_BAND;
    if (m < 0 && m > -UPPER_BAND) return -UPPER_BAND;
    return m;
}

inline bool setBothMotors(float rightMotor, float leftMotor){
    if (rightMotor > 100 || rightMotor < -100) return false;
    if (leftMotor > 100 || leftMotor < -100) return false;

    //TODO might have to update this to make it more robust
    rightMotor = applyMotorBand(rightMotor);
    leftMotor  = applyMotorBand(leftMotor);

    if (rightMotor == leftMotor){ // Calibration to make it go straight
        leftMotor += calibration_factor;
    }

    bool rightReturn = setMotor(RIGHT, rightMotor);
    bool leftReturn = setMotor(LEFT, leftMotor);

    return rightReturn && leftReturn;
}

inline void stopMotor(Channel chan){

    int pin1, pin2;
    if (chan == RIGHT) {
        pin1 = R1; pin2 = R2;
    } else {
        pin1 = L1; pin2 = L2;
    }

    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
}

inline void stopBothMotors(){
    analogWrite(L1, 0);
    analogWrite(R1, 0);
    analogWrite(L2, 0);
    analogWrite(R2, 0);
}

static void beginJob(const MotorJob& j);

void startMotorQueue();

void pauseMotorQueue();

bool startMotorJob(float right_percent, float left_percent, uint32_t duration_ms);

void serviceMotorJob();

bool queueMotorJob(float right_percent, float left_percent, uint32_t duration_ms);

void abortMotorQueue(bool clear_pending = true);

MotorSpeeds getCurSpeeds();

#endif //LAB2_IMU_FUNCTIONS_H