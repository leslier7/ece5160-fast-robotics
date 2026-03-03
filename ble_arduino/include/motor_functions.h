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
  bool active = false;
  uint32_t stop_at_ms = 0;
  int left_percent = 0;
  int right_percent = 0;
};


// Percent from -100 to 100 to set direction as well
inline bool setMotor(channel chan, float percent){

    if (percent > 100 || percent < -100) return false;
    if (percent == 100) percent = 99; //Making sure to always have a bit of duty cycle
    if (percent == -100) percent = -99;

    if (chan == LEFT) percent = -percent;   // Inverting one motor to keep direction the same

    bool forward = true;
    if(percent < 0) forward = false;

    int val = (int)((abs(percent) / 100.0) * 255);
    Serial.printf("Val: %d\n", val);

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

inline bool setBothMotors(float rightMotor, float leftMotor){
    if (rightMotor > 100 || rightMotor < -100) return false;
    if (leftMotor > 100 || leftMotor < -100) return false;

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

void startMotorJob(int right_percent, int left_percent, uint32_t duration_ms);

void serviceMotorJob();


#endif //LAB2_IMU_FUNCTIONS_H