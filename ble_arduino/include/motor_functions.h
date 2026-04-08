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

constexpr float MOTOR_LOWER_BAND = 5.0f;
constexpr float MOTOR_UPPER_BAND = 20.0f;
constexpr float MOTOR_BRAKE_CMD = 1000.0f;

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

// // Percent from -100 to 100 to set direction as well
// inline bool setMotor(channel chan, float percent){

//     if (percent > 100 || percent < -100) return false;
//     if (percent == 100) percent = 99; //Making sure to always have a bit of duty cycle
//     if (percent == -100) percent = -99;

//     if (chan == LEFT) percent = -percent;   // Inverting one motor to keep direction the same

//     bool forward = true;
//     if(percent < 0) forward = false;

//     int val = (int)((abs(percent) / 100.0) * 255);
//     //DEBUG_PRINTF("Val: %d\n", val);

//     int pin1, pin2;

//     if (chan == RIGHT) {
//         pin1 = R1; pin2 = R2;
//     } else {
//         pin1 = L1; pin2 = L2;
//     }

//     if (forward) {
//         analogWrite(pin1, val);
//         analogWrite(pin2, 0);
//     } else {
//         analogWrite(pin1, 0);
//         analogWrite(pin2, val);
//     }

//     return true;
// }

void setCurSpeeds(float left_percent, float right_percent);

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
    setCurSpeeds(0,0);

    analogWrite(L1, 0);
    analogWrite(R1, 0);
    analogWrite(L2, 0);
    analogWrite(R2, 0);
}

inline void brakeMotor(Channel chan){
    int pin1, pin2;
    if (chan == RIGHT) {
        pin1 = R1; pin2 = R2;
    } else {
        pin1 = L1; pin2 = L2;
    }

    analogWrite(pin1, 255);
    analogWrite(pin2, 255);
}

inline void brakeBothMotors(){
    analogWrite(L1, 255);
    analogWrite(R1, 255);
    analogWrite(L2, 255);
    analogWrite(R2, 255);
}

inline bool setMotor(Channel chan, float percent){

    if (percent == MOTOR_BRAKE_CMD) {
        brakeMotor(chan);
        return true;
    }

    if (percent > 100 || percent < -100) return false;
    if (percent == 100) percent = 99;
    if (percent == -100) percent = -99;

    if (chan == LEFT) percent = -percent;

    bool forward = true;
    if(percent < 0) forward = false;

    int val = (int)((abs(percent) / 100.0f) * 255.0f);

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

// inline float applyMotorBand(float m) {

//     if (fabs(m) < MOTOR_LOWER_BAND) return 0; 
//     if (m > 0 && m < MOTOR_UPPER_BAND) return MOTOR_UPPER_BAND; 
//     if (m < 0 && m > -MOTOR_UPPER_BAND) return -MOTOR_UPPER_BAND; 
//     return m;
// }

inline bool driveMotorCommand(Channel chan, float cmd) {
    if (cmd == MOTOR_BRAKE_CMD) {
        brakeMotor(chan);
        return true;
    }
    return setMotor(chan, cmd);
}

inline float applyMotorBand(float m) {
    const float LOWER_BAND = 5.0f;
    const float UPPER_BAND = 30.0f;
    const float NEG_LOWER_BAND = 5.0f;
    const float NEG_UPPER_BAND = 35.0f;

    if (m > 0 && m < LOWER_BAND) return MOTOR_BRAKE_CMD;
    if (m >= LOWER_BAND && m < UPPER_BAND) return UPPER_BAND;

    if (m < 0 && m > -NEG_LOWER_BAND) return MOTOR_BRAKE_CMD;
    if (m <= -NEG_LOWER_BAND && m > -NEG_UPPER_BAND) return -NEG_UPPER_BAND;

    return m;
}

inline bool setBothMotors(float leftMotor, float rightMotor){
    if (rightMotor > 100 || rightMotor < -100) return false;
    if (leftMotor > 100 || leftMotor < -100) return false;

    setCurSpeeds(leftMotor, rightMotor); // set the recorded motor speed

    //TODO might have to update this to make it more robust
    rightMotor = applyMotorBand(rightMotor);
    leftMotor  = applyMotorBand(leftMotor);

    if (rightMotor == leftMotor && rightMotor != MOTOR_BRAKE_CMD){ // Calibration to make it go straight
        if(leftMotor != 100){
            leftMotor += calibration_factor;
        } else if (leftMotor == 100){ //if running at 100%, subtract from the right wheel instead of adding to the left
            rightMotor -= calibration_factor;
        }
        
    }

    // bool rightReturn = setMotor(RIGHT, rightMotor);
    // bool leftReturn = setMotor(LEFT, leftMotor);

    bool rightReturn = driveMotorCommand(RIGHT, rightMotor);
    bool leftReturn  = driveMotorCommand(LEFT, leftMotor);

    return rightReturn && leftReturn;
}

static void beginJob(const MotorJob& j);

void startMotorQueue();

void pauseMotorQueue();

bool isMotorQueueBusy();

bool isMotorQueueIdle();

bool startMotorJob(float right_percent, float left_percent, uint32_t duration_ms);

void serviceMotorJob();

bool queueMotorJob(float right_percent, float left_percent, uint32_t duration_ms);

void abortMotorQueue(bool clear_pending = true);

MotorSpeeds getCurSpeeds();

#endif //LAB2_IMU_FUNCTIONS_H