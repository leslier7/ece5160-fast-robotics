#include "motor_functions.h"

static MotorJob motor_job;

void startMotorJob(int right_percent, int left_percent, uint32_t duration_ms){
    motor_job.active = true;
    motor_job.right_percent = right_percent;
    motor_job.left_percent = left_percent;
    motor_job.stop_at_ms = millis() + duration_ms;
    setBothMotors(right_percent, left_percent);
}

void serviceMotorJob(){
    if (!motor_job.active) return;
  // wrap-safe compare
  if ((int32_t)(millis() - motor_job.stop_at_ms) >= 0) {
    motor_job.active = false;
    stopBothMotors();
  }
}