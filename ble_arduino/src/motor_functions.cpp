#include "motor_functions.h"
#include "globals.h"
#include "pid.h"

float calibration_factor = 13.85;

static bool motor_queue_enabled = false;

MotorJobQueue motor_q;

MotorSpeeds cur_speeds;

void startMotorQueue() {
  motor_queue_enabled = true;
}

void pauseMotorQueue() {
  motor_queue_enabled = false;
}

bool startMotorJob(float right_percent, float left_percent, uint32_t duration_ms) {
  // enqueue and let serviceMotorJob() start it
  return queueMotorJob(right_percent, left_percent, duration_ms);
}

static void beginJob(const MotorJob& j) {
  motor_job.active = true;
  motor_job.right_percent = j.right_percent;
  motor_job.left_percent  = j.left_percent;
  motor_job.stop_at_ms    = millis() + j.duration_ms;
  setBothMotors(j.left_percent, j.right_percent);
}

void serviceMotorJob() {
  // If a job is running, always service its timeout (so it can stop)
  if (motor_job.active) {
    if ((int32_t)(millis() - motor_job.stop_at_ms) >= 0) {
      motor_job.active = false;
      stopBothMotors();
    } else {
      return; // still running
    }
  }

  // If idle, only start next job if queue is enabled
  if (!motor_queue_enabled) return;

  // Try to start next job
  MotorJob next;
  if (motor_q.pop(next)) {
    beginJob(next);
  } else {
    // Queue empty so auto-disable
    motor_queue_enabled = false;
  }
}

bool queueMotorJob(float right_percent, float left_percent, uint32_t duration_ms) {
  MotorJob j{
    right_percent,
    left_percent,
    duration_ms
  };
  return motor_q.push(j);
}

void abortMotorQueue(bool clear_pending) {
  motor_queue_enabled = false;
  motor_job.active = false;
  stopBothMotors();
  if (clear_pending) motor_q.clear();
}

bool isMotorQueueBusy() {
    return motor_queue_enabled;
}

bool isMotorQueueIdle() {
    return !motor_queue_enabled;
}

void setCurSpeeds(float left_percent, float right_percent){
  cur_speeds.left_percent = left_percent;
  cur_speeds.right_percent = right_percent;
}

MotorSpeeds getCurSpeeds(){
  if (motor_job.active) {
    return { (float)(motor_job.left_percent), (float)(motor_job.right_percent) };
  } else if (pid_controller.running){
    return {cur_speeds.left_percent, cur_speeds.right_percent};
  } else {
    return { 0.0f, 0.0f };
  }
}