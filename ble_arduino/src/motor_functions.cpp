#include "motor_functions.h"

static bool motor_queue_enabled = false;

MotorJobQueue motor_q;

void startMotorQueue() {
  motor_queue_enabled = true;
}

void pauseMotorQueue() {          // pauses after current job finishes (or immediately, your choice)
  motor_queue_enabled = false;
}

bool startMotorJob(float right_percent, float left_percent, uint32_t duration_ms) {
  // enqueue and let serviceMotorJob() start it deterministically
  return queueMotorJob(right_percent, left_percent, duration_ms);
}

static void beginJob(const MotorJob& j) {
  motor_job.active = true;
  motor_job.right_percent = j.right_percent;
  motor_job.left_percent  = j.left_percent;
  motor_job.stop_at_ms    = millis() + j.duration_ms;
  setBothMotors(j.right_percent, j.left_percent);
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