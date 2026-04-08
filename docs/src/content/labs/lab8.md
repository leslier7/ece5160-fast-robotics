---
title: "Lab 8: Stunts"
date: 2026-03-25
description: "Performing a drift stunt with the car"
cover: "../../assets/lab8-cover.jpg"
---

## Objective
The objective of this lab was to perform either a flip or a drift stunt with the car. I chose to do the drift.

## Lab work

### Lab Tasks

#### The Drift

<figure style="max-width: 720px; margin: 1rem 0;">
  <iframe
    width="100%"
    height="405"
    src="https://www.youtube-nocookie.com/embed/8XUSd-_2tUo"
    title="Lab 8 drift video"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
</figure>


Warning: flashing lights for the slowmo video (thanks PWM lighting)
<figure style="max-width: 720px; margin: 1rem 0;">
  <iframe
    width="100%"
    height="405"
    src="https://www.youtube-nocookie.com/embed/McoNjh6ok6E"
    title="Lab 8 slow motion drift"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
</figure>

For this lab, I decided to do the drift stunt. This meant that I had to drive my car into the wall as fast as I could, then have it turn 180 degrees, and then return to the starting line. I started the lab by doing entirely open-loop control. I used the motor control queue I implemented in lab 4 to set a command to move forward for a set amount of time, spin the wheels in opposite directions, and then return to the start. This worked, but it wasn't the best solution, especially the turn. I found that the following sequence worked pretty well as a starting point:

```python
sequence = ["0|0|2000","100|100|850", "-100|100|150", "100|100|850"]
```

Where the pattern is "left_motor|right_motor|duration in milliseconds"

#### Car Modifications
To make my car drift better, I had to reduce wheel friction. To do this, I added duct tape to the wheels. But first, I covered the wheels in masking tape. I did two layers of tape because Professor Farrell mentioned that duct tape alone left a sticky residue on the wheels, which makes the mapping labs very difficult since I need to spin the wheels in small increments. The modified car is shown in Figure 1.

<div style="display: flex; gap: 1rem; flex-wrap: wrap;">
  <figure style="flex: 1; min-width: 300px;">
    <img src="/ece5160-fast-robotics/assets/lab8/tapedwheels.jpg" alt="" style="width: 100%;">
    <figcaption>Figure 1: Taped wheels to reduce friction.</figcaption>
  </figure>
</div>

#### Home Setup
I did the final recording for the stunt at my apartment. I measured out the 4m and 3ft required for the stunt, and then marked them using tape. Figures 2, 3, and 4 show the home setup

<div style="display: flex; gap: 1rem; flex-wrap: wrap;">
  <figure style="flex: 1; min-width: 300px;">
    <img src="/ece5160-fast-robotics/assets/lab8/3ft.jpg" alt="" style="width: 100%;">
    <figcaption>Figure 2: 3ft from the wall</figcaption>
  </figure>
  <figure style="flex: 1; min-width: 300px;">
    <img src="/ece5160-fast-robotics/assets/lab8/3ft-close.jpg" alt="" style="width: 100%;">
    <figcaption>Figure 3: 3ft close up</figcaption>
  </figure>
  <figure style="flex: 1; min-width: 300px;">
    <img src="/ece5160-fast-robotics/assets/lab8/4m.jpg" alt="" style="width: 100%;">
    <figcaption>Figure 4: 4m from the wall (~13ft and my tape measure was only 12, hence the ruler)</figcaption>
  </figure>
</div>

#### Closed Loop Control
To make the turn easier, I decided to add a PID loop using the IMU to turn a fixed angle. But since I was now using both the motor queue and a PID controller, I had to build a state machine. I had five states: START, TOWARD_WALL, DRIFT, RETURN, and END. I also added the command to start the stunt, which allowed me to adjust some parameters to simplify rapid testing.

##### **START**
In the START state, I grabbed the current yaw for later use in the turn. It also reset the PID controller. It then transitioned to the TOWARD_WALL state.

##### **TOWARD_WALL**
In the TOWARD_WALL state, I added a command to the motor queue to drive both wheels at max speed for a set time. The time was one of the parameters that I sent over Bluetooth with the drift command. The state machine would remain in this state until the motor queue was empty. After the queue was empty, it would transition to the DRIFT state. During the transition, it set the PID controller's setpoint and started it.

##### **DRIFT**
For DRIFT, it ran the PID controller to spin the car. After calculating the PID percentage, it checked if it had crossed the setpoint. If it had not crossed the setpoint, the state machine remained in the DRIFT state. If the setpoint was crossed, it would set up the RETURN state by stopping the PID controller, adding a quick break command to drift, and then adding the return drive by maxing the wheels for the same amount of time it took to drive towards the wall. It then would go to the RETURN state.

##### **RETURN**
In the RETURN state, it just checked if the motor queue was empty. If the queue was still running, it remained in the RETURN state. When the queue was empty, it would transition to the END state.

##### **END**
The end state was very simple. It just set a global drift flag to false, so the car knew the stunt was over. It then always transitioned back to the START state to wait for the next drift.

<figure>
<figcaption>Drift State Machine Code</figcaption>

```cpp
void driftStateTick(){
    switch(DriftState){
        case START:
            if(drift_running){
                //queueMotorJob(100, 100, 850);
                queueMotorJob(100, 100, drive_time);
                //startMotorJob(100, 100, 850);
                startMotorQueue();
                DriftState = TOWARD_WALL;
            } else {
                DriftState = START;
            }
            break;
        case TOWARD_WALL:
            //this assumes that serviceMotorQueue is called in main
            if(isMotorQueueBusy()) {
                DriftState = TOWARD_WALL;
            } else if (isMotorQueueIdle()) {
                if(yaw < 180){
                    setSetpoint(imu_pid, yaw+turn_angle);
                } else if (yaw > 180) {
                    setSetpoint(imu_pid, yaw-turn_angle);
                } else {
                    setSetpoint(imu_pid,0);
                }
                startPID(imu_pid);

                prev_yaw_error = imu_pid.setpoint - yaw;
                if (prev_yaw_error > 180.0f) prev_yaw_error -= 360.0f;
                if (prev_yaw_error < -180.0f) prev_yaw_error += 360.0f;
                DriftState = DRIFT;
            }
            break;
        case DRIFT: {

            float pid_percent = updatePID(imu_pid);
            setBothMotors(pid_percent, -pid_percent); //Spin motors in oposite directions

            // Within +- 10 degrees of setpoint
            float yaw_error = imu_pid.setpoint - yaw;
            if (yaw_error > 180.0f) yaw_error -= 360.0f;
            if (yaw_error < -180.0f) yaw_error += 360.0f;

            bool crossed_target =
                (prev_yaw_error > 0.0f && yaw_error < 0.0f) ||
                (prev_yaw_error < 0.0f && yaw_error > 0.0f);

            if (fabs(yaw_error) <= (float)angle_zone || crossed_target) {
                stopPID(imu_pid);
                stopBothMotors();
                queueMotorJob(-pid_percent, pid_percent, break_time);   // brief brake to shed angular momentum
                queueMotorJob(100, 100, drive_time);
                startMotorQueue();
                DriftState = RETURN;
            } else {
                DriftState = DRIFT;
            }
            prev_yaw_error = yaw_error;
            break;
        }
        case RETURN:
            // Wait for queue to empty
            if(isMotorQueueBusy()){
                DriftState = RETURN;
            } else if (isMotorQueueIdle()){
                DriftState = END;
            }
            break;
        case END:
            drift_running = false;
            DriftState = START;
            break;
    }

}
```

</figure>

#### Improvements
While my stunt did work, there are some things that could have been improved. The main one would be using the ToF sensor to detect the wall rather than just timing. I used the timing approach for a few reasons. The main one is that the TOF sensors are slow, and this stunt is fast. The Kalman Filter could have helped solve this problem, but because I took a slip week for Lab 7, I hadn't finished it yet. I also wanted another excuse to use the motor queue, since I had already implemented it in Lab 4, making it easy to send a sequence of motor commands. Another improvement I would have liked to make was using the IMU to drive the car straight while going to/returning from the wall. I used a calibration factor to keep the car straight, but had to recalibrate both because of the tape and because, when I calibrated it, I characterized it at 40%, not 100%. A better version would drive straight but use a tiny PID controller to make micro-adjustments to the speeds, keeping the car straight for much longer. This would have taken more time than I had, so I didn't implement it.

#### Bloopers

<figure style="max-width: 720px; margin: 1rem 0;">
  <iframe
    width="100%"
    height="405"
    src="https://www.youtube-nocookie.com/embed/cu3Sz0E2QtI"
    title="Lab 8 Bloopers"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
</figure>

### Collaborations
I talked with Immanuel Koshy about some high level content, but mostly worked on my own. I used AI to help format the writeup.