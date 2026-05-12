// ============================================================
//  Lab 12 — Inverted Pendulum / Wheelie Controller  (FIXED)
//
//  Fixes vs previous version:
//    - SET_GAINS parser rewritten — safe prefix match, no false hits
//    - KF_ALPHA1/ALPHA2 names corrected (were swapped)
//    - KF Ad/Bd matrices corrected for pitch pendulum dynamics
//    - kf_step blocked until kf_ready to prevent garbage first step
//    - GET_RUN_DATA: `now` scoped correctly for streaming block
//    - BLE.poll() called every loop iteration regardless of state
//    - Complementary filter gyro axis configurable via GYRO_AXIS
//    - prev_pid_time properly reset on kf_init
//    - Gains printed correctly in START_PID
//    - FALLEN_ANGLE now correctly defaults to 20 deg (new frame)
//    - KF alpha constants fixed: alpha1=5.36 (gravity), alpha2=45 (motor)
//
//  Pitch convention (complementary filter):
//    Flat on ground  →  ~0 deg
//    Upright/wheelie →  ~82 deg  (tune SP to match your car)
//    Upside down     →  ~180 deg
//
//  BLE Commands:
//    START_PID   (10)  → Phase A: hand-place upright, balance
//    START_RUN   (8)   → Phase B: full pop sequence
//    GET_RUN_DATA(9)   → stream logged data
//    STOP_ROBOT  (12)  → emergency stop
//    SET_GAINS   (11)  → see parse section for all keys
//
//  Log packet format:
//    T:<ms>|D1:<raw_pitch>|D2:<kf_pitch>|MOT:<pwm>|ERR:<pid_err>|VEL:<kf_rate>
// ============================================================

#include "ICM_20948.h"
#include <ArduinoBLE.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include "EString.h"
#include <Wire.h>

using namespace BLA;

// ========================
//    BLE UUIDs  (UNCHANGED)
// ========================
#define BLE_UUID_TEST_SERVICE "312f9813-7fbd-4dca-9b2f-17b3e7fc82f4"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"

#define AD0_VAL 1

// ========================
//    MOTOR PINS  (from motor_functions.h)
// ========================
#define L1 2
#define L2 3
#define R1 15
#define R2 14

const int DEADBAND_L = 60;
const int DEADBAND_R = 45;

// ========================
//    GYRO AXIS SELECTION
//    Check Serial with car upright: if pitch_rate is negative when
//    car tips forward, flip to -gyrX(). Try Y if X gives no response.
// ========================
#define GYRO_AXIS  (myICM.gyrY())   // change to gyrX() or -gyrY() if needed

// ========================
//    LOG
// ========================
#define LOG_SIZE 4000
unsigned long log_time [LOG_SIZE];
float         log_pitch[LOG_SIZE];
float         log_kfp  [LOG_SIZE];
int           log_pwm  [LOG_SIZE];
float         log_err  [LOG_SIZE];
float         log_vel  [LOG_SIZE];
float         log_gyro [LOG_SIZE];   // raw gyro pitch-rate (deg/s) — for sysID
int           log_idx  = 0;

// ========================
//    PID GAINS  (tunable via SET_GAINS)
//    Starting point from reference implementation scaled to degrees:
//      Kp = 0.04  (angle gain)
//      Kd = 0.002 (rate gain — on KF pitch-rate, already smooth)
//      Ki = 0.0   (add only after Kp/Kd stable)
// ========================
float Kp           = 0.04f;
float Ki           = 0.00f;
float Kd           = 0.002f;
float SETPOINT_DEG = 82.0f;    // tune to your physical balance point
float WINDUP_LIMIT = 200.0f;

#define RUN_DURATION_MS  15000   // hard safety timeout (ms)
#define FALLEN_ANGLE     20.0f   // abort if pitch drops below this (deg)
#define BALANCE_WINDOW   35.0f   // integral only active within this window (deg)

// ========================
//    POP SEQUENCE  (tunable via SET_GAINS)
// ========================
int   POP_DRIVE_MS  = 200;    // ms driving forward
int   POP_BRAKE_MS  = 300;    // ms braking hard
float HANDOFF_ANGLE = 55.0f;  // pitch (deg) at which balance loop takes over
int   POP_DRIVE_PWM = 220;
int   POP_BRAKE_PWM = 200;

// ========================
//    KF PARAMETERS
//    alpha1 = 5.36  deg/s² per deg   (gravity — from drop sysID)
//    alpha2 = 45.0  deg/s² per unit-u (motor  — from step sysID)
//    DT     = 0.0177s  (complementary filter / DMP rate)
//    Model: theta_ddot = alpha1*theta - alpha2*u
//    x = [theta, theta_dot]
//    Ad = I + DT*[[0,1],[alpha1,0]]
//    Bd = DT*[[0],[-alpha2]]
//    C  = [1, 0]
// ========================
float kf_sigma1 = 50.0f;   // pitch process noise (deg)     — high = trust DMP more
float kf_sigma2 = 50.0f;   // pitch-rate process noise (deg/s)
float kf_sigma3 =  1.0f;   // DMP measurement noise (deg)   — low  = trust DMP more

const float KF_ALPHA1 = 5.36f;    // gravity term
const float KF_ALPHA2 = 45.0f;    // motor torque term
const float KF_DT     = 0.0177f;  // seconds

Matrix<2,2> Ad;
Matrix<2,1> Bd;
Matrix<1,2> C_kf;
Matrix<2,2> Sigma_u;
Matrix<1,1> Sigma_z;

Matrix<2,1> kf_mu;
Matrix<2,2> kf_sigma;
bool        kf_ready = false;

// ========================
//    BLE
// ========================
ICM_20948_I2C myICM;
BLEService testService(BLE_UUID_TEST_SERVICE);
BLEStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, 550);
BLEFloatCharacteristic  tx_characteristic_float (BLE_UUID_TX_FLOAT,  BLERead | BLENotify);
BLEStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, 550);
EString tx_estring_value;

unsigned long last_ble_check = 0;
#define BLE_CHECK_INTERVAL_MS 250

enum CommandTypes {
    PING              = 0,
    SEND_TWO_INTS     = 1,
    SEND_THREE_FLOATS = 2,
    ECHO              = 3,
    DUMP_LOGS         = 4,
    GET_TIME_MILLIS   = 5,
    GET_IMU_DATA      = 6,
    START_LOGGING     = 7,
    START_RUN         = 8,
    GET_RUN_DATA      = 9,
    START_PID         = 10,
    SET_GAINS         = 11,
    STOP_ROBOT        = 12,
    SYSID_DROP        = 13    // free-fall from balance → extract alpha1 & alpha2
};

enum RobotState {
    STATE_IDLE,
    STATE_BALANCE,
    STATE_POP_DRIVE,
    STATE_POP_BRAKE,
    STATE_POP_WAIT,
    STATE_BALANCING,
    STATE_SYSID_DROP   // motors off, log phi + gyro for regression
};
RobotState robot_state = STATE_IDLE;

unsigned long state_enter_time = 0;
unsigned long run_start_time   = 0;

// ========================
//    SENSOR STATE
// ========================
float         current_pitch = 0.0f;
bool          imu_updated   = false;
float         gyro_pitch    = 0.0f;
float         raw_gyro_rate = 0.0f;   // deg/s from hardware gyro (for sysID)
unsigned long last_imu_time = 0;

// ========================
//    PID STATE
// ========================
float         integral      = 0.0f;
float         prev_error    = 0.0f;
unsigned long prev_pid_time = 0;

// ========================
//    BLE STREAMING
// ========================
bool is_sending_run = false;
int  run_send_index = 0;

// ========================
//    FORWARD DECLARATIONS
// ========================
void handle_command(BLEDevice central, BLECharacteristic characteristic);
void stop_motors();

// ========================
//    KF
// ========================
void build_kf_matrices() {
    // Ad = I + DT * [[0, 1], [alpha1, 0]]
    Ad = { 1.0f,               KF_DT,
           KF_ALPHA1 * KF_DT,  1.0f  };

    // Bd = DT * [[0], [-alpha2]]
    Bd = { 0.0f,
          -KF_ALPHA2 * KF_DT };

    // Measure pitch directly
    C_kf = { 1.0f, 0.0f };

    Sigma_u = { kf_sigma1 * kf_sigma1, 0.0f,
                0.0f,                   kf_sigma2 * kf_sigma2 };

    Sigma_z = { kf_sigma3 * kf_sigma3 };
}

void kf_init(float pitch_deg) {
    // State is [phi, phi_dot] where phi = theta - SETPOINT_DEG.
    // Linearising around the balance point means phi=0 is the equilibrium,
    // so phi_ddot = alpha1*phi - alpha2*u is correct (no constant offset).
    kf_mu    = { pitch_deg - SETPOINT_DEG, 0.0f };
    kf_sigma = { 25.0f, 0.0f,
                  0.0f, 25.0f };
    kf_ready = true;
}

// u_norm: signed normalised motor command [-1, 1]
// y_deg:  pitch measurement (deg)
// do_update: true when fresh DMP reading available this loop
void kf_step(float u_norm, float y_deg, bool do_update) {
    Matrix<1,1> u_mat;
    u_mat(0,0) = u_norm;

    Matrix<2,1> mu_p    = Ad * kf_mu + Bd * u_mat;
    Matrix<2,2> sigma_p = Ad * kf_sigma * ~Ad + Sigma_u;

    if (do_update) {
        Matrix<2,1> CT      = ~C_kf;
        Matrix<1,1> sigma_m = C_kf * sigma_p * CT + Sigma_z;
        Matrix<1,1> sigma_m_inv;
        sigma_m_inv(0,0) = 1.0f / sigma_m(0,0);
        Matrix<2,1> kf_gain = sigma_p * CT * sigma_m_inv;
        Matrix<1,1> y_mat;
        y_mat(0,0) = y_deg;
        Matrix<1,1> innov = y_mat - C_kf * mu_p;
        kf_mu = mu_p + kf_gain * innov;
        Matrix<2,2> I2 = { 1.0f, 0.0f, 0.0f, 1.0f };
        kf_sigma = (I2 - kf_gain * C_kf) * sigma_p;
    } else {
        kf_mu    = mu_p;
        kf_sigma = sigma_p;
    }
}

float kf_pitch()     { return SETPOINT_DEG + kf_mu(0); }  // phi → absolute theta
float kf_pitchrate() { return kf_mu(1); }

// ========================
//    MOTOR HELPERS  (ported from motor_functions.h/.cpp)
// ========================

void stop_motors() {
    analogWrite(L1, 0); analogWrite(L2, 0);
    analogWrite(R1, 0); analogWrite(R2, 0);
}

void brake_motors() {
    analogWrite(L1, 255); analogWrite(L2, 255);
    analogWrite(R1, 255); analogWrite(R2, 255);
}

// Accepts a raw PWM value (0–255).
// LEFT motor polarity is inverted (matching motor_functions.h) so both
// motors drive in the same physical direction for the same sign.
void drive_forward(int speed) {
    speed = constrain(speed, 0, 255);
    if (speed == 0) { stop_motors(); return; }
    analogWrite(R1, speed); analogWrite(R2, 0);      // right: forward
    analogWrite(L1, 0);     analogWrite(L2, speed);  // left: inverted forward
}

void drive_backward(int speed) {
    speed = constrain(speed, 0, 255);
    if (speed == 0) { stop_motors(); return; }
    analogWrite(R1, 0);     analogWrite(R2, speed);  // right: backward
    analogWrite(L1, speed); analogWrite(L2, 0);      // left: inverted backward
}

// pid_out > 0 → forward, pid_out < 0 → backward  (range ±255, same as before)
void drive_pid_output(float pid_out) {
    int speed = (int)fabsf(pid_out);
    if (speed < 1) { stop_motors(); return; }
    if (speed < 40) speed = 40;   //Adding a deadzone to help motors get started
    if (pid_out > 0) drive_backward(speed);
    else             drive_forward(speed);
}

// ========================
//    PID
// ========================
void reset_pid_state() {
    integral      = 0.0f;
    prev_error    = 0.0f;
    prev_pid_time = 0;
}

// Returns signed PWM. Returns 0.0 if fallen.
float run_pid(float pitch_kf, float pitchrate_kf, unsigned long now) {
    if (pitch_kf < FALLEN_ANGLE) return 0.0f;

    float error = SETPOINT_DEG - pitch_kf;
    float dt_s  = (prev_pid_time == 0) ? 0.0f
                                       : (float)(now - prev_pid_time) / 1000.0f;

    float p_term = Kp * error;

    // Integral — only inside balance window, clamp with anti-windup
    if (fabsf(error) <= BALANCE_WINDOW && dt_s > 0.0f) {
        integral += error * dt_s;
        integral  = constrain(integral, -WINDUP_LIMIT, WINDUP_LIMIT);
    } else if (fabsf(error) > BALANCE_WINDOW) {
        integral = 0.0f;   // reset outside window
    }
    float i_term = Ki * integral;

    // Derivative on measurement (KF rate is smooth — no extra filter needed)
    float d_term = -Kd * pitchrate_kf;

    float pid_out = p_term + i_term + d_term;
    pid_out = constrain(pid_out, -255.0f, 255.0f);

    prev_pid_time = now;
    return pid_out;
}

// ========================
//    IMU
// ========================
bool init_dmp() {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) return false;

    myICM.startupDefault();   // enable raw accel + gyro alongside DMP

    bool success = true;
    success &= (myICM.initializeDMP()                                           == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0)                      == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO()                                              == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP()                                               == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP()                                                == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO()                                               == ICM_20948_Stat_Ok);
    return success;
}

// Complementary filter pitch.
// Flat ≈ 0 deg, upright/wheelie ≈ 82 deg, upside-down ≈ ±180 deg.
// Returns true if a fresh DMP quaternion was read this call.
bool read_dmp_pitch() {
    icm_20948_DMP_data_t data;
    bool got_data = false;
    do {
        myICM.readDMPdataFromFIFO(&data);
        if (myICM.status == ICM_20948_Stat_Ok ||
            myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) {
            if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
                double q0 = sqrt(max(0.0, 1.0 - q1*q1 - q2*q2 - q3*q3));
                double sinp = 2.0 * (q0*q2 - q3*q1);
                sinp = constrain((double)sinp, -1.0, 1.0);
                current_pitch = (float)(asin(sinp) * 180.0 / PI);
                // Also grab raw gyro so we have theta_dot for sysID regression.
                myICM.getAGMT();
                raw_gyro_rate = GYRO_AXIS;
                got_data = true;
            }
        }
    } while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail);
    return got_data;
}

// ========================
//    SETUP
// ========================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);
    pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
    stop_motors();

    if (!init_dmp()) {
        Serial.println("DMP FAIL");
        while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(200); }
    }
    Serial.println("DMP ready.");

    build_kf_matrices();

    if (!BLE.begin()) { Serial.println("BLE FAIL"); while (1); }
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(rx_characteristic_string);
    BLE.addService(testService);
    rx_characteristic_string.setEventHandler(BLEWritten, handle_command);
    BLE.setConnectionInterval(6, 3200);
    BLE.advertise();

    Serial.println("BLE active — Artemis BLE [Lab 12 Wheelie]");
    Serial.print("Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.print(Kd);
    Serial.print(" SP="); Serial.println(SETPOINT_DEG);
}

// ========================
//    MAIN LOOP
// ========================
void loop() {
    unsigned long now = millis();

    // ── 1. IMU ───────────────────────────────────────────────
    imu_updated = read_dmp_pitch();

    // ── 2. BLE ───────────────────────────────────────────────
    // Always poll so commands arrive promptly
    BLE.poll();

    // Connection safety check during active runs
    bool active = (robot_state != STATE_IDLE);
    if (active && now - last_ble_check >= BLE_CHECK_INTERVAL_MS) {
        last_ble_check = now;
        BLEDevice central = BLE.central();
        if (!central || !central.connected()) {
            stop_motors();
            robot_state = STATE_IDLE;
            Serial.println("BLE lost — stopped.");
        }
    }

    // ── 3. KF PREDICT/UPDATE  (only when KF is initialised) ──
    if (kf_ready) {
        // Use last logged PWM as u (0 on first step — safe)
        float u_norm = (log_idx > 0)
                       ? ((float)log_pwm[log_idx - 1] / 255.0f)
                       : 0.0f;
        kf_step(u_norm, current_pitch - SETPOINT_DEG, imu_updated);
    }

    // ── 4. STATE MACHINE ─────────────────────────────────────
    switch (robot_state) {

        // ── IDLE ─────────────────────────────────────────────
        case STATE_IDLE:
            break;

        // ── PHASE A: BALANCE (hand-placed) ───────────────────
        case STATE_BALANCE: {
            // Hard timeout
            if (now - run_start_time >= RUN_DURATION_MS) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Timeout. Send cmd 9 for data.");
                break;
            }

            // Debug print every 300ms
            static unsigned long last_print = 0;
            if (now - last_print >= 300) {
                last_print = now;
                Serial.print("pitch="); Serial.print(current_pitch, 2);
                Serial.print(" kf=");   Serial.print(kf_ready ? kf_pitch() : 0.0f, 2);
                Serial.print(" u=");
                Serial.println(log_idx > 0 ? log_pwm[log_idx-1] : 0);
            }

            // Wait for car to be upright before engaging
            if (!kf_ready) {
                if (imu_updated && current_pitch > 50.0f) {
                    kf_init(current_pitch);
                    reset_pid_state();
                    prev_pid_time = now;
                    Serial.print("KF init at pitch="); Serial.println(current_pitch);
                }
                stop_motors();
                break;
            }

            float pk = kf_pitch();
            float vk = kf_pitchrate();

            // Fallen check
            if (pk < FALLEN_ANGLE) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.print("Fallen. pitch="); Serial.println(pk);
                break;
            }

            float pid_out = run_pid(pk, vk, now);
            drive_pid_output(pid_out);

            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = pk;
                log_pwm  [log_idx] = (int)pid_out;
                log_err  [log_idx] = SETPOINT_DEG - pk;
                log_vel  [log_idx] = vk;
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 1: DRIVE FORWARD ────────────────────
        case STATE_POP_DRIVE: {
            drive_forward(POP_DRIVE_PWM);

            if (now - state_enter_time >= (unsigned long)POP_DRIVE_MS) {
                state_enter_time = now;
                robot_state = STATE_POP_BRAKE;
                Serial.println("Pop: braking...");
            }
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = kf_ready ? kf_pitch() : current_pitch;
                log_pwm  [log_idx] = POP_DRIVE_PWM;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 2: BRAKE ────────────────────────────
        case STATE_POP_BRAKE: {
            drive_backward(POP_BRAKE_PWM);

            if (now - state_enter_time >= (unsigned long)POP_BRAKE_MS) {
                stop_motors();
                state_enter_time = now;
                robot_state = STATE_POP_WAIT;
                Serial.println("Pop: waiting for handoff...");
            }
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = kf_ready ? kf_pitch() : current_pitch;
                log_pwm  [log_idx] = -POP_BRAKE_PWM;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 3: WAIT FOR PITCH THRESHOLD ─────────
        case STATE_POP_WAIT: {
            // Timeout — car didn't flip enough
            if (now - state_enter_time > 600) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Handoff timeout — no flip. Abort.");
                break;
            }

            float pk = kf_ready ? kf_pitch() : current_pitch;

            if (pk >= HANDOFF_ANGLE) {
                kf_init(current_pitch);    // re-init KF fresh at handoff point
                reset_pid_state();
                prev_pid_time    = now;
                state_enter_time = now;
                robot_state      = STATE_BALANCING;
                Serial.print("Handoff at pitch="); Serial.println(pk);
            }

            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = pk;
                log_pwm  [log_idx] = 0;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 4: BALANCE AFTER POP ────────────────
        case STATE_BALANCING: {
            if (now - run_start_time >= RUN_DURATION_MS) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Balancing timeout. Send cmd 9.");
                break;
            }

            float pk = kf_ready ? kf_pitch() : current_pitch;
            float vk = kf_ready ? kf_pitchrate() : 0.0f;

            if (pk < FALLEN_ANGLE) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.print("Fallen after pop. pitch="); Serial.println(pk);
                break;
            }

            float pid_out = run_pid(pk, vk, now);
            drive_pid_output(pid_out);

            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = pk;
                log_pwm  [log_idx] = (int)pid_out;
                log_err  [log_idx] = SETPOINT_DEG - pk;
                log_vel  [log_idx] = vk;
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }

        // ── SYSID DROP: motors off, log phi + gyro ────────────
        // Place car near balance, send SYSID_DROP (cmd 13), release.
        // Python fits phi_ddot = alpha1*phi and alpha2 from the residual.
        case STATE_SYSID_DROP: {
            stop_motors();
            float pk = kf_ready ? kf_pitch() : current_pitch;
            // Stop recording if fallen or 3-second window elapsed
            if (pk < FALLEN_ANGLE || (now - run_start_time > 3000)) {
                robot_state = STATE_IDLE;
                Serial.print("SysID done. "); Serial.print(log_idx);
                Serial.println(" pts. Send cmd 9 for data.");
                break;
            }
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;                      // absolute theta
                log_kfp  [log_idx] = current_pitch - SETPOINT_DEG;       // phi
                log_pwm  [log_idx] = 0;
                log_err  [log_idx] = current_pitch - SETPOINT_DEG;       // phi (redundant but clear)
                log_vel  [log_idx] = raw_gyro_rate;                       // phi_dot (measured)
                log_gyro [log_idx] = raw_gyro_rate;
                log_idx++;
            }
            break;
        }
    }

    // ── 5. BLE DATA STREAM ───────────────────────────────────
    static unsigned long last_send_t = 0;
    if (is_sending_run && (now - last_send_t >= 10)) {
        if (run_send_index < log_idx) {
            tx_estring_value.clear();
            tx_estring_value.append("T:");    tx_estring_value.append((int)log_time [run_send_index]);
            tx_estring_value.append("|D1:");  tx_estring_value.append(log_pitch[run_send_index]);
            tx_estring_value.append("|D2:");  tx_estring_value.append(log_kfp  [run_send_index]);
            tx_estring_value.append("|MOT:"); tx_estring_value.append(log_pwm  [run_send_index]);
            tx_estring_value.append("|ERR:"); tx_estring_value.append(log_err  [run_send_index]);
            tx_estring_value.append("|VEL:"); tx_estring_value.append(log_vel  [run_send_index]);
            tx_estring_value.append("|GYR:"); tx_estring_value.append(log_gyro [run_send_index]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            run_send_index++;
            last_send_t = now;
        } else {
            is_sending_run = false;
            Serial.println("Transfer complete.");
        }
    }
}

// ========================
//    COMMAND HANDLER
// ========================

// Safe key-value parser: finds "KEY" and reads value up to next '|' or end.
// Returns NAN if key not found. Avoids false matches (e.g. "Ki" inside "Kp").
static float parse_float(const String& s, const char* key) {
    int i = s.indexOf(key);
    while (i != -1) {
        // Make sure the char before key is '|' or start (prevents "SKp" matching "Kp")
        bool valid_start = (i == 0) || (s.charAt(i - 1) == '|');
        if (valid_start) {
            int vs = i + strlen(key);
            int ve = s.indexOf('|', vs);
            if (ve == -1) ve = s.length();
            return s.substring(vs, ve).toFloat();
        }
        i = s.indexOf(key, i + 1);
    }
    return NAN;
}

static int parse_int(const String& s, const char* key) {
    int i = s.indexOf(key);
    while (i != -1) {
        bool valid_start = (i == 0) || (s.charAt(i - 1) == '|');
        if (valid_start) {
            int vs = i + strlen(key);
            int ve = s.indexOf('|', vs);
            if (ve == -1) ve = s.length();
            return s.substring(vs, ve).toInt();
        }
        i = s.indexOf(key, i + 1);
    }
    return INT_MIN;   // sentinel for "not found"
}

void handle_command(BLEDevice central, BLECharacteristic characteristic) {
    String cmd_str = rx_characteristic_string.value();
    int cmd_type   = cmd_str.toInt();
    Serial.print("CMD: "); Serial.println(cmd_type);

    switch (cmd_type) {

        // ── Phase A: hand-placed balance ─────────────────────
        case START_PID:
            stop_motors();
            log_idx          = 0;
            run_start_time   = millis();
            state_enter_time = millis();
            last_ble_check   = millis();
            reset_pid_state();
            kf_ready         = false;
            gyro_pitch       = 0.0f;
            last_imu_time    = 0;
            is_sending_run   = false;
            robot_state      = STATE_BALANCE;
            Serial.println(">>> Phase A: BALANCE <<<");
            Serial.print("Kp="); Serial.print(Kp);
            Serial.print(" Ki="); Serial.print(Ki);
            Serial.print(" Kd="); Serial.print(Kd);
            Serial.print(" SP="); Serial.println(SETPOINT_DEG);
            break;

        // ── Phase B: full pop sequence ────────────────────────
        case START_RUN:
            stop_motors();
            log_idx          = 0;
            run_start_time   = millis();
            state_enter_time = millis();
            last_ble_check   = millis();
            reset_pid_state();
            kf_init(current_pitch);   // init KF at current (flat) pitch
            gyro_pitch       = current_pitch;
            last_imu_time    = 0;
            is_sending_run   = false;
            robot_state      = STATE_POP_DRIVE;
            Serial.println(">>> Phase B: POP SEQUENCE <<<");
            Serial.print("DriveMS="); Serial.print(POP_DRIVE_MS);
            Serial.print(" BrakeMS="); Serial.print(POP_BRAKE_MS);
            Serial.print(" HandoffDeg="); Serial.println(HANDOFF_ANGLE);
            break;

        // ── Emergency stop ────────────────────────────────────
        case STOP_ROBOT:
            stop_motors();
            robot_state = STATE_IDLE;
            Serial.println(">>> STOP <<<");
            break;

        // ── SysID drop: motors off, log free-fall from balance ─
        // Procedure: hold car upright near SETPOINT_DEG, send this
        // command, then release. Robot logs for up to 3 s then idles.
        // Retrieve with GET_RUN_DATA (cmd 9). Python fits alpha1 & alpha2.
        case SYSID_DROP:
            stop_motors();
            log_idx        = 0;
            run_start_time = millis();
            kf_init(current_pitch);
            is_sending_run = false;
            robot_state    = STATE_SYSID_DROP;
            Serial.println(">>> SysID DROP — release car now <<<");
            Serial.print("SP="); Serial.print(SETPOINT_DEG);
            Serial.print(" current_pitch="); Serial.println(current_pitch);
            break;

        // ── Stream data ───────────────────────────────────────
        case GET_RUN_DATA:
            stop_motors();
            robot_state = STATE_IDLE;
            if (log_idx > 0) {
                is_sending_run = true;
                run_send_index = 0;
                Serial.print("Sending "); Serial.print(log_idx); Serial.println(" pts...");
            } else {
                Serial.println("No data to send.");
            }
            break;

        // ── Tune all parameters ───────────────────────────────
        case SET_GAINS: {
            stop_motors();
            robot_state = STATE_IDLE;
            bool kf_changed = false;
            float v; int vi;

            // PID gains
            v = parse_float(cmd_str, "Kp:"); if (!isnan(v)) Kp = v;
            v = parse_float(cmd_str, "Ki:"); if (!isnan(v)) {
                Ki = v;
                WINDUP_LIMIT = (Ki > 0) ? (150.0f / Ki) : 200.0f;
            }
            v = parse_float(cmd_str, "Kd:"); if (!isnan(v)) Kd = v;
            v = parse_float(cmd_str, "SP:"); if (!isnan(v)) SETPOINT_DEG = v;

            // KF sigmas
            v = parse_float(cmd_str, "S1:"); if (!isnan(v)) { kf_sigma1 = v; kf_changed = true; }
            v = parse_float(cmd_str, "S2:"); if (!isnan(v)) { kf_sigma2 = v; kf_changed = true; }
            v = parse_float(cmd_str, "S3:"); if (!isnan(v)) { kf_sigma3 = v; kf_changed = true; }
            if (kf_changed) build_kf_matrices();

            // Pop sequence timing
            vi = parse_int(cmd_str, "PD:");   if (vi != INT_MIN) POP_DRIVE_MS  = vi;
            vi = parse_int(cmd_str, "PB:");   if (vi != INT_MIN) POP_BRAKE_MS  = vi;
            v  = parse_float(cmd_str, "HO:"); if (!isnan(v))     HANDOFF_ANGLE = v;
            vi = parse_int(cmd_str, "DPWM:"); if (vi != INT_MIN) POP_DRIVE_PWM = vi;
            vi = parse_int(cmd_str, "BPWM:"); if (vi != INT_MIN) POP_BRAKE_PWM = vi;

            Serial.println("=== GAINS SET ===");
            Serial.print("Kp:"); Serial.print(Kp);
            Serial.print(" Ki:"); Serial.print(Ki);
            Serial.print(" Kd:"); Serial.print(Kd, 4);
            Serial.print(" SP:"); Serial.println(SETPOINT_DEG);
            Serial.print("KF sigma: "); Serial.print(kf_sigma1);
            Serial.print(" / "); Serial.print(kf_sigma2);
            Serial.print(" / "); Serial.println(kf_sigma3);
            Serial.print("Pop: DriveMS="); Serial.print(POP_DRIVE_MS);
            Serial.print(" BrakeMS="); Serial.print(POP_BRAKE_MS);
            Serial.print(" HandoffDeg="); Serial.println(HANDOFF_ANGLE);
            break;
        }

        default:
            Serial.println("Unknown cmd.");
            break;
    }
}