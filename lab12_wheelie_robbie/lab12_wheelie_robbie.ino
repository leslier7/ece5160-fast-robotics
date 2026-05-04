// ============================================================
//  Lab 12 — Inverted Pendulum / Wheelie Controller
//
//  State machine:
//    IDLE       → waiting for BLE command
//    BALANCE    → (Phase A) car placed upright by hand, PID+KF holds it
//    POP_DRIVE  → (Phase B) drive forward at full speed to build momentum
//    POP_BRAKE  → hard brake to pitch the car back
//    POP_WAIT   → wait until pitch crosses HANDOFF_ANGLE before balancing
//    BALANCING  → same PID+KF loop as Phase A, triggered after pop
//
//  BLE Commands (backward-compatible with Labs 6 & 7):
//    START_PID   (10)  → Phase A: place car upright, balances immediately
//    START_RUN   (8)   → Phase B: full pop sequence (drive→brake→balance)
//    GET_RUN_DATA(9)   → stream logged data
//    STOP_ROBOT  (12)  → emergency stop
//    SET_GAINS   (11)  "11|Kp:4.0|Kd:0.08|Ki:0.0"     → PID gains
//                      "11|SP:0.0"                       → balance setpoint (deg)
//                      "11|S1:3|S2:15|S3:2"             → KF sigmas
//                      "11|PD:200|PB:300|HO:60"         → pop timing & handoff angle
//                      (all combinable in one string)
//
//  Log format (Python-compatible):
//    T:<ms>|D1:<raw_pitch_deg>|D2:<kf_pitch_deg>|MOT:<pwm>|ERR:<pid_err>|VEL:<kf_rate>
//
//  KF model  (pitch in degrees, upright = 0 deg):
//    theta_ddot = alpha1*theta - alpha2*u
//    alpha1 = 5.36  deg/s^2 per deg  (gravity, from sysID drop data)
//    alpha2 = 45.0  deg/s^2 per unit-u (motor torque, from sysID step data)
//    DT     = 0.0177s  (DMP ~56Hz)
//    Ad = [[1.0000, 0.0177], [0.0949, 1.0000]]
//    Bd = [[0.0],            [-0.7965]]
//    C  = [[1.0, 0.0]]
//    sigma1=3 deg, sigma2=15 deg/s, sigma3=2 deg
//
//  IMU Pitch Convention (current_pitch = roll - 90):
//    Flat on ground  →  current_pitch ≈ -90 deg
//    Upright/wheelie →  current_pitch ≈   0 deg
//    Balance setpoint = 0.0 deg (tunable via SP:)
// ============================================================

#include "ICM_20948.h"
#include <ArduinoBLE.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include "EString.h"
#include <Wire.h>

using namespace BLA;

// ========================
//    BLE UUIDs  (UNCHANGED from Labs 6/7)
// ========================
#define BLE_UUID_TEST_SERVICE "312f9813-7fbd-4dca-9b2f-17b3e7fc82f4"
#define BLE_UUID_RX_STRING    "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_STRING    "f235a225-6735-4d73-94cb-ee5dfce9ba83"
#define BLE_UUID_TX_FLOAT     "27616294-3063-4ecc-b60b-3470ddef2938"

#define AD0_VAL  1

// ========================
//    MOTOR PINS  (from motor_functions.h)
// ========================
#define L1 2
#define L2 3
#define R1 15
#define R2 14

// ========================
//    LOG
// ========================
#define LOG_SIZE 4000
unsigned long log_time [LOG_SIZE];
float         log_pitch[LOG_SIZE];   // raw DMP pitch (deg)
float         log_kfp  [LOG_SIZE];   // KF pitch estimate (deg)
int           log_pwm  [LOG_SIZE];   // motor PWM command (signed)
float         log_err  [LOG_SIZE];   // PID error (deg)
float         log_vel  [LOG_SIZE];   // KF pitch-rate estimate (deg/s)
int           log_idx  = 0;

// ========================
//    PID GAINS  (tunable via SET_GAINS)
//    Starting values from sysID:
//      Kp=4.0 → at 10 deg error → 40 PWM, scales to 255 at ~60 deg
//      Kd=0.08 → at 100 deg/s rate → 8 PWM  (KF rate is smooth, so Kd can be large)
//      Ki=0.0  → start without integral to avoid windup during tuning
// ========================
float Kp           =  4.0f;
float Ki           =  0.0f;
float Kd           =  0.08f;
float SETPOINT_DEG =  0.0f;   // 0 = upright in corrected frame; tune ±5 deg as needed
bool  windup_on    = true;
float WINDUP_LIMIT = 200.0f;

#define RUN_DURATION_MS   15000   // hard safety timeout (ms)
#define FALLEN_ANGLE      (-60.0f) // pitch below this = fallen, abort (e.g. -60 deg)
#define BALANCE_WINDOW    (35.0f)  // only run PID if |error| < this (deg from setpoint)

// ========================
//    POP SEQUENCE PARAMETERS  (tunable via SET_GAINS PD/PB/HO)
// ========================
int   POP_DRIVE_MS   = 200;    // ms to drive forward at full speed
int   POP_BRAKE_MS   = 300;    // ms to brake hard
int   POP_BACK_MS    = 300;    // ms to drive backward after brake
float HANDOFF_ANGLE  = 55.0f;  // pitch must exceed this to hand off to balance loop
                                // (pitch > HANDOFF_ANGLE means car is rising past 55 deg)
int   POP_DRIVE_PWM  = 255;    // forward PWM during pop drive phase
int   POP_BRAKE_PWM  = 255;    // reverse PWM during braking phase
int   POP_BACK_PWM   = 255;    // reverse PWM during back phase

// ========================
//    KF PARAMETERS
//    alpha1=5.36, alpha2=45.0, DT=0.0177s
//    Tunable at runtime via SET_GAINS S1/S2/S3
// ========================
float kf_sigma1 =  3.0f;   // pitch process noise (deg)
float kf_sigma2 = 15.0f;   // pitch-rate process noise (deg/s)
float kf_sigma3 =  2.0f;   // DMP measurement noise (deg)

// Fixed model params — from sysID (do not change at runtime)
const float KF_ALPHA2 = 5.36f;   // deg/s^2 per deg
const float KF_ALPHA1 = 45.0f;   // deg/s^2 per unit-u
const float KF_DT     = 0.0177f; // s

// KF matrices (computed once in build_kf_matrices())
Matrix<2,2> Ad;
Matrix<2,1> Bd;
Matrix<1,2> C_kf;
Matrix<2,2> Sigma_u;
Matrix<1,1> Sigma_z;

// KF state
Matrix<2,1> kf_mu;      // [pitch_deg, pitch_rate_deg_s]
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
    START_RUN         = 8,   // full pop sequence
    GET_RUN_DATA      = 9,
    START_PID         = 10,  // Phase A: hand-placement balance only
    SET_GAINS         = 11,
    STOP_ROBOT        = 12
};

// ========================
//    STATE MACHINE
// ========================
enum RobotState {
    STATE_IDLE,
    STATE_BALANCE,     // Phase A: hand-placed, balance only
    STATE_POP_DRIVE,   // Phase B step 1: drive forward
    STATE_POP_BRAKE,   // Phase B step 2: brake
    STATE_POP_BACK,   // Phase B step 3: drive back
    STATE_POP_WAIT,    // Phase B step 4: wait for pitch threshold
    STATE_BALANCING    // Phase B step 5: PID balance after pop
};
RobotState robot_state = STATE_IDLE;

unsigned long state_enter_time = 0;
unsigned long run_start_time   = 0;

// ========================
//    SENSOR STATE
// ========================
float         current_pitch = 0.0f;   // corrected pitch: flat=-90, upright=0
bool          imu_updated   = false;

// ========================
//    PID STATE
// ========================
float         integral       = 0.0f;
float         prev_error     = 0.0f;
float         prev_pitch     = 0.0f;
float         filtered_d     = 0.0f;
unsigned long prev_pid_time  = 0;

// ========================
//    BLE STREAMING
// ========================
bool is_sending_run  = false;
int  run_send_index  = 0;

// ========================
//    KF HELPERS
// ========================

void build_kf_matrices() {
    // Continuous model: theta_ddot = alpha1*theta - alpha2*u
    // A_c = [[0, 1], [alpha1, 0]]
    // B_c = [[0], [-alpha2]]
    // Euler discretisation: Ad = I + DT*A_c,  Bd = DT*B_c
    Ad = {1.0f,               KF_DT,
          KF_ALPHA1 * KF_DT,  1.0f  };

    Bd = {0.0f,
          -KF_ALPHA2 * KF_DT};

    // Measure pitch directly
    C_kf = {1.0f, 0.0f};

    Sigma_u = {kf_sigma1 * kf_sigma1, 0.0f,
               0.0f,                   kf_sigma2 * kf_sigma2};

    Sigma_z = {kf_sigma3 * kf_sigma3};
}

void kf_init(float pitch_deg) {
    kf_mu    = {pitch_deg, 0.0f};
    kf_sigma = {5.0f, 0.0f,
                0.0f, 5.0f};
    kf_ready = true;
}

// u_norm: signed normalised motor input, range [-1, 1]
// y_deg:  raw DMP pitch reading (deg)
// do_update: true when fresh DMP reading available
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
        Matrix<1,1> y_innov = y_mat - C_kf * mu_p;
        kf_mu = mu_p + kf_gain * y_innov;

        Matrix<2,2> I2 = {1.0f, 0.0f, 0.0f, 1.0f};
        kf_sigma = (I2 - kf_gain * C_kf) * sigma_p;
    } else {
        kf_mu    = mu_p;
        kf_sigma = sigma_p;
    }
}

float kf_pitch()    { return kf_mu(0); }   // estimated pitch (deg)
float kf_pitchrate(){ return kf_mu(1); }   // estimated pitch rate (deg/s)

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
    if (pid_out > 0) drive_forward(speed);
    else             drive_backward(speed);
}

// ========================
//    PID STEP  (shared by STATE_BALANCE and STATE_BALANCING)
//    Returns: signed PWM to apply (already constrained to ±255)
// ========================
float run_pid(float pitch_kf, float pitchrate_kf, unsigned long now) {

    float error = SETPOINT_DEG - pitch_kf;

    // Safety: if car has fallen, return 0 so caller can abort
    if (pitch_kf < FALLEN_ANGLE) return 0.0f;

    // If outside balance window don't wind up integral
    if (fabsf(error) > BALANCE_WINDOW) {
        integral = 0.0f;
    }

    float dt_s = (prev_pid_time == 0) ? 0.0f : (float)(now - prev_pid_time) / 1000.0f;

    // Proportional
    float p_term = Kp * error;

    // Integral  (only inside balance window)
    if (fabsf(error) <= BALANCE_WINDOW && dt_s > 0) {
        integral += error * dt_s;
        if (windup_on)
            integral = constrain(integral, -WINDUP_LIMIT, WINDUP_LIMIT);
    }
    float i_term = Ki * integral;

    // Derivative on measurement (use KF rate — already smooth, no extra filter needed)
    // D term opposes pitch rate regardless of setpoint direction
    float d_term = -Kd * pitchrate_kf;

    float pid_out = p_term + i_term + d_term;
    pid_out = constrain(pid_out, -255.0f, 255.0f);

    prev_error   = error;
    prev_pid_time = now;

    return pid_out;
}

// ========================
//    RESET PID STATE
// ========================
void reset_pid_state() {
    integral      = 0.0f;
    prev_error    = 0.0f;
    prev_pitch    = 0.0f;
    filtered_d    = 0.0f;
    prev_pid_time = 0;
}

// ========================
//    IMU
// ========================
bool init_dmp() {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) return false;
    Serial.println("IMU started correctly");
    bool success = true;
    success &= (myICM.initializeDMP()                                             == ICM_20948_Stat_Ok);
    delay(500);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR)   == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0)                        == ICM_20948_Stat_Ok);
    success &= (myICM.enableFIFO()                                                == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP()                                                 == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP()                                                  == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO()                                                 == ICM_20948_Stat_Ok);
    return success;
}

// Returns true if a fresh reading was obtained this call.
// Pitch convention: flat on ground ≈ -90 deg, upright (wheelie) ≈ 0 deg
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
                double t0  = +2.0 * (q0*q1 + q2*q3);
                double t1  = +1.0 - 2.0*(q1*q1 + q2*q2);
                double roll = atan2(t0, t1) * 180.0 / PI;
                current_pitch = (float)(roll - 90.0);   // flat=-90, upright=0
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
        Serial.println("DMP FAIL — halting");
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

    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    Serial.println("BLE active — Artemis BLE [Lab 12 Wheelie]");
    Serial.print("Kp="); Serial.print(Kp);
    Serial.print(" Ki="); Serial.print(Ki);
    Serial.print(" Kd="); Serial.print(Kd);
    Serial.print(" SP="); Serial.print(SETPOINT_DEG);
    Serial.print(" KF sigma="); Serial.print(kf_sigma1);
    Serial.print("/"); Serial.print(kf_sigma2);
    Serial.print("/"); Serial.println(kf_sigma3);
}

// ========================
//    MAIN LOOP
// ========================
void loop() {

    // ── 1. READ IMU ──────────────────────────────────────────
    imu_updated = read_dmp_pitch();

    // ── 2. BLE POLL / SAFETY ────────────────────────────────
    bool balancing = (robot_state == STATE_BALANCE   ||
                      robot_state == STATE_BALANCING ||
                      robot_state == STATE_POP_DRIVE ||
                      robot_state == STATE_POP_BRAKE ||
                      robot_state == STATE_POP_BACK  ||
                      robot_state == STATE_POP_WAIT);

    if (balancing) {
        unsigned long now_ble = millis();
        if (now_ble - last_ble_check >= BLE_CHECK_INTERVAL_MS) {
            last_ble_check = now_ble;
            BLEDevice central = BLE.central();
            if (!central || !central.connected()) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("BLE lost — stopped.");
            }
        }
    } else {
        BLE.poll();
    }

    // ── 3. STATE MACHINE ────────────────────────────────────
    unsigned long now = millis();

    // KF update runs for any active state (predict every loop, update when DMP fires)
    if (robot_state != STATE_IDLE && kf_ready) {
        float u_norm = (log_idx > 0) ? ((float)log_pwm[log_idx - 1] / 255.0f) : 0.0f;
        kf_step(u_norm, current_pitch, imu_updated);
    }

    switch (robot_state) {

        // ── IDLE ──────────────────────────────────────────────
        case STATE_IDLE:
            // nothing; waiting for BLE command
            break;

        // ── PHASE A: BALANCE (hand-placed) ──────────────────
        case STATE_BALANCE: {
            if (now - run_start_time >= RUN_DURATION_MS) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Balance timeout. Send cmd 9 for data.");
                break;
            }

            if (!kf_ready) {
                if (imu_updated) kf_init(current_pitch);
                break;
            }

            float pk = kf_pitch();
            float vk = kf_pitchrate();

            // Safety: if car has fallen abort
            if (pk < FALLEN_ANGLE) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Fallen — stopped.");
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
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 1: POP — DRIVE FORWARD ──────────────
        case STATE_POP_DRIVE: {
            drive_forward(POP_DRIVE_PWM);
            if (now - state_enter_time >= (unsigned long)POP_DRIVE_MS) {
                state_enter_time = now;
                robot_state = STATE_POP_BRAKE;
                Serial.println("Pop: braking...");
            }
            // Log during drive phase
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = kf_ready ? kf_pitch() : current_pitch;
                log_pwm  [log_idx] = POP_DRIVE_PWM;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 2: POP — BRAKE ──────────────────────
        case STATE_POP_BRAKE: {
            brake_motors();
            if (now - state_enter_time >= (unsigned long)POP_BRAKE_MS) {
                stop_motors();
                state_enter_time = now;
                robot_state = STATE_POP_BACK;
                Serial.println("Pop: breaking...");
            }
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = kf_ready ? kf_pitch() : current_pitch;
                log_pwm  [log_idx] = -POP_BRAKE_PWM;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_idx++;
            }
            break;
        }

        case STATE_POP_BACK: {
            drive_backward(POP_BACK_PWM);
            if (now - state_enter_time >= (unsigned long)POP_BACK_MS) {
                stop_motors();
                state_enter_time = now;
                robot_state = STATE_POP_WAIT;
                Serial.println("Pop: waiting for handoff angle...");
            }
            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = kf_ready ? kf_pitch() : current_pitch;
                log_pwm  [log_idx] = -POP_BACK_PWM;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 3: WAIT FOR PITCH THRESHOLD ─────────
        case STATE_POP_WAIT: {
            // Safety timeout — if car hasn't risen in 500ms, abort
            if (now - state_enter_time > 500) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Handoff timeout — car didn't rise. Abort.");
                break;
            }

            float pk = kf_ready ? kf_pitch() : current_pitch;

            // Hand off when pitch exceeds handoff angle
            // (pitch rises from ~-90 toward 0; HANDOFF_ANGLE e.g. -35 means ≈55 deg from flat)
            float handoff_threshold = SETPOINT_DEG - HANDOFF_ANGLE;  // e.g. 0 - 55 = -35
            if (pk >= handoff_threshold) {
                reset_pid_state();
                robot_state = STATE_BALANCING;
                Serial.print("Handoff! pitch="); Serial.println(pk);
            }

            if (log_idx < LOG_SIZE) {
                log_time [log_idx] = now;
                log_pitch[log_idx] = current_pitch;
                log_kfp  [log_idx] = pk;
                log_pwm  [log_idx] = 0;
                log_err  [log_idx] = 0;
                log_vel  [log_idx] = kf_ready ? kf_pitchrate() : 0;
                log_idx++;
            }
            break;
        }

        // ── PHASE B STEP 4: BALANCE AFTER POP ────────────────
        case STATE_BALANCING: {
            if (now - run_start_time >= RUN_DURATION_MS) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Balancing timeout. Send cmd 9 for data.");
                break;
            }

            if (!kf_ready) {
                if (imu_updated) kf_init(current_pitch);
                break;
            }

            float pk = kf_pitch();
            float vk = kf_pitchrate();

            if (pk < FALLEN_ANGLE) {
                stop_motors();
                robot_state = STATE_IDLE;
                Serial.println("Fallen after pop — stopped.");
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
                log_idx++;
            }
            break;
        }
    }

    // ── 4. STREAM DATA OVER BLE ───────────────────────────────
    static unsigned long last_send_time = 0;
    if (is_sending_run && (now - last_send_time > 10)) {
        if (run_send_index < log_idx) {
            tx_estring_value.clear();
            tx_estring_value.append("T:");   tx_estring_value.append((int)log_time [run_send_index]);
            tx_estring_value.append("|D1:"); tx_estring_value.append(log_pitch[run_send_index]);
            tx_estring_value.append("|D2:"); tx_estring_value.append(log_kfp  [run_send_index]);
            tx_estring_value.append("|MOT:");tx_estring_value.append(log_pwm  [run_send_index]);
            tx_estring_value.append("|ERR:");tx_estring_value.append(log_err  [run_send_index]);
            tx_estring_value.append("|VEL:");tx_estring_value.append(log_vel  [run_send_index]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            run_send_index++;
            last_send_time = now;
        } else {
            is_sending_run = false;
            Serial.println("Transfer complete.");
        }
    }
}

// ========================
//    COMMAND HANDLER
// ========================
void handle_command(BLEDevice central, BLECharacteristic characteristic) {
    String cmd_str = rx_characteristic_string.value();
    int cmd_type   = cmd_str.toInt();
    Serial.print("CMD: "); Serial.println(cmd_type);

    switch (cmd_type) {

        // ── Phase A: balance from hand-placed upright ────────
        case START_PID:
            log_idx        = 0;
            run_start_time = millis();
            state_enter_time = millis();
            last_ble_check = millis();
            reset_pid_state();
            kf_ready       = false;
            is_sending_run = false;
            robot_state    = STATE_BALANCE;
            Serial.println(">>> Phase A: BALANCE <<<");
            Serial.print("Kp="); Serial.print(Kp);
            Serial.print(" Ki="); Serial.print(Ki);
            Serial.print(" Kd="); Serial.print(Kd);
            Serial.print(" SP="); Serial.println(SETPOINT_DEG);
            break;

        // ── Phase B: full pop sequence ───────────────────────
       
       case START_RUN:
            log_idx          = 0;
            run_start_time   = millis();
            state_enter_time = millis();
            last_ble_check   = millis();
            reset_pid_state();
            
            // START KF IMMEDIATELY! 
            // This allows the math to track the car during the flip
            kf_init(current_pitch); 
            
            is_sending_run   = false;
            robot_state      = STATE_POP_DRIVE;
            Serial.println(">>> Phase B: POP SEQUENCE <<<");
            // ...
            break;

        // ── Emergency stop ───────────────────────────────────
        case STOP_ROBOT:
            stop_motors();
            robot_state = STATE_IDLE;
            Serial.println(">>> STOP <<<");
            break;

        // ── Stream logged data ───────────────────────────────
        case GET_RUN_DATA:
            stop_motors();
            robot_state = STATE_IDLE;
            if (log_idx > 0) {
                is_sending_run = true;
                run_send_index = 0;
                Serial.print("Sending "); Serial.print(log_idx); Serial.println(" pts...");
            } else {
                Serial.println("No data.");
            }
            break;

        // ── Tune gains and KF sigmas ─────────────────────────
        case SET_GAINS: {
            stop_motors();
            robot_state = STATE_IDLE;

            bool kf_changed = false;

            auto parse_f = [&](const char* key) -> float {
                int i = cmd_str.indexOf(key);
                if (i == -1) return NAN;
                int s = i + strlen(key);
                int e = cmd_str.indexOf('|', s);
                if (e == -1) e = cmd_str.length();
                return cmd_str.substring(s, e).toFloat();
            };
            auto parse_i = [&](const char* key) -> int {
                int i = cmd_str.indexOf(key);
                if (i == -1) return -999;
                int s = i + strlen(key);
                int e = cmd_str.indexOf('|', s);
                if (e == -1) e = cmd_str.length();
                return cmd_str.substring(s, e).toInt();
            };

            float v; int vi;
            v = parse_f("Kp:"); if (!isnan(v)) Kp = v;
            v = parse_f("Ki:"); if (!isnan(v)) { Ki = v; if (Ki > 0) WINDUP_LIMIT = 150.0f / Ki; }
            v = parse_f("Kd:"); if (!isnan(v)) Kd = v;
            v = parse_f("SP:"); if (!isnan(v)) SETPOINT_DEG = v;

            v = parse_f("S1:"); if (!isnan(v)) { kf_sigma1 = v; kf_changed = true; }
            v = parse_f("S2:"); if (!isnan(v)) { kf_sigma2 = v; kf_changed = true; }
            v = parse_f("S3:"); if (!isnan(v)) { kf_sigma3 = v; kf_changed = true; }
            if (kf_changed) build_kf_matrices();

            vi = parse_i("PD:"); if (vi != -999) POP_DRIVE_MS  = vi;
            vi = parse_i("PB:"); if (vi != -999) POP_BRAKE_MS  = vi;
            vi = parse_i("PBK:");  if (vi != -999) POP_BACK_MS   = vi;
            v  = parse_f("HO:"); if (!isnan(v))  HANDOFF_ANGLE = v;
            vi = parse_i("DPWM:"); if (vi != -999) POP_DRIVE_PWM = vi;
            vi = parse_i("BPWM:"); if (vi != -999) POP_BRAKE_PWM = vi;
            vi = parse_i("BKPWM:");if (vi != -999) POP_BACK_PWM  = vi;

            Serial.println("=== GAINS SET ===");
            Serial.print("Kp:"); Serial.print(Kp);
            Serial.print(" Ki:"); Serial.print(Ki);
            Serial.print(" Kd:"); Serial.print(Kd);
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