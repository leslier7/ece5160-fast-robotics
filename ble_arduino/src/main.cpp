
#include <Arduino.h>
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include "debug.h"
#include "commands.h"
#include "ble_config.h"
#include "data_collection.h"
#include "imu_functions.h"
#include "distance_functions.h"
#include "motor_functions.h"
#include "pid.h"

//////////// Global Variables ////////////
SFEVL53L1X distanceSensorFront;
SFEVL53L1X distanceSensorSide;

bool light_value = false;
unsigned long prev_time;
unsigned long prev_rec_time;

PIDController pid_controller;
float pid_percent;
float commanded_percent;
MotorSpeeds pid_speeds;

bool bothSetup = false;

// Interupt function
void timerISR(void)
{
    // Clear the interrupt
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

    // code goes here
}

void
setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    //analogWriteFrequency(20000.0); //TODO play with this

    bleSetup();

    initIMU(myICM);

    initDMP(myICM);

    //TODO check if sensors already have unique IDs, then just begin
    //bool side_setup = setupSensor(distanceSensorSide, true);
    //bool front_setup = setupSensor(distanceSensorFront, false);

    bothSetup = setupBothSensors(distanceSensorFront, distanceSensorSide);

    //Only block startup with ToF
    #ifdef TOF
    if(!bothSetup){
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_PRINTF("Failed to start both distance sensors");
        while(1); // TODO maybe change this and set some kind of bluetooth status checker
    } else {
        DEBUG_PRINTLN("Started both distance sensors");
    }
    #endif

    #ifdef TOF
    //ToF PID
    initPID(pid_controller, 0.3, 0.03, 0.003, 0.8, readFrontDist, 1);
    #elif  defined(IMU)
    //ToF PID
    initPID(pid_controller, 0.3, 0.03, 0.003, 0.8, readYaw, 1);
    #endif

    stopBothMotors(); // Make sure to drive the pins to a known state to start
    
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    // // Interupt code if desired
    // // Configure timer 0A
    // am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
    //     AM_HAL_CTIMER_FN_REPEAT |
    //     AM_HAL_CTIMER_INT_ENABLE |
    //     AM_HAL_CTIMER_XT_32_768KHZ);

    // // Set compare value
    // am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 32768/1000, 0); // 1 kHz (just change the divisor to change the frequency)

    // // Enable interrupt in NVIC
    // NVIC_EnableIRQ(CTIMER_IRQn);

    // // Register interrupt handler
    // am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERA0, timerISR);

    // // Enable timer interrupt
    // am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    // // Start timer
    // am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    // Debugging struct sizes
    // DEBUG_PRINT("Size of attitude struct: ");
    // DEBUG_PRINTLN(sizeof(Attitude));
    // DEBUG_PRINT("Size of Comp filter struct: ");
    // DEBUG_PRINTLN(sizeof(CompFilter));

    //TODO make this bluetooth controlled
    //TODO setpoint is in mm from sensor, but I want it in cm
    //startPID(pid_controller, 300);
}


static unsigned long prev_debug_ms = 0;

void
loop()
{   
    //TOOD get rid of all of this
    //updateDistance(cur_dists, distanceSensorFront, distanceSensorSide);

    // Bluetooth processing
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        DEBUG_PRINT("Connected to: ");
        DEBUG_PRINTLN(central.address());

        // While central is connected
        while (central.connected()) {
            
            //bool imu_updated = updateIMU();

            if(bothSetup){
                updateDistance(cur_dists, distanceSensorFront, distanceSensorSide);
                DEBUG_PRINTF("\nYaw value: %f  Front Sensor value: %d  Front Sensor status: %d", yaw, cur_dists.front, cur_dists.front_status);
            }

            pred_dists = predictDistances(cur_dists, prev_dists);

            updateYaw(&yaw, myICM);

            // Handle PID        
            if(pid_controller.running){

                #ifdef TOF
                // ToF PID
                if(cur_dists.front_status == 4 && cur_dists.front < 2){ //target is very far away, run at constant speed
                    commanded_percent = 40.0f;
                    setBothMotors(commanded_percent, commanded_percent);
                } else if (cur_dists.front < 5000){ // got valid sensor data, use PID. But only if values arent too high
                    pid_percent = updatePID(pid_controller);
                    commanded_percent = pid_percent;
                    setBothMotors(pid_percent, pid_percent);
                } 
                #elif defined(IMU)
                // IMU PID
                pid_percent = updatePID(pid_controller);
                
                setBothMotors(pid_percent, -pid_percent); //Spin motors in oposite directions
                
                #endif
            }

            

            //digitalWrite(LED_BUILTIN, imu_updated);

            //Used for lab 7 to brake the car before it hits the wall
            //TODO check and make sure that this data is actually valid
            if(cur_dists.front > 600){
                serviceMotorJob();
            } else {
                brakeBothMotors();
            }
            
            
            // Collect IMU data
            if(recording){
                collectDriveData(time_data, yaw_data, dist_data, motor_data);

                // // Record on a timer
                // if ((millis() - prev_rec_time) >= 4) { //Record 7.5 seconds of data
                //     //collectAllData(time_data, temp_data, imu_data, dist_data, motor_data); //TODO figure out how to transmit the sample rate more effectivly for PID control and TOF
                    
                //     prev_rec_time = millis();
                // }
            }

            // #ifdef DEBUG_ENABLED
            // if (millis() - prev_debug_ms >= 100) {
            //     //DEBUG_PRINTF("Yaw value: %.2f  PID value: %.2f\n", yaw, pid_percent);

            //     int last = (yaw_data.index + DATA_ARR_SIZE - 1) % DATA_ARR_SIZE;
            //     DEBUG_PRINTF("live_yaw=%.2f recorded_yaw=%.2f\n", yaw, yaw_data.value[last]);
            //     prev_debug_ms = millis();
            // }
            // #endif

            
            
            // if(cur_dists.front_updated){
            //     prev_dists.front = cur_dists.front;
            // }
            // if(cur_dists.side_updated){
            //     prev_dists.side = cur_dists.side;
            // }

            // Heartbeat
            if ((millis() - prev_time) >= 500) {
                digitalWrite(LED_BUILTIN, light_value);
                light_value = !light_value;
                prev_time = millis();
            }
            

            BLE.poll();
            // Send data
            write_data();

            // Read data
            read_data();
        }

        DEBUG_PRINTLN("Disconnected");
        stopBothMotors();
    }
}
