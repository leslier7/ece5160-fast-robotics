
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

PIDController pid_controller;

void
setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    //analogWriteFrequency(20000.0); //TODO play with this

    bleSetup();

    initIMU(myICM);


    //TODO check if sensors already have unique IDs, then just begin
    // bool side_setup = setupSensor(distanceSensorSide, true);
    // bool front_setup = setupSensor(distanceSensorFront, false);

    bool bothSetup = setupBothSensors(distanceSensorFront, distanceSensorSide);

    if(!bothSetup){
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_PRINTF("Failed to start both distance sensors");
        while(1); // TODO maybe change this and set some kind of bluetooth status checker
    } else {
        DEBUG_PRINTLN("Started both distance sensors");
    }

    //TODO make it easier to modify the PID values
    initPID(pid_controller, 0.3, 0.03, 0.003, readFrontDist, 1);
    
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    // Debugging struct sizes
    // DEBUG_PRINT("Size of attitude struct: ");
    // DEBUG_PRINTLN(sizeof(Attitude));
    // DEBUG_PRINT("Size of Comp filter struct: ");
    // DEBUG_PRINTLN(sizeof(CompFilter));

    //TODO make this bluetooth controlled
    //TODO setpoint is in mm from sensor, but I want it in cm
    startPID(pid_controller, 300);
}



void
loop()
{   
    //TOOD get rid of all of this
    updateDistance(cur_dists, distanceSensorFront, distanceSensorSide);

    //running PID as an experiment
    float pid_percent = updatePID(pid_controller);

    
    DEBUG_PRINTF("Front Sensor value: %d    PID value: %.2f\n", cur_dists.front , pid_percent);

    setBothMotors(pid_percent, pid_percent);

    // Bluetooth processing
    // Listen for connections
    // BLEDevice central = BLE.central();

    // // If a central is connected to the peripheral
    // if (central) {
    //     DEBUG_PRINT("Connected to: ");
    //     DEBUG_PRINTLN(central.address());

    //     // While central is connected
    //     while (central.connected()) {
            
    //         bool imu_updated = updateIMU();

    //         updateDistance(cur_dists, distanceSensorFront, distanceSensorSide);

    //         //digitalWrite(LED_BUILTIN, imu_updated);

    //         serviceMotorJob();

    //         BLE.poll();
    //         // Send data
    //         write_data();

    //         // Read data
    //         read_data();

    //         // Collect IMU data
    //         if(recording){
    //             collectAllData(time_data, temp_data, imu_data, dist_data, motor_data);
    //         }
            
    //         // Heartbeat
    //         if ((millis() - prev_time) >= 2000) {
    //             digitalWrite(LED_BUILTIN, light_value);
    //             light_value = !light_value;
    //             prev_time = millis();
    //         }
            
    //     }

    //     DEBUG_PRINTLN("Disconnected");
    //     stopBothMotors();
    // }
}
