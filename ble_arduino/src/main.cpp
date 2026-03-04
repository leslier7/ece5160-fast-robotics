
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

//////////// Global Variables ////////////
SFEVL53L1X distanceSensorFront;
SFEVL53L1X distanceSensorSide;

void
setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    //analogWriteFrequency(20000.0); //TODO play with this

    bleSetup();

    initIMU(myICM);

    bool front_setup = setupSensor(distanceSensorFront, true);
    bool side_setup = setupSensor(distanceSensorSide, false);

    if(!front_setup || !side_setup){
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_PRINTF("Failed to start both distance sensors");
        while(1); // TODO maybe change this and set some kind of bluetooth status checker
    } else {
        DEBUG_PRINTLN("Started both distance sensors");
    }

    
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
}



void
loop()
{   
    // Bluetooth processing
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        DEBUG_PRINT("Connected to: ");
        DEBUG_PRINTLN(central.address());

        // While central is connected
        while (central.connected()) {
            
            bool imu_updated = updateIMU();

            updateDistance(cur_dists, distanceSensorFront, distanceSensorSide);

            digitalWrite(LED_BUILTIN, imu_updated);

            serviceMotorJob();

            BLE.poll();
            // Send data
            write_data();

            // Read data
            read_data();

            // Collect IMU data
            if(recording){
                collectAllData(time_data, temp_data, imu_data, dist_data);
            }
            

        }

        DEBUG_PRINTLN("Disconnected");
    }
}
