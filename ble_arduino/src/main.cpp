
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


//////////// Global Variables ////////////


void
setup()
{
    Serial.begin(115200);

    bleSetup();

    initIMU(myICM);

    pinMode(LED_BUILTIN, OUTPUT);
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
            
            updateIMU();

            BLE.poll();
            // Send data
            write_data();

            // Read data
            read_data();

            // Collect IMU data
            if(recording){
                collectAllData(time_data, temp_data, imu_data);
            }
            

        }

        DEBUG_PRINTLN("Disconnected");
    }
}
