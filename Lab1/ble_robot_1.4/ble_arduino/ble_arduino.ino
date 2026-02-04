
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "4843fd62-068c-4a37-af34-e724c6a05681"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

#define TIME_ARR_SIZE 30
unsigned long time_values[TIME_ARR_SIZE];
unsigned long time_index = 0;

int temp_values[TIME_ARR_SIZE];
int temp_index = 0;

//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    DATA_RATE,
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
    char char_arr[MAX_MSG_SIZE];

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            
            float float_a, float_b, float_c;

            // Extract first float from command string
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract second float from command string
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;

            // Extract third float from command string
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);


            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO: {

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            Serial.print("ECHO: ");
            Serial.println(char_arr);

            EString temp_string = EString();
            temp_string.clear();
            temp_string.append("Robot recieved: ");
            temp_string.append(char_arr);

        
            tx_estring_value.clear();
            tx_estring_value.append(temp_string.c_str());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            
            break;
        }
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        case GET_TIME_MILLIS:

            snprintf(char_arr, MAX_MSG_SIZE, "T:%lu", millis());

            tx_estring_value.clear();
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;
        case SEND_TIME_DATA: {

            Serial.println("Sending time data!");

            EString temp_string = EString();
            int tx_result = -1;

            for(int i = 0; i < TIME_ARR_SIZE; i++){
                char value_str[20];
                snprintf(value_str, sizeof(value_str), "%lu", time_values[i]);
                
                // Check if adding this value would exceed MAX_MSG_SIZE
                // Account for comma separator and null terminator
                int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
                
                if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
                    // Send current packet before it overflows
                    tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
                    Serial.print("Sent packet: ");
                    Serial.println(temp_string.c_str());
                    
                    // Small delay to allow BLE stack to process
                    delay(10);
                    
                    // Reset for next packet
                    temp_string.clear();
                }
                
                // Add comma if not first item in current packet
                if (temp_string.get_length() > 0) {
                    temp_string.append(",");
                }
                temp_string.append(value_str);
            }

            // Send any remaining data
            if (temp_string.get_length() > 0) {
                tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
                Serial.print("Sent packet: ");
                Serial.println(temp_string.c_str());
                delay(10);
            }

            // Send end marker
            tx_result = tx_characteristic_string.writeValue("end");
            Serial.print("Serial Transmission Result: ");
            Serial.println(tx_result);
            Serial.println("Finished sending array");

            break;
        }

        case GET_TEMP_READINGS: {
            Serial.println("Sending temp readings");

            EString temp_string = EString();
            int tx_result = -1;

            for(int i = 0; i < TIME_ARR_SIZE; i++){
                char value_str[30];
                snprintf(value_str, sizeof(value_str), "%lu:%d", time_values[i], temp_values[i]);
                
                // Check if adding this value would exceed MAX_MSG_SIZE
                // Account for comma separator and null terminator
                int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
                
                if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
                    // Send current packet before it overflows
                    tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
                    Serial.print("Sent packet: ");
                    Serial.println(temp_string.c_str());
                    
                    // Small delay to allow BLE stack to process
                    delay(10);
                    
                    // Reset for next packet
                    temp_string.clear();
                }
                
                // Add comma if not first item in current packet
                if (temp_string.get_length() > 0) {
                    temp_string.append(",");
                }
                temp_string.append(value_str);
            }

            // Send any remaining data
            if (temp_string.get_length() > 0) {
                tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
                Serial.print("Sent packet: ");
                Serial.println(temp_string.c_str());
                delay(10);
            }

            // Send end marker
            tx_result = tx_characteristic_string.writeValue("end");
            Serial.print("Serial Transmission Result: ");
            Serial.println(tx_result);
            Serial.println("Finished sending array");

            break;
        }

        case DATA_RATE: {
            int byte_size;

            // Extract first float from command string
            success = robot_cmd.get_next_value(byte_size);
            if (!success)
                return;

            // Clamp to valid range
            if (byte_size >= MAX_MSG_SIZE) {
                byte_size = MAX_MSG_SIZE - 1;
            }

            // Fast string construction using memset
            char reply[MAX_MSG_SIZE];
            memset(reply, 'X', byte_size);
            reply[byte_size] = '\0';  // Null terminate

            // Send the message
            tx_characteristic_string.writeValue(reply);

            // Force immediate transmission
            BLE.poll();

            Serial.print("Sent ");
            Serial.print(byte_size);
            Serial.println(" byte reply");

            break;
        }

        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void send_time(){
    char char_arr[MAX_MSG_SIZE];

    snprintf(char_arr, MAX_MSG_SIZE, "T:%lu", millis());

    tx_estring_value.clear();
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
}

void collect_time(){

    if(time_index < TIME_ARR_SIZE) { 
        time_values[time_index] = millis();
        //Serial.print("Collected time at t: ");
        //Serial.println(time_values[time_index]);
        time_index++;
    } else { // Overflows start overwriting old data
        time_index = 0;
        time_values[time_index] = millis();
        //Serial.println("Time values overflowed");
    }

}

void collect_temps(){
    collect_time(); //Collect time so it corresponds to the temp
    if(temp_index < TIME_ARR_SIZE) { 
        temp_values[temp_index] = getTempDegC();
        //Serial.print("Collected time at t: ");
        //Serial.println(time_values[time_index]);
        temp_index++;
    } else { // Overflows start overwriting old data
        temp_index = 0;
        temp_values[temp_index] = getTempDegC();
        //Serial.println("Time values overflowed");
    }
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            BLE.poll();
            // Send data
            write_data();

            // Read data
            read_data();

            // Send current time
            //send_time();

            // Collect temps with times
            collect_temps();

        }

        Serial.println("Disconnected");
    }
}
