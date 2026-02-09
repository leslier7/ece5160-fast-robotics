#include "commands.h"
#include "ble_config.h"
#include "data_collection.h"

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

            for(int i = 0; i < DATA_ARR_SIZE; i++){
                char value_str[20];
                snprintf(value_str, sizeof(value_str), "%lu", time_data.values[i]);
                
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

            for(int i = 0; i < DATA_ARR_SIZE; i++){
                char value_str[30];
                snprintf(value_str, sizeof(value_str), "%lu:%d", time_data.values[i], temp_data.values[i]);
                
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

        case GET_IMU_READINGS: {
            Serial.println("Sending IMU readings");

            EString temp_string = EString();
            int tx_result = -1;

            for(int i = 0; i < DATA_ARR_SIZE; i++){
                char value_str[30];
                snprintf(value_str, sizeof(value_str), "%lu:%.3f:%.3f:%.3f", time_data.values[i], imu_data.values[i].pitch, imu_data.values[i].roll, imu_data.values[i].yaw);
                
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

        case START_RECORDING: {
            clearData(time_data);
            clearData(temp_data);
            clearData(imu_data);
            recording = true;

            EString temp_string = EString();
            temp_string.clear();
            temp_string.append("Started recording");

        
            tx_estring_value.clear();
            tx_estring_value.append(temp_string.c_str());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            break;
        }

        case STOP_RECORDING: {
            recording = false;
            EString temp_string = EString();
            temp_string.clear();
            temp_string.append("Stopped recording");

        
            tx_estring_value.clear();
            tx_estring_value.append(temp_string.c_str());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

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