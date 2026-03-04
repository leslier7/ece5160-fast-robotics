#include "commands.h"
#include "ble_config.h"
#include "data_collection.h"
#include "debug.h"
#include "motor_functions.h"

static bool handle_ping();
static bool handle_send_two_ints();
static bool handle_send_three_floats();
static bool handle_echo();
static bool handle_dance();
static bool handle_set_vel();
static bool handle_get_time_millis();
static bool handle_send_time_data();
static bool handle_get_temp_readings();
static bool handle_data_rate();
static bool handle_get_imu_readings();
static bool handle_start_recording();
static bool handle_stop_recording();
static bool handle_get_dist_readings();
static bool handle_get_all_readings();
static bool handle_set_motor_job();
static bool handle_set_motor_sequence();

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

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
            handle_ping();
            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            handle_send_two_ints();
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            handle_send_three_floats();
            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:
            handle_echo();
            break;
        /*
         * DANCE
         */
        case DANCE:
            handle_dance();
            break;
        /*
         * SET_VEL
         */
        case SET_VEL:
            handle_set_vel();
            break;
        case GET_TIME_MILLIS:
            handle_get_time_millis();
            break;
        case SEND_TIME_DATA:
            handle_send_time_data();
            break;
        case GET_TEMP_READINGS:
            handle_get_temp_readings();
            break;
        case DATA_RATE:
            handle_data_rate();
            break;
        case GET_IMU_READINGS:
            handle_get_imu_readings();
            break;
        case START_RECORDING:
            handle_start_recording();
            break;
        case STOP_RECORDING:
            handle_stop_recording();
            break;
        case GET_DIST_READINGS:
            handle_get_dist_readings();
            break;
        case GET_ALL_READINGS:
            handle_get_all_readings();
            break;
        case SET_MOTOR_JOB:
            handle_set_motor_job();
            break;
        case SET_MOTOR_SEQUENCE:
            handle_set_motor_sequence();
            break;
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            DEBUG_PRINT("Invalid Command Type: ");
            DEBUG_PRINTLN(cmd_type);
            break;
    }
}

static bool handle_ping() {
    tx_estring_value.clear();
    tx_estring_value.append("PONG");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    DEBUG_PRINT("Sent back: ");
    DEBUG_PRINTLN(tx_estring_value.c_str());

    return true;
}

static bool handle_send_two_ints() {
    int int_a, int_b;
    bool success;

    // Extract the next value from the command string as an integer
    success = robot_cmd.get_next_value(int_a);
    if (!success)
        return false;

    // Extract the next value from the command string as an integer
    success = robot_cmd.get_next_value(int_b);
    if (!success)
        return false;

    DEBUG_PRINT("Two Integers: ");
    DEBUG_PRINT(int_a);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(int_b);
    
    return true;
}

static bool handle_send_three_floats() {
    float float_a, float_b, float_c;
    bool success;

    // Extract first float from command string
    success = robot_cmd.get_next_value(float_a);
    if (!success)
        return false;

    // Extract second float from command string
    success = robot_cmd.get_next_value(float_b);
    if (!success)
        return false;

    // Extract third float from command string
    success = robot_cmd.get_next_value(float_c);
    if (!success)
        return false;

    DEBUG_PRINT("Three floats: ");
    DEBUG_PRINT(float_a);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(float_b);
    DEBUG_PRINT(", ");
    DEBUG_PRINTLN(float_c);

    return true;
}

static bool handle_echo() {
    char char_arr[MAX_MSG_SIZE];
    bool success;

    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);
    if (!success)
        return false;

    DEBUG_PRINT("ECHO: ");
    DEBUG_PRINTLN(char_arr);

    EString temp_string = EString();
    temp_string.clear();
    temp_string.append("Robot received: ");
    temp_string.append(char_arr);

    tx_estring_value.clear();
    tx_estring_value.append(temp_string.c_str());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    
    return true;
}

static bool handle_dance() {
    DEBUG_PRINTLN("Look Ma, I'm Dancin'!");

    return true;
}

static bool handle_set_vel() {

    return true;
}

static bool handle_get_time_millis() {
    char char_arr[MAX_MSG_SIZE];

    snprintf(char_arr, MAX_MSG_SIZE, "T:%lu", millis());

    tx_estring_value.clear();
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    return true;
}

static bool handle_send_time_data() {
    DEBUG_PRINTLN("Sending time data!");

    EString temp_string = EString();
    int tx_result = -1;

    recording = false; // Pause recording while transmitting
    int start = time_data.index;
    for(int k = 0; k < DATA_ARR_SIZE; k++){
        int i = (start + k) % DATA_ARR_SIZE; // chronological order
        
        char value_str[20];
        snprintf(value_str, sizeof(value_str), "%lu", time_data.values[i]);
        
        // Check if adding this value would exceed MAX_MSG_SIZE
        // Account for comma separator and null terminator
        int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
        
        if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
            // Send current packet before it overflows
            tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
            DEBUG_PRINT("Sent packet: ");
            DEBUG_PRINTLN(temp_string.c_str());
            
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
        DEBUG_PRINT("Sent packet: ");
        DEBUG_PRINTLN(temp_string.c_str());
        delay(10);
    }

    // Send end marker
    tx_result = tx_characteristic_string.writeValue("end");
    DEBUG_PRINT("Serial Transmission Result: ");
    DEBUG_PRINTLN(tx_result);
    DEBUG_PRINTLN("Finished sending array");

    recording = true;

    return true;
}

static bool handle_get_temp_readings() {
    DEBUG_PRINTLN("Sending temp readings");

    EString temp_string = EString();
    int tx_result = -1;

    recording = false; // Pause recording while transmitting
    int start = temp_data.index;
    for(int k = 0; k < DATA_ARR_SIZE; k++){
        int i = (start + k) % DATA_ARR_SIZE; // chronological order


        char value_str[30];
        snprintf(value_str, sizeof(value_str), "%lu:%d", time_data.values[i], temp_data.values[i]);
        
        // Check if adding this value would exceed MAX_MSG_SIZE
        // Account for comma separator and null terminator
        int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
        
        if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
            // Send current packet before it overflows
            tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
            DEBUG_PRINT("Sent packet: ");
            DEBUG_PRINTLN(temp_string.c_str());
            
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
        DEBUG_PRINT("Sent packet: ");
        DEBUG_PRINTLN(temp_string.c_str());
        delay(10);
    }

    // Send end marker
    tx_result = tx_characteristic_string.writeValue("end");
    DEBUG_PRINT("Serial Transmission Result: ");
    DEBUG_PRINTLN(tx_result);
    DEBUG_PRINTLN("Finished sending array");

    recording = true;

    return true;
}

static bool handle_data_rate() {
    int byte_size;
    bool success;

    // Extract first float from command string
    success = robot_cmd.get_next_value(byte_size);
    if (!success)
        return false;

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

    DEBUG_PRINT("Sent ");
    DEBUG_PRINT(byte_size);
    DEBUG_PRINTLN(" byte reply");

    return true;
}

static bool handle_get_imu_readings() {
    DEBUG_PRINTLN("Sending IMU readings");

    EString temp_string = EString();
    int tx_result = -1;


    recording = false; // Pause recording while transmitting
    int start = imu_data.index;
    for(int k = 0; k < DATA_ARR_SIZE; k++){
        int i = (start + k) % DATA_ARR_SIZE; // chronological order

        char value_str[30];
        snprintf(value_str, sizeof(value_str), "%lu:%.3f:%.3f:%.3f", time_data.values[i], imu_data.values[i].pitch, imu_data.values[i].roll, imu_data.values[i].yaw);
        
        // Check if adding this value would exceed MAX_MSG_SIZE
        // Account for comma separator and null terminator
        int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
        
        if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
            // Send current packet before it overflows
            tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
            DEBUG_PRINT("Sent packet: ");
            DEBUG_PRINTLN(temp_string.c_str());
            
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
        DEBUG_PRINT("Sent packet: ");
        DEBUG_PRINTLN(temp_string.c_str());
        delay(10);
    }

    // Send end marker
    tx_result = tx_characteristic_string.writeValue("end");
    DEBUG_PRINT("Serial Transmission Result: ");
    DEBUG_PRINTLN(tx_result);
    DEBUG_PRINTLN("Finished sending array");

    recording = true; // Resume recording after transmit
    return true;
}

static bool handle_start_recording() {
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

    return true;
}

static bool handle_stop_recording() {
    recording = false;
    EString temp_string = EString();
    temp_string.clear();
    temp_string.append("Stopped recording");

    tx_estring_value.clear();
    tx_estring_value.append(temp_string.c_str());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    return true;
}

static bool handle_get_dist_readings() {
    DEBUG_PRINTLN("Sending all readings");

    EString temp_string = EString();
    int tx_result = -1;


    recording = false; // Pause recording while transmitting
    int start = dist_data.index;
    for(int k = 0; k < DATA_ARR_SIZE; k++){
        int i = (start + k) % DATA_ARR_SIZE; // chronological order

        char value_str[30];
        snprintf(value_str, sizeof(value_str), "%lu:%d:%d", time_data.values[i], dist_data.values[i].front, dist_data.values[i].side);
        
        // Check if adding this value would exceed MAX_MSG_SIZE
        // Account for comma separator and null terminator
        int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
        
        if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
            // Send current packet before it overflows
            tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
            DEBUG_PRINT("Sent packet: ");
            DEBUG_PRINTLN(temp_string.c_str());
            
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
        DEBUG_PRINT("Sent packet: ");
        DEBUG_PRINTLN(temp_string.c_str());
        delay(10);
    }

    // Send end marker
    tx_result = tx_characteristic_string.writeValue("end");
    DEBUG_PRINT("Serial Transmission Result: ");
    DEBUG_PRINTLN(tx_result);
    DEBUG_PRINTLN("Finished sending array");

    recording = true; // Resume recording after transmit
    return true;
}

static bool handle_get_all_readings() {
    DEBUG_PRINTLN("Sending all readings");

    EString temp_string = EString();
    int tx_result = -1;


    recording = false; // Pause recording while transmitting
    int start = dist_data.index;
    for(int k = 0; k < DATA_ARR_SIZE; k++){
        int i = (start + k) % DATA_ARR_SIZE; // chronological order

        char value_str[64];
        snprintf(value_str, sizeof(value_str), "%lu:%.3f:%.3f:%.3f:%d:%d", time_data.values[i], imu_data.values[i].pitch, imu_data.values[i].roll, imu_data.values[i].yaw, dist_data.values[i].front, dist_data.values[i].side);

        // Check if adding this value would exceed MAX_MSG_SIZE
        // Account for comma separator and null terminator
        int needed_len = strlen(value_str) + (temp_string.get_length() > 0 ? 1 : 0);
        
        if (temp_string.get_length() + needed_len >= MAX_MSG_SIZE - 1) {
            // Send current packet before it overflows
            tx_result = tx_characteristic_string.writeValue(temp_string.c_str());
            DEBUG_PRINT("Sent packet: ");
            DEBUG_PRINTLN(temp_string.c_str());
            
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
        DEBUG_PRINT("Sent packet: ");
        DEBUG_PRINTLN(temp_string.c_str());
        delay(10);
    }

    // Send end marker
    tx_result = tx_characteristic_string.writeValue("end");
    DEBUG_PRINT("Serial Transmission Result: ");
    DEBUG_PRINTLN(tx_result);
    DEBUG_PRINTLN("Finished sending array");

    recording = true; // Resume recording after transmit
    return true;
}

static bool handle_set_motor_job() {
    float right_percent, left_percent;
    int duration_ms_i;
    bool success;
    char char_arr[MAX_MSG_SIZE];

    // Extract first float from command string
    success = robot_cmd.get_next_value(left_percent);
    if (!success)
        return false;

    // Extract second float from command string
    success = robot_cmd.get_next_value(right_percent);
    if (!success)
        return false;

    // Extract third float from command string
    success = robot_cmd.get_next_value(duration_ms_i);
    if (!success)
        return false;

    if (duration_ms_i < 0) duration_ms_i = 0; // bounds checking just in case
    uint32_t duration_ms = (uint32_t)duration_ms_i;

    DEBUG_PRINTF("Left percent: %f   Right percent: %f   duration (ms): %d\n", left_percent, right_percent, duration_ms);

    snprintf(char_arr, MAX_MSG_SIZE, "l:%f,r:%f,d:%d", left_percent, right_percent, duration_ms);

    EString temp_string = EString();
    temp_string.clear();
    temp_string.append("Robot received: ");
    temp_string.append(char_arr);

    tx_estring_value.clear();
    tx_estring_value.append(temp_string.c_str());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    startMotorJob(right_percent, left_percent, duration_ms); // Start after the bluetooth return msg so that doesnt interfere
    startMotorQueue();

    return true;
}

static bool handle_set_motor_sequence() {
    float right_percent, left_percent;
    int duration_ms_i;
    bool success;
    int jobs_queued = 0;

    // Keep reading triplets until we run out of tokens
    while (true) {
        success = robot_cmd.get_next_value(left_percent);
        if (!success) break;

        success = robot_cmd.get_next_value(right_percent);
        if (!success) break;

        success = robot_cmd.get_next_value(duration_ms_i);
        if (!success) break;

        if (duration_ms_i < 0) duration_ms_i = 0;
        uint32_t duration_ms = (uint32_t)duration_ms_i;

        DEBUG_PRINTF("Left percent: %f   Right percent: %f   duration (ms): %d\n", left_percent, right_percent, duration_ms);

        if (!queueMotorJob(right_percent, left_percent, duration_ms)) {
            DEBUG_PRINTLN("Motor queue full!");
            break;
        }
        jobs_queued++;
    }

    // Send back how many jobs were queued
    char char_arr[MAX_MSG_SIZE];
    snprintf(char_arr, MAX_MSG_SIZE, "Queued %d motor jobs", jobs_queued);
    tx_estring_value.clear();
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    startMotorQueue();

    return jobs_queued > 0;
}