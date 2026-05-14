#include <stdint.h>
#include <stdio.h>
#include <math.h>      // for fminf, fmaxf
#include <string.h>    // for strlen
#include <inttypes.h> //for formatting
#include <driver/uart.h>
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include "motor_values.h"
#include "uart_slcan.h"


static QueueHandle_t uart_queue = NULL;
static slcan_frame_list_t slcan_streams;  // static = invisible outside this file
const char *TAG_UART = "Testudog_UART"; // FOR LOGGING
//Holds first characters that identifies each command
const char COMMANDS_SLCAN[SUPPORTED_COMMANDS] = {'t','O','C'};
//Control access and flow of the process
bool state_slcan_channel = true;

/**
 * @brief Pack motor state data into SLCAN hexadecimal format
 *
 * Encodes motor position, velocity, current, temperature, and status into
 * a hexadecimal string suitable for transmission via UART in SLCAN protocol.
 * All values are saturated to their valid ranges and converted to fixed-width
 * integer representations:
 * - Position: 16 bits
 * - Velocity: 12 bits
 * - Torque: 12 bits
 * - Temperature: 16 bits
 * - Motor status: 8 bits
 *
 * @param msg Pointer to character buffer where hex string is written
 * @param msg_size Size of the msg buffer
 * @param pos Position in radians [-12.5, 12.5]
 * @param vel Velocity in rad/s [-76.0, 76.0]
 * @param curr Current in A [-60.0, 60.0]
 * @param temp_c Temperature in Celsius [-40, 215]
 * @param mot_st Motor status/error code (8-bit value)
 *
 * @note All input values are automatically clamped to their valid ranges
 * @see unpack_reply() for the corresponding decode function
 */
static void pack_motor_state_to_slcan(char * msg, size_t msg_size, float pos, float vel,float curr, float temp_c, uint8_t mot_st) {

    //Linear Mapping (Converting physical units to raw integers)
    pos = fminf(fmaxf(P_MIN, pos), P_MAX);
    vel = fminf(fmaxf(V_MIN, vel), V_MAX);
    curr = fminf(fmaxf(I_MIN, curr),I_MAX);
    temp_c = fminf(fmaxf(C_MIN, temp_c), C_MAX);

    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int curr_int = float_to_uint(curr, I_MIN, I_MAX, 12);
    int temp_c_int = float_to_uint(temp_c, C_MIN, C_MAX, 16);

    // 3. Masking ensures not bits exist outside the target range
    const uint16_t p_int16 = p_int & 0xFFFF;
    const uint16_t v_int12 = v_int & 0xFFF;
    const uint16_t curr_int12 = curr_int & 0xFFF;
    const uint16_t temp_c_int16 = temp_c_int & 0xFFFF;

    //format a hex string that represents 8 bytes of data that's the max of standard SLCAN
    snprintf(msg, msg_size, "%04x%03x%03x%04x%02x", p_int16, v_int12, curr_int12, temp_c_int16, mot_st);
}

/**
 * @brief Parse an SLCAN standard frame string into a structured format
 *
 * Decodes a standard SLCAN frame string (format: 'tIIIDDDDDDDD...') into
 * components: CAN ID, data length code, and data bytes. SLCAN format:
 * - 't' prefix (standard frame)
 * - 3 hex digits for CAN ID
 * - 1 digit for DLC (0-8)
 * - 2 hex digits per data byte (up to 8 bytes)
 *
 * @param input Pointer to SLCAN frame string (not null-terminated required,
 *              but must be at least 21 characters for valid 8-byte frame)
 * @param frame_can Pointer to slcan_frame_t structure to store parsed results
 *
 * @return true if frame was successfully parsed, false on validation errors
 *         (invalid format, wrong length, DLC > LENGTH_SLCAN_DATA)
 *
 * @note Only parses standard frames (starting with 't'), not extended frames or special commands
 * @see decode_slcan() for processing multiple frames from a buffer
 */
static bool parse_slcan( const char* input, slcan_frame_t *frame_can){

    // 1. Basic validation, considering  standard frames only
    if (input == NULL || input[0] != 't') {
        return false;
    }
    // 2. Length check: A standard 't' frame is:
    // 't' + 3(ID) + 1(DLC) + (2 * DLC) data bytes
    // Length is (DLC 8): 21 chars.
    size_t input_len = strlen(input);
    if (input_len < EXPECTED_SIZE_SLCAN_STD){
        ESP_LOGI(TAG_UART,"Total Frame has less than %u characters for processing", EXPECTED_SIZE_SLCAN_STD);
        return false;
    } 
    //id
    // 3. Parse ID (3 hex digits)
    char can_id[4];
    memcpy(can_id, &input[1], 3);
    can_id[3] = '\0';
    frame_can->id = (uint32_t) strtol(can_id, NULL, 16);
    // 4. Parse DLC (1 digit)
    frame_can->dlc = input[4] - '0';// '0' is 48 as uint8_t, refer to ASCII
    if (frame_can->dlc > LENGTH_SLCAN_DATA){
        ESP_LOGI(TAG_UART, "Data has more than %u bytes ", LENGTH_SLCAN_DATA);
        return false;
    } 
    // 5. Parse Data bytes
    for (uint8_t idx = 0; idx < frame_can->dlc; idx++) {
        char byte_hex[3];
        byte_hex[0] = input[5 + (idx * 2)];
        byte_hex[1] = input[6 + (idx * 2)];
        byte_hex[2] = '\0';
        frame_can->data[idx] = (uint8_t)strtol(byte_hex, NULL, 16);
    }
    return true;
}

/**
 * @brief Decode multiple SLCAN frames from a UART buffer
 *
 * Processes a UART receive buffer containing one or more SLCAN frames separated
 * by carriage returns ('\r'). Extracts each frame, validates it using parse_slcan(),
 * and stores valid frames in the output list. Automatically skips malformed frames
 * and handles noise before the 't' prefix.
 *
 * @param uart_buffer Pointer to raw UART receive buffer (may contain noise/extra data)
 * @param length_buffer_uart Length of uart_buffer in bytes
 * @param out_list Pointer to slcan_frame_list_t to store decoded frames.
 *                 count is reset to 0 at the start of this function.
 *
 * @note Modifies uart_buffer in-place by inserting null terminators
 * @note out_list->count will not exceed MAX_FRAMES_PER_BUFFER; excess frames are silently dropped
 * @see parse_slcan() to parse individual frames
 * @see receive_slcan() for the public interface
 */
static void decode_slcan(uint8_t *uart_buffer, int length_buffer_uart, slcan_frame_list_t *out_list) {
    out_list->count = 0;
    char *ptr = (char *)uart_buffer;
    char *frame_end;
    // Use strchr to find each terminator in the buffer
    //? Use instead strcmp to accept Commands defined by multiple characters?
    while ((frame_end = strchr(ptr, '\r')) != NULL) {
        *frame_end = '\0'; // Terminate the individual SLCAN string

        // Find the actual start of the command (skip any \n or noise)
        char *frame_start = strchr(ptr, 't');

        if (frame_start != NULL) {
            slcan_frame_t frame_process;
            if (parse_slcan(frame_start, &frame_process)) {
                // Safety check: don't overflow our "vector"
                if (out_list->count < MAX_FRAMES_PER_BUFFER) {
                    out_list->frames[out_list->count++] = frame_process;
                } else {
                    ESP_LOGI(TAG_UART, "Buffer full, skipping frame");
                }
            }
        }
        ptr = frame_end + 1; // Move to the next potential frame
    }
}

/**
 * @brief Handle special SLCAN control commands
 *
 * Processes special commands that control the SLCAN channel state or query its status:
 * - Command 1: Open the SLCAN channel (enable motor CAN message transmission)
 * - Command 2: Close the SLCAN channel (disable motor CAN message transmission)
 * - Command 0 or other: Refers to standard command slcan frame with prefix 't'
 *
 * @param special_cmd Command code: 1 (open), 2 (close), or other
 *
 * @note Affects global state_slcan_channel variable
 * @see search_and_set_special_command() to extract and process commands from UART buffer
 */
void set_special_command(uint8_t special_cmd){
    switch(special_cmd){
        case 2:
            state_slcan_channel = false;
            ESP_LOGI(TAG_UART, "Closed SLCAN Channel");
            break;
        case 1:
            state_slcan_channel = true;
            ESP_LOGI(TAG_UART, "Opened SLCAN Channel");
            break;
        default:
            char* channel_state_action = state_slcan_channel ? "Open" : "Close";
            ESP_LOGI(TAG_UART, "SLCAN CHANNEL STATUS: %s", channel_state_action);
            break;
    }
}

/**
 * @brief Search for and execute special SLCAN commands in a UART buffer
 *
 * Scans a UART receive buffer for special command characters ('t', 'O', 'C')
 * as defined in COMMANDS_SLCAN and executes the corresponding set_special_command().
 * Currently supports:
 * - 't': Standard frame (not a special command, skipped)
 * - 'O': Open SLCAN channel (command index 1)
 * - 'C': Close SLCAN channel (command index 2)
 *
 * @param uart_buffer Pointer to UART receive buffer containing potential special commands
 *
 * @note Processes commands separated by carriage returns ('\r')
 * @note First character of each line is checked against COMMANDS_SLCAN array
 * @note Modifies global state_slcan_channel variable based on commands
 * @see set_special_command() for the command execution logic
 */
void search_and_set_special_command(uint8_t *uart_buffer){
    char *ptr = (char *)uart_buffer;
    char *frame_end;
    // Use strchr to find each terminator in the buffer
    while ((frame_end = strchr(ptr, '\r')) != NULL) {
        // check the FIRST char of the frame
        char check_command = *ptr;

        for (int idx = SUPPORTED_COMMANDS - 1; idx >= 0; idx--) {
            if (check_command == COMMANDS_SLCAN[idx]) {
                set_special_command(idx);
                break; // found a match, stop searching
            }
        }

        ptr = frame_end + 1; // ✅ advance past current frame
    }
}

/**
 * @brief Receive and parse SLCAN frames from UART
 *
 * Reads available data from the UART port, decodes SLCAN frames, and checks for
 * special control commands. Returns a list of parsed frames ready for transmission
 * to the CAN bus via the motor controller.
 *
 * @param uart_buffer Pointer to buffer for storing raw UART data (minimum max_len_uart bytes)
 * @param max_len_uart Maximum number of bytes to read from UART (should account for null terminator)
 *
 * @return Pointer to static slcan_frame_list_t containing decoded frames.
 *         count is 0 if no frames received or on read errors.
 *         Pointer remains valid until next call to receive_slcan().
 *
 * @note The returned pointer references a static variable; copy data if persistence needed
 * @note Automatically processes special control commands found in the buffer
 * @see transmit_slcan() for sending motor state in the opposite direction
 * @see decode_slcan() for the frame parsing logic
 */
const slcan_frame_list_t* receive_slcan(uint8_t *uart_buffer, size_t max_len_uart) {
    // Reset the count at the start
    slcan_streams.count = 0;

    int len_received = uart_read_bytes(PORT_UART, uart_buffer, max_len_uart - 1, UART_TICKS);

    if (len_received <= 0) return &slcan_streams;
    // Ensure the whole buffer is null-terminated at the end of the data received
    uart_buffer[len_received] = '\0';
    decode_slcan(uart_buffer, len_received, &slcan_streams);
    return &slcan_streams;
}

/**
 * @brief Unpack motor state data from a CAN reply message
 *
 * Decodes an 8-byte CAN reply message from an MIT-mode motor into motor state
 * parameters:
 * - Motor position (16 bits)
 * - Motor velocity (12 bits)
 * - Motor torque (12 bits)
 * - Motor temperature (8 bits, raw value)
 * - Motor error code (8 bits)
 *
 * @param msg Pointer to 8-byte CAN message received from motor
 *            Byte layout:
 *            - [0]: Motor driver ID
 *            - [1:2]: Position data (16 bits)
 *            - [3:4]: Velocity data (12 bits + 4 padding)
 *            - [4:5]: Current data (4 bits + 8 bits)
 *            - [6]: Temperature (raw, offset by -40°C)
 *            - [7]: Motor error code
 *
 * @return motor_state structure containing:
 *         - driver_id: Motor CAN ID
 *         - position: Position in radians
 *         - velocity: Velocity in rad/s
 *         - current: Current in A
 *         - temperature: Temperature in °C (adjusted from raw value)
 *         - motor_error: Motor error/fault code
 *
 * @note Temperature returned is adjusted: raw_temp - 40 = °C
 * @see pack_motor_state_to_slcan() for the inverse operation
 */
const motor_state unpack_reply(uint8_t* msg){
    /// unpack ints from can buffer ///
    const uint8_t id = msg[0]; //Driver ID
    const int pos_int = (msg[1]<<8)|msg[2]; // Motor Position Data
    const int vel_int = (msg[3]<<4)|(msg[4]>>4); // Motor Speed Data
    const int curr_int = ((msg[4]&0xF)<<8)|msg[5]; //Motor Current Data
    const int tempt_int = msg[6] ; // Temperature range: -40~215
    const uint8_t motor_error = msg[7] ; // motor error code
     /// convert ints to floats ///
    const float pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    const float vel = uint_to_float(vel_int, V_MIN, V_MAX, 12);
    const float curr = uint_to_float(curr_int, I_MIN, I_MAX, 12);
    const float tempt = tempt_int;
    motor_state packet_received = {id, pos, vel, curr, tempt-40, motor_error};
    return packet_received;
}


/**
 * @brief Transmit motor state via UART in SLCAN format
 *
 * Encodes a motor_state structure into SLCAN format and transmits it via UART
 * to the Jetson or other controller. The message is formatted as:
 * 't' + 3-digit CAN ID + '8' (DLC) + 16 hex digits (8 bytes) + '\r'
 *
 * The transmitted data includes:
 * - Motor position (16 bits)
 * - Motor velocity (12 bits)
 * - Motor torque (12 bits)
 * - Motor temperature (16 bits)
 * - Motor status/error (8 bits)
 *
 * @param info_motor motor_state structure containing the data to transmit
 *                   - driver_id: Motor CAN ID (will be formatted as 3 hex digits)
 *                   - position, velocity, torque: Control state values
 *                   - temperature: Motor temperature
 *                   - motor_error: Motor status code
 *
 * @note Always transmits DLC=8 (fixed 8-byte payload)
 * @note Logs detailed motor state and SLCAN message for debugging
 * @note Requires uart_init() to have been called first
 * @see receive_slcan() for receiving SLCAN commands from Jetson
 * @see unpack_reply() for decoding incoming motor states
 */
void transmit_slcan(const motor_state info_motor){

    char slcan_command_transmit[LENGTH_SLCAN_DATA*4]; //22 bytes 0-20 tiiildd..[CR] , byte 21 \0
    //Convert the float to its binary representation (4 bytes for single-precision, 8 for double).

    char data_motor[LENGTH_SLCAN_DATA *4];//buffer to store the string of data
    //creates the data hex string ,Numbers of dd pairs must match the data length DLC
    ESP_LOGI(TAG_UART, "Motor State ID: %u | P: %.2f rad | V: %.2f rad/s | I: %.2f A | T: %.2f C | err: %u",
                info_motor.driver_id,
                info_motor.position,
                info_motor.velocity,
                info_motor.current,
                info_motor.temperature,
                info_motor.motor_error
            );
    pack_motor_state_to_slcan(
                data_motor,
                sizeof(data_motor),
                info_motor.position,
                info_motor.velocity,
                info_motor.current,
                info_motor.temperature,
                info_motor.motor_error);

    //Embeddding the hex string into an SLCAN transmit command.
    // Build SLCAN: tIIILDDDDDDDDDDDDDDD\r\0 , \0 is added by the snprintf at the end
    //"t%03" PRIx32 ensure the CAN ID is 3 characters fixed width
    //PRIx32: This is a macro (from <inttypes.h>) that ensures a uint32_t variable is printed as hexadecimal in a way
    //that is portable across different processors.
    //"8%.16s\r DLC that is always 8 and precision of 16 characters to represent
    //as string the 8 bytes (2 char per byte in hex)
    snprintf(slcan_command_transmit, sizeof(slcan_command_transmit),
        "t%03" PRIx32 "8%.16s\r", info_motor.driver_id, data_motor);

    //send command via UART
    ESP_LOGI(TAG_UART,"Sending to UART slcan  %s", slcan_command_transmit);
    uart_write_bytes(PORT_UART, slcan_command_transmit, strlen(slcan_command_transmit));
}

/**
 * @brief Print UART buffer status and check for potential overflow
 *
 * Queries the UART driver to determine how much data is currently waiting
 * in the RX ring buffer. Logs the current buffered size and issues a warning
 * if the buffer is over 90% full, indicating potential packet loss if the
 * receive task doesn't have sufficient priority.
 *
 * @note Useful for debugging UART communication issues and task scheduling problems
 * @note A nearly-full buffer warning suggests the receive task needs higher priority
 */
void print_UART_status() {
    size_t buffered_size;
    // Check how many bytes are currently waiting in the RX ring buffer
    uart_get_buffered_data_len(PORT_UART, &buffered_size);

    ESP_LOGI(TAG_UART, "UART Stats | RX Buffer: %u/%d bytes",
            buffered_size, BUF_SIZE);

    if (buffered_size > (BUF_SIZE * 0.9)) {
        ESP_LOGW(TAG_UART, "WARNING: UART RX Buffer is nearly full! Check task priority.");
    }
}

/**
 * @brief Initialize UART interface for SLCAN communication
 *
 * Configures and initializes UART2 (PORT_UART) with the following settings:
 * - Baud rate: 115200
 * - Data bits: 8
 * - Stop bits: 1
 * - Parity: Disabled
 * - Flow control: Disabled (software control can be enabled if needed)
 * - RX/TX pins: UART_RXD_PIN (16), UART_TXD_PIN (17)
 * - RX ring buffer size: BUF_SIZE (2048 bytes)
 * - TX ring buffer size: BUF_SIZE (2048 bytes)
 * - Event queue size: EVENT_QUEUE_SIZE (12 events)
 *
 * Creates FreeRTOS queues for UART events and enables event detection.
 * Waits 1 second for initialization to complete.
 *
 * @note Must be called once before any UART communication
 * @note Automatically installs the UART driver with interrupt support
 * @see PORT_UART, BUF_SIZE, UART_BAUD_RATE for configuration constants
 */
void uart_init(){
    //Last zero means no interrupts
    ESP_ERROR_CHECK(uart_driver_install(PORT_UART, BUF_SIZE, BUF_SIZE, EVENT_QUEUE_SIZE, &uart_queue, 0));
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,// use software control if disabled
        .rx_flow_ctrl_thresh = UART_RTS_THRESHOLD,
    };
    ESP_ERROR_CHECK(uart_param_config(PORT_UART, &uart_config));
    //Set UART Pins
    ESP_ERROR_CHECK(uart_set_pin(PORT_UART, UART_TXD_PIN, UART_RXD_PIN, -1, -1));
    //
    ESP_LOGI(TAG_UART, "UART testudog controller started \n");
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait x second
}
