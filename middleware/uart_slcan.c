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


const char *TAG_UART = "Testudog_UART"; // FOR LOGGING
const char COMMANDS_SLCAN[SUPPORTED_COMMANDS] = {'t','O','C'};
bool state_slcan_channel = true;
static QueueHandle_t uart_queue = NULL;
static slcan_frame_list_t slcan_streams;  // static = invisible outside this file


static void pack_motor_state_to_slcan(char * msg, size_t msg_size, float pos, float vel,float t_ff, float temp_c, uint8_t mot_st) {

    //Linear Mapping (Converting physical units to raw integers)
    pos = fminf(fmaxf(P_MIN, pos), P_MAX);
    vel = fminf(fmaxf(V_MIN, vel), V_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    temp_c = fminf(fmaxf(C_MIN, temp_c), C_MAX);
    
    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    int temp_c_int = float_to_uint(temp_c, C_MIN, C_MAX, 16);

    // 3. Masking ensures not bits exist outside the target range
    const uint16_t p_int16 = p_int & 0xFFFF;
    const uint16_t v_int12 = v_int & 0xFFF;
    const uint16_t t_int12 = t_int & 0xFFF;
    const uint16_t temp_c_int16 = temp_c_int & 0xFFFF;

    //format a hex string that represents 8 bytes of data that's the max of standard SLCAN 
    snprintf(msg, msg_size, "%04x%03x%03x%04x%02x", p_int16, v_int12, t_int12, temp_c_int16, mot_st);
}

//from ESP32 to MOTORS 
static bool parse_slcan( const char* input, slcan_frame_t *frame_can){
    
    // 1. Basic validation, considering  standard frames only
    if (input == NULL || input[0] != 't') {
        return false;
    }
    // 2. Length check: A standard 't' frame is:
    // 't' + 3(ID) + 1(DLC) + (2 * DLC) data bytes
    // Length is (DLC 8): 21 chars.
    size_t input_len = strlen(input);
    if (input_len < 21) return false;
    //id
    // 3. Parse ID (3 hex digits)
    char can_id[4];
    memcpy(can_id, &input[1], 3);
    can_id[3] = '\0';
    frame_can->id = (uint32_t) strtol(can_id, NULL, 16);
    // 4. Parse DLC (1 digit)
    frame_can->dlc = input[4] - '0';// '0' is 48 as uint8_t, refer to ASCII
    if (frame_can->dlc > LENGTH_SLCAN_DATA) return false;
    // 6. Parse Data bytes
    for (uint8_t idx = 0; idx < frame_can->dlc; idx++) {
        char byte_hex[3];
        byte_hex[0] = input[5 + (idx * 2)];
        byte_hex[1] = input[6 + (idx * 2)];
        byte_hex[2] = '\0';
        frame_can->data[idx] = (uint8_t)strtol(byte_hex, NULL, 16);
    }
    return true;
}

//receive data from the JETSON in SLCAN format and convert it to send it to the TWAI/CAN Controller
static void decode_slcan(uint8_t *uart_buffer, int length_buffer_uart, slcan_frame_list_t *out_list) {
    out_list->count = 0;
    char *ptr = (char *)uart_buffer;
    char *frame_end;
    // Use strchr to find each terminator in the buffer
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

//Currently only supports Open, Close Channel  special commands
void search_and_set_special_command(uint8_t *uart_buffer){
    char *ptr = (char *)uart_buffer;
    char *frame_end;
    // Use strchr to find each terminator in the buffer
    while ((frame_end = strchr(ptr, '\r')) != NULL) {
        // check the FIRST char of the frame
        char check_command = *ptr;
        // 
        for (int idx = SUPPORTED_COMMANDS - 1; idx >= 0; idx--) {
            if (check_command == COMMANDS_SLCAN[idx]) {
                set_special_command(idx);
                break; // found a match, stop searching
            }
        }

        ptr = frame_end + 1; // ✅ advance past current frame
    }
}



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

const motor_state unpack_reply(uint8_t* msg){
    /// unpack ints from can buffer ///
    const uint8_t id = msg[0]; //Driver ID
    const int pos_int = (msg[1]<<8)|msg[2]; // Motor Position Data
    const int vel_int = (msg[3]<<4)|(msg[4]>>4); // Motor Speed Data
    const int tor_int = ((msg[4]&0xF)<<8)|msg[5]; //Motor Torque Data
    const int tempt_int = msg[6] ; // Temperature range: -40~215
    const uint8_t motor_error = msg[7] ; // motor error code
     /// convert ints to floats ///
    const float pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    const float vel = uint_to_float(vel_int, V_MIN, V_MAX, 12);
    const float tor = uint_to_float(tor_int, T_MIN, T_MAX, 12);
    const float tempt = tempt_int;
    motor_state packet_received = {id, pos, vel, tor, tempt-40, motor_error};
    return packet_received;
}

//Send motor data via UART to the Jetson in SLCAN Format
void transmit_slcan(const motor_state info_motor){

    char slcan_command_transmit[LENGTH_SLCAN_DATA*4]; //22 bytes 0-20 tiiildd..[CR] , byte 21 \0
    //Convert the float to its binary representation (4 bytes for single-precision, 8 for double).

    char data_motor[LENGTH_SLCAN_DATA *4];//consider \0 in the string and an extra byte to secure no overflow
    //creates the data hex string ,Numbers of dd pairs must match the data length DLC
    ESP_LOGI(TAG_UART, "Motor ID: %u pos: %.2f vel: %.2f torque: %.2f temp: %.2f err: %u",
                info_motor.driver_id,
                info_motor.position,
                info_motor.velocity,
                info_motor.torque,
                info_motor.temperature,
                info_motor.motor_error
            );
    pack_motor_state_to_slcan( 
                data_motor, 
                sizeof(data_motor),
                info_motor.position,
                info_motor.velocity,
                info_motor.torque,
                info_motor.temperature,
                info_motor.motor_error);

    //Embeddding the hex string into an SLCAN transmit command.
    // Build SLCAN: tIIILDDDDDDDDDDDDDDD\r\0 , \0 is added by the snprintf at the end
    //"t%03" PRIx32 ensure the CAN ID is 3 characters fixed width
    //PRIx32: This is a macro (from <inttypes.h>) that ensures a uint32_t variable is printed as hexadecimal in a way that is portable across different processors.
    //"8%.16s\r DLC that is always 8 and precision of 16 characters to represent as string the 8 bytes (2 char per byte in hex)
    snprintf(slcan_command_transmit, sizeof(slcan_command_transmit),
         "t%03" PRIx32 "8%.16s\r", info_motor.driver_id, data_motor);

    //send command via UART 
    ESP_LOGI(TAG_UART,"Sending to UART slcan  %s", slcan_command_transmit);
    uart_write_bytes(PORT_UART, slcan_command_transmit, strlen(slcan_command_transmit));
}

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
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait a second   
}
