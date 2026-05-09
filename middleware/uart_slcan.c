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
QueueHandle_t uart_queue = NULL;
static slcan_frame_list_t slcan_streams;  // static = invisible outside this file
static void pack_motor_state_to_slcan(char * msg, size_t msg_size, float pos, float vel,float t_ff, float temp_c, uint8_t mot_st) {

    pos = fminf(fmaxf(P_MIN, pos), P_MAX);
    vel = fminf(fmaxf(V_MIN, vel), V_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    temp_c = fminf(fmaxf(C_MIN, temp_c), C_MAX);
    
    /// convert floats to unsigned ints ///
    int p_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    int temp_c_int = float_to_uint(temp_c, C_MIN, C_MAX, 16);

    const uint16_t p_int16 = p_int & 0xFFFF;
    const uint16_t v_int12 = v_int & 0xFFF;
    const uint16_t t_int12 = t_int & 0xFFF;
    const uint16_t temp_c_int16 = temp_c_int & 0xFFFF;

    //format a hex string that represents 8 bytes of data that's the max of standard SLCAN 
    snprintf(msg, msg_size, "%04x%03x%03x%04x%01x\r", p_int16, v_int12, t_int12, temp_c_int16, mot_st);
}

//from ESP32 to MOTORS 
bool parse_slcan( const char* input, slcan_frame_t *frame_can){
    
    //only considering standard frames by now
    if(input[0] != 't' ) return false;
    //id
    char can_id[4] = {input[1], input[2], input[3],'\0'};
    frame_can->id = (uint32_t) strtol(can_id, NULL, 16);
    //dlc
    frame_can->dlc = input[4] - '0';
    if (frame_can->dlc > LENGTH_SLCAN_DATA) return false;
    //data
    for (size_t idx = 0; idx < frame_can-> dlc; idx++ ){
        char byte_str[3] = {input[5+(idx*2)],input[6+(idx*2)],'\0'};
        frame_can->data[idx] = (uint8_t) strtol(byte_str, NULL, 16);
    }
    return true;
}

//receive data from the JETSON in SLCAN format and convert it to send it to the TWAI/CAN Controller
void decode_slcan(uint8_t *uart_buffer, int length_buffer_uart, slcan_frame_list_t *out_list) {
    out_list->count = 0;
    size_t start = 0;
    for (size_t idx = 0; idx < length_buffer_uart; idx++) {
        if (uart_buffer[idx] == '\r') {
            uart_buffer[idx] = '\0'; // Null-terminate frame
            
            slcan_frame_t frame_process;
            if (parse_slcan((char*)&uart_buffer[start], &frame_process)) {
                
                // Safety check: don't overflow our "vector"
                if (out_list->count < MAX_FRAMES_PER_BUFFER) {
                    out_list->frames[out_list->count++] = frame_process;
                } else {
                    ESP_LOGW(TAG_UART, "Frame list full, dropping message");
                }
            }
            start = idx + 1;
        }
    }
}
const slcan_frame_list_t* receive_slcan(uint8_t *uart_buffer, size_t max_len_uart) {
    // Reset the count at the start
    slcan_streams.count = 0;

    int len_received = uart_read_bytes(UART_NUM_2, uart_buffer, max_len_uart - 1, UART_TICKS);
    
    if (len_received <= 0) {
        // No log here to avoid flooding the console if idle
        return &slcan_streams;
    }

    decode_slcan(uart_buffer, len_received, &slcan_streams);

    // Final check for malformed data (missing \r at the end or non supported commands)
    if (len_received > 0 && uart_buffer[len_received-1] != '\0') {
        uart_buffer[len_received] = '\0';
        ESP_LOGI(TAG_UART, "Partial data received: %s", uart_buffer);
    }
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
    pack_motor_state_to_slcan( 
                data_motor, 
                sizeof(data_motor),
                info_motor.position,
                info_motor.velocity,
                info_motor.torque,
                info_motor.temperature,
                info_motor.motor_error);
    //Embeddding the hex string into an SLCAN transmit command.
    // Build SLCAN: tIIILDDDDDDDDDDDDDDD\r\0
    //convert DLC
    const char length_char = LENGTH_SLCAN_DATA + '0';  // 8 → '8'
    snprintf(slcan_command_transmit, sizeof(slcan_command_transmit),
         "t%03" PRIx32 "%c%.16s", info_motor.driver_id, length_char, data_motor);
    //send command via UART 
    ESP_LOGI(TAG_UART,"Sending to UART  %s \n", slcan_command_transmit);
    uart_write_bytes(UART_NUM_2, &slcan_command_transmit, strlen(slcan_command_transmit));
}


void uart_init(){
    //Last zero means no interrupts
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE, BUF_SIZE, EVENT_QUEUE_SIZE, &uart_queue, 0));
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,//enable for real implementation
        .rx_flow_ctrl_thresh = UART_RTS_THRESHOLD,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    //Set UART Pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TXD_PIN, UART_RXD_PIN, UART_RTS_PIN, UART_CTS_PIN));
    //
    ESP_LOGI(TAG_UART, "UART testudog controller started \n"); 
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait a second   
}
