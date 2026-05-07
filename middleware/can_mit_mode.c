#include <math.h>
#include <string.h>
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_log.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "can_mit_mode.h"


const uint8_t START_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFC };
const uint8_t EXIT_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
const uint8_t SET_ZERO_POSITION[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
const uint8_t READ_MOTOR[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
const char *TAG = "testudog_node_can"; // FOR LOGGING

static QueueHandle_t rx_queue = NULL;

int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void comm_can_transmit_eid(const uint32_t driver_id, uint8_t *data, size_t data_length,const twai_node_handle_t node_hdl) {
    twai_frame_t tx_msg = {
        .header.id = driver_id,  // Message ID
        .header.ide = false,     // Use 29-bit extended ID format
        .header.dlc = LENGTH_CAN_BUFFER,
        .buffer = data,        // Pointer to data to transmit
        .buffer_len = data_length//sizeof(send_buff),  // Length of data to transmit
    };
    ESP_ERROR_CHECK(twai_node_transmit(node_hdl, &tx_msg, 0));  // Timeout = 0: returns immediately if queue is full
}

motor_state unpack_reply(uint8_t* msg){
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
    const float tor = uint_to_float(tor_int, -T_MIN, T_MAX, 12);
    const float tempt = tempt_int;
    motor_state packet_received = {id, pos, vel, tor, tempt-40, motor_error};
    return packet_received;
}

void pack_cmd( uint8_t * msg,  float p_des,  float v_des,  float kp,  float kd,  float t_ff) {
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    const int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    const int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    const int kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    const int kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    const int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    /// pack ints into the can buffer ///
    msg[0] = p_int>>8; // Position High 8
    msg[1] = p_int&0xFF; // Position Low 8
    msg[2] = v_int>>4; // Speed High 8 bits
    msg[3] = ((v_int&0xF)<<4)|(kp_int>>8); // Speed Low 4 bits KP High 4 bits
    msg[4] = kp_int&0xFF; // KP Low 8 bits
    msg[5] = kd_int>>4; // Kd High 8 bits
    msg[6] = ((kd_int&0xF)<<4)|(t_int>>8); // KP Low 4 bits Torque High 4 bits
    msg[7] = t_int&0xff; // Torque Low 8 bits
}

// bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx){
//     uint8_t recv_buff[LENGTH_CAN_BUFFER];
//     twai_frame_t rx_frame = {
//         .buffer = recv_buff,
//         .buffer_len = sizeof(recv_buff),
//     };
//     esp_err_t error = twai_node_receive_from_isr(handle, &rx_frame);
//     if (ESP_OK == error) {
//         // receive ok, do something here
//         motor_state data_received = unpack_reply(rx_frame.buffer);
//         ESP_LOGI(TAG, "FRAME RECEIVED IS \n Motor ID: %u \n position: %.2f \n velocity: %.2f \n torque: %.2f \n temperature: %.2f \nMotor error code: %u \n",
//             data_received.driver_id,
//             data_received.position,
//             data_received.velocity,
//             data_received.torque,
//             data_received.temperature,
//             data_received.motor_error
//             );
//         can_rx_msg_t msg = {
//         .id = rx_frame.header.id,
//         .dlc = rx_frame.buffer_len,
//         };
//         memcpy(msg.data, rx_frame.buffer, msg.dlc);  // ← COPY data! 
//         BaseType_t need_yield = pdFALSE;
//         xQueueSendFromISR(rx_queue, &msg, &need_yield);
//         portYIELD_FROM_ISR(need_yield);    
//         return true;
//     }
//      ESP_LOGI(TAG, "Failed to receive in ISR: %s", esp_err_to_name(error)); 
//      return false;
// }

bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    // This is the fastest way to see if hardware is actually talking
    esp_rom_printf("\a--- CAN MESSAGE RECEIVED! ---\n"); 
    
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = { .buffer = recv_buff, .buffer_len = 8 };
    
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        esp_rom_printf("ID: 0x%lx Data: %02x %02x %02x...\n", 
                        rx_frame.header.id, rx_frame.buffer[0], rx_frame.buffer[1]);
        return true;
    }
    return false;
}

static void rx_printer_task(void *arg)
{
    can_rx_msg_t msg;
    while (1) {
        if (xQueueReceive(rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Received CAN ID: 0x%x, DLC: %u", msg.id, msg.dlc);
            for (uint8_t i = 0; i < msg.dlc; i++) {
                ESP_LOGI(TAG, "Byte %u: 0x%02x", i, msg.data[i]);
            }
        }
    }
    vTaskDelete( NULL );
}


void can_mit_mode_init() {
    rx_queue = xQueueCreate(NUMBERS_MOTORS * 2 , sizeof(can_rx_msg_t));
    if (!rx_queue) {
        ESP_LOGI(TAG, "Failed to create RX queue");
        return;
    }
    xTaskCreatePinnedToCore(rx_printer_task, "can_rx_print", 4096, NULL, 8, NULL, tskNO_AFFINITY);

}


