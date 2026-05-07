#include <math.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include "can_mit_mode.h"


const uint8_t START_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFC };
const uint8_t EXIT_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
const uint8_t SET_ZERO_POSITION[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
const uint8_t READ_MOTOR[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
const char *TAG = "Testudog_CAN"; // FOR LOGGING

static QueueHandle_t can_rx_queue = NULL;
static SemaphoreHandle_t can_mutex = NULL;

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

void comm_can_transmit_eid(const uint32_t driver_id, uint8_t *data) {
      twai_message_t tx_msg = {
        .identifier = driver_id;
        .extd = 0;
        .rtr = 0;
        .data_length_code = LENGTH_CAN_BUFFER;
        .data = data;

      }
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, pdMS_TO_TICKS(100)));  // Timeout = 0: returns immediately if queue is full
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

void can_mit_mode_init() {
    // rx_queue = xQueueCreate(NUMBERS_MOTORS * 2 , sizeof(can_rx_msg_t));
    // if (!rx_queue) {
    //     ESP_LOGI(TAG, "Failed to create RX queue");
    //     return;
    // }
    // xTaskCreatePinnedToCore(rx_printer_task, "can_rx_print", 4096, NULL, 8, NULL, tskNO_AFFINITY);

}


