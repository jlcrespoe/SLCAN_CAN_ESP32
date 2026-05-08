#include <math.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include "motor_values.h"
#include "can_mit_mode.h"


const uint8_t START_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFC };
const uint8_t EXIT_MIT_MODE[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
const uint8_t SET_ZERO_POSITION[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
const uint8_t READ_MOTOR[LENGTH_CAN_BUFFER] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
const char *TAG_CAN = "Testudog_CAN"; // FOR LOGGING



void comm_can_transmit(const uint32_t driver_id, const uint8_t *data) {
      twai_message_t tx_msg = {
        .identifier = driver_id,
        .extd = 0,
        .rtr = 0,
        .data_length_code = LENGTH_CAN_BUFFER,

      };
      memcpy(tx_msg.data, data, tx_msg.data_length_code);
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
    esp_log_level_set(TAG_CAN, ESP_LOG_INFO);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_SENDER_TX_GPIO, TWAI_SENDER_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 100;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGI(TAG_CAN, "TWAI install failed");
        while (1);
    }
    // xTaskCreatePinnedToCore(rx_printer_task, "can_rx_print", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG_CAN, "TWAI Node created done.\n");
}

void init_motors(){
    for(uint8_t motor_id = TESTUDOG_MOTOR_0; motor_id < NUMBERS_MOTORS ; motor_id++){
        ESP_LOGI(TAG_CAN, "Starting MIT Mode on motor TESTUDOG 0x%u....", motor_id);
        comm_can_transmit((uint32_t) motor_id, (uint8_t *)START_MIT_MODE);
        vTaskDelay(pdMS_TO_TICKS(500)); //wait half second
        ESP_LOGI(TAG_CAN, "MIT Mode started on motor TESTUDOG 0x%u....", motor_id);
    }
}


