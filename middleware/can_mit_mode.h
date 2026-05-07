#ifndef CAN_MIT_MODE_H
#define CAN_MIT_MODE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_twai_types.h"

// ========================================================================
// TWAI/CAN Parameters and constants for ESP32 WROOM UE
// ========================================================================
#define TWAI_SENDER_TX_GPIO     GPIO_NUM_18
#define TWAI_SENDER_RX_GPIO     GPIO_NUM_19
#define TWAI_QUEUE_DEPTH        100 //total motors testudog 
#define TWAI_BITRATE            1000000  // 1MB  kbps bitrate
#define LENGTH_CAN_BUFFER 8 //# max number of bytes data of standard CAN FRAME
#define NUMBERS_MOTORS 12 // Numbers of motors testudog change accordingly
// ========================================================================
// MIT MODE CONTROL COMMUNICATION PROTOCOL CONSTANTS FOR CUBERMARS MOTOR AK80-6
// ========================================================================
/// limit data to be within bounds ///
#define P_MAX  12.5f
#define P_MIN  -P_MAX
#define V_MAX  76.0f
#define V_MIN  -V_MAX
#define T_MAX  12.0f
#define T_MIN  -T_MAX
#define C_MAX 215
#define C_MIN -40
#define Kp_MIN  0
#define Kp_MAX  500.0f
#define Kd_MIN  0
#define Kd_MAX  5.0f
#define Test_Pos 0.0f
// ========================================================================
// CAN Motors ID
// ========================================================================
typedef enum {
    TESTUDOG_MOTOR_0 = 0x00,
    TESTUDOG_MOTOR_1,
    TESTUDOG_MOTOR_2,
    TESTUDOG_MOTOR_3,
    TESTUDOG_MOTOR_4,
    TESTUDOG_MOTOR_5,
    TESTUDOG_MOTOR_6,
    TESTUDOG_MOTOR_7,
    TESTUDOG_MOTOR_8,
    TESTUDOG_MOTOR_9,
    TESTUDOG_MOTOR_10, 
    TESTUDOG_MOTOR_11, 
}TESTUDOG_CAN_IDS;

// ========================================================================
// Structures for sending and receiving in MIT MODE via CAN BUS
// ========================================================================
//receive
typedef struct {
    uint32_t driver_id;
    float position;       
    float velocity;         
    float torque;
    float temperature;
    uint8_t motor_error;        
} motor_state;
//send
typedef struct {
    uint32_t driver_id;
    float position;       
    float velocity;         
    float torque;
    float k_propotional;
    float k_derivate;      
} motor_parameters;  

//be able to see values
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[LENGTH_CAN_BUFFER];
} can_rx_msg_t;

// ========================================================================
// CAN/TWAI MIT MODE Special CAN Codes
// ========================================================================
extern const uint8_t START_MIT_MODE[LENGTH_CAN_BUFFER];
extern const uint8_t EXIT_MIT_MODE[LENGTH_CAN_BUFFER];
extern const uint8_t SET_ZERO_POSITION[LENGTH_CAN_BUFFER];
extern const uint8_t READ_MOTOR[LENGTH_CAN_BUFFER];
extern const char *TAG; // FOR LOGGING


int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void comm_can_transmit_eid(const uint32_t driver_id, uint8_t *data, size_t data_length,const twai_node_handle_t node_hdl);
void pack_cmd( uint8_t * msg, const float p_des, const float v_des, const float kp, const float kd, const float t_ff);
void can_mit_mode_init();
motor_state unpack_reply(uint8_t* msg);
bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);
#endif // CAN_MIT_MODE_H