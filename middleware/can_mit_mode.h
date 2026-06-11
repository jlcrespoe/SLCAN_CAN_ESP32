#ifndef CAN_MIT_MODE_H
#define CAN_MIT_MODE_H

#ifdef __cplusplus
extern "C" {          // ← tells C++ linker: look for plain C names
#endif


#include <stdint.h>
#include <stddef.h>
#include <esp_twai_types.h>

// ========================================================================
// TWAI/CAN Parameters and constants for ESP32
// ========================================================================
#define TWAI_TX_GPIO GPIO_NUM_5 // CAN TX PIN
#define TWAI_RX_GPIO GPIO_NUM_4 //CAN RX PIN
#define TWAI_QUEUE_DEPTH 100 // Set at your own criteria
#define TWAI_BITRATE 1000000  // 1MB  kbps bitrate
#define LENGTH_CAN_BUFFER 8 //# max number of bytes data of standard CAN FRAME
#define NUMBERS_MOTORS 12 // Numbers of motors testudog change accordingly

// ========================================================================
// CAN Motors ID & MIT Mode
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
} TESTUDOG_CAN_IDS;

//MIT MODES ID COMMANDS
typedef enum {
    START_READ_MIT = 0x00,
    EXIT_MIT,
    SET_HOME_MIT,
} MIT_MODE_CMD;

// ========================================================================
// Structures for moto data and MIT Commands
// ========================================================================

//send motor data
typedef struct {
    uint32_t driver_id;
    float position;
    float velocity;
    float torque;
    float k_propotional;
    float k_derivate;
} motor_parameters;

//send check
typedef struct {
    float position;       
    float velocity;         
    float torque;
    float k_proportional;
    float k_derivate; 
} motor_control;

//send mit commands
typedef struct {
    uint8_t command[LENGTH_CAN_BUFFER];
    int action;
} motor_command;


extern const char *TAG_CAN;
void comm_can_transmit(const uint32_t driver_id, const uint8_t *data, int type_id);
const motor_control unpack_command(uint8_t* msg);
void can_mit_mode_init();
void print_CAN_status();
void command_to_all_motors(int action);
void pack_mit_command( uint8_t * msg,  float p_des,  float v_des,  float kp,  float kd,  float t_ff);
int is_special_command(uint8_t * msg);

#ifdef __cplusplus
}           // ← closes extern "C" {
#endif

#endif // CAN_MIT_MODE_H