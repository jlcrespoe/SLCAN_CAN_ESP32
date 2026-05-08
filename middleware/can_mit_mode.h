#ifndef CAN_MIT_MODE_H
#define CAN_MIT_MODE_H

#ifdef __cplusplus
extern "C" {          // ← tells C++ linker: look for plain C names
#endif


#include <stdint.h>
#include <stddef.h>
#include <esp_twai_types.h>

// ========================================================================
// TWAI/CAN Parameters and constants for ESP32 WROOM UE
// ========================================================================
#define TWAI_SENDER_TX_GPIO     18 // CAN TX PIN
#define TWAI_SENDER_RX_GPIO     19 //CAN RX PIN
#define TWAI_QUEUE_DEPTH        100 // Set at your own criteria
#define TWAI_BITRATE            1000000  // 1MB  kbps bitrate
#define LENGTH_CAN_BUFFER 8 //# max number of bytes data of standard CAN FRAME
#define NUMBERS_MOTORS 12 // Numbers of motors testudog change accordingly

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
extern const char *TAG_CAN; // FOR LOGGING

void comm_can_transmit(const uint32_t driver_id, const uint8_t *data);
void can_mit_mode_init();
void init_motors();

#ifdef __cplusplus
}           // ← closes extern "C" {
#endif

#endif // CAN_MIT_MODE_H