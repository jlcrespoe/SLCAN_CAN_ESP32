#ifndef UART_SLCAN_H
#define UART_SLCAN_H

#ifdef __cplusplus
extern "C" {          // ← tells C++ linker: look for plain C names
#endif

// ========================================================================
// UART Parameters and constants for ESP32
// ========================================================================

#define PORT_UART UART_NUM_2
#define UART_RXD_PIN 16
#define UART_TXD_PIN 17
#define UART_RTS_PIN (UART_PIN_NO_CHANGE) //18 not using it
#define UART_CTS_PIN (UART_PIN_NO_CHANGE) //19 not using it

#define UART_BAUD_RATE     115200
#define UART_RTS_THRESHOLD 122
#define UART_TICKS 100

#define BUF_SIZE 2048
#define LENGTH_UART_BUFFER 128
#define LENGTH_SLCAN_DATA 8 // equivalent to LENGTH_CAN_BUFFER
#define EVENT_QUEUE_SIZE 12
#define SUPPORTED_COMMANDS 3

#define MAX_FRAMES_PER_BUFFER 10 // Adjust based on expected UART traffic

// ========================================================================
// Structures to process slcan frames & motor state
// ========================================================================

//receive
typedef struct {
    uint32_t driver_id;
    float position;
    float velocity;
    float current;
    float temperature;
    uint8_t motor_error;
} motor_state;



typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[LENGTH_SLCAN_DATA];
    bool is_extended;
    bool is_rtr;
} slcan_frame_t;

typedef struct {
    slcan_frame_t frames[MAX_FRAMES_PER_BUFFER];
    size_t count;
} slcan_frame_list_t;

extern const char *TAG_UART;
extern const char COMMANDS_SLCAN[SUPPORTED_COMMANDS];
extern bool state_slcan_channel;
const motor_state unpack_reply(uint8_t* msg);
const slcan_frame_list_t* receive_slcan(uint8_t *uart_buffer, size_t max_len_uart);
void transmit_slcan(const motor_state info_motor);
void print_UART_status();
void uart_init();

#ifdef __cplusplus
}           // ← closes extern "C" {
#endif


#endif // UART_SLCAN_H