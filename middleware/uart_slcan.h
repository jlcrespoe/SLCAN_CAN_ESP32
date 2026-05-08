#ifndef UART_SLCAN_H
#define UART_SLCAN_H


#ifdef __cplusplus
extern "C" {          // ← tells C++ linker: look for plain C names
#endif

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

#define MAX_FRAMES_PER_BUFFER 10 // Adjust based on expected UART traffic


//receive
typedef struct {
    uint32_t driver_id;
    float position;       
    float velocity;         
    float torque;
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

extern const char *TAG_UART; // FOR LOGGING
//void pack_motor_state_to_slcan( char * msg, size_t msg_size, float pos, float vel,float t_ff, float temp_c, uint8_t mot_st);
//void decode_slcan(uint8_t *uart_buffer);
//bool parse_slcan( const char* input, slcan_frame_t *frame_can);
motor_state unpack_reply(uint8_t* msg);
void receive_slcan( uint8_t *uart_buffer, size_t max_len_uart, slcan_frame_list_t *out_list );
void transmit_slcan(const motor_state info_motor);
void uart_init();


#ifdef __cplusplus
}           // ← closes extern "C" {
#endif


#endif // CAN_MIT_MODE_H