#include <driver/twai.h>
#include "uart_slcan.h"
#include "can_mit_mode.h"


#define TWAI_SLNT_PIN 2 //For use with transeceiver TJA1051T/3, must be SLNT PIN LOW TO be active

// ============================================================
// TASK 1: SLCAN (UART) → Decode → Encode MIT MODE → CAN Bus TX
// Receives SLCAN frames from Jetson and forwards to CAN bus
// ============================================================
void slcan_to_can_task(void *pvParameters) {
    uint8_t uart_receive[256]; // Ensure this matches your expected MTU
    for (;;) {
            // 1. Receive and Decode
            // We pass the address of our list struct to be filled
            const slcan_frame_list_t *streams_can = receive_slcan(uart_receive, sizeof(uart_receive));

            // 2. Check if we actually got frames (count > 0)
            if (streams_can -> count == 0) {
                ESP_LOGI(TAG_TDG, "Waiting for SLCAN Frames.");
                // Optional: short delay to prevent watchdog issues if UART is empty
                vTaskDelay(pdMS_TO_TICKS(10)); 
                continue;
            }
            // 3. Iterate through the frames and send to TWAI
            for (size_t idx = 0; idx < streams_can -> count; idx++) {

                const slcan_frame_t *command = &streams_can -> frames[idx];
                const uint8_t* data_can = (uint8_t *)command->data;
                for (uint8_t idx = 0; idx < LENGTH_CAN_BUFFER; idx++) {
                    ESP_LOGI(TAG_UART,"%02X ", msg.data[idx]);
                }
                    ESP_LOGI(TAG_UART,"\n");
                // Send to CAN bus
                comm_can_transmit(
                    command->id, 
                    data_can
                );
            }
        // 4. Small yield to let other tasks run
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ============================================================
// TASK 2: CAN Bus RX → Decode MIT MODE → Encode SLCAN Format → SLCAN (UART)
// Safely processes CAN frames and forwards to Jetson via UART
// ============================================================
void can_to_slcan_task(void *pvParameters) {
    twai_message_t  can_msg;
    for (;;) {
        // Blocks here until ISR pushes a frame — no CPU wasted
          if(twai_receive(&can_msg, pdMS_TO_TICKS(200)) == ESP_OK) {
            // Now safe to log and process
            ESP_LOGI(TAG_CAN, "Received ID: 0x%lx DLC: %d", can_msg.id, can_msg.dlc);
            motor_state motor_data = unpack_reply(can_msg.data);
            ESP_LOGI(TAG_CAN, "Motor ID: %u pos: %.2f vel: %.2f torque: %.2f temp: %.2f err: %u",
                motor_data.driver_id,
                motor_data.position,
                motor_data.velocity,
                motor_data.torque,
                motor_data.temperature,
                motor_data.motor_error
            );
              transmit_slcan(motor_data);
          } else {
            ESP_LOGI(TAG_CAN, "Waiting for CAN frames...");
        }
    }
}

void setup(){
//Set up TWAI/CAN Controller
can_mit_mode_init();
//Set up UAART Controller
uart_init();
//set motors to MIT MODE 
//init_motors();
#if defined(TWAI_SLNT_PIN)
  pinMode(TWAI_SLNT_PIN, OUTPUT);
  digitalWrite(TWAI_SLNT_PIN, LOW);  // LOW = normal operation
#endif

xTaskCreatePinnedToCore(slcan_to_can_task, "slcan_to_can_task", 1024*4, NULL, 5, NULL, 0); 
// CAN → SLCAN:  slightly higher so motor replies are processed quickly
xTaskCreatePinnedToCore(can_to_slcan_task, "can_to_slcan_task", 1024*4, NULL, 6, NULL, 1);  


}

void loop(){

}