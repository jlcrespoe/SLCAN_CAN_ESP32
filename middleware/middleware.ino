#include <driver/twai.h>
#include <driver/uart.h>
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
        //if the channel is close can't send from JETSON
        if(state_slcan_channel){
            // 1. Receive and Decode
            // We pass the address of our list struct to be filled
            const slcan_frame_list_t *streams_can = receive_slcan(uart_receive, sizeof(uart_receive));

            // 2. Check if we actually got frames (count > 0)
            if (streams_can -> count == 0) {
                print_UART_status();
                // Optional: short delay to prevent watchdog issues if UART is empty
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;

            }
            ESP_LOGI(TAG_UART, "Received a total of %u streams", streams_can -> count);
            // 3. Iterate through the frames and send to TWAI
            for (size_t idx = 0; idx < streams_can -> count; idx++) {

                const slcan_frame_t *command = &streams_can -> frames[idx];
                const uint8_t* data_can = (uint8_t *)command->data;

                // Send to CAN bus
                comm_can_transmit(
                    command->id,
                    data_can
                );

            //Search for special commands
            //search_and_set_special_command(uart_receive);

            // 4. Small yield to let other tasks run
            vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            ESP_LOGI(TAG_UART, "UART Channel is closed");
        }
        //Check for special commands
        //search_and_set_special_command(uart_receive);
    }
}

// ============================================================
// TASK 2: CAN Bus RX → Decode MIT MODE → Encode SLCAN Format → SLCAN (UART)
// Safely processes CAN frames and forwards to Jetson via UART
// ============================================================
void can_to_slcan_task(void *pvParameters) {
    twai_message_t  can_msg;
    for (;;) {
        //if the channel is close can't receive from CAN
        if(state_slcan_channel){
          //Receive message
        esp_err_t err = twai_receive(&can_msg, pdMS_TO_TICKS(100));
        if(err != ESP_OK) {
            print_CAN_status();
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        // Now safe to log and process
        ESP_LOGI(TAG_CAN, "Received ID: 0x%lx DLC: %d", can_msg.identifier, can_msg.data_length_code);
        motor_state motor_data = unpack_reply(can_msg.data);
        transmit_slcan(motor_data);
        }
        //Check for special commands
        //search_and_set_special_command(uart_receive);
        }

}

void setup(){
    //Don't forget to setup Core Level Debug to INFO To watch logs!!! Tools -> Core Debug
    esp_log_level_set(TAG_CAN, ESP_LOG_INFO);
    esp_log_level_set(TAG_UART, ESP_LOG_INFO);
    //Set up TWAI/CAN Controller
    can_mit_mode_init();
    ESP_LOGI(TAG_CAN, "HEY CAN READY");
    //Set up UAART Controller
    uart_init();
    ESP_LOGI(TAG_UART, "HEY UART READY");
    //start all motors to MIT MODE
    //command_to_all_motors(START_READ_MIT);
    #if defined(TWAI_SLNT_PIN)
        pinMode(TWAI_SLNT_PIN, OUTPUT);
        digitalWrite(TWAI_SLNT_PIN, LOW);  // LOW = normal operation
        ESP_LOGI(TAG_UART, "HEY SLNT PIN READY");
    #endif

    xTaskCreatePinnedToCore(slcan_to_can_task, "slcan_to_can_task", 1024*4, NULL, 5, NULL, 0);  
    xTaskCreatePinnedToCore(can_to_slcan_task, "can_to_slcan_task", 1024*4, NULL, 6, NULL, 1);  

    ESP_LOGI(TAG_CAN, "SCRIPT READY");

}

void loop(){
    //Put any code here
}
