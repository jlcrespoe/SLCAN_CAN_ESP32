#include <driver/twai.h>
#include <driver/uart.h>
#include "uart_slcan.h"
#include "can_mit_mode.h"


#define TWAI_SLNT_PIN 2 //For use with transeceiver TJA1051T/3, must be SLNT PIN LOW TO be active

void printStatus() {
  twai_status_info_t status;
  twai_get_status_info(&status);
  ESP_LOGI(TAG_CAN, "  State: %d | TX err: %d | RX err: %d | TX pending: %d\n",
    status.state,
    status.tx_error_counter,
    status.rx_error_counter,
    status.msgs_to_tx);
}

void test_uart(void *pvParameters){
    const char* test_str = "This is a test string.\n";
    uint8_t data[128];
    for(;;){
        ESP_LOGI(TAG_UART, "Sending test string");
        uart_write_bytes(PORT_UART, (const char*)test_str, strlen(test_str));
        vTaskDelay(pdMS_TO_TICKS(1000)); 

        ESP_LOGI(TAG_UART, "Checking if received string");
        int length_uart = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(PORT_UART, (size_t*)&length_uart));
        length_uart = uart_read_bytes(PORT_UART, data, length_uart, 100);
        ESP_LOGI(TAG_UART,"length_uart received is %i",length_uart);
        for(size_t idx = 0; idx < length_uart; idx++){
           ESP_LOGI(TAG_UART,"VALUE IS %c", data[idx]); 
        }
        uart_flush(PORT_UART); //clear the buffer

    }

}


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
            ESP_LOGI(TAG_UART, "Waiting for SLCAN Frames.");
            // Optional: short delay to prevent watchdog issues if UART is empty
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            continue;
            
        }
        // 3. Iterate through the frames and send to TWAI
        for (size_t idx = 0; idx < streams_can -> count; idx++) {

            const slcan_frame_t *command = &streams_can -> frames[idx];
            const uint8_t* data_can = (uint8_t *)command->data;
            char hex_str[LENGTH_CAN_BUFFER * 3 + 1]; // "AA BB CC ..." + \0
            for (uint8_t byte_idx = 0; byte_idx < LENGTH_CAN_BUFFER; byte_idx++) {
                snprintf(hex_str + (byte_idx * 3), 4, "%02X ", data_can[byte_idx]);
            }
            ESP_LOGI(TAG_UART, "CAN data: %s", hex_str);
            // Send to CAN bus
            comm_can_transmit(
                command->id, 
                data_can
            );
            uart_flush(PORT_UART); //clear the buffer
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
          //Receive message
          esp_err_t err = twai_receive(&can_msg, pdMS_TO_TICKS(100));
          if(err == ESP_OK) {
            // Now safe to log and process
            ESP_LOGI(TAG_CAN, "Received ID: 0x%lx DLC: %d", can_msg.identifier, can_msg.data_length_code);
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
            //ESP_LOGI(TAG_CAN, "Waiting for CAN frames...");
            printStatus();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  
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
    //set motors to MIT MODE 
    //init_motors();
    #if defined(TWAI_SLNT_PIN)
    pinMode(TWAI_SLNT_PIN, OUTPUT);
    digitalWrite(TWAI_SLNT_PIN, LOW);  // LOW = normal operation
    ESP_LOGI(TAG_UART, "HEY SLNT PIN READY");
    #endif

    //xTaskCreatePinnedToCore(slcan_to_can_task, "slcan_to_can_task", 1024*4, NULL, 5, NULL, 0); 
    xTaskCreatePinnedToCore(test_uart, "test_uart", 1024*4, NULL, 5, NULL, 0); 
    xTaskCreatePinnedToCore(can_to_slcan_task, "can_to_slcan_task", 1024*4, NULL, 6, NULL, 1);  

    ESP_LOGI(TAG_CAN, "SCRIPT READY");

}

void loop(){
    
}
