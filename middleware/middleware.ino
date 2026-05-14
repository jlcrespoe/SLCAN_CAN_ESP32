#include <driver/twai.h>
#include <driver/uart.h>
#include "uart_slcan.h"
#include "can_mit_mode.h"

#define TWAI_SLNT_PIN 2 //For use with transeceiver TJA1051T/3, must be SLNT PIN LOW TO be active

// ============================================================
// TASK 1: SLCAN (UART) → Decode → Encode MIT MODE → CAN Bus TX
// Receives SLCAN frames from Jetson and forwards to CAN bus
// ============================================================
/**
 * @brief FreeRTOS task that relays SLCAN frames from UART to CAN bus
 *
 * Continuously receives SLCAN-formatted frames from the Jetson controller via UART,
 * decodes them, and transmits the resulting CAN messages to all 12 motors on the
 * CAN bus. Acts as a bridge between the host controller and the motor drivers.
 *
 * Task behavior:
 * - Checks if SLCAN channel is open (state_slcan_channel == true)
 * - Receives and decodes multiple SLCAN frames in one read
 * - Extracts motor ID and raw CAN data from each frame
 * - Transmits each frame to the corresponding motor via comm_can_transmit()
 * - Yields to other tasks with 10ms delay between frames
 * - If channel closed, logs status and waits 1 second before retry
 * - If no frames received, prints UART buffer status and waits 1 second
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 *
 * @note This task runs indefinitely and should be created during setup()
 * @note Channel can be controlled via special SLCAN commands (O=open, C=close)
 * @note Requires uart_init() and can_mit_mode_init() to be called first
 * @see receive_slcan() for UART frame reception
 * @see comm_can_transmit() for CAN message transmission
 * @see state_slcan_channel global variable for channel control
 */
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
/**
 * @brief FreeRTOS task that relays motor state data from CAN bus to UART
 *
 * Continuously monitors the CAN bus for incoming motor state messages,
 * decodes the MIT-mode response frames, and transmits the motor state
 * information back to the Jetson controller via UART in SLCAN format.
 * Acts as the return path for motor feedback.
 *
 * Task behavior:
 * - Checks if SLCAN channel is open (state_slcan_channel == true)
 * - Waits up to 100ms for CAN messages on the bus
 * - Decodes received 8-byte MIT mode reply using unpack_reply()
 * - Converts motor state to SLCAN format and transmits via UART
 * - If channel closed, logs status and waits 1 second before retry
 * - If no message received within timeout, prints CAN status and waits 1 second
 * - Logs detailed motor state information (ID, position, velocity, etc.)
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 *
 * @note This task runs indefinitely and should be created during setup()
 * @note Channel state controlled by special commands from the host
 * @note Requires uart_init() and can_mit_mode_init() to be called first
 * @see twai_receive() for CAN message reception
 * @see unpack_reply() for decoding motor state from CAN data
 * @see transmit_slcan() for UART transmission
 * @see state_slcan_channel global variable for channel control
 */
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

/**
 * @brief Arduino setup function - initializes all hardware and FreeRTOS tasks
 *
 * Performs one-time initialization of the ESP32 system:
 * 1. Sets up logging levels for CAN and UART subsystems to INFO level
 * 2. Initializes the TWAI/CAN interface at 1 Mbps for motor communication
 * 3. Initializes UART2 at 115200 baud for Jetson controller communication
 * 4. Configures the CAN transceiver silent pin (if available) to enable operation
 * 5. Creates and starts two FreeRTOS tasks for bidirectional communication:
 *    - slcan_to_can_task: Relays UART commands to motors via CAN
 *    - can_to_slcan_task: Relays motor feedback from CAN back to UART
 *
 * Optional (currently commented out):
 * - Sends START_READ_MIT command to all 12 motors to enter MIT mode on startup
 *
 * @note Configure Core Debug Level to INFO via Tools -> Core Debug in Arduino IDE
 *       to enable detailed logging output for debugging
 * @note The TWAI_SLNT_PIN (GPIO 2) is used with TJA1051T/3 transceiver;
 *       LOW = active, HIGH = silent/standby
 * @note Both FreeRTOS tasks are created but may not run until scheduler starts
 * @see can_mit_mode_init() for CAN/TWAI initialization
 * @see uart_init() for UART initialization
 * @see slcan_to_can_task() for command relay task
 * @see can_to_slcan_task() for feedback relay task
 */
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
