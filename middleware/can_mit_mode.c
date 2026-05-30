#include <sys/_intsup.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <esp_err.h>
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <esp_log.h>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include "motor_values.h"
#include "can_mit_mode.h"

//Hold the general Special CAN Codes 
static const  motor_command MOTOR_CMDS_MIT[] = {
    {START_READ_MIT, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}}, //enter MIT Mode or read motors if already MIT Mode started 
    {EXIT_MIT, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}}, //exit MIT Mode
    {SET_HOME_MIT, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}} //set zero position
};

static const size_t NUM_CMDS = sizeof(MOTOR_CMDS_MIT) / sizeof(MOTOR_CMDS_MIT[0]);

const char *TAG_CAN = "Testudog_CAN"; // FOR LOGGING

/**
 * @brief Transmit CAN message to a specific motor
 *
 * Prepares and transmits a CAN message to the specified motor ID via the TWAI interface.
 * Converts the data to a CAN frame, logs the transmission in hexadecimal format, and
 * sends it with a 100ms timeout.
 *
 * @param driver_id The CAN ID of the motor (0x00 to 0x0B for motors 0-11)
 * @param data Pointer to an 8-byte array containing the CAN message payload
 *
 * @note The function blocks until the message is queued or timeout occurs
 * @see comm_can_transmit() is typically called after pack_mit_command()
 */
void comm_can_transmit(const uint32_t driver_id, const uint8_t *data) {
    twai_message_t tx_msg = {
    .identifier = driver_id,
    .extd = 0,
    .rtr = 0,
    .data_length_code = LENGTH_CAN_BUFFER,

    };
    memcpy(tx_msg.data, data, tx_msg.data_length_code);

    const int CAN_cmd_check = is_special_command(tx_msg.data);
    if(CAN_cmd_check < 0){
        const motor_control motor_control_frame = unpack_command(tx_msg.data);
        ESP_LOGI(TAG_CAN, "Control cmd to Motor ID: %u | KP: %.2f | KD: %.2f | P: %.2f rad | V: %.2f rad/s | T: %.2f N/m",
                driver_id,
                motor_control_frame.k_proportional,
                motor_control_frame.k_derivate,
                motor_control_frame.position,
                motor_control_frame.velocity,
                motor_control_frame.torque
            );
    }

    char hex_str[LENGTH_CAN_BUFFER * 3 + 1]; // "AA BB CC ..." + \0
    for (uint8_t byte_idx = 0; byte_idx < LENGTH_CAN_BUFFER; byte_idx++) {
        snprintf(hex_str + (byte_idx * 3), 4, "%02X ", data[byte_idx]);
    }
    ESP_LOGI(TAG_CAN, "Sending CAN data: %s", hex_str);
    // Timeout = 0: returns immediately if queue is full
    ESP_ERROR_CHECK(twai_transmit(&tx_msg, pdMS_TO_TICKS(100)));
}

/**
 * @brief Initialize the TWAI/CAN interface for MIT mode communication
 *
 * Sets up the TWAI driver with:
 * - 1 Mbps bitrate configuration
 * - TX queue length of x messages
 * - Acceptance of all CAN IDs (no filtering)
 * - Normal mode operation (not loopback)
 *
 * Installs the driver, starts it, and waits 1 second for initialization to complete.
 * Enters infinite loop if driver installation or start fails.
 *
 * @see TWAI_TX_GPIO, TWAI_RX_GPIO, LENGTH_CAN_BUFFER definitions
 */
void can_mit_mode_init() {

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = TWAI_QUEUE_DEPTH;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGI(TAG_CAN, "TWAI install failed");
        while (1);
    }

    if (twai_start() != ESP_OK) {
        ESP_LOGI(TAG_CAN, "TWAI start failed");
        while (1);
    }

    ESP_LOGI(TAG_CAN, "TWAI Node created done.\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait a second
}

/**
 * @brief Print the current CAN/TWAI bus status
 *
 * Retrieves and logs the status information from the TWAI driver, including:
 * - Current bus state:
 *      0x000. ESP_OK: Status information retrieved
        0x102 ESP_ERR_INVALID_ARG: Arguments are invalid
        0x103 ESP_ERR_INVALID_STATE: TWAI driver is not installed
 * - TX error counter
 * - RX error counter
 * - Number of pending TX messages
 *
 * @note Useful for debugging CAN communication issues
 */
void print_CAN_status() {
    twai_status_info_t stat_info;
    twai_get_status_info(&stat_info);
    ESP_LOGI(TAG_CAN, "  State: %d | TX err: %d | RX err: %d | TX pending: %d\n",
    stat_info.state,
    stat_info.tx_error_counter,
    stat_info.rx_error_counter,
    stat_info.msgs_to_tx);
}

/**
 * @brief Send the same MIT mode command to all motors sequentially
 *
 * Iterates through all N motors (TESTUDOG_MOTOR_0 to TESTUDOG_MOTOR_11) and sends
 * the specified MIT mode command to each motor with a delay between commands.
 * This is typically used for:
 * - Entering MIT mode on all motors (START_READ_MIT)
 * - Exiting MIT mode on all motors (EXIT_MIT)
 * - Setting home position on all motors (SET_HOME_MIT)
 *
 * @param action The MIT mode command index: START_READ_MIT (0), EXIT_MIT (1), or SET_HOME_MIT (2)
 *
 * @see MOTOR_CMDS_MIT array for available commands
 * @see NUMBERS_MOTORS constant for motor count
 */
void command_to_all_motors(int action){

    for(uint8_t motor_id = TESTUDOG_MOTOR_0; motor_id < NUMBERS_MOTORS ; motor_id++){
        ESP_LOGI(TAG_CAN, "MIT action %i mode on TESTUDOG motor 0x%u....", action, motor_id);
        comm_can_transmit((uint32_t) motor_id, (uint8_t *)MOTOR_CMDS_MIT[action].command);
        vTaskDelay(pdMS_TO_TICKS(500)); //wait half second
        ESP_LOGI(TAG_CAN, "Done on %u | %u",motor_id, NUMBERS_MOTORS);
    }
}

/**
 * @brief Pack motor control parameters into CAN message format
 *
 * Encodes desired position, velocity, and PD control gains into an 8-byte CAN message
 * following the MIT actuator communication protocol:
 * - Position (p_des): 16 bits
 * - Velocity (v_des): 12 bits
 * - Proportional gain (kp): 12 bits
 * - Derivative gain (kd): 12 bits
 * - Feedforward torque (t_ff): 12 bits
 *
 * All input values are clamped to their min/max bounds before encoding.
 *
 * @param msg Pointer to 8-byte array where the packed message is stored
 * @param p_des Desired position in radians [-12.5, 12.5]
 * @param v_des Desired velocity in rad/s [-76.0, 76.0]
 * @param kp Proportional gain [0, 500.0]
 * @param kd Derivative gain [0, 5.0]
 * @param t_ff Feedforward torque in Nm [-12.0, 12.0]
 *
 * @note Values are automatically clamped to valid ranges before encoding
 * @see comm_can_transmit() to send the packed message
 */
void pack_mit_command( uint8_t * msg,  float p_des,  float v_des,  float kp,  float kd,  float t_ff) {
    p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
    v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
    kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
    kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
    t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    const uint32_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    const uint32_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    const uint32_t kp_int = float_to_uint(kp, Kp_MIN, Kp_MAX, 12);
    const uint32_t kd_int = float_to_uint(kd, Kd_MIN, Kd_MAX, 12);
    const uint32_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
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


/**
 * @brief Check if a CAN message is a special MIT mode command
 *
 * Compares the provided 8-byte CAN message against known special commands
 * (START_READ_MIT, EXIT_MIT, SET_HOME_MIT). These commands have specific
 * reserved byte patterns and are used for motor mode control rather than
 * continuous motor parameter updates.
 *
 * @param msg Pointer to 8-byte CAN message to check
 *
 * @return Command index if special command found:
 *         - 0: START_READ_MIT (enter MIT mode / read motor state)
 *         - 1: EXIT_MIT (exit MIT mode)
 *         - 2: SET_HOME_MIT (set zero home position)
 *         Return -1 if message is not a recognized special command
 *
 * @note Special commands trigger mode changes and are logged differently
 * @note Used to filter and identify control messages vs. regular motor commands
 * @see MOTOR_CMDS_MIT array for the actual command byte patterns
 * @see pack_mit_command() for standard motor control encoding
 * @see command_to_all_motors() which uses these command indices
 */
int is_special_command(uint8_t * msg){
    for(uint8_t idx =0; idx < NUM_CMDS; idx ++){
        if (memcmp(msg, MOTOR_CMDS_MIT[idx].command, sizeof(msg)) == 0) {
            ESP_LOGI(TAG_CAN, "Special Command found for %u", idx);
            return idx;
        }
    }
    return -1;
}

/**
 * @brief Unpack motor control command from a CAN message
 *
 * Decodes an 8-byte CAN message into motor control parameters following
 * the MIT actuator communication protocol (V1 motor format). Extracts the
 * desired position, velocity, and PD gains from bit-packed integer fields
 * and converts them back to floating-point physical units.
 *
 * Byte layout (V1 motor format currently in use):
 * - Bytes [0:1]: Position command (16 bits)
 * - Bytes [2:3]: Velocity command (12 bits) + KP high (4 bits)
 * - Bytes [3:4]: KP command (12 bits)
 * - Bytes [5:6]: KD command (12 bits) + Torque high (4 bits)
 * - Bytes [6:7]: Torque command (12 bits)
 *
 * Note: V3 motor format is available but commented out. Switch by uncommenting
 * the V3 section if using V3 motors with different byte ordering.
 *
 * @param msg Pointer to 8-byte CAN message from motor controller
 *
 * @return motor_control structure containing:
 *         - k_propotional (kp): Proportional gain [0, 500.0]
 *         - k_derivate (kd): Derivative gain [0, 5.0]
 *         - position: Desired position in radians [-12.5, 12.5]
 *         - velocity: Desired velocity in rad/s [-76.0, 76.0]
 *         - torque: Feedforward torque in Nm [-12.0, 12.0]
 *
 * @note This is the inverse operation of pack_mit_command()
 * @note Currently uses V1 motor byte layout; V3 format available as alternative
 * @see pack_mit_command() for the encoding operation
 * @see float_to_uint() and uint_to_float() for conversion details
 */
const motor_control unpack_command(uint8_t* msg){
    /// unpack ints from can buffer ///
    //FOR V3 MOTOR
    //  const uint32_t kp_int = (msg[0]<<4)|(msg[1]>>4); // KP value command
    //  const uint32_t kd_int = ((msg[1]&0xF)<<8)|msg[2];  // KD value command
    //  const uint32_t pos_int = (msg[3]<<8)|msg[4]; // Motor Position command
    //  const uint32_t vel_int = (msg[5]<<4)|(msg[6]>>4); // Motor Speed command

    //for v1 motor current default
    const uint32_t pos_int = (msg[0]<<8)|msg[1]; // Motor Position command
    const uint32_t vel_int = (msg[2]<<4)|(msg[3]>>4); // Motor Speed command
    const uint32_t kp_int = ((msg[3]&0xF)<<8)|msg[4]; // KP value command
    const uint32_t kd_int = (msg[5]<<4)|(msg[6]>>4);  // KD value command


    const int tor_int = ((msg[6]&0xF)<<8)|msg[7]; //Motor Torque command


     /// convert ints to floats ///
    const float kp = uint_to_float(kp_int, Kp_MIN, Kp_MAX, 12);
    const float kd = uint_to_float(kd_int, Kd_MIN, Kd_MAX, 12);
    const float pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    const float vel = uint_to_float(vel_int, V_MIN, V_MAX, 12);
    const float tor = uint_to_float(tor_int, T_MIN, T_MAX, 12);

    motor_control command_received = {pos, vel, tor, kp, kd};
    return command_received;

}


