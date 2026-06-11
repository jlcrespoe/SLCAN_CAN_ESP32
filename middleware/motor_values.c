#include <stdint.h>
#include "motor_values.h"


// Index 0: Standard Frame Constants

const motor_config_t MOTOR_SPECS[2] = {
    [0] = { // --- STANDARD FRAME (AK80-6: Completely Symmetric) ---
        // Sending
        .p_min_send = -12.5f,  .p_max_send = 12.5f, // radians
        .v_min_send = -76.0f,  .v_max_send = 76.0f, // rad/s
        .t_min_send = -12.0f,  .t_max_send = 12.0f, // N*m
        // Receiving (Identical to sending for standard MIT mode)
        .p_min_recv = -12.5f,  .p_max_recv = 12.5f,
        .v_min_recv = -76.0f,  .v_max_recv = 76.0f,
        .t_min_recv = -12.0f,  .t_max_recv = 12.0f,
        // Tunings & Status
        .kp_min     = 0.0f,    .kp_max     = 500.0f,
        .kd_min     = 0.0f,    .kd_max     = 5.0f,
        .c_min      = -40.0f,  .c_max      = 215.0f, //celcius
        .i_min      = -60.0f,  .i_max      = 60.0f // Amperes(A)
    },
    [1] = { // --- EXTENDED FRAME (AK60-6: Asymmetric Engineering Bounds) ---
        // Sending (Software Control Boundaries)
        .p_min_send = -12.56f, .p_max_send = 12.56f, 
        .v_min_send = -60.0f,  .v_max_send = 60.0f, 
        .t_min_send = -12.0f,  .t_max_send = 12.0f,
        // Receiving (Hardware Telemetry Window converted to SI units)
        .p_min_recv = -55.8505f, .p_max_recv = 55.8505f, // +/- 3200 degrees
        .v_min_recv = -2393.595f,.v_max_recv = 2393.595f,// +/- 320000 rpm
        .t_min_recv = -12.0f,    .t_max_recv = 12.0f,
        // Tunings & Status
        .kp_min     = 0.0f,    .kp_max     = 500.0f,
        .kd_min     = 0.0f,    .kd_max     = 5.0f,
        .c_min      = -20.0f,  .c_max      = 127.0f,
        .i_min      = -60.0f,  .i_max      = 60.0f
    }
};

/**
 * @brief Convert a floating-point value to an unsigned integer with scaling
 *
 * Performs linear mapping from a floating-point range [x_min, x_max] to an
 * unsigned integer range [0, 2^bits - 1]. This is used to encode motor control
 * parameters (position, velocity, torque, etc.) into fixed-width integer formats
 * for CAN message transmission.
 *
 * @param x The floating-point value to convert
 * @param x_min Minimum value of the floating-point range
 * @param x_max Maximum value of the floating-point range
 * @param bits Number of bits to use for the unsigned integer (e.g., 8, 12, 16)
 *
 * @return Unsigned integer representation of x in the range [0, 2^bits - 1]
 *
 * @note Input x is clamped to [x_min, x_max] before conversion
 * @see uint_to_float() for the inverse operation
 */
uint32_t float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    uint32_t result = (uint32_t) ((x- x_min)*((float)((1U<<bits)/span)));
    if (result >= (1U << bits)) result = (1U << bits) - 1;  // prevent overflow
    return result;
}

/**
 * @brief Convert an unsigned integer back to a floating-point value with scaling
 *
 * Performs the inverse of float_to_uint(), mapping from an unsigned integer
 * [0, 2^bits - 1] back to the floating-point range [x_min, x_max].
 * Used to decode motor state parameters received from CAN messages.
 *
 * @param x_int The unsigned integer value to convert
 * @param x_min Minimum value of the floating-point range
 * @param x_max Maximum value of the floating-point range
 * @param bits Number of bits used in the unsigned integer representation
 *
 * @return Floating-point value corresponding to x_int in the range [x_min, x_max]
 *
 * @see float_to_uint() for the forward conversion
 */
float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1U << bits)-1)) + offset;
}