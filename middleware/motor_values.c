#include "motor_values.h"

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
int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x- x_min)*((float)((1<<bits)/span)));
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
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}