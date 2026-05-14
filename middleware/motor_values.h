#ifndef MOTOR_VALUES_H
#define MOTOR_VALUES_H


#ifdef __cplusplus
extern "C" {          // ← tells C++ linker: look for plain C names
#endif

// ========================================================================
// MIT MODE CONTROL COMMUNICATION PROTOCOL CONSTANTS FOR CUBERMARS MOTOR AK80-6
// check documentation at https://www.cubemars.com/images/file/20240102/1704163364483999.pdf
// ========================================================================
/// limit data to be within bounds ///
#define P_MAX  12.5f
#define P_MIN  -P_MAX
#define V_MAX  76.0f
#define V_MIN  -V_MAX
#define T_MAX  12.0f
#define T_MIN  -T_MAX
#define C_MAX 215
#define C_MIN -40
#define Kp_MIN  0
#define Kp_MAX  500.0f
#define Kd_MIN  0
#define Kd_MAX  5.0f
#define Test_Pos 0.0f
#define I_MAX 60
#define I_MIN -I_MAX

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);


#ifdef __cplusplus
}           // ← closes extern "C" {
#endif

#endif // MOTOR_VALUES_H