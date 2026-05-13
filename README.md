### Middleware MCU SLCAN~UART/CAN for ESP32

The following project converts a ESP32 as a bridge for communicate between device with UART Peripheral following the SLCAN format with a set of CAN Nodes, specifically CubeMars Motors usin CAN BUS Peripheral. Applications for control robotics.

### Target Devices
| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

# SLCAN

### Supported commands

- `O[CR]` - Open the CAN channel
- `C[CR]` - Close the CAN channel
- `tiiildd...[CR]` - Transmit a standard (11bit) CAN frame (`iii` = Id., `l` = DLC, `dd` = Data)

Note: Channel configuration commands must be sent before opening the channel. The channel must be opened before transmitting frames.

### CANable SLCAN Protocol (Option 1)

Supported commands

- `O` - Open channel
- `C` - Close channel
- `tIIILDD...` - Transmit data frame (Standard ID) [ID, length, data]

Note: Channel configuration commands must be sent before opening the channel. The channel must be opened before transmitting frames.

**Note: The firmware currently does not provide any ACK/NACK feedback for serial commands.**

Note: The implementation currently does not support CAN FD commands and frame format.

## SLCAN Functions

```C
const motor_state unpack_reply(uint8_t* msg);
const slcan_frame_list_t* receive_slcan(uint8_t *uart_buffer, size_t max_len_uart);
void transmit_slcan(const motor_state info_motor);
void print_UART_status();
void uart_init();
```

## CAN functions

```C
void comm_can_transmit(const uint32_t driver_id, const uint8_t *data);
void can_mit_mode_init();
void print_CAN_status();
void command_to_all_motors(int action);
void pack_mit_command( uint8_t * msg,  float p_des,  float v_des,  float kp,  float kd,  float t_ff);
```

### Development tools

Arduino IDE 2.3.8 or ESP-IDF(requires file configuration)

#### Arduino IDE

Verify the corresponding board configuration.

Important to setup esp-idf packages and boards via
Preferences-> Additional Boards Manager URL

and paste

```sh
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

Verify and upload code

### ESP-IDF

### Configure the project

* Set the target of the build (where `{IDF_TARGET}` stands for the target chip such as `esp32` or `esp32s2`).
* Then run `menuconfig` to configure the example.

```sh
idf.py set-target {IDF_TARGET}
idf.py menuconfig
```

* Under `Example Configuration`, configure the pin assignments using the options `TX GPIO Number` and `RX GPIO Number` according to how the target was connected to the transceiver. By default, `TX GPIO Number` and `RX GPIO Number` are set to the following values:
  * On the ESP32, `TX GPIO Number` and `RX GPIO Number` default to `21` and `22` respectively
  * On other chips, `TX GPIO Number` and `RX GPIO Number` default to `0` and `2` respectively

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```sh
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Hardware Required

This project requires only a single target (e.g., an ESP32 or ESP32-S2) and a device with Uart Serial port.
The target must be connected to:
- An external transceiver (e.g., a TJA1051T/3 transceiver). This connection usually consists of a TX and, RX signal and GND. See can_mit_mode.h to set them.

- An external Uart Serial port given the UART Pins used. See uart_slcan.h to set them.


### Troubleshooting & Testing Devices

These devices help to depurate errors and check frames:

USB-CAN Adapter and use of cangaroo CAN BUS analyzer software https://github.com/Schildkroet/CANgaroo

USB-TTL adapter and use of any Serial Monitor with line endings option
eg: VSCODE Serial Monitor https://marketplace.visualstudio.com/items?itemName=ms-vscode.vscode-serial-monitor

### Example Log Output from target

### Video Demonstration


### Contact

E-Mail: jlcrespoe@unal.edu.co
