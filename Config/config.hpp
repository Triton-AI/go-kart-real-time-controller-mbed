/**
 * @file config.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-11
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

// *************
// Build Options
// *************

// *************
// Communication
// *************
// Choose one of the available interfaces
//#define COMM_USB_SERIAL
#define COMM_UART_SERIAL
//#define COMM_ETHERNET  // not implemented
//#define COMM_CAN  // not implemented

// UART-specific settings
#define BAUD_RATE 115200
//For the rs232 connector
#define UART_RX_PIN PD_2
#define UART_TX_PIN PC_12
//For the radio
//#define UART_RX_PIN PE_7
//#define UART_TX_PIN PE_8

// Generic comm settings
#define RECV_BUFFER_SIZE 32
// millisecond to wait between each serial/ethernet/can read
#define WAIT_READ_MS 5
// outbound packet queue size
#define SEND_QUEUE_SIZE 10
// interval of sending sensor packets
#define SEND_SENSOR_INTERVAL_MS 50

// *********
// Watchdogs
// *********
// What's the update frequency of the watchdog
#define DEFAULT_WD_INTERVAL_MS 2
#define DEFAULT_WD_MAX_INACTIVITY_MS 100
// How often should the MCU send heartbeat by default
#define DEFAULT_MCU_HEARTBEAT_INTERVAL_MS 1000
#define DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS 2000
// How often should the MCU expect heartbeat from PC by default
#define DEFAULT_PC_HEARTBEAT_INTERVAL_MS 1000
#define DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS 2000
// In active mode, how often should the MCU expect control command from PC
#define DEFAULT_CTL_CMD_INTERVAL_MS 10
#define DEFAULT_CTL_CMD_LOST_TOLERANCE_MS 200
// How often should actuation be active
#define DEFAULT_ACTUATION_INTERVAL_MS 1000
#define DEFAULT_ACTUATION_LOST_TOLERANCE_MS 2000
// How often should sensor polling happen
#define DEFAULT_SENSOR_POLL_INTERVAL_MS 10
#define DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS 100

// *********
// Actuation
// *********
// Throttle
#define THROTTLE_PWM_PIN PA_5
#define CAN_THROTTLE CAN_2
#define VESC_THROTTLE_ID 1
// Braking
#define CAN1_RX PD_0
#define CAN1_TX PD_1
#define CAN1_BAUDRATE 500000
#define CAN_STEER CAN_1 // Which CAN bus to use for steering [CAN_1 | CAN_2]
#define MAX_BRAKE_VAL 2500
#define MIN_BRAKE_VAL 1500
// Steering
#define CAN2_RX PB_5
#define CAN2_TX PB_6
#define CAN2_BAUDRATE 500000
#define CAN_BRAKE CAN_2 // Which CAN bus to use for brake [CAN_1 | CAN_2]
#define MAX_STEER_DEG 221
#define MIN_STEER_DEG 118
#define NEUTRAL_STEER_DEG 180
#define MAX_STEER_SPEED_ERPM 1000.0
#define MAX_CURRENT_MA 3000.0
#define STEER_P 20.0
#define STEER_I 0.0
#define STEER_D 1.0
#define STEER_DEADBAND_DEG 0.5
#define PID_INTERVAL_MS 10
#define VESC_STEERING_ID 2

// *******
// Sensors
// *******
// PWM steering encoder
#define STEER_ENCODER_PIN PC_7
//test
// *****
// ESTOP
// *****
#define ESTOP_PIN PD_8

#endif // CONFIG_HPP_
