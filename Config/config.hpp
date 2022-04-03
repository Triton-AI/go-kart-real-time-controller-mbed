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
#define UART_RX_PIN PC_12
#define UART_TX_PIN PD_2

// Generic comm settings
#define RECV_BUFFER_SIZE 32
// millisecond to wait between each serial/ethernet/can read
#define WAIT_READ_MS 5
#define SEND_QUEUE_SIZE 10

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

// *********
// Actuation
// *********
#define THROTTLE_PWM_PIN PA_5
#define CAN1_RX PD_0
#define CAN1_TX PD_1
#define MAX_BRAKE_VAL 2500
#define MIN_BRAKE_VAL 1500

#endif // CONFIG_HPP_