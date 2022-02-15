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
#define COMM_USB_SERIAL
//#define COMM_UART_SERIAL
//#define COMM_ETHERNET
//#define COMM_CAN

// *********
// Watchdogs
// *********
// What's the update frequency of the watchdog
#define DEFAULT_WD_INTERVAL_MS 2
#define DEFAULT_WD_MAX_INACTIVITY_MS 100
// How often should the MCU send heartbeat by default
#define DEFAULT_MCU_HEARTBEAT_INTERVAL_MS 100
#define DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS 1000
// How often should the MCU expect heartbeat from PC by default
#define DEFAULT_PC_HEARTBEAT_INTERVAL_MS 100
#define DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS 1000
// In active mode, how often should the MCU expect control command from PC
#define DEFAULT_CTL_CMD_INTERVAL_MS 10
#define DEFAULT_CTL_CMD_LOST_TOLERANCE_MS 200
#endif // CONFIG_HPP_