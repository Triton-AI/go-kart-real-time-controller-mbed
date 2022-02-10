/**
 * @file comm.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef COMM_HPP_
#define COMM_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "mbed.h"
#include "USBSerial.h"

#include "watchable.hpp"

#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

// Choose a comm interface
#define COMM_USB_SERIAL
// #define COMM_UART_SERIAL
// #define COMM_ETHERNET
// #define COMM_CAN

namespace tritonai {
namespace gkc {
class CommManager : public Watchable {
public:
  static constexpr uint32_t WATCHDOG_UPDATE_MS = 100;
  static constexpr uint32_t WATCHDOG_MAX_MS = 500;

  explicit CommManager(GkcPacketSubscriber *sub);
  size_t send(const GkcPacket &packet);

  // Watchable interface
  void watchdog_callback();

protected:
  std::unique_ptr<GkcPacketFactory> factory_;

#ifdef COMM_USB_SERIAL
  std::unique_ptr<USBSerial> usb_serial_;
#endif

  void recv_callback();
};
} // namespace gkc
} // namespace tritonai

#endif // COMM_HPP_