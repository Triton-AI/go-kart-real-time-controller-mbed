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
#include <queue>

#include "BufferedSerial.h"
#include "USBSerial.h"
#include "mbed.h"

#include "config.hpp"
#include "Watchdog/watchable.hpp"

#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

// Choose a comm interface in config.hpp

namespace tritonai {
namespace gkc {
class CommManager : public Watchable {
public:
  explicit CommManager(GkcPacketSubscriber *sub);
  void send(const GkcPacket &packet);

protected:
  std::unique_ptr<GkcPacketFactory> factory_;
  Queue<GkcBuffer, SEND_QUEUE_SIZE> send_queue_;
  std::queue<std::shared_ptr<GkcBuffer>> send_queue_data_;
  Thread send_thread;
#ifdef COMM_USB_SERIAL
  std::unique_ptr<USBSerial> usb_serial_;
#endif

#ifdef COMM_UART_SERIAL
  std::unique_ptr<BufferedSerial> uart_serial_;
  Thread uart_serial_thread_;
#endif

  void recv_callback();
  void watchdog_callback();
  void send_thread_impl();
  size_t send_impl(const GkcBuffer &buffer);
};
} // namespace gkc
} // namespace tritonai

#endif // COMM_HPP_