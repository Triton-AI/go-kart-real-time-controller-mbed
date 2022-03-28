/**
 * @file comm.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include <chrono>
#include <memory>

#include "comm.hpp"
#include "mbed.h"

namespace tritonai {
namespace gkc {
CommManager::CommManager(GkcPacketSubscriber *sub)
    : Watchable(WATCHDOG_UPDATE_MS, WATCHDOG_MAX_MS),
      factory_(
          std::make_unique<GkcPacketFactory>(sub, GkcPacketUtils::debug_cout)) {
  attach(callback(this, &CommManager::watchdog_callback));
#ifdef COMM_USB_SERIAL
  usb_serial_ = std::make_unique<USBSerial>();
  usb_serial_->attach(this, &CommManager::recv_callback);
#endif

#ifdef COMM_UART_SERIAL
  uart_serial_ = std::make_unique<BufferedSerial>(UART_RX_PIN, UART_TX_PIN, BAUD_RATE);
  uart_serial_thread_.start(mbed::callback(this, &CommManager::recv_callback));
#endif
}

size_t CommManager::send(const GkcPacket &packet) {
  auto to_send = factory_->Send(packet);

#ifdef COMM_USB_SERIAL
#define SERIAL_VAR usb_serial_
#endif
#ifdef COMM_UART_SERIAL
#define SERIAL_VAR uart_serial_
#endif
  if (!SERIAL_VAR->writable()) {
    return 0;
  }
  return SERIAL_VAR->write(to_send->data(), to_send->size());
}

void CommManager::watchdog_callback() {}

void CommManager::recv_callback() {
#ifdef COMM_USB_SERIAL
  static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
  inc_count();
  do {
    auto num_byte_read = usb_serial_->read(buffer.data(), buffer.size());
    if (num_byte_read > 0) {
      factory_->Receive(
          GkcBuffer(buffer.begin(), buffer.begin() + num_byte_read));
    }
  } while (usb_serial_->available());
#endif

#ifdef COMM_UART_SERIAL
  static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
  static auto wait_time = std::chrono::milliseconds(WAIT_READ_MS);
  Timer sleep_timer;
  while (!ThisThread::flags_get()) {
    inc_count();
    sleep_timer.start();
    if (uart_serial_->readable()) {
      auto num_byte_read = uart_serial_->read(buffer.data(), buffer.size());
      if (num_byte_read > 0) {
        factory_->Receive(
            GkcBuffer(buffer.begin(), buffer.end() + num_byte_read));
      }
    }
    sleep_timer.stop();
    auto sleep_time = sleep_timer.elapsed_time() - wait_time;
    if (sleep_time > std::chrono::milliseconds(0)) {
      ThisThread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time));
    }
    // TODO(haoru): log the number of frequence compromises (sleep_time >
    // wait_time)
  }
#endif
}

} // namespace gkc
} // namespace tritonai
