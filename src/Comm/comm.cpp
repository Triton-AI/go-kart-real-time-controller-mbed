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
#include <ratio>
#include <string>

#include "Kernel.h"
#include "comm.hpp"
#include "mbed.h"

namespace tritonai {
namespace gkc {
CommManager::CommManager(GkcPacketSubscriber *sub)
    : Watchable(DEFAULT_COMM_POLL_INTERVAL_MS, DEFAULT_COMM_POLL_LOST_TOLERANCE_MS, "CommManager"),
      factory_(
          std::make_unique<GkcPacketFactory>(sub, GkcPacketUtils::debug_cout)) {
  attach(callback(this, &CommManager::watchdog_callback));
#ifdef COMM_USB_SERIAL
  usb_serial_ = std::make_unique<USBSerial>();
  usb_serial_->attach(this, &CommManager::recv_callback);
#endif

#ifdef COMM_UART_SERIAL
  uart_serial_ =
      std::make_unique<BufferedSerial>(UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
  uart_serial_thread_.start(mbed::callback(this, &CommManager::recv_callback));
#endif

  send_thread.start(callback(this, &CommManager::send_thread_impl));
}

void CommManager::send(const GkcPacket &packet) {
  auto to_send = factory_->Send(packet);
  if (send_queue_.try_put(to_send.get())) {
    send_queue_data_.push(to_send);
  }
}

size_t CommManager::send_impl(const GkcBuffer &buffer) {
#ifdef COMM_USB_SERIAL
#define SERIAL_VAR usb_serial_
#endif
#ifdef COMM_UART_SERIAL
#define SERIAL_VAR uart_serial_
#endif
  if (!SERIAL_VAR->writable()) {
    return 0;
  }
  return SERIAL_VAR->write(buffer.data(), buffer.size());
}

void CommManager::watchdog_callback() {
  std::cout << "CommManager Timeout detected" << std::endl;
  NVIC_SystemReset();
}

void CommManager::recv_callback() {
#ifdef COMM_USB_SERIAL
  static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
  inc_count();
  do {
    auto num_byte_read = usb_serial_->read(buffer.data(), buffer.size());
    if (num_byte_read > 0) {
      factory_->Receive(RawGkcBuffer{buffer.data(), buffer.size()});
    }
  } while (usb_serial_->available());
#endif

#ifdef COMM_UART_SERIAL
  // std::cout << "Starting comm receive" << std::endl;
  static auto buffer = GkcBuffer(RECV_BUFFER_SIZE, 0);
  static auto wait_time = std::chrono::milliseconds(WAIT_READ_MS);
  while (!ThisThread::flags_get()) {
    inc_count();
    if (uart_serial_->readable()) {
      auto num_byte_read = uart_serial_->read(buffer.data(), buffer.size());
      if (num_byte_read > 0) {
        RawGkcBuffer buff;
        buff.data = buffer.data();
        buff.size = num_byte_read;

        // std::cout << "Got something of size " << num_byte_read << "\n";
        // cout << "Data: ";

        // for (int i = 0; i < num_byte_read; i++) {
        //   cout << hex << static_cast<int>(buffer[i]) << " ";
        // }

        // cout << endl;

        factory_->Receive(buff);
      }
    }
    // TODO(haoru): log the number of frequence compromises (sleep_time >
    // wait_time)
  }

#endif
}

void CommManager::send_thread_impl() {
  while (!ThisThread::flags_get()) {
    GkcBuffer *buf_to_send;
    send_queue_.try_get_for(Kernel::wait_for_u32_forever, &buf_to_send);
    send_impl(*buf_to_send);
    send_queue_data_.pop();
  }
}
} // namespace gkc
} // namespace tritonai
