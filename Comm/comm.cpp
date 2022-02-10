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

#include "comm.hpp"
#include "mbed.h"
#include <memory>

namespace tritonai {
namespace gkc {
CommManager::CommManager(GkcPacketSubscriber *sub)
    : Watchable(WATCHDOG_UPDATE_MS, WATCHDOG_MAX_MS),
      factory_(
          std::make_unique<GkcPacketFactory>(sub, GkcPacketUtils::debug_cout)) {
#ifdef COMM_USB_SERIAL
  usb_serial_ = std::make_unique<USBSerial>();
#endif
}

size_t CommManager::send(const GkcPacket &packet) {
  auto to_send = factory_->Send(packet);

#ifdef COMM_USB_SERIAL
  if (!usb_serial_->writable()) {
    return 0;
  }
  return usb_serial_->write(to_send->data(), to_send->size());
#endif
}

void CommManager::watchdog_callback() {}

void CommManager::recv_callback() {
  static auto buffer = GkcBuffer(512, 0);
  inc_count();
#ifdef COMM_USB_SERIAL
  do {
    auto num_byte_read = usb_serial_->read(buffer.data(), buffer.size());
    if (num_byte_read > 0) {
      factory_->Receive(
          GkcBuffer(buffer.begin(), buffer.begin() + num_byte_read));
    }
  } while (usb_serial_->available());
#endif
}

} // namespace gkc
} // namespace tritonai
