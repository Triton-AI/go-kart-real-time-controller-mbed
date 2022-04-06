/**
 * @file sensor_reader.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-03
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef SENSOR_READER_HPP_
#define SENSOR_READER_HPP_
#include "Mutex.h"
#include "config.hpp"
#include "gkc_packets.hpp"
#include "watchable.hpp"
#include <chrono>
#include <cstdint>
#include <vector>

namespace tritonai {
namespace gkc {

class ISensorProvider {
public:
  ISensorProvider() {}
  virtual bool is_ready() = 0;
  virtual void populate_reading(SensorGkcPacket &pkt) = 0;
};

class SensorReader : Watchable {
public:
  SensorReader();
  void register_provider(ISensorProvider *provider);
  void remove_provider(ISensorProvider *provider);
  const SensorGkcPacket &get_packet() const { return pkt_; }
  void set_poll_interval(std::chrono::milliseconds val) {
    poll_interval_ = val;
  }
  std::chrono::milliseconds get_poll_interval() { return poll_interval_; }

protected:
  SensorGkcPacket pkt_{};
  std::vector<ISensorProvider *> providers_{};
  Mutex providers_lock_;
  std::chrono::milliseconds poll_interval_{DEFAULT_SENSOR_POLL_INTERVAL_MS};

  Thread sensor_poll_thread;
  void sensor_poll_thread_impl();

  // TODO include QEI.h and add a pointer to qei
};
} // namespace gkc
} // namespace tritonai

#endif // SENSOR_READER_HPP_