/**
 * @file sensor_reader.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-03
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "sensor_reader.hpp"
#include "ThisThread.h"
#include "config.hpp"
#include "watchdog.hpp"
#include <cstdio>
#include <string>

namespace tritonai {
namespace gkc {
SensorReader::SensorReader()
    : Watchable(DEFAULT_SENSOR_POLL_INTERVAL_MS,
                DEFAULT_SENSOR_POLL_LOST_TOLERANCE_MS) {
  std::cout << "Initializing sensor" << std::endl;
  sensor_poll_thread.start(
      callback(this, &SensorReader::sensor_poll_thread_impl));
  std::cout << "Sensor initialized" << std::endl;
}

void SensorReader::sensor_poll_thread_impl() {
  while (!ThisThread::flags_get()) {
    providers_lock_.lock();
    for (auto &provider : providers_) {
      if (provider->is_ready()) {
        provider->populate_reading(pkt_);
      }
    }
    providers_lock_.unlock();
    ThisThread::sleep_for(poll_interval_);
  }
}

void SensorReader::register_provider(ISensorProvider *provider) {
  providers_lock_.lock();
  providers_.push_back(provider);
  providers_lock_.unlock();
}

void SensorReader::remove_provider(ISensorProvider *provider) {
  providers_lock_.lock();
  providers_.erase(std::remove(providers_.begin(), providers_.end(), provider),
                   providers_.end());
  providers_lock_.unlock();
}
} // namespace gkc
} // namespace tritonai