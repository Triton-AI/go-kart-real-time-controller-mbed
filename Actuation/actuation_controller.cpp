/**
 * @file actuation_controller.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-02
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "actuation_controller.hpp"
#include "Kernel.h"
#include "ThisThread.h"
#include "config.hpp"
#include "watchdog.hpp"

namespace tritonai {
namespace gkc {

template <typename T> T clamp(const T &min, const T &max, const T &val) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

PwmOut throttle_pin(THROTTLE_PWM_PIN);

ActuationController::ActuationController()
    : Watchable(DEFAULT_ACTUATION_INTERVAL_MS,
                DEFAULT_ACTUATION_LOST_TOLERANCE_MS) {
  throttle_thread.start(
      callback(this, &ActuationController::throttle_thread_impl));
  steering_thread.start(
      callback(this, &ActuationController::steering_thread_impl));
  brake_thread.start(callback(this, &ActuationController::brake_thread_impl));
}

void ActuationController::throttle_thread_impl() {
  throttle_pin.period(0.0001f);

  float *cmd;
  while (!ThisThread::flags_get()) {
    throttle_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    current_throttle_cmd = clamp<float>(0.0, 1.0, *cmd);
    delete cmd;
    throttle_pin.write(current_throttle_cmd);
  }
}

void ActuationController::steering_thread_impl() {}

void ActuationController::brake_thread_impl() {}
} // namespace gkc
} // namespace tritonai