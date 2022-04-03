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
#include <cmath>
#include <cstdint>

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

template <typename T> T non_linear_map(const T &val) {
  return (exp(val * val * val) - 1) / (exp(1) - 1);
}

template <typename S, typename D>
D map_range(const S &source, const S &source_min, const S &source_max,
            const D &dest_min, const D &dest_max) {
  float source_f = clamp<S>(source_min, source_max, source);
  source_f = (source_f - source_min) / (source_max - source_min);
  return static_cast<D>(source_f * (dest_max - dest_min) + dest_min);
}

PwmOut throttle_pin(THROTTLE_PWM_PIN);
CAN can1(CAN1_RX, CAN1_TX, 500000);

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
    throttle_pin.write(non_linear_map(current_throttle_cmd));
  }
}

void ActuationController::steering_thread_impl() {}

void ActuationController::brake_thread_impl() {
  // CAN frame format
  // |       Byte 1      |           Byte 2            |
  // | 8 LSB of position | CE | ME | 5 MSB of position |
  //
  // CE: enable actuator clutch (free move vs. motor engaged)
  // ME: enable actuator motor
  // 5 MSB of position in byte 2 occupies its 5 LSB
  // Position has a total of 8 + 5 = 13 bits (0 - 8192). 1000 = 1 inch
  static unsigned char message[8] = {0x0F, 0x4A, 0x00, 0xC0, 0, 0, 0, 0};
  float *cmd;
  uint16_t brake_output = 0;

  while (!ThisThread::flags_get()) {
    brake_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    current_brake_cmd = clamp<float>(0.0, 1.0, *cmd);
    delete cmd;
    brake_output = map_range<float, uint16_t>(current_brake_cmd, 0.0, 1.0,
                                              MIN_BRAKE_VAL, MAX_BRAKE_VAL);
    message[2] = brake_output & 0xFF;
    message[3] = 0xC0 | ((brake_output >> 8) & 0x1F);
    can1.write(CANMessage(0x00FF0000, message, 8, CANData, CANExtended));
  }
}
} // namespace gkc
} // namespace tritonai