/**
 * @file actuation_controller.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-02
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef ACTUATION_CONTROLLER_HPP_
#define ACTUATION_CONTROLLER_HPP_

#include "Queue.h"
#include "mbed.h"
#include "watchable.hpp"
#include "watchdog.hpp"

namespace tritonai {
namespace gkc {
class ActuationController : public Watchable {
public:
  ActuationController();

  float get_throttle_cmd() const { return current_throttle_cmd; };
  float get_steering_cmd() const { return current_steering_cmd; };
  float get_brake_cmd() const { return current_brake_cmd; };

  bool set_throttle_cmd(float *cmd) { return throttle_cmd_queue.try_put(cmd); };
  bool set_steering_cmd(float *cmd) { return steering_cmd_queue.try_put(cmd); };
  bool set_brake_cmd(float *cmd) { return brake_cmd_queue.try_put(cmd); };

protected:
  Queue<float, 10> throttle_cmd_queue;
  Queue<float, 10> steering_cmd_queue;
  Queue<float, 10> brake_cmd_queue;

  float current_throttle_cmd {0.0};
  float current_steering_cmd {0.0};
  float current_brake_cmd {0.0};

  Thread throttle_thread;
  Thread steering_thread;
  Thread brake_thread;

  void throttle_thread_impl();
  void steering_thread_impl();
  void brake_thread_impl();
};
} // namespace gkc
} // namespace tritonai
#endif // ACTUATION_CONTROLLER_HPP_