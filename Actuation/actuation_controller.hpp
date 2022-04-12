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

#include "Mutex.h"
#include "Queue.h"
#include "logger.hpp"
#include "mbed.h"
#include "pid_controller.hpp"
#include "sensor_reader.hpp"
#include "watchable.hpp"
#include "watchdog.hpp"

namespace tritonai {
namespace gkc {
struct ActuationSensors {
  float steering_rad = 0.0;
  float brake_psi = 0.0;
  float fl_rad = 0.0;
  float fr_rad = 0.0;
  float rl_rad = 0.0;
  float rr_rad = 0.0;
  int32_t steering_output = 0;
};

class ActuationController : public Watchable, public ISensorProvider {
public:
  explicit ActuationController(ILogger *logger);

  float get_throttle_cmd() const { return current_throttle_cmd; };
  float get_steering_cmd() const { return current_steering_cmd; };
  float get_brake_cmd() const { return current_brake_cmd; };

  bool set_throttle_cmd(float *cmd) { return throttle_cmd_queue.try_put(cmd); };
  bool set_steering_cmd(float *cmd) { return steering_cmd_queue.try_put(cmd); };
  bool set_brake_cmd(float *cmd) { return brake_cmd_queue.try_put(cmd); };

  bool is_ready();
  void populate_reading(SensorGkcPacket &pkt);

protected:
  Queue<float, 10> throttle_cmd_queue;
  Queue<float, 10> steering_cmd_queue;
  Queue<float, 10> brake_cmd_queue;

  float current_throttle_cmd{0.0};
  float current_steering_cmd{0.0};
  float current_brake_cmd{0.0};

  Thread throttle_thread;
  Thread steering_thread;
  Thread steering_pid_thread;
  Thread brake_thread;
  Thread sensor_poll_thread;

  ActuationSensors sensors{};

  void throttle_thread_impl();
  void steering_thread_impl();
  void steering_pid_thread_impl();
  void brake_thread_impl();
  void sensor_poll_thread_impl();

  PidController steering_pid;
  ILogger *logger;
};
} // namespace gkc
} // namespace tritonai
#endif // ACTUATION_CONTROLLER_HPP_