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
  int steering_wraps = 0;
  float steering_speed = 0.0;
  bool hasSwitched = false;
  int32_t steering_output = 0;
};

/**
 * @brief
 * 
 * The ActuationController class is responsible for controlling the actuators of the autonomous kart. It provides an interface to control the throttle, steering, and brake by setting the desired commands using the set_throttle_cmd(), set_steering_cmd(), and set_brake_cmd() functions. The current values of the commands can be obtained using the get_throttle_cmd(), get_steering_cmd(), and get_brake_cmd() functions.
 *
 * The ActuationController class inherits from two other classes, Watchable and ISensorProvider. Watchable implements things for the watchdog, but I think it is only partly implemented. ISensorProvider requires the implementation of the populate_reading() function, which is used to populate a SensorGkcPacket with sensor information.
 *
 * The class uses three queues, one for each actuator, to store the desired commands. The throttle_thread_impl(), steering_thread_impl(), steering_pid_thread_impl(), and brake_thread_impl() functions are executed continuously and in parallel in different threads. These functions retrieve the commands from the queues and transform them into commands for the actuators.
 *
 * The ActuationController class also includes a sensor_poll_thread_impl() function that retrieves sensor data and stores it in the ActuationSensors object. This data can then be used in the populate_reading() function to populate a SensorGkcPacket with the sensor information.
 *
 * The ActuationController class includes a PidController object for the steering actuator, which can be used to implement a PID loop to achieve precise control of the steering.
 *
 * Finally, the ActuationController class includes an ILogger object that is used for logging.
 */


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
  void steer_speed_pid_thread_impl();
  void brake_thread_impl();
  void sensor_poll_thread_impl();

  PidController steering_pid;
  PidController steering_vel_pid;
  ILogger *logger;
};
} // namespace gkc
} // namespace tritonai
#endif // ACTUATION_CONTROLLER_HPP_