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
#include "PwmIn.h"
#include "ThisThread.h"
#include "Thread.h"
#include "config.hpp"
#include "global_profilers.hpp"
#include "watchdog.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

#define M_PI 3.14159265

namespace tritonai {
namespace gkc {

template <typename T>
constexpr T clamp(const T &min, const T &max, const T &val) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

template <typename T> constexpr T non_linear_map(const T &val) {
  return (exp(val * val * val) - 1) / (exp(1) - 1);
}

template <typename S, typename D>
constexpr D map_range(const S &source, const S &source_min, const S &source_max,
                      const D &dest_min, const D &dest_max) {
  float source_f = clamp<S>(source_min, source_max, source);
  source_f = (source_f - source_min) / (source_max - source_min);
  return static_cast<D>(source_f * (dest_max - dest_min) + dest_min);
}

template <typename S, typename D> constexpr D deg_to_rad(const S &deg) {
  return static_cast<D>(deg * M_PI / 180.0);
}

bool ActuationController::is_ready() {
  //  return sensor_poll_thread.get_state() == Thread::Running ||
  //         sensor_poll_thread.get_state() == Thread::WaitingDelay;
  return false;
}

void ActuationController::populate_reading(SensorGkcPacket &pkt) {
  pkt.values.steering_angle_rad = sensors.steering_rad;
  pkt.values.brake_pressure = sensors.brake_psi;
  pkt.values.wheel_speed_fl = sensors.fl_rad;
  pkt.values.wheel_speed_fr = sensors.fr_rad;
  pkt.values.wheel_speed_rl = sensors.rl_rad;
  pkt.values.wheel_speed_rr = sensors.rr_rad;
  pkt.values.servo_angle_rad = static_cast<float>(sensors.steering_output);
}

PwmOut throttle_pin(THROTTLE_PWM_PIN);
PwmIn steer_encoder(STEER_ENCODER_PIN);
CAN CAN_1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);
CAN CAN_2(CAN2_RX, CAN2_TX, CAN2_BAUDRATE);

PidCoefficients steering_pid_coeff{STEER_P,
                                   STEER_I,
                                   STEER_D,
                                   -MAX_STEER_SPEED_ERPM,
                                   MAX_STEER_SPEED_ERPM,
                                   -MAX_STEER_SPEED_ERPM,
                                   MAX_STEER_SPEED_ERPM};

ActuationController::ActuationController(ILogger *logger)
    : Watchable(DEFAULT_ACTUATION_INTERVAL_MS,
                DEFAULT_ACTUATION_LOST_TOLERANCE_MS),
      ISensorProvider(),
      current_steering_cmd(deg_to_rad<int32_t, float>(NEUTRAL_STEER_DEG)),
      steering_pid("steering", steering_pid_coeff), logger(logger) {
  std::cout << "Initializing actuation" << std::endl;
  throttle_thread.start(
      callback(this, &ActuationController::throttle_thread_impl));
  steering_thread.start(
      callback(this, &ActuationController::steering_thread_impl));
  steering_pid_thread.start(
      callback(this, &ActuationController::steering_pid_thread_impl));
  brake_thread.start(callback(this, &ActuationController::brake_thread_impl));
  sensor_poll_thread.start(
      callback(this, &ActuationController::sensor_poll_thread_impl));
  std::cout << "Actuation initialized" << std::endl;
}

void ActuationController::throttle_thread_impl() {
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  throttle_pin.period(0.0001f);
  float *cmd;

  while (!ThisThread::flags_get()) {
    throttle_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    current_throttle_cmd = clamp<float>(0.0, 1.0, *cmd);
    delete cmd;
    throttle_pin.write(non_linear_map(current_throttle_cmd));
  }
}

void ActuationController::steering_pid_thread_impl() {
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  static constexpr std::chrono::milliseconds pid_interval(
      static_cast<uint32_t>(PID_INTERVAL_MS));
#define RPM_EXTENDED_ID 0x03
#define CURRENT_EXTENDED_ID 0x01
  static constexpr uint32_t rpm_id =
      (static_cast<uint32_t>(RPM_EXTENDED_ID) << sizeof(uint8_t) * 8) |
      static_cast<uint32_t>(VESC_ID);
  static constexpr uint32_t current_id =
      (static_cast<uint32_t>(CURRENT_EXTENDED_ID) << sizeof(uint8_t) * 8) |
      static_cast<uint32_t>(VESC_ID);

  while (!ThisThread::flags_get()) {
    sensors.steering_output = static_cast<int32_t>(steering_pid.update(
        current_steering_cmd - sensors.steering_rad, PID_INTERVAL_MS / 1000.0));
    if (abs(sensors.steering_rad - current_steering_cmd) <
        deg_to_rad<float, float>(STEER_DEADBAND_DEG)) {
      steering_pid.reset_integral_error(0.0);
      sensors.steering_output = 0;
    }
    auto data = sensors.steering_output;
    bool write_success = CAN_STEER.write(CANMessage(
        rpm_id, reinterpret_cast<uint8_t *>(&data), 4, CANData, CANExtended));
    ThisThread::sleep_for(pid_interval);
  }
}

void ActuationController::steering_thread_impl() {
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  float *cmd;
  while (!ThisThread::flags_get()) {
    steering_cmd_queue.try_get_for(Kernel::wait_for_u32_forever, &cmd);
    *cmd = clamp<float>(-1.0, 1.0, *cmd);
    *cmd =
        map_range<float, float>(*cmd, -1.0, 1.0, MIN_STEER_DEG, MAX_STEER_DEG);
    *cmd = deg_to_rad<float, float>(*cmd);
    current_steering_cmd = *cmd;
    delete cmd;
  }
}

void ActuationController::brake_thread_impl() {
  // CAN frame format
  // |       Byte 1      |           Byte 2            |
  // | 8 LSB of position | CE | ME | 5 MSB of position |
  //
  // CE: enable actuator clutch (free move vs. motor engaged)
  // ME: enable actuator motor
  // 5 MSB of position in byte 2 occupies its 5 LSB
  // Position has a total of 8 + 5 = 13 bits (0 - 8192). 1000 = 1 inch
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
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
    CAN_BRAKE.write(CANMessage(0x00FF0000, message, 8, CANData, CANExtended));
  }
}

void ActuationController::sensor_poll_thread_impl() {
  ThisThread::sleep_for(std::chrono::milliseconds(1000));
  static constexpr std::chrono::milliseconds poll_interval{
      DEFAULT_SENSOR_POLL_INTERVAL_MS};
  while (!ThisThread::flags_get()) {
    sensors.steering_rad = steer_encoder.dutycycle();
    sensors.steering_rad =
        map_range<float, float>(sensors.steering_rad, 0.0, 1.0, 0.0, 2 * M_PI);
    ThisThread::sleep_for(poll_interval);
  }
}
} // namespace gkc
} // namespace tritonai