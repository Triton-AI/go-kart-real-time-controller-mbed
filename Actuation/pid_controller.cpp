
/**
 * @file pid_controller.cpp
 * @author
 * @brief
 * @version 0.1
 * @date 2022-04-02
 *
 * @copyright Copyright 2022 Triton AI
 *
 * This file implements the PID controller with functions to take inthe current error
 * and time elapsed since last error measurement.
 *
 *
 *
 *
 */

#include <algorithm>
#include <cmath>
#include <string>

#include "pid_controller.hpp"

namespace tritonai {
namespace gkc {
//Creates an object with a specified name and coefficient inputs.
PidController::PidController(std::string const &name,
                             PidCoefficients const &coefficients)
    : name_{name}, coefficients_{coefficients} {}
//creates uninitalized object with default arguments
PidController::PidController() : name_{"uninitialized"}, coefficients_{{}} {}

void PidController::reset_integral_error(float integral_error) {
  integral_error_ = integral_error;
}

template <typename T> T clamp(const T &val, const T &min, const T &max) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

//returns current value of integral error
float PidController::integral_error() { return integral_error_; }

//Takes the error value "new erroe" and the time elapsed "actual_dt"
//and calculates the PID components using stored coefficients.
//
float PidController::update(float new_error, float actual_dt) {
  last_error_ = error_;
  error_ = new_error;

  // Calculate & saturate integral error.

  integral_error_ += error_ * actual_dt;
  integral_error_ =
      clamp<float>(integral_error_, coefficients_.min_i, coefficients_.max_i);

  // Calculate dt error.

  const auto dt_error = (error_ - last_error_) / actual_dt;

  // Calculate control.

  const auto p = error_ * coefficients_.k_p;
  const auto i = integral_error_ * coefficients_.k_i;
  const auto d = dt_error * coefficients_.k_d;

  const auto cmd = p + i + d;

  // Clamp control.

  if (cmd <= coefficients_.min_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to min %f", cmd, coefficients_.min_cmd);
    return coefficients_.min_cmd;
  }

  if (cmd >= coefficients_.max_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to max %f", cmd, coefficients_.max_cmd);
    return coefficients_.max_cmd;
  }

  return cmd;
}

//returns the PID coefficients used in the controller.
const PidCoefficients &PidController::params() const { return coefficients_; }
} // namespace gkc
} // namespace tritonai
