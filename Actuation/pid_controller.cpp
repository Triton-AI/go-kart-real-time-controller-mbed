#include <algorithm>
#include <cmath>
#include <string>

#include "pid_controller.hpp"

namespace tritonai {
namespace gkc {

PidController::PidController(std::string const &name,
                             PidCoefficients const &coefficients)
    : name_{name}, coefficients_{coefficients} {}

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

float PidController::integral_error() { return integral_error_; }

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

const PidCoefficients &PidController::params() const { return coefficients_; }
} // namespace gkc
} // namespace tritonai
