#ifndef RACE_VEHICLE_CONTROLLER__EXTERNAL__PID_CONTROLLER_HPP_
#define RACE_VEHICLE_CONTROLLER__EXTERNAL__PID_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <string>

namespace tritonai {
namespace gkc {

struct PidCoefficients {
  float k_p{};
  float k_i{};
  float k_d{};
  float min_cmd{};
  float max_cmd{};
  float min_i{};
  float max_i{};
};

class PidController {
public:
  explicit PidController(std::string const &name,
                         PidCoefficients const &coefficients);

  PidController();

  void reset_integral_error(float integral_error);

  float integral_error();

  float update(float new_error, float actual_dt);

  const PidCoefficients &params() const;

private:
  std::string name_;
  PidCoefficients coefficients_;

  float integral_error_{};
  float last_error_{};
  float error_{};
};
} // namespace gkc
} // namespace tritonai

#endif // RACE_VEHICLE_CONTROLLER__EXTERNAL__PID_CONTROLLER_HPP_
