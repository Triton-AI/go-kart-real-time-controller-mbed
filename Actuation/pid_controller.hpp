#ifndef RACE_VEHICLE_CONTROLLER__EXTERNAL__PID_CONTROLLER_HPP_
#define RACE_VEHICLE_CONTROLLER__EXTERNAL__PID_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <string>


/**
 * @file pid_controller.hpp
 * @author
 * @brief
 * @version 0.1
 * @date 2022-04-02
 *
 * @copyright Copyright 2022 Triton AI
 *
 * This header file is used for defining a Structure for the PID coefficients
 * and a class for the PID controller itself.
 * A PID controller is a feedback control loop used to control v process.
 * AKA a Proportional-Integral-Derivative control loop.
 * The measured error input produces an output to reduce the error and reach a target value.
 *
 *
 *
 */
namespace tritonai {
namespace gkc {

struct PidCoefficients {
    //These coefficients are used in the PIC control structure.
    //k_p,k_i and k_d represent proportional, integral and derivative respectively
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
    //Takes in a name and the coefficients
  explicit PidController(std::string const &name,
                         PidCoefficients const &coefficients);

  PidController();
    //Sets the integral error
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
