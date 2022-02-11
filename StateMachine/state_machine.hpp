/**
 * @file state_machine.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef STATE_MACHINE_HPP_
#define STATE_MACHINE_HPP_

#include "mbed.h"

#include "tai_gokart_packet/gkc_packet_utils.hpp"

namespace tritonai {
namespace gkc {
enum StateTransitionResult {
  SUCCESS = 0,
  FAILURE = 1,
  ERROR = 2,
  EMERGENCY_STOP = 3,
  FAILURE_INVALID_TRANSITION = 4
};

class GkcStateMachine {
public:
  GkcStateMachine();

  StateTransitionResult initialize();
  StateTransitionResult deactivate();
  StateTransitionResult activate();
  StateTransitionResult shutdown();
  StateTransitionResult emergency_stop();
  StateTransitionResult reinitialize();

  GkcLifecycle get_state() const;

protected:
  virtual StateTransitionResult
  on_initialize(const GkcLifecycle &last_state) = 0;

  virtual StateTransitionResult
  on_deactivate(const GkcLifecycle &last_state) = 0;

  virtual StateTransitionResult on_activate(const GkcLifecycle &last_state) = 0;

  virtual StateTransitionResult on_shutdown(const GkcLifecycle &last_state) = 0;

  virtual StateTransitionResult
  on_emergency_stop(const GkcLifecycle &last_state) = 0;

  virtual StateTransitionResult
  on_reinitialize(const GkcLifecycle &last_state) = 0;

private:
  GkcLifecycle state_;
};
} // namespace gkc
} // namespace tritonai

#endif // STATE_MACHINE_HPP_
