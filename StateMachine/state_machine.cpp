/**
 * @file state_machine.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "state_machine.hpp"
#include <iostream>

namespace tritonai {
namespace gkc {
GkcStateMachine::GkcStateMachine() : state_(GkcLifecycle::Uninitialized) {}

StateTransitionResult GkcStateMachine::initialize() {
  if (state_ != GkcLifecycle::Uninitialized) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  state_ = GkcLifecycle::Initializing;
  const auto result = on_initialize(state_);
  if (result == StateTransitionResult::SUCCESS) {
    state_ = GkcLifecycle::Inactive;
  } else {
    state_ = GkcLifecycle::Uninitialized;
  }
  return result;
}

StateTransitionResult GkcStateMachine::deactivate() {
  if (state_ != GkcLifecycle::Active) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  const auto result = on_deactivate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

StateTransitionResult GkcStateMachine::activate() {
  if (state_ != GkcLifecycle::Inactive) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  const auto result = on_activate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Active;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

StateTransitionResult GkcStateMachine::shutdown() {
  if (state_ != GkcLifecycle::Inactive && state_ != GkcLifecycle::Active) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  const auto result = on_shutdown(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Emergency;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

StateTransitionResult GkcStateMachine::emergency_stop() {
  if (state_ == GkcLifecycle::Uninitialized ||
      state_ == GkcLifecycle::Initializing) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  const auto result = on_emergency_stop(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Emergency;
    break;
  case StateTransitionResult::FAILURE:
  case StateTransitionResult::ERROR:
    std::cout << "Resetting MCU for Estop Failure";
    NVIC_SystemReset();
  default:
    break;
  }
  return result;
}

StateTransitionResult GkcStateMachine::reinitialize() {
  if (state_ != GkcLifecycle::Emergency) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  const auto result = on_reinitialize(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Uninitialized;
    break;
  default:
    break;
  }
  return result;
}

GkcLifecycle GkcStateMachine::get_state() const { return state_; }
} // namespace gkc
} // namespace tritonai
