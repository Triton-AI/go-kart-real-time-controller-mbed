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
/**
 * @brief GkcStateMachine constructor
 * Constructs a GkcStateMachine object, which has the methods for transitioning
 * between states of the GkcLifecycle.
 */
namespace tritonai {
namespace gkc {
GkcStateMachine::GkcStateMachine() : state_(GkcLifecycle::Uninitialized) {}

/**
 * @brief Transitions the state machine to the initializing state if possible.
 * Has to check if the current state is uninitialized, if not, it fails.
 * If the current state is uninitialized, sets the state to initializing, calls
 * on_initialize, and sets the state to inactive initialization is successful.
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::initialize() {
  // Checks that the current state is uninitialized
  if (state_ != GkcLifecycle::Uninitialized) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Sets the state to initializing and calls on_initialize
  state_ = GkcLifecycle::Initializing;
  const auto result = on_initialize(state_);
  // Sets the state based on whether or not initialization was successful
  if (result == StateTransitionResult::SUCCESS) {
    state_ = GkcLifecycle::Inactive;
  } else {
    state_ = GkcLifecycle::Uninitialized;
  }
  return result;
}

/**
 * @brief Transitions the state machine to the inactive state if possible.
 * Has to check if the current state is active, if not, it fails.
 * If the current state is active, calls on_deactivate, and sets the state to
 * inactive if deactivation is successful. If deactivation is not successful,
 * it calls the emergency_stop function and changes states.
 * @return StateTransitionResult
 */

StateTransitionResult GkcStateMachine::deactivate() {
  // Checks that the current state is active
  if (state_ != GkcLifecycle::Active) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_deactivate and sets state to inactive if transition is successful
  const auto result = on_deactivate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  // Calls the emergency_stop function and changes states if necessary
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

/**
 * @brief Transitions the state machine to the active state if possible.
 * Has to check if the current state is inactive, if not, it fails.
 * If the current state is inactive, calls on_activate, and sets the state to
 * active if activation is successful. If activation is not successful, it
 * calls the emergency_stop function and changes states.
 * @return StateTransitionResult
 */

StateTransitionResult GkcStateMachine::activate() {
  // Checks that the current state is inactive
  // if (state_ != GkcLifecycle::Inactive) {
  //   std::cout << "BAD OOGA BOOGA\n";
  //   return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  // }
  // Calls on_activate and sets state to active if transition is successful
  const auto result = on_activate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Active;
    break;
  // Calls the emergency_stop function and changes states if necessary
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

/**
 * @brief Transitions the state machine to the shutdown state if possible.
 * Has to check if the current state is inactive or active, if not, it fails.
 * If the current state is inactive or active, calls on_shutdown, and sets the
 * state to emergency if shutdown is successful. If shutdown is not successful,
 * it calls the emergency_stop function and changes states.
 * @return StateTransitionResult
 */

StateTransitionResult GkcStateMachine::shutdown() {
  // Checks that the current state is inactive or active
  if (state_ != GkcLifecycle::Inactive && state_ != GkcLifecycle::Active) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_shutdown and sets state to emergency
  const auto result = on_shutdown(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Emergency;
    break;
  // Calls the emergency_stop function if the transition fails
  case StateTransitionResult::EMERGENCY_STOP:
    on_emergency_stop(state_);
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    break;
  }
  return result;
}

/**
 * @brief Transitions the state machine to the emergency state.
 * Checks whether the current state is not uninitialized or initializing,
 * if it is, it fails. Otherwise, it calls on_emergency_stop and sets the state
 * to emergency. If the transition fails, it resets the MCU.
 *
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::emergency_stop() {
  // Checks that the current state is not uninitialized or initializing
  if (state_ == GkcLifecycle::Uninitialized ||
      state_ == GkcLifecycle::Initializing) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_emergency_stop and checks the result
  const auto result = on_emergency_stop(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Emergency;
    break;
  // Resets the MCU if the transition fails
  case StateTransitionResult::FAILURE:
  case StateTransitionResult::ERROR:
    std::cout << "Resetting MCU for Estop Failure";
    NVIC_SystemReset();
  default:
    break;
  }
  return result;
}

/**
 * @brief Transitions the state machine to the uninitialized state if possible.
 * Checks whether the current state is emergency, if it is, it fails. Otherwise,
 * it calls on_reinitialize and sets the state to uninitialized if the
 * transition is successful.
 *
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::reinitialize() {
  // Checks that the current state is emergency
  if (state_ != GkcLifecycle::Emergency) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_reinitialize and sets state to uninitialized if transition is
  // successful
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
