/**
 * @file controller.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI hello this is a test
 *
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <iomanip>

#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

#include "StateMachine/state_machine.hpp"
#include "tai_gokart_packet/version.hpp"

#include "Controller/controller.hpp"
#include "config.hpp"

#ifndef CONFIG_HPP_
#error "No configuration macro found"
#endif

namespace tritonai {
namespace gkc {
Controller::Controller()
    : GkcStateMachine(), GkcPacketSubscriber(),
      Watchable(DEFAULT_MCU_HEARTBEAT_INTERVAL_MS,
                DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS),
      ISensorProvider(), comm_(this), actuation_(this), sensor_(), ILogger(),
      pc_hb_watcher_(DEFAULT_PC_HEARTBEAT_INTERVAL_MS,
                     DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS),
      ctl_cmd_watcher_(DEFAULT_CTL_CMD_INTERVAL_MS,
                       DEFAULT_CTL_CMD_LOST_TOLERANCE_MS),
      watchdog_(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS,
                DEFAULT_WD_INTERVAL_MS),
      estop_interrupt(ESTOP_PIN) {
  // std::cout << "Initializing Controller class" << std::endl;
  // attach(callback(this, &Controller::watchdog_callback));
  // pc_hb_watcher_.attach(callback(this, &Controller::watchdog_callback));
  // ctl_cmd_watcher_.attach(callback(this, &Controller::watchdog_callback));
  // Watchable::activate();packet_callback

  // watchdog_.add_to_watchlist(this);
  // watchdog_.add_to_watchlist(&pc_hb_watcher_);
  // watchdog_.add_to_watchlist(&ctl_cmd_watcher_);

  // state_poll_thread.start(callback(this, &Controller::poll_state));

  sensor_.register_provider(&actuation_);

  // sensor_.register_provider(this);

  // estop_interrupt.rise(callback(this,
  // &Controller::estop_interrupt_callback));

  // std::cout << "Controller class initialized" << std::endl;
}

void Controller::poll_state() {
  while (1) {
    auto ss = "";
    switch (static_cast<GkcLifecycle>(get_state())) {
    case GkcLifecycle::Uninitialized:
      ss = "Uninit";
      break;
    case GkcLifecycle::Initializing:
      ss = "Initing";
      break;
    case GkcLifecycle::Active:
      ss = "Active";
      break;
    case Inactive:
      ss = "Inactive";
      break;
    case Emergency:
      ss = "Estop";
      break;
    }

    std::cout << "State: " << ss << std::endl;
    ThisThread::sleep_for(500ms);
  }
}

/**
 *
 * @brief This function would be called when the watchdog timer expires.
 */
void Controller::watchdog_callback() {
  // std::cout << "Controller watchdog triggered" << std::endl;
  if (get_state() != GkcLifecycle::Uninitialized &&
      get_state() != GkcLifecycle::Emergency) {
    emergency_stop();
  }
}

/**
 *
 * @brief One of the packet callback functions.
 * This one sends a HandShake1 Packet to the MRC.
 * @param pkt GkcPacket for HandShake1
 */
void Controller::packet_callback(const Handshake1GkcPacket &packet) {
  // std::cout << "Handshake received" << std::endl;
  if (get_state() == GkcLifecycle::Uninitialized) {
    Handshake2GkcPacket pkt;
    pkt.seq_number = packet.seq_number + 1;
    comm_.send(pkt);
  } else {
    send_log(LogPacket::Severity::WARNING,
             "Handshake received while not in uninitialized mode. Ignoring.");
  }
}

void Controller::packet_callback(const Handshake2GkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Handshake #2 received which should not be sent to MCU. Ignoring.");
}

/**
 *
 * @brief One of the packet callback functions.
 * This one creates a packet with the firmware version and sends it to the MRC.
 * @param pkt GkcPacket for firmware
 */
void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet) {
  cout << "Received a GetFirmwareVersionGkcPacket!" << endl;
  FirmwareVersionGkcPacket pkt;
  pkt.major = GkcPacketLibVersion::MAJOR;
  pkt.minor = GkcPacketLibVersion::MINOR;
  pkt.patch = GkcPacketLibVersion::PATCH;
  comm_.send(pkt);
}

/**
 *
 * @brief One of the packet callback functions.
 * This one receives a firmware version and logs that its not the sender's
 * responsibility.
 * @param pkt FirmwareVersionGkcPacket for firmware
 */
void Controller::packet_callback(const FirmwareVersionGkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Firmware version received, but checking version is PC's "
           "responsibility. Ignoring.");
}

/**
 *
 * @brief One of the packet callback functions.
 * This one determines if the RTC can be reset. It will only be reset
 * if the the "magic number" or password matches.
 * @param pkt packet that tells to reset
 */
void Controller::packet_callback(const ResetMcuGkcPacket &packet) {
  if (get_state() != GkcLifecycle::Uninitialized) {
    send_log(LogPacket::Severity::WARNING,
             "MCU can only be reset in uninitialized state. Ignoring.");
    return;
  }

  static constexpr uint32_t MAGIC = 0xAAAAAAAA;
  if (packet.magic_number == MAGIC) {
    send_log(LogPacket::Severity::INFO, "Reset Acknowledged. MCU Resetting.");
    NVIC_SystemReset();
  } else {
    send_log(LogPacket::Severity::WARNING,
             "Incorrect magic number in reset packet. Ignoring.");
  }
}

void Controller::packet_callback(const HeartbeatGkcPacket &packet) {
  static uint8_t last_count = packet.rolling_counter;
  if (last_count != packet.rolling_counter) {
    pc_hb_watcher_.inc_count();
  }
  last_count = packet.rolling_counter;
}

void Controller::packet_callback(const ConfigGkcPacket &packet) {
  initialize_thread.start(
      callback(this, &Controller::initialize_thread_callback));
}

void Controller::packet_callback(const StateTransitionGkcPacket &packet) {
  switch (static_cast<GkcLifecycle>(packet.requested_state)) {
  case GkcLifecycle::Uninitialized:
    GkcStateMachine::reinitialize();
    break;
  case GkcLifecycle::Initializing:
    send_log(LogPacket::Severity::WARNING,
             "Initializing state can only be entered upon reciving config "
             "packet. Ignoring.");
    break;
  case GkcLifecycle::Active:
    GkcStateMachine::activate();
    break;
  case Inactive:
    GkcStateMachine::deactivate();
    break;
  case Emergency:
    GkcStateMachine::emergency_stop();
    break;
  }
}

void Controller::packet_callback(const ControlGkcPacket &packet) {
  std::stringstream s;
  s << "[Control] thr: " << packet.throttle << ", brk: " << packet.brake
    << ", str: " << packet.steering;
  send_log(LogPacket::Severity::INFO, s.str());

  if (get_state() == GkcLifecycle::Active) {
    this->set_actuation_values(packet.steering, packet.throttle, packet.brake);
  } else {
    std::cout << "ERROR: Trying to do serial control with non-active state!\n";
    ThisThread::sleep_for(500ms);
  }
}

void Controller::set_actuation_values(float steerVal, float throttleVal,
                                      float breakVal) {
  // std::cout << "I am being called!!!!!" << std::endl;
  actuation_.set_throttle_cmd(new float(throttleVal)); // was set to steer
                                                       // before
  // std::cout << "throttle: " << throttleVal << endl;
  float h1 = actuation_.get_throttle_cmd();
  // std::cout << h1 << std::endl;
  actuation_.set_brake_cmd(new float(breakVal)); // setr to throttle before
  // std::cout << "brake: " << breakVal << endl;
  float h2 = actuation_.get_brake_cmd();
  // std::cout << h2 << std::endl;
  actuation_.set_steering_cmd(new float(steerVal)); // set to break
  // std::cout << "steering: " << steerVal << endl;
  float h3 = actuation_.get_steering_cmd();
  // std::cout <<"from controller.cpp " << h3 <<  std::endl;
}

void Controller::packet_callback(const SensorGkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "A sensor packet was received which MCU will ignore.");
}

void Controller::packet_callback(const Shutdown1GkcPacket &packet) {
  if (get_state() != GkcLifecycle::Inactive &&
      get_state() != GkcLifecycle::Active) {
    send_log(
        LogPacket::Severity::WARNING,
        "Vehicle can only be shutdown in inactive or active mode. Ignoring.");
    return;
  }

  send_log(LogPacket::Severity::INFO, "Shutdown Acknowledged. Shutting down.");
  Shutdown2GkcPacket pkt;
  pkt.seq_number = packet.seq_number + 1;
  comm_.send(pkt);
  GkcStateMachine::shutdown();
}

void Controller::packet_callback(const Shutdown2GkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Shutdown #2 received which should not be sent to MCU. Ignoring.");
}

void Controller::packet_callback(const LogPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Log packet received which should not be sent to MCU. Ignoring.");
}

bool Controller::is_ready() { return true; }

void Controller::populate_reading(SensorGkcPacket &pkt) {
  // pkt.values.fault_warning = static_cast<bool>(estop_interrupt.read());
}

void Controller::initialize_thread_callback() { initialize(); }

void Controller::send_log(const LogPacket::Severity &severity,
                          const std::string &what) {
  LogPacket pkt;
  pkt.level = severity;
  pkt.what = what;
  comm_.send(pkt);
}

void Controller::heartbeat_thread_callback() {
  uint8_t rolling_counter = 0;
  HeartbeatGkcPacket pkt;
  Timer hb_timer;
  while (!ThisThread::flags_get()) {
    hb_timer.start();
    inc_count();
    pkt.rolling_counter = rolling_counter++;
    pkt.state = get_state();
    comm_.send(pkt);
    hb_timer.stop();
    auto sleep_time = std::chrono::milliseconds(get_update_interval()) -
                      hb_timer.elapsed_time();
    if (sleep_time > std::chrono::milliseconds(0)) {
      ThisThread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time));
    }
  }
}

void Controller::sensor_poll_thread_callback() {
  Timer sensor_timer;
  while (!ThisThread::flags_get()) {
    sensor_timer.start();
    comm_.send(sensor_.get_packet());
    sensor_timer.stop();
    auto sleep_time = std::chrono::milliseconds(get_update_interval()) -
                      sensor_timer.elapsed_time();
    if (sleep_time > std::chrono::milliseconds(0)) {
      ThisThread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time));
    }
  }
}

void Controller::estop_interrupt_callback() {
  send_log(LogPacket::Severity::WARNING, "[ESTOP TRIGGERED!!!]");
  emergency_stop();
}

StateTransitionResult
Controller::on_initialize(const GkcLifecycle &last_state) {
  // std::cout << "Start initialization" << std::endl;
  // if (estop_interrupt.read()) {
  //  send_log(LogPacket::Severity::ERROR,
  //           "ESTOP is still on. Initialization failed.");
  //  return StateTransitionResult::FAILURE;
  // }
  pc_hb_watcher_.activate();
  heartbeat_thread.start(
      callback(this, &Controller::heartbeat_thread_callback));
  sensor_poll_thread.start(
      callback(this, &Controller::sensor_poll_thread_callback));
  initialize_thread_callback();
  return StateTransitionResult::SUCCESS;
}

StateTransitionResult
Controller::on_deactivate(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.deactivate();
  // TODO(haoru): disallow actuation
  return StateTransitionResult::SUCCESS;
}

void Controller::deactivate_controller() {
  // GkcStateMachine::deactivate();
  ctl_cmd_watcher_.deactivate();
  pc_hb_watcher_.deactivate();
  heartbeat_thread.terminate();
  sensor_poll_thread.terminate();
}

void Controller::activate_controller() {
  // GkcStateMachine::activate();
  ctl_cmd_watcher_.activate();
  pc_hb_watcher_.activate();
  heartbeat_thread.start(
      callback(this, &Controller::heartbeat_thread_callback));
  sensor_poll_thread.start(
      callback(this, &Controller::sensor_poll_thread_callback));
}

StateTransitionResult Controller::on_activate(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.activate();
  // TODO(haoru): allow actuation
  return StateTransitionResult::SUCCESS;
}

StateTransitionResult Controller::on_shutdown(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.deactivate();
  // TODO(haoru): disallow actuation
  pc_hb_watcher_.deactivate();
  return StateTransitionResult::SUCCESS;
}

StateTransitionResult
Controller::on_emergency_stop(const GkcLifecycle &last_state) {
  // TODO
  return StateTransitionResult::SUCCESS;
}

StateTransitionResult
Controller::on_reinitialize(const GkcLifecycle &last_state) {
  // TODO
  return StateTransitionResult::SUCCESS;
}

} // namespace gkc
} // namespace tritonai