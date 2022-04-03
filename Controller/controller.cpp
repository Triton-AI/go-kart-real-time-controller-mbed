/**
 * @file controller.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "gkc_packet_utils.hpp"
#include "gkc_packets.hpp"

#include "tai_gokart_packet/version.hpp"

#include "config.hpp"
#include "controller.hpp"

#ifndef CONFIG_HPP_
#error "No configuration macro found"
#endif

namespace tritonai {
namespace gkc {
Controller::Controller()
    : GkcStateMachine(), GkcPacketSubscriber(),
      Watchable(DEFAULT_MCU_HEARTBEAT_INTERVAL_MS,
                DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS),
      comm_(this), actuation_(),
      pc_hb_watcher_(DEFAULT_PC_HEARTBEAT_INTERVAL_MS,
                     DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS),
      ctl_cmd_watcher_(DEFAULT_CTL_CMD_INTERVAL_MS,
                       DEFAULT_CTL_CMD_LOST_TOLERANCE_MS),
      watchdog_(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS,
                DEFAULT_WD_INTERVAL_MS) {
  std::cout << "Initializing Controller class" << std::endl;
  attach(callback(this, &Controller::watchdog_callback));
  pc_hb_watcher_.attach(callback(this, &Controller::watchdog_callback));
  ctl_cmd_watcher_.attach(callback(this, &Controller::watchdog_callback));
  Watchable::activate();

  watchdog_.add_to_watchlist(this);
  watchdog_.add_to_watchlist(&pc_hb_watcher_);
  watchdog_.add_to_watchlist(&ctl_cmd_watcher_);
  std::cout << "Controller class initialized" << std::endl;
}

void Controller::watchdog_callback() {
  std::cout << "Controller watchdog triggered" << std::endl;
  if (get_state() != GkcLifecycle::Uninitialized &&
      get_state() != GkcLifecycle::Emergency) {
    emergency_stop();
  }
}

void Controller::packet_callback(const Handshake1GkcPacket &packet) {
  std::cout << "Handshake received" << std::endl;
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

void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet) {
  FirmwareVersionGkcPacket pkt;
  pkt.major = GkcPacketLibVersion::MAJOR;
  pkt.minor = GkcPacketLibVersion::MINOR;
  pkt.patch = GkcPacketLibVersion::PATCH;
  comm_.send(pkt);
}

void Controller::packet_callback(const FirmwareVersionGkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Firmware version received, but checking version is PC's "
           "responsibility. Ignoring.");
}

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
  case Shutdown:
    send_log(LogPacket::Severity::WARNING,
             "Shutdown state can only be entered upon reciving shutdown "
             "request. Ignoring.");
    break;
  case Emergency:
    GkcStateMachine::emergency_stop();
    break;
  }
}

void Controller::packet_callback(const ControlGkcPacket &packet) {
  // TODO
  // std::stringstream s;
  // s << "[Control] thr: " << packet.throttle << ", brk: " << packet.brake
  //   << ", str: " << packet.steering;
  // send_log(LogPacket::Severity::INFO, s.str());
  if (get_state() == GkcLifecycle::Active)
  {
      actuation_.set_throttle_cmd(new float(packet.throttle));
  }
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
  SensorGkcPacket pkt;
  while (!ThisThread::flags_get()) {
    sensor_timer.start();
    // TODO
    sensor_timer.stop();
    comm_.send(pkt);
    auto sleep_time = std::chrono::milliseconds(get_update_interval()) -
                      sensor_timer.elapsed_time();
    if (sleep_time > std::chrono::milliseconds(0)) {
      ThisThread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time));
    }
  }
}

StateTransitionResult
Controller::on_initialize(const GkcLifecycle &last_state) {
  std::cout << "Start initialization" << std::endl;
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