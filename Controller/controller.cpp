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

  /**
   * @brief Construct a new Controller:: Controller object
   * it declares the wachdog parameters and callback functions, and adds actuation_ to the sensor list (because the actuator has sensros like the steering encoder, and throttle speed) 
   * 
   */
Controller::Controller()
    : GkcStateMachine(), GkcPacketSubscriber(),
      Watchable(DEFAULT_MCU_HEARTBEAT_INTERVAL_MS,
                DEFAULT_MCU_HEARTBEAT_LOST_TOLERANCE_MS),
      comm_(this), actuation_(this), sensor_(),
      pc_hb_watcher_(DEFAULT_PC_HEARTBEAT_INTERVAL_MS,
                     DEFAULT_PC_HEARTBEAT_LOST_TOLERANCE_MS),         //Define the parameters of the harrbeat watchdog
      ctl_cmd_watcher_(DEFAULT_CTL_CMD_INTERVAL_MS,
                       DEFAULT_CTL_CMD_LOST_TOLERANCE_MS),            //Define the oarameters of the controll commands watchdog
      watchdog_(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS,
                DEFAULT_WD_INTERVAL_MS) {
  std::cout << "Initializing Controller class" << std::endl;
  //Define the functions that are called when the wachdog is triggered.
  attach(callback(this, &Controller::watchdog_callback));
  pc_hb_watcher_.attach(callback(this, &Controller::watchdog_callback));
  ctl_cmd_watcher_.attach(callback(this, &Controller::watchdog_callback));
  //Do other things necesary for the watchdog
  Watchable::activate();
  watchdog_.add_to_watchlist(this);
  watchdog_.add_to_watchlist(&pc_hb_watcher_);
  watchdog_.add_to_watchlist(&ctl_cmd_watcher_);

  //Register the actuators as a sesor provider. The actuator class has sensors and we register as one of the -----------------------
  sensor_.register_provider(&actuation_);

  std::cout << "Controller class initialized" << std::endl;
}

/**
 * @brief This function will be called when one of the wachdogs is triggered
 * It is configured as the wachdog callback function on Controller::Controller()
 * If we are not on the Unizialized state or Emergency state its the state machine to emergecy_stop state.
 * Then, to change what the kart does when the wachdog is triggered (there is an emergecy stop) you should change Controller::on_emergency_stop()
 * 
 */
void Controller::watchdog_callback() {
  std::cout << "Controller watchdog triggered" << std::endl;
  if (get_state() != GkcLifecycle::Uninitialized &&
      get_state() != GkcLifecycle::Emergency) {
    emergency_stop();
  }
}

/**
 * @brief This function is cakked whenever we receive a Handshake1GkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It sends back a Handshake2GkcPacket with a seq_number 1 higher than the received one.
 * If the state machine is not on Unitialized mode it will raise a warning, as it should be receiving this type of packet at that point
 * 
 * @param packet the received packet. It contains the received information.
 */
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

/**
 * @brief This function is cakked whenever we receive a Handshake2GkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It raises a warning because as the communication protocol is defined the RTC should be receiving this type of packets.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const Handshake2GkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Handshake #2 received which should not be sent to MCU. Ignoring.");
}

/**
 * @brief This function is cakked whenever we receive a GetFirmwareVersionGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It sends back a FirmwareVersionGkcPacket with the version of the communication library running on the RTC.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet) {
  FirmwareVersionGkcPacket pkt;
  pkt.major = GkcPacketLibVersion::MAJOR;
  pkt.minor = GkcPacketLibVersion::MINOR;
  pkt.patch = GkcPacketLibVersion::PATCH;
  comm_.send(pkt);
}

/**
 * @brief This function is cakked whenever we receive a FirmwareVersionGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It raises a warning because as the communication protocol is defined the RTC should be receiving this type of packets. It is the main robot computer to check the version compatibility
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const FirmwareVersionGkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Firmware version received, but checking version is PC's "
           "responsibility. Ignoring.");
}

/**
 * @brief This function is cakked whenever we receive a ResetMcuGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * If the state machine is not on the Unitialized state it raises a warning because it can only be reseted on that state.
 * It checks that the received packet contains the 'magic' number. This is like a pasword to reset it so not anyonw can reset it.
 * 
 * @param packet the received packet. It contains the received information.
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

/**
 * @brief This function is cakked whenever we receive a HeartbeatGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * The porpuse of this packet is to be sent periodically to test communication is working. We have a watchdog that is monitoring that we aare receiving this type of packet. If we don't for longer than the watchdog is configured on the watchdog will call the function Controller::watchdog_callback() whcich will set the state machine to emergency_stop and stop the kart.
 * This function kicks the watchdog 'pc_hb_watcher_' so it knows we are still receiving communication.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const HeartbeatGkcPacket &packet) {
  static uint8_t last_count = packet.rolling_counter;
  if (last_count != packet.rolling_counter) {
    pc_hb_watcher_.inc_count();
  }
  last_count = packet.rolling_counter;
}

/**
 * @brief This function is cakked whenever we receive a ConfigGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * This function runs the function Controller::initialize_thread_callback() on the thread 'initialize_thread' because the function is long and these functions have to be short to not block the communication. Running it on another thread won't block the communication. ???
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const ConfigGkcPacket &packet) {
  initialize_thread.start(
      callback(this, &Controller::initialize_thread_callback));
}

void Controller::initialize_thread_callback() { initialize(); }

/**
 * @brief This function is cakked whenever we receive a StateTransitionGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * This packet will change the state machine to the one indicated. Some states are unaccesible using this type os packet and it raises a warning if we try.
 * 
 * @param packet the received packet. It contains the received information.
 */
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
  /*case Shutdown:
    send_log(LogPacket::Severity::WARNING,
             "Shutdown state can only be entered upon reciving shutdown "
             "request. Ignoring.");
    break;*/
  case Emergency:
    GkcStateMachine::emergency_stop();
    break;
  }
}

/**
 * @brief This function is cakked whenever we receive a ControlGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * This packet will make the kart to move the actuators. It does so by interfacing the Actuator class (actuation_) with the methods set_throttle_cmd, set_brake_cmd, set_steering_cmd
 * It checks the state machine is on Active state.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const ControlGkcPacket &packet) {
  // TODO
  // std::stringstream s;
  // s << "[Control] thr: " << packet.throttle << ", brk: " << packet.brake
  //  << ", str: " << packet.steering;
  // send_log(LogPacket::Severity::INFO, s.str());
  if (get_state() == GkcLifecycle::Active) {
    //it does new float(.) because the functions receive a float* and not a float. That way we create a float* that has the value of what we want.
    actuation_.set_throttle_cmd(new float(packet.throttle));
    actuation_.set_brake_cmd(new float(packet.brake));
    actuation_.set_steering_cmd(new float(packet.steering));
  }
}

/**
 * @brief This function is cakked whenever we receive a SensorGkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It raises a warning because as the communication protocol is defined the RTC should be receiving this type of packets. We should send sensor values to the computer, not the other way round.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const SensorGkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "A sensor packet was received which MCU will ignore.");
}

/**
 * @brief This function is cakked whenever we receive a Shutdown1GkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * This packet shuts down the RTC. It checks the state machine is on Inactive, or Active state. It sets the state machine to Shutdown state
 * 
 * @param packet the received packet. It contains the received information.
 */
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

/**
 * @brief This function is cakked whenever we receive a Shutdown2GkcPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It raises a warning because as the communication protocol is defined the RTC should be receiving this type of packet.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const Shutdown2GkcPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Shutdown #2 received which should not be sent to MCU. Ignoring.");
}

/**
 * @brief This function is cakked whenever we receive a LogPacket.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcPacketSubscriber.
 * More information about the communication protocol on https://github.com/Triton-AI/go-kart-real-time-controller/blob/main/ROS2/src/tai_gokart_packet/design/Packet_API.md
 * 
 * It raises a warning because as the communication protocol is defined the RTC should be receiving this type of packet.
 * 
 * @param packet the received packet. It contains the received information.
 */
void Controller::packet_callback(const LogPacket &packet) {
  send_log(LogPacket::Severity::WARNING,
           "Log packet received which should not be sent to MCU. Ignoring.");
}


/**
 * @brief It uses the CommManager (comm_) to send the log
 We 'promise' to implement this function because we inherate from ILogger
 * 
 * @param severity severity of the log (eg. LogPacket::Severity::WARNING)
 * @param what The text on the log
 */
void Controller::send_log(const LogPacket::Severity &severity,
                          const std::string &what) {
  LogPacket pkt;
  pkt.level = severity;
  pkt.what = what;
  comm_.send(pkt);
}

/**
 * @brief it sends a HeartbeatGkcPacket to the main computer at the frecuency using the period from the watchdog.
 * This function is running on the thread 'heartbeat_thread', and it is configured to run on Controller::on_initialize()
 * 
 */
void Controller::heartbeat_thread_callback() {
  uint8_t rolling_counter = 0;
  HeartbeatGkcPacket pkt;
  Timer hb_timer;
  //Infinite loop. Equivalent to while(1);
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

/**
 * @brief it sends a sensor packet to the main computer at the frecuency using the period from the watchdog.
 * It does sensor_.get_packet() to get the packet populated with all the sensors info.
 * 
 */
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

/**
 * @brief This function is called whenever the state machine goes to the Initialize state.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * It activates the harbeat watchdog, starts the thread that sends harbeat packets to the main computer, and the thread that reads the sensors.
 * It starths the function GkcStateMachine::initialize()
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
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


/**
 * @brief This function is called whenever the state machine goes to the Actuation paused (deactivate) state.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * As specified on the state machine diagram on the link it deactivates the controll commands wachdog, because we will stop receiving that type of commands
 * TODO: disallow actuation
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
StateTransitionResult
Controller::on_deactivate(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.deactivate();
  // TODO(haoru): disallow actuation
  return StateTransitionResult::SUCCESS;
}

/**
 * @brief This function is called whenever the state machine goes to the Actuation Active (activate) state.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * As specified on the state machine diagram on the link it arms the controll commands wachdog, because we will start receiving that type of commands
 * TODO: allow actuation
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
StateTransitionResult Controller::on_activate(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.activate();
  // TODO(haoru): allow actuation
  return StateTransitionResult::SUCCESS;
}

/**
 * @brief This function is called whenever the state machine goes from Actuation paused to Emergency Stop through a shutdown command from the computer.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * It deactiaves the watchdogs
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
StateTransitionResult Controller::on_shutdown(const GkcLifecycle &last_state) {
  ctl_cmd_watcher_.deactivate();
  // TODO(haoru): disallow actuation
  pc_hb_watcher_.deactivate();
  return StateTransitionResult::SUCCESS;
}

/**
  * @brief This function is called whenever the state machine goes to the Emergency stop state.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * We still have to implement the emergency stop protocol
 * TODO: implement what to do on the mergency stop
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
StateTransitionResult
Controller::on_emergency_stop(const GkcLifecycle &last_state) {
  // TODO
  return StateTransitionResult::SUCCESS;
}

/**
 * @brief This function is called whenever the state machine goes from emergency stop to unitialized through a reinitialize command from the computer.
 * This function is one of the functions we 'promised' to implement when we inherited from GkcStateMachine.
 * More information about the state machine on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
 * 
 * TODO: implement what to do
 * 
 * @param last_state  The previous state
 * @return StateTransitionResult
 */
StateTransitionResult
Controller::on_reinitialize(const GkcLifecycle &last_state) {
  // TODO
  return StateTransitionResult::SUCCESS;
}

} // namespace gkc
} // namespace tritonai