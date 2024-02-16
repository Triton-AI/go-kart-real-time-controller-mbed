#include "Controller/controller.hpp"
#include "config.h"

#include "tai_gokart_packet/gkc_packets.hpp"

#include <chrono>
#include <iostream>

namespace tritonai::gkc
{
  // TODO: remove this function in production
  void Controller::keep_alive()
  {
    GkcLifecycle last_state = GkcLifecycle::Emergency;
    while(1){
      ThisThread::sleep_for(std::chrono::milliseconds(100));
      // State to string
      std::string state;
      switch(get_state())
      {
        case GkcLifecycle::Uninitialized:
          state = "Uninitialized";
          break;
        case GkcLifecycle::Inactive:
          state = "Inactive";
          break;
        case GkcLifecycle::Active:
          state = "Active";
          break;
        case GkcLifecycle::Emergency:
          state = "EmergencyStop";
          break;
        case GkcLifecycle::Initializing:
          state = "Initializing";
          break;
        default:
          state = "Unknown";
          break;
      }
      if (last_state != get_state())
      {
        send_log(LogPacket::Severity::WARNING, "Controller state: " + state);
        last_state = get_state();
      }
      this->inc_count();
    }
  }

  // Controller initialization
  Controller::Controller() :
    Watchable(DEFAULT_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_CONTROLLER_POLL_LOST_TOLERANCE_MS, "Controller"), // Initializes the controller with default values
    GkcStateMachine(), // Initializes the state machine
    _severity(LogPacket::Severity::WARNING), // Initializes the severity of the logger
    _comm(this), // Passes the controller as the subscriber to the comm manager
    _watchdog(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS, DEFAULT_WD_WAKEUP_INTERVAL_MS), // Initializes the watchdog with default values
    _sensor_reader(), // Initializes the sensor reader
    _actuation(this), // Passes the controller as the logger to the actuation controller
    _rc_controller(this) // Passes the controller as the packet subscriber to the RC controller
  {
    // Attaches the watchdog callback to the controller
    attach(callback(this, &Controller::watchdog_callback));

    // Add a timer that calls keep alive every 100ms
    _keep_alive_thread.start(callback(this, &Controller::keep_alive));

    // Adds the all the objects to the watchlist
    _watchdog.add_to_watchlist(this); // Adds the controller to the watchlist
    _watchdog.add_to_watchlist(&_comm); // Adds the comm manager to the watchlist
    _watchdog.add_to_watchlist(&_sensor_reader); // Adds the sensor reader to the watchlist
    _watchdog.add_to_watchlist(&_rc_controller); // Adds the RC controller to the watchlist

    _watchdog.arm(); // Arms the watchdog

    send_log(LogPacket::Severity::INFO, "Controller initialized");
  }

  // Wathdog API IMPLEMENTATION
  void Controller::watchdog_callback()
  {
    send_log(LogPacket::Severity::ERROR, "Controller watchdog trigger");
    NVIC_SystemReset();
  }

  // ILogger API IMPLEMENTATION
  void Controller::send_log(const LogPacket::Severity &severity, const std::string &what)
  {
    
    if(severity == LogPacket::Severity::FATAL && _severity <= severity)
      std::cerr << "Fatal: " << what << std::endl;
    else if(severity == LogPacket::Severity::ERROR && _severity <= severity)
      std::cerr << "Error: " << what << std::endl;
    else if(severity == LogPacket::Severity::WARNING && _severity <= severity)
      std::cerr << "Warning: " << what << std::endl;
    else if(severity == LogPacket::Severity::INFO && _severity <= severity)
      std::cout << "Info: " << what << std::endl;
    // else
    //   std::cout << "Debug: " << what << std::endl;

  }

  // PACKET CALLBACKS API IMPLEMENTATION
  void Controller::packet_callback(const Handshake1GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Handshake1GkcPacket received");

    // If the controller is uninitialized, initialize it
    if(get_state() == GkcLifecycle::Uninitialized)
    {
      send_log(LogPacket::Severity::INFO, "Controller transitioning to Initializing");
      initialize();
    }

    // If the controller has successfully initialized, send a Handshake2GkcPacket
    if(get_state() == GkcLifecycle::Inactive)
    {
      Handshake2GkcPacket response;
      response.seq_number += packet.seq_number + 1;
      _comm.send(response);
    }
  }

  void Controller::packet_callback(const Handshake2GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Handshake2GkcPacket received, should not happen, ignoring.");
  }

  void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "GetFirmwareVersionGkcPacket received");
  }

  void Controller::packet_callback(const FirmwareVersionGkcPacket &packet)
  {
    std::cout << "FirmwareVersionGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ResetMcuGkcPacket &packet)
  {
    std::cout << "ResetMcuGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const HeartbeatGkcPacket &packet)
  {
    std::cout << "HeartbeatGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ConfigGkcPacket &packet)
  {
    std::cout << "ConfigGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const StateTransitionGkcPacket &packet)
  {
    std::cout << "StateTransitionGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ControlGkcPacket &packet)
  {
    std::cout << "ControlGkcPacket received" << std::endl;
    _actuation.set_throttle_cmd(new float(packet.throttle));
    _actuation.set_steering_cmd(new float(packet.steering));
    _actuation.set_brake_cmd(new float(packet.brake));
  }

  void Controller::packet_callback(const SensorGkcPacket &packet)
  {
    std::cout << "SensorGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Shutdown1GkcPacket &packet)
  {
    std::cout << "Shutdown1GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Shutdown2GkcPacket &packet)
  {
    std::cout << "Shutdown2GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const LogPacket &packet)
  {
    std::cout << "LogPacket received" << std::endl;
  }

  void Controller::packet_callback(const RCControlGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, 
            "RCControlGkcPacket received: throttle: " + std::to_string((int)(packet.throttle * 100)) + "%, " +
            "steering: " + std::to_string((int)(packet.steering * 100)) + "%, " +
            "brake: " + std::to_string((int)(packet.brake * 100)) + "%, " +
            "autonomy_mode: " + std::to_string(packet.autonomy_mode) + ", " +
            "is_active: " + std::to_string(packet.is_active)
    );


    _actuation.set_throttle_cmd(new float(packet.throttle));
    _actuation.set_steering_cmd(new float(packet.steering));
    _actuation.set_brake_cmd(new float(packet.brake));

  }

  // GkcStateMachine API IMPLEMENTATION
  StateTransitionResult Controller::on_initialize(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller initializing");
    return StateTransitionResult::SUCCESS;
  }

  StateTransitionResult Controller::on_deactivate(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller deactivating");
    return StateTransitionResult::SUCCESS;
  }

  StateTransitionResult Controller::on_activate(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller activating");
    return StateTransitionResult::SUCCESS;
  }

  StateTransitionResult Controller::on_emergency_stop(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller emergency stopping");
    return StateTransitionResult::SUCCESS;
  }

  StateTransitionResult Controller::on_reinitialize(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller reinitializing");
    return StateTransitionResult::SUCCESS;
  }
} // namespace tritonai::gkc