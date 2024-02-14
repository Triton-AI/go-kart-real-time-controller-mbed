#include "Controller/controller.hpp"
#include "config.h"

#include <iostream>

namespace tritonai::gkc
{
  Controller::Controller() :
    _comm(this), // Passes the controller as the subscriber to the comm manager
    _watchdog(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS, DEFAULT_WD_WAKEUP_INTERVAL_MS), // Initializes the watchdog with default values
    _sensor_reader() // Initializes the sensor reader
  {
    _watchdog.add_to_watchlist(&_comm); // Adds the comm manager to the watchlist
    _watchdog.add_to_watchlist(&_sensor_reader); // Adds the sensor reader to the watchlist

    _watchdog.arm(); // Arms the watchdog

    std::cout << "Controller created" << std::endl;
  }

  void Controller::packet_callback(const Handshake1GkcPacket &packet)
  {
    std::cout << "Handshake1GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Handshake2GkcPacket &packet)
  {
    std::cout << "Handshake2GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet)
  {
    std::cout << "GetFirmwareVersionGkcPacket received" << std::endl;
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
} // namespace tritonai::gkc