/**
 * @file gkc_packets.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Packet structures
 * @version 0.1
 * @date 2021-10-30
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */
#include "tai_gokart_packet/gkc_packets.hpp"
#include <chrono>
#include <algorithm>
#include <memory>
#include <string>
namespace tritonai
{
namespace gkc
{

RawGkcPacket::RawGkcPacket()
: payload_size(0), checksum(0), payload(GkcBuffer()) {}

RawGkcPacket::RawGkcPacket(const GkcBuffer & payload)
{
  payload_size = payload.size();
  this->payload = GkcBuffer(payload_size, 0);
  std::copy(payload.begin(), payload.end(), this->payload.begin());
  checksum = GkcPacketUtils::calc_crc16(payload);
}

RawGkcPacket::RawGkcPacket(const RawGkcPacket & packet) {*this = packet;}

RawGkcPacket & RawGkcPacket::operator=(const RawGkcPacket & packet)
{
  payload_size = packet.payload_size;
  payload = GkcBuffer(payload_size, 0);
  std::copy(packet.payload.begin(), packet.payload.end(), payload.begin());
  checksum = packet.checksum;
  return *this;
}

std::shared_ptr<GkcBuffer> RawGkcPacket::encode()
{
  static constexpr uint8_t NUM_NON_PAYLOAD_BYTES = 5;
  auto buffer = std::make_unique<GkcBuffer>(payload_size + NUM_NON_PAYLOAD_BYTES, 0);
  auto pos_payload_size =
    GkcPacketUtils::write_to_buffer(buffer->begin(), static_cast<uint8_t>(START_BYTE));
  auto pos_payload = GkcPacketUtils::write_to_buffer(pos_payload_size, payload_size);
  auto pos_checksum = std::copy(payload.begin(), payload.end(), pos_payload);
  auto pos_end_byte = GkcPacketUtils::write_to_buffer(pos_checksum, checksum);
  auto pos_end = GkcPacketUtils::write_to_buffer(pos_end_byte, static_cast<uint8_t>(END_BYTE));
  if (pos_end != buffer->end()) {
    // Sanity check: encoding should use the entire buffer
    std::cerr << "Error when encoding raw gkc packet. Potential payload size mismatch.";
  }
  return buffer;
}

GkcPacket::~GkcPacket()
{
}

RawGkcPacket::SharedPtr GkcPacket::encode() const
{
  return std::make_shared<RawGkcPacket>();
}

void GkcPacket::decode(const RawGkcPacket & raw)
{
  (void)raw;
}

void GkcPacket::publish(GkcPacketSubscriber & sub)
{
  (void)sub;
}

void GkcPacketUtils::debug_cout(std::string str)
{
  std::cout << str << std::endl;
}

/*
Handshake 1
*/
RawGkcPacket::SharedPtr Handshake1GkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(5, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint32_t>(payload.begin() + 1, seq_number);
  return std::make_unique<RawGkcPacket>(payload);
}

void Handshake1GkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer<uint32_t>(
    raw.payload.begin() + 1,
    seq_number);
}

/*
Handshake 2
*/
RawGkcPacket::SharedPtr Handshake2GkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(5, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint32_t>(payload.begin() + 1, seq_number);
  return std::make_unique<RawGkcPacket>(payload);
}

void Handshake2GkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer<uint32_t>(
    raw.payload.begin() + 1,
    seq_number);
}

/*
Get firmware
*/
RawGkcPacket::SharedPtr GetFirmwareVersionGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(1, 0);
  payload[0] = FIRST_BYTE;
  return std::make_unique<RawGkcPacket>(payload);
}

void GetFirmwareVersionGkcPacket::decode(const RawGkcPacket & raw)
{
  (void)raw;
}

/*
Firmware
*/
RawGkcPacket::SharedPtr FirmwareVersionGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(4, 0);
  payload[0] = FIRST_BYTE;
  payload[1] = major;
  payload[2] = minor;
  payload[3] = patch;
  return std::make_unique<RawGkcPacket>(payload);
}

void FirmwareVersionGkcPacket::decode(const RawGkcPacket & raw)
{
  major = raw.payload[1];
  minor = raw.payload[2];
  patch = raw.payload[3];
}

/*
Reset IMU
*/
RawGkcPacket::SharedPtr ResetRTCGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(5, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint32_t>(payload.begin() + 1, magic_number);
  return std::make_unique<RawGkcPacket>(payload);
}

void ResetRTCGkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer<uint32_t>(
    raw.payload.begin() + 1,
    magic_number);
}

/*
Heartbeat
*/
RawGkcPacket::SharedPtr HeartbeatGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(3, 0);
  payload[0] = FIRST_BYTE;
  payload[1] = rolling_counter;
  payload[2] = state;
  return std::make_unique<RawGkcPacket>(payload);
}

void HeartbeatGkcPacket::decode(const RawGkcPacket & raw)
{
  rolling_counter = raw.payload[1];
  state = raw.payload[2];
}

/*
Config
*/
RawGkcPacket::SharedPtr ConfigGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(sizeof(Configurables) + 1, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer(payload.begin() + 1, values);
  return std::make_unique<RawGkcPacket>(payload);
}

void ConfigGkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer(
    raw.payload.begin() + 1,
    values);
}

/*
State Transition
*/
RawGkcPacket::SharedPtr StateTransitionGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(2, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer(payload.begin() + 1, requested_state);
  return std::make_unique<RawGkcPacket>(payload);
}

void StateTransitionGkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer(
    raw.payload.begin() + 1,
    requested_state);
}

/*
Control
*/
RawGkcPacket::SharedPtr ControlGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(13, 0);
  payload[0] = FIRST_BYTE;
  auto pos_steering = GkcPacketUtils::write_to_buffer(payload.begin() + 1, throttle);
  auto pos_brake = GkcPacketUtils::write_to_buffer(pos_steering, steering);
  GkcPacketUtils::write_to_buffer(pos_brake, brake);
  return std::make_unique<RawGkcPacket>(payload);
}

void ControlGkcPacket::decode(const RawGkcPacket & raw)
{
  auto pos_steering = GkcPacketUtils::read_from_buffer(raw.payload.begin() + 1, throttle);
  auto pos_brake = GkcPacketUtils::read_from_buffer(pos_steering, steering);
  GkcPacketUtils::read_from_buffer(pos_brake, brake);
}

/*
RCControl
*/
RawGkcPacket::SharedPtr RCControlGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(13, 0);
  payload[0] = FIRST_BYTE;
  auto pos_steering = GkcPacketUtils::write_to_buffer(payload.begin() + 1, throttle);
  auto pos_brake = GkcPacketUtils::write_to_buffer(pos_steering, steering);
  GkcPacketUtils::write_to_buffer(pos_brake, brake);
  return std::make_unique<RawGkcPacket>(payload);
}

void RCControlGkcPacket::decode(const RawGkcPacket & raw)
{
  auto pos_steering = GkcPacketUtils::read_from_buffer(raw.payload.begin() + 1, throttle);
  auto pos_brake = GkcPacketUtils::read_from_buffer(pos_steering, steering);
  GkcPacketUtils::read_from_buffer(pos_brake, brake);
}

/*
Sensors
*/
RawGkcPacket::SharedPtr SensorGkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(sizeof(SensorValues) + 1, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer(payload.begin() + 1, values);
  return std::make_unique<RawGkcPacket>(payload);
}

void SensorGkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer(
    raw.payload.begin() + 1,
    values);
}

/*
Shutdown 1
*/
RawGkcPacket::SharedPtr Shutdown1GkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(5, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint32_t>(payload.begin() + 1, seq_number);
  return std::make_unique<RawGkcPacket>(payload);
}

void Shutdown1GkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer<uint32_t>(
    raw.payload.begin() + 1,
    seq_number);
}

/*
Shutdown 2
*/
RawGkcPacket::SharedPtr Shutdown2GkcPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(5, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint32_t>(payload.begin() + 1, seq_number);
  return std::make_unique<RawGkcPacket>(payload);
}

void Shutdown2GkcPacket::decode(const RawGkcPacket & raw)
{
  GkcPacketUtils::read_from_buffer<uint32_t>(
    raw.payload.begin() + 1,
    seq_number);
}

/*
Log
*/
RawGkcPacket::SharedPtr LogPacket::encode() const
{
  GkcBuffer payload = GkcBuffer(what.size() + 1 + 1, 0);
  payload[0] = FIRST_BYTE;
  GkcPacketUtils::write_to_buffer<uint8_t>(payload.begin() + 1, static_cast<uint8_t>(level));
  std::copy(what.begin(), what.end(), payload.begin() + 2);
  return std::make_unique<RawGkcPacket>(payload);
}

void LogPacket::decode(const RawGkcPacket & raw)
{
  uint8_t level_num = 0;
  auto what_begin = GkcPacketUtils::read_from_buffer<uint8_t>(
    raw.payload.begin() + 1,
    level_num);
  level = static_cast<Severity>(level_num);
  what = std::string(what_begin, raw.payload.end());
}
}  // namespace gkc
}  // namespace tritonai
