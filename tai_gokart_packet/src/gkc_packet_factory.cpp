/**
 * @file gkc_packet_factory.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2021-11-03
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */

#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include <cassert>
#include <memory>
#include <string>
#include <algorithm>
namespace tritonai
{
namespace gkc
{
GkcPacketFactory::GkcPacketFactory(GkcPacketSubscriber * sub, void(*debug)(std::string))
{
  this->_debug = debug;
  this->_sub = sub;
}

void GkcPacketFactory::Receive(const GkcBuffer & buffer)
{
  static constexpr int MIN_PACKET_SIZE = 6;
  static constexpr int MIN_PAYLOAD_SIZE = 1;
  static constexpr int NUM_NON_PAYLOAD_BYTE = 5;
  static constexpr int NUM_BYTE_BEFORE_PAYLOAD = 2;
  static int start_idx = 0;

  _buffer.reserve(_buffer.size() + buffer.size());
  _buffer.insert(_buffer.end(), buffer.begin(), buffer.end());

  // Look for the start byte
look_for_next_start: for (const auto & byte : _buffer) {
    if (byte == RawGkcPacket::START_BYTE) {
      break;
    }
    ++start_idx;
  }

  if (_buffer.size() <= static_cast<uint32_t>(start_idx + 1)) {
    // No packet start found. erase buffer.
    _buffer = GkcBuffer();
    start_idx = 0;
    return;
  }

  // Are there enough bytes to form a packet?
  if (_buffer.size() - start_idx >= MIN_PACKET_SIZE) {
    uint8_t payload_size = _buffer[start_idx + 1];
    if (static_cast<uint32_t>(start_idx + payload_size + NUM_NON_PAYLOAD_BYTE) > _buffer.size()) {
      // Need more bytes to complete a packet. Wait for the next receive.
      return;
    }
    // Check packet completeness
    if ((_buffer[start_idx + NUM_NON_PAYLOAD_BYTE + payload_size - 1] != RawGkcPacket::END_BYTE) ||
      (payload_size < MIN_PAYLOAD_SIZE))
    {
      _debug("Packet malformed. Potentially out-of-sync.");
      _buffer.erase(_buffer.begin(), _buffer.begin() + start_idx + 1);
      start_idx = 0;
      goto look_for_next_start;
    }

    // Find payload and checksum
    const GkcBuffer payload =
      GkcBuffer(
      _buffer.begin() + start_idx + NUM_BYTE_BEFORE_PAYLOAD,
      _buffer.begin() + start_idx + NUM_BYTE_BEFORE_PAYLOAD + payload_size);
    uint16_t checksum =
      *reinterpret_cast<uint16_t *>(&_buffer[start_idx + NUM_BYTE_BEFORE_PAYLOAD + payload_size]);

    // Check checksum
    if (GkcPacketUtils::calc_crc16(payload) != checksum) {
      _debug("Possible packet corruption. Dropping packet.");
      _buffer.erase(
        _buffer.begin(),
        _buffer.begin() + start_idx + NUM_NON_PAYLOAD_BYTE + payload_size);
      start_idx = 0;
      goto look_for_next_start;
    }

    auto raw_packet = RawGkcPacket(payload);
    uint8_t fb = payload[0];
    auto packet = fb_lookup.find(fb)->second();
    packet->decode(raw_packet);
    packet->publish(*(this->_sub));

    // One packet found. Are there others?
    // First erase the parsed packet from buffer
    start_idx = 0;
    _buffer.erase(
      _buffer.begin(),
      _buffer.begin() + start_idx + NUM_NON_PAYLOAD_BYTE + payload_size);
    // Then go to look for the next packet
    goto look_for_next_start;
  } else {
    // Need more bytes to complete a packet. Wait for the next receive.
    return;
  }
}

std::shared_ptr<GkcBuffer> GkcPacketFactory::Send(const GkcPacket::SharedPtr & packet)
{
  return packet->encode()->encode();
}
std::shared_ptr<GkcBuffer> GkcPacketFactory::Send(const GkcPacket & packet)
{
  return packet.encode()->encode();
}
}  // namespace gkc
}  // namespace tritonai
