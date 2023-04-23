# Gokart Packet Library

## Introduction

This packet library includes

- `gkc_packets.hpp`: class definition of all packets;
- `gkc_packet_factory.hpp`: factory class for encoding and decoding packets;
- `gkc_packet_subscriber.hpp`: interface for receiving packets from the packet factory.
- `gkc_packet_utils.hpp`: utility functions for working with packets.

Note:

1. `gkc` means "gokart controller";
2. All packets are subclasses of `GkcPacket`;

## How to use

Suppose you are writing a microcontroller code that uses this library to communicate with the PC.

1. Have two of your classes ready: one presumably called `message_handler` that takes care of the inbound/outbound messages on the high level. The other one maybe called `comm_handler` that directly reads/writes bytes onto some communication lines.
2. Your `message_handler` needs to subclass `GkcPacketSubscriber` and implements all of its interface methods. They are the different callbacks when the factory receives every message.
3. Declare an instance of `GkcPacketFactory` by passing your `message_handler` and a debug callback (maybe `&GkcPacketUtils::debug_cout`) to the constructor. Your `GkcPacketFactory` could live inside your `message_handler`, for example.
4. Your `comm_handler` should convert incoming bytes into a `std::vector<uint8_t>` which is typedef-ed as `GkcBuffer` should you include `gkc_packets.hpp`. Then it should call `GkcPacketFactory::Receive` and pass the buffer to the factory. The factory will parse the bytes and trigger the callbacks to your `message_handler`.
5. When your `message_handler` whats to send out messages, It should call `GkcPacketFactory::Send` and be handed back with a `GkcBuffer` ready to be sent down the communication line using your `comm_handler`.

Note:

1. Make sure that the `GkcBuffer` passed to `GkcPacketFactory::Receive` only includes valid data stream. For example, Do not initialize a vector of size 256, dump the 42 bytes received into it, and call the callback on the entire vector - this will corrupt the packets, and some low-level communication libraries do this. Use something like `num_bytes_received` to remove out-of-range bytes.
2. Preallocate space when initializing `GkcBuffer`. For example:
```cpp
#include <memory>
#include "tai_gokart_packet/gkc_packets.hpp"

// your incoming buffer
unsigned char* some_buffer = ...
// length of the buffer
int len_buffer = 16;

// pre-allocate enough space
GkcBuffer gkc_buffer(len_buffer, 0);
// fast copy
std::copy(some_buffer, some_buffer + len_buffer, gkc_buffer.begin());

// Pass the buffer to the factory. 
packet_factory.receive(gkc_buffer);
```

## An Example

```cpp
#include <memory>
#include "tai_gokart_packet/gkc_packets.hpp"
#include "your_project/child_of_gkc_packet_subscriber.hpp"

// `your_project/child_of_gkc_packet_subscriber.hpp` contains a derived class of GkcPacketSubscriber
int main()
{
 auto packet_sub = ChildOfGkcPacketSubscriber();
 auto factory = tritonai::gkc::GkcPacketFactory(&sub, tritonai::gkc::GkcPacketUtils::debug_cout);

 // Example 1: I want to send a log packet
 auto packet = tritonai::gkc::LogPacket();
 packet.level = tritonai::gkc::LogPacket::Severity::FATAL;
 packet.what = "Hello World";
 // Let the factory encode it into a buffer (std::vector<uint8_t>)
 auto buffer_to_send = factory.Send(packet);
 // Now I call some random serial library to send it out
 Serial.write(buffer_to_send);

 // Example 2: I have an incoming buffer of stuff to be decoded
 while (true)  // Keep receiving
 {
  static auto recv_buffer = tritonai::gkc::GkcBuffer(2048, 0);
  // Read something using a random serial library that tells me how many bytes are read
  auto bytes_received = Serial.receive(GkcBuffer);
  // Take the valid range of the buffer
  auto valid_buffer_received = 
       tritonai::gkc::GkcBuffer(recv_buffer.begin(), recv_buffer.begin() + bytes_received);
  // Pass the valid received bytes to the factory
  factory.Receive(valid_buffer_received);
  // Now `packet_sub` will have its callback triggered,
  // if valid packets exist.
 }

 return 0;
}

```

## Packet API

Review the [packet API documentation](Packet_API.md) for more details.
