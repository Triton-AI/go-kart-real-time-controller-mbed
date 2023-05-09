/**
 * @file logger.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-011
 *
 * @copyright Copyright 2022 Triton AI
 *
 */
#ifndef LOGGER_HPP_
#define LOGGER_HPP_
#include "tai_gokart_packet/gkc_packets.hpp"
#include <string>

namespace tritonai {
namespace gkc {
/**
 * @brief Logger interface
 * Abstract class for logger, allowing one to implement different loggers
 * of different severities and destinations.
 * 
 */
class ILogger {
public:
  ILogger() {}
  // sends a log packet to the logger with the given severity and message
  virtual void send_log(const LogPacket::Severity &severity,
                        const std::string &what) = 0;
};
} // namespace gkc
} // namespace tritonai
#endif
