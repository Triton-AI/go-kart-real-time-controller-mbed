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
#include "gkc_packets.hpp"
#include <string>

namespace tritonai {
namespace gkc {
class ILogger {
public:
  ILogger() {}
  virtual void send_log(const LogPacket::Severity &severity,
                        const std::string &what) = 0;
};
} // namespace gkc
} // namespace tritonai
#endif
