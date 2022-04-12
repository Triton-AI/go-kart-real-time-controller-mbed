/**
 * @file profiler.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-05
 *
 * @copyright Copyright 2022 Triton AI
 *
 */
#ifndef PROFILER_HPP_
#define PROFILER_HPP_
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mbed.h>
#include <sstream>
#include <string>
#include <vector>
#define AVERAGE_WINDOW_SIZE 10
namespace tritonai {
namespace gkc {
class Profiler {
public:
  Profiler(const char *name) : name_(name) {}
  void start_timer() {
    if (!profiling_) {
      timer_.reset();
      timer_.start();
      profiling_ = true;
    }
  }
  void stop_timer() {
    timer_.stop();
    profiling_ = false;
    if (buffer_.size() == AVERAGE_WINDOW_SIZE) {
      buffer_.erase(buffer_.begin());
    }
    buffer_.push_back(timer_.elapsed_time());
  }
  std::chrono::microseconds get_last_time() const { return buffer_.back(); }
  std::chrono::microseconds get_average_time() const {
    std::chrono::microseconds total(0);
    for (const auto &time : buffer_) {
      total += time;
    }
    return total / buffer_.size();
  }

  std::string get_name() const { return name_; }
  std::string dump(const bool &newline = true) const {
    std::stringstream ss;
    ss << "[Profiler " << get_name()
       << "]: last_time (us): " << get_last_time().count()
       << ", ave_time (us): " << get_average_time().count()
       << (newline ? "\n" : "\r");
    return ss.str();
  }

protected:
  Timer timer_;
  std::vector<std::chrono::microseconds> buffer_;
  std::string name_;
  bool profiling_{false};
};
} // namespace gkc
} // namespace tritonai
#endif
