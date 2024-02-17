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
/**
 * @brief Provides functionality for profiling code performance
 * 
 */
class Profiler {
public:
/**
 * @brief Construct a new Profiler object
 * @param name of the section being profiled
 */
  Profiler(const char *name) : name_(name) {}

/**
 * @brief Start the timer
 * Resets the timer and starts it
 */
  void start_timer() {
    if (!profiling_) {
      timer_.reset();
      timer_.start();
      profiling_ = true;
    }
  }
/**
 * @brief Stops the timer
 * 
 */
  void stop_timer() {
    timer_.stop();
    profiling_ = false;
    if (buffer_.size() == AVERAGE_WINDOW_SIZE) {
      buffer_.erase(buffer_.begin());
    }
    buffer_.push_back(timer_.elapsed_time());
  }
/**
 * @brief Getter for the last time on the buffer
 * 
 * @return std::chrono::microseconds 
 */
  std::chrono::microseconds get_last_time() const { return buffer_.back(); }
/**
 * @brief Getter for the average time
 * Gets the average time object by summing up all the times in the buffer and
 * dividing by the size of the buffer
 * 
 * @return std::chrono::microseconds 
 */
  std::chrono::microseconds get_average_time() const {
    std::chrono::microseconds total(0);
    for (const auto &time : buffer_) {
      total += time;
    }
    return total / buffer_.size();
  }
/**
 * @brief Getter for the name object
 * 
 * @return std::string 
 */
  std::string get_name() const { return name_; }
/**
 * @brief Dumps the profiler information to a string
 * Gives the profiler, the last time, and the average time.
 * @param newline whether to add a newline character at the end of the string
 * @return std::string 
 */
  std::string dump(const bool &newline = true) const {
    std::stringstream ss;
    ss << "[Profiler " << get_name()
       << "]: last_time (us): " << get_last_time().count()
       << ", ave_time (us): " << get_average_time().count()
       << (newline ? "\n" : "\r");
    return ss.str();
  }
/**
 * @brief Protected variables and functions for the profiler
 * timer is the timer object
 * buffer is the buffer of times
 * name is the name of the profiler
 */
protected:
  Timer timer_;
  std::vector<std::chrono::microseconds> buffer_;
  std::string name_;
  bool profiling_{false};
};
} // namespace gkc
} // namespace tritonai
#endif
