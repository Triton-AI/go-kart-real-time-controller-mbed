#ifndef WATCHABLE_HPP_
#define WATCHABLE_HPP_

#include <stdint.h>
#include <iostream>
#include <string>
#include "mbed.h"

namespace tritonai {
namespace gkc {
class Watchable {
public:
  Watchable(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms)
      : update_interval_ms(update_interval_ms),
        max_inactivity_limit_ms(max_inactivity_limit_ms) {}

  void activate() { active = true; }
  void deactivate() { active = false; }
  void inc_count() { ++rolling_counter; }
  uint32_t get_update_interval() { return update_interval_ms; }
  void set_update_interval(const uint32_t &update_interval_ms) {
    this->update_interval_ms = update_interval_ms;
  }
  bool is_activated() { return active; }
  uint32_t get_max_inactivity_limit_ms() { return max_inactivity_limit_ms; }
  virtual bool check_activity() {
    bool activity = last_check_rolling_counter_val != rolling_counter;
    last_check_rolling_counter_val = rolling_counter;
    return activity;
  }
  void attach(Callback<void ()> func) { callback_func_ = func; }
  void watchdog_trigger() { callback_func_(); }

protected:
  bool active = false;
  uint32_t rolling_counter = 0;
  uint32_t update_interval_ms = 0;
  uint32_t max_inactivity_limit_ms = 0;
  Callback<void ()> callback_func_;
private:
  uint32_t last_check_rolling_counter_val = 0;
};
} // namespace gkc
} // namespace tritonai

#endif // WATCHABLE_HPP_
