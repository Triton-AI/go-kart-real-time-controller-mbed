#include "watchdog.hpp"
#include "mbed.h"
#include <chrono>
#include <functional>
#include <iostream>

namespace tritonai {
namespace gkc {
Watchdog::Watchdog(uint32_t update_interval_ms,
                   uint32_t max_inactivity_limit_ms, uint32_t wakeup_every_ms)
    : Watchable(update_interval_ms, max_inactivity_limit_ms),
      watchdog_interval_ms_(wakeup_every_ms) {
  add_to_watchlist(this);
}

void Watchdog::add_to_watchlist(Watchable *to_watch) {
  watchlist.push_back(WatchlistEntry(to_watch, 0));
}

void Watchdog::arm() {
  activate();
  running_ = true;
  watch.start(callback(this, &Watchdog::watch_thread));
}

void Watchdog::disarm() {
  if (watch.get_state() == Thread::State::Running) {
    running_ = false;
    watch.join();
  }
  running_ = false;

  // Reset inactivity timers
  for (auto entry : watchlist) {
    entry.second = 0;
  }
  deactivate();
}

void Watchdog::watchdog_callback() {
    std::cout << "Watchdog timeout" << std::endl;
}

void Watchdog::watch_thread() {
  auto time = Kernel::Clock::now();
  while (running_) {
    auto time_elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Kernel::Clock::now() - time);
    for (auto entry : watchlist) {
      if (entry.second > entry.first->get_update_interval()) {
        // Enough time has passed to checkup on this watchable object
        if (entry.first->check_activity() || !entry.first->is_activated()) {
          // Activity found. Reset counter.
          entry.second = 0;
        } else {
          // No activity. Increment inactivity counter.
          entry.second += time_elapsed_ms.count();
          if (entry.second > entry.first->get_max_inactivity_limit_ms()) {
            // Watchdog triggered.
            entry.first->watchdog_callback();
          }
        }
      } else {
        entry.second += time_elapsed_ms.count();
      }
    }
    inc_count();  // Signal the watchdog is alive
    ThisThread::sleep_for(std::chrono::milliseconds(watchdog_interval_ms_));
  }
}
} // namespace gkc
} // namespace tritonai