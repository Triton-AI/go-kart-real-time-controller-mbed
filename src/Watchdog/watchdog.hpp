#ifndef WATCHDOG_HPP_
#define WATCHDOG_HPP_

#include <cstdint>
#include <stdint.h>
#include <utility>
#include <vector>

#include "watchable.hpp"

using tritonai::gkc::Watchable;
namespace tritonai {
namespace gkc {
class Watchdog : public Watchable {
public:
  Watchdog() = delete;
  Watchdog(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms,
           uint32_t wakeup_every_ms);

  void add_to_watchlist(Watchable *to_watch);
  void arm();
  void disarm();

  void watchdog_callback(); // Watchable API

protected:
  typedef uint32_t TimeElapsed;
  typedef std::pair<Watchable *, TimeElapsed> WatchlistEntry;
  typedef std::vector<WatchlistEntry> Watchlist;
  Watchlist watchlist{};
  Thread watch_thread{osPriorityNormal, OS_STACK_SIZE, nullptr, "watch_thread"};
  uint32_t watchdog_interval_ms_;

  void start_watch_thread();
};
} // namespace gkc
} // namespace tritonai
#endif // WATCHDOG_HPP_
