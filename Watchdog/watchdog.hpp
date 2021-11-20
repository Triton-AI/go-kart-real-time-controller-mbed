
#ifndef WATCHDOG_HPP_
#define WATCHDOG_HPP_

#include <cstdint>
#include <stdint.h>
#include <utility>
#include <vector>

#include "watchable.hpp"

using tritonai::gkc::Watchable;
namespace tritonai{
namespace gkc{
    class Watchdog: public Watchable{
    public:
       Watchdog(uint32_t update_interval_ms, uint32_t max_inactivity_limit_ms);

       void add_to_watchlist(Watchable* to_watch);
       void arm();
       void disarm();

       void watchdog_callback(); // Watchable API

    protected:
    typedef std::vector<std::pair<Watchable*, uint32_t>> Watchlist;
        Watchlist watchlist;
        uint32_t watchdog_interval_ms = 0;
    
        void watch_thread_();

    };
}
}





#endif // WATCHDOG_HPP_