
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
    class Watchdog{
    public:
       Watchdog();

       void watch(Watchable* to_watch);
       void watch_thread();

    protected:
    typedef std::vector<std::pair<Watchable*, uint32_t>> Watchlist;
        Watchlist watchlist;
        uint32_t watchdog_interval_ms = 0;


    };
}
}





#endif // WATCHDOG_HPP_