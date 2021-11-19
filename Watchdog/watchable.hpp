#ifndef WATCHABLE_HPP_
#define WATCHABLE_HPP_

#include <stdint.h>

namespace tritonai{
namespace gkc{
    class Watchable{
    public:
        Watchable();

        void activate(){ active = true; }
        void deactivate(){ active = false; }
        void inc_count(){ ++rolling_counter; }
        uint32_t get_update_interval(){ return update_interval_ms; }
        void set_update_interval(const uint32_t& update_interval_ms){ this->update_interval_ms = update_interval_ms; }
        // TODO getter and setter for max_inactivity_limit_ms
        bool check_activity(){ 
            bool activity = last_check_rolling_counter_val != rolling_counter; 
            last_check_rolling_counter_val = rolling_counter;
            return activity;
        }

    protected:
        bool active = false;
        uint32_t rolling_counter = 0;
        uint32_t last_check_rolling_counter_val = 0;
        uint32_t update_interval_ms = 0;
        uint32_t max_inactivity_limit_ms = 0;
        
    };
}
}





#endif // WATCHABLE_HPP_