#include "watchdog.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <string>

namespace tritonai {
namespace gkc {   //defines a constructor for a class called "Watchdog" in the "gkc" namespace, which is a part of the "tritonai" namespace. 
Watchdog::Watchdog(uint32_t update_interval_ms,
                   uint32_t max_inactivity_limit_ms, uint32_t wakeup_every_ms)
    : Watchable(update_interval_ms, max_inactivity_limit_ms, "Watchdog"), 
    //Watchable constructor is called with the "update_interval_ms" and "max_inactivity_limit_ms" arguments
    //the result is used to initialize the Watchdog object.
      watchdog_interval_ms_(wakeup_every_ms) { 
        //"watchdog_interval_ms_" member variable is initialized with the "wakeup_every_ms" argument.
  add_to_watchlist(this);//The constructor then adds the current Watchdog object to the watchlist for monitoring purposes
  attach(callback(this, &Watchdog::watchdog_callback));//attaches a callback function named "watchdog_callback" to the Watchable object
  watch_thread.start(callback(this, &Watchdog::start_watch_thread));
  //starts a new thread by calling the "start_watch_thread" function.
}

// takes a pointer to an object of type Watchable as its argument, named to_watch.
void Watchdog::add_to_watchlist(Watchable *to_watch) {
  watchlist.push_back(WatchlistEntry(to_watch, 0));
  //function adds an element to a watchlist maintained by the Watchdog class, 
  //intialized at 0 
}

void Watchdog::arm() { activate(); }//calls the activate function of the Watchdog object

void Watchdog::disarm() {
  deactivate();
  // Basically resets inactivity timers  for each entry in the watchlist 
  for (auto entry : watchlist) {//iterates over each element of the 'watchlist'
    entry.second = 0;//set the secondmember of the 'watchhlistEntry object to 0 
  }
}

void Watchdog::watchdog_callback() {//would be called when the watchdog timer expires 
  std::cout << "Watchdog timeout" << std::endl;
  // Restart the system 
  NVIC_SystemReset();
}
//start a thread that continuously monitors the watchlist nd checks for any activity.
void Watchdog::start_watch_thread() {
  static auto last_time = Kernel::Clock::now();//initializes a static variable last_time with the current time
  while (1) {//infinite while  loop 
    if (is_activated()) {
      auto time_elapsed_ms = //calculates the time elapsed since the last iteration
          std::chrono::duration_cast<std::chrono::milliseconds>(
              Kernel::Clock::now() - last_time); //updates inactivity timers for each element in the watchlist vector
      for (auto &entry : watchlist) {//check each element in the watchlist vector 
        if (entry.second > entry.first->get_update_interval()) {
          // Enough time has passed to checkup on this watchable object
          if (entry.first->check_activity() || !entry.first->is_activated()) {
            // Activity found. Reset counter.
            entry.second = 0;
            std::cout << "Activity found in " << entry.first->get_name()
                      << std::endl; 
          } else {
            // No activity. Increment inactivity counter.
            entry.second += time_elapsed_ms.count();
            if (entry.second > entry.first->get_max_inactivity_limit_ms()) {
              // inactivity timer exceeds the maximum inactivity limit
              // Watchdog triggered.
              entry.first->watchdog_trigger();
            }
          }
        } else {
          entry.second += time_elapsed_ms.count();
        }
      }
    }

    inc_count(); // Signal the watchdog is alive
    last_time = Kernel::Clock::now();//update with current time 
    ThisThread::sleep_for(std::chrono::milliseconds(watchdog_interval_ms_));
  }
}
} // namespace gkc
} // namespace tritonai