#include <iostream>
#include <string>

#include "mbed.h"
#include "watchable.hpp"
#include "watchdog.hpp"
// main() runs in its own thread in the OS
// START
class Blinker : public tritonai::gkc::Watchable {
public:
  Blinker(uint32_t a, uint32_t b) : Watchable(a, b) {
    attach(callback(this, &Blinker::watchdog_callback));
  }
  int BlinkLoop() {
    // Initalizing User LEDs
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
    DigitalOut led3(LED3);
    InterruptIn button(
        PC_13); // Interrupts MCU when Blue Button (PC_13) is Pressed
    button.rise(callback(this, &Blinker::add_counter));
    while (true) {
      // Bit counter
      led1 = counter & 1; // counter mod 1 to turn on LED1
      led2 = counter & 2; // counter mod 2 to turn on LED2
      led3 = counter & 4; // counter mod 4 to turn on LED3
    }
  }
  void watchdog_callback() {
    // NVIC_SystemReset();
    std::cout << "watchdog triggered" << std::to_string(rolling_counter) << std::endl;
  } // ???
private:
  void add_counter() {
    counter++;
    inc_count();
  }
  int counter = 0;
};
int main() {
  Blinker blinker(100, 5000); // Object named after its Class
  blinker.activate();
  tritonai::gkc::Watchdog watchdog(100, 5000, 10);
  watchdog.add_to_watchlist(&blinker);
  watchdog.arm();
  blinker.BlinkLoop();
}
