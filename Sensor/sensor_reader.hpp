
#ifndef SENSOR_READER_HPP_
#define SENSOR_READER_HPP_
#include "watchable.hpp"
#include <cstdint>

// TODO add mbed libaray to create hardware interrupt

namespace tritonai{
namespace gkc{
    class SensorReader: Watchable{
        // TODO sendable API? link the class with comm manager
    public:
        SensorReader(uint32_t send_sensor_interval_ms);

        void send_sensor_values(); //TODO change void to sensor packet

    protected:

        uint32_t encoder_rpm; //create code to count and time inturrupts on motor
        float voltage;
        float steering_rad;
        float brake_pressure;
        float tire_temp;
        float tire_pressure;

        //TODO include QEI.h and add a pointer to qei
    };
}
}






#endif // SENSOR_READER_HPP_