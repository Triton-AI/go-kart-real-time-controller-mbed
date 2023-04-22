#include "mbed.h"
#include "InterfaceCAN.h"
#include <vector>

class CANOpener {
public:
    CANOpener(PinName rd, PinName td, int hz, const uint32_t vesc_id);
    bool move_motor_cur(int32_t current);
    bool move_motor_pos(int32_t pos);
    bool move_motor_pos_at_rpm(int32_t pos, int rpm);
    bool stop_motor();
    bool read_pos(int &pos_now);
    bool read_erpm(int &erpm_now);

private:
    CAN* can;
    uint32_t VESC_ID;

    static constexpr uint32_t FORMULATE_ID(const uint32_t &vesc_id, const uint32_t &mode_id) {
        return (static_cast<uint32_t>(mode_id)
            << sizeof(uint8_t) * 8) |
            static_cast<uint32_t>(vesc_id);
    }

    // Function to encode message within uint8_t array
    void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) {
        buffer[(*index)++] = number >> 24;
        buffer[(*index)++] = number >> 16;
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
    }
};