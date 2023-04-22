#include "CANOpener.hpp"

#define VESC_CURRENT_EXTENDED_ID 1
#define VESC_POS_EXTENDED_ID 4
#define VESC_CAN_PACKET_STATUS_1 9
#define VESC_CAN_PACKET_STATUS_4 16

using namespace tritonai::gkc;

CANOpener::CANOpener(PinName rd, PinName td, int hz, uint32_t vesc_id) {
    this->can = new CAN(rd, td, hz);
    this->VESC_ID = vesc_id;
}

bool CANOpener::move_motor_cur(int32_t current) {
    uint8_t message[4] = {0, 0, 0, 0};
    int32_t idx = 0;

    buffer_append_uint32(&message[0], current, &idx);
    CANMessage msg = CANMessage(FORMULATE_ID(this->VESC_ID, VESC_CURRENT_EXTENDED_ID), &message[0],
                                  sizeof(message), CANData, CANExtended);

    return this->can->write(msg);
}

bool CANOpener::stop_motor() {
    return this->move_motor_cur(0);
}

bool CANOpener::move_motor_pos(int32_t pos) {
    uint8_t message[4] = {0, 0, 0, 0};
    int32_t idx = 0;

    buffer_append_uint32(&message[0], pos * 1000000, &idx);
    CANMessage msg = CANMessage(FORMULATE_ID(this->VESC_ID, VESC_POS_EXTENDED_ID), &message[0],
                                  sizeof(message), CANData, CANExtended);

    return this->can->write(msg);
}

bool CANOpener::read_pos(int &pos_now) {
    CANMessage msg;

    if(this->can->read(msg)) {

        if(msg.id == FORMULATE_ID(this->VESC_ID, VESC_CAN_PACKET_STATUS_4)) {
            pos_now = (msg.data[7] | msg.data[6] << 8) / 50;
        }

        return true;
    }
    return false;
}


bool CANOpener::read_erpm(int &erpm_now) {
    CANMessage msg;

    if(this->can->read(msg)) {

        if(msg.id == FORMULATE_ID(this->VESC_ID, VESC_CAN_PACKET_STATUS_1)) {
            erpm_now = static_cast<int>((msg.data[0] << 24) | (msg.data[1] << 16) | (msg.data[2] << 8) | msg.data[3]);
        }

        return true;
    }
    return false;
}