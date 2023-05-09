#include "elrs_receiver.hpp"

elrc_receiver::elrc_receiver(PinName RX_pin, PinName TX_pin, 
        char targetIn, char message_typeIn,char message_sizeIn): serial_port(RX_pin,TX_pin){
    messageSize = message_sizeIn;
    messageType = message_typeIn;
    messageTarget = targetIn;    
    messageAvailable = false;
    serial_port.set_baud(115200);
    serial_port.set_format(
    /* bits */ 8,
    /* parity */ SerialBase::None,
    /* stop bit */ 1
    );
    serial_port.set_blocking(false);
}


bool elrc_receiver::gatherData(){
    void* channelsPointer = (void*)(&structuredChannel);
    messageAvailable = false;

    int messageBufferRequest = messageSize*2+6;

    bool hasRead = serial_port.read(payload,256);
    if(!hasRead)return messageAvailable;
    //printf("%x %x %x ", payload[0], payload[1], payload[2]);
    //printf("%x %x %x ", payload[3], payload[4], payload[5]);
    //printf("%x %x %x\n", payload[6], payload[7], payload[8]);
    int sm;
    for(sm = 0; !(payload[sm] == messageTarget && payload[sm+1] == messageType
    && payload[sm+2] == messageSize) && sm < messageSize; sm++);
    if(sm >= messageSize) return messageAvailable;
    
    messageAvailable = true;

    //serial_port.sync();
    std::memcpy(channelsPointer, (void*)(payload+sm+3), 22);
    structToArray();        
    serial_port.sync();
    return messageAvailable;
}

void elrc_receiver::structToArray() {
    busValues[0] = structuredChannel.ch0;
    busValues[1] = structuredChannel.ch1;
    busValues[2] = structuredChannel.ch2;
    busValues[3] = structuredChannel.ch3;
    busValues[4] = structuredChannel.ch4;
    busValues[5] = structuredChannel.ch5;
    busValues[6] = structuredChannel.ch6;
    busValues[7] = structuredChannel.ch7;
    busValues[8] = structuredChannel.ch8;
    busValues[9] = structuredChannel.ch9;
    busValues[10] = structuredChannel.ch10;
    busValues[11] = structuredChannel.ch11;
    busValues[12] = structuredChannel.ch12;
    busValues[13] = structuredChannel.ch13;
    busValues[14] = structuredChannel.ch14;    
    busValues[15] = structuredChannel.ch15;    
}