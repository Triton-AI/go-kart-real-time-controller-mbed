#include <stdint.h>
#include "mbed.h"


// default values are for BETAFPV
// either change header for the message type or 
// pass it into the connstructor
#define EXPECTED_CRFS_SIZE 0x16
#define DESIZED_CRFS_TYPE 0x18
#define ELRS_TARGET 0xc8

//elrs bit format as descrived in
typedef struct crsf_channels_s
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED crsf_channels_t;

class elrc_receiver{
public:
    elrc_receiver(PinName RX_pin, PinName TX_pin, 
        char messageTargetIn = ELRS_TARGET, char messageTypeIn = DESIZED_CRFS_TYPE,
        char messageSizeIn = EXPECTED_CRFS_SIZE);
    
    // call it every time you want to gather data
    // returns success or no success on gathering data
    bool gatherData();

    //changes from 11 bit structure to array to make easier to access
    void structToArray();

    //returns pointer to array of data values
    const uint16_t* busData(){ 
        messageAvailable = false;
        return busValues; }

    //tells whether new values have been created since
    // last time data was accessed
    bool messageAvailable;

private:
    //where the serial is to be writen to
    char payload[256];

    //struct to copy the memory into
    crsf_channels_t structuredChannel;

    //change depending on the message type and device type
    char messageSize;
    char messageType;
    char messageTarget;

    //formated array access
    uint16_t busValues[16];

    //serial port
    BufferedSerial serial_port;
};
