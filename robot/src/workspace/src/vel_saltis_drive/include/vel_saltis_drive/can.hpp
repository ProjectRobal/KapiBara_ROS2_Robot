#pragma once

#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include <map>

extern "C"
{
    #include <unistd.h>
    #include <net/if.h>
    #include <sys/ioctl.h>
    #include <sys/socket.h>

    #include <linux/can.h>
    #include <linux/can/raw.h>

}

#include "frame.hpp"

#include "can_msg.hpp"

#include "common.hpp"


class CANBridge
{
    
    int can_sock;

    // CanFrame frames[PACKET_TYPE_COUNT];

    std::map<packet_type_t,CanFrame> frames;

    public:

    CANBridge()
    {
        this->frames[Encoder] = CanFrame(sizeof(speed_msg_t),Encoder);
    }

    bool start(const char* can_name,uint32_t id=VEL_SALTIS_ID,uint32_t mask=0xF);

    /*
        data - data to send
        size - data size
        target - CAN bus id
        id - a device register bank id
        offset - register bank offset in bytes
    */
    void send(uint8_t* data,uint32_t size,uint16_t target,uint16_t id);

    // send just header
    void send_id(uint16_t target,uint16_t id);

    const CanFrame* recive();

    ~CANBridge()
    {
        close(this->can_sock);
    }

};