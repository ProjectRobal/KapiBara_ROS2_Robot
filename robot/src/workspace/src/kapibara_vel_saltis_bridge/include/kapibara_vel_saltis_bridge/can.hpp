#pragma once

#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

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

#include "datatypes.h"

#include "common.hpp"


class CANBridge
{
    
    int can_sock;

    CanFrame frames[PACKET_TYPE_COUNT];

    public:

    CANBridge()
    : frames{
        CanFrame(sizeof(ping_msg),PING),
        CanFrame(sizeof(imu_raw_t),IMU),
        CanFrame(sizeof(orientation_t),Orientation),
        CanFrame(sizeof(encoder_t),Encoder),
        CanFrame(sizeof(tof_t),TOF),
        CanFrame(CONFIG_MAX_BUFFER_SIZE,GENERAL_CFG_DATA),
        CanFrame(sizeof(ack_msg_t),ACK)
    }
    {}

    void set_recive_size(uint32_t size)
    {
        this->frames[5].overide_size(size);
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