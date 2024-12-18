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

#include <map>

#include "frame.hpp"

#include "datatypes.h"

#include "common.hpp"


class CANBridge
{
    
    int can_sock;

    // CanFrame frames[PACKET_TYPE_COUNT];

    std::map<packet_type_t,CanFrame> frames;

    public:

    CANBridge()
    {
        this->frames[PING] = CanFrame(sizeof(ping_msg),PING);
        this->frames[IMU] = CanFrame(sizeof(imu_raw_t),IMU);
        this->frames[Orientation] = CanFrame(sizeof(orientation_t),Orientation);
        this->frames[Encoder] = CanFrame(sizeof(encoder_t),Encoder);
        this->frames[TOF] = CanFrame(sizeof(tof_t),TOF);
        this->frames[GENERAL_CFG_DATA] = CanFrame(CONFIG_MAX_BUFFER_SIZE,GENERAL_CFG_DATA);
        this->frames[MAG] = CanFrame(sizeof(mag_t),MAG);
        
        this->frames[ACK] = CanFrame(sizeof(ack_msg_t),ACK);

    }

    void set_recive_size(uint32_t size)
    {
        this->frames[GENERAL_CFG_DATA].overide_size(size);
    }

    bool start(const char* can_name,uint32_t id=VEL_SALTIS_RX_ID,uint32_t mask=0xF);

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