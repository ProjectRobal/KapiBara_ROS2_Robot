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
        CanFrame(sizeof(ack_msg_t),ACK),
        CanFrame(sizeof(imu_cfg_t),IMU_CFG),
        CanFrame(sizeof(aux_cfg_t),AUX_CFG),
        CanFrame(sizeof(pid_cfg_t),PID_CFG),
        CanFrame(sizeof(servo_cfg_t),SERVO_CFG),
        CanFrame(sizeof(motor_cfg_t),MOTOR_CFG),
        CanFrame(sizeof(fusion_cfg_t),FUSION_CFG),
    }
    {}

    bool start(const char* can_name);

    /*
        data - data to send
        size - data size
        target - CAN bus id
        id - a device register bank id
        offset - register bank offset in bytes
    */
    void send(uint8_t* data,uint32_t size,uint16_t target,uint16_t id,uint16_t offset);

    const CanFrame* recive();

    ~CANBridge()
    {
        close(this->can_sock);
    }

};