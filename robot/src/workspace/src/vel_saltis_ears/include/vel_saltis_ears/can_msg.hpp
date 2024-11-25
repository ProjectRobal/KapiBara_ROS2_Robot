#pragma once

#include <cstdint>

enum packet_type_t
{
    Encoder=3
};

struct motor_msg_t
{
    int32_t speed_left;
    int32_t speed_right;
};

struct servo_msg_t
{
    uint8_t left;
    uint8_t right;
};

#define ENCODER_ID 3

struct speed_msg_t
{
    int32_t speed_left;
    int32_t speed_right;
    int32_t distance_left;
    int32_t distance_right;
    int32_t raw_left;
    int32_t raw_right;
};