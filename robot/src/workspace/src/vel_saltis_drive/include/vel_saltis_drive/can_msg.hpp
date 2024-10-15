#pragma once

#include <cstdint>

enum packet_type_t
{
    Encoder=3
};

struct motor_msg_t
{
    int16_t speed_left;
    int16_t speed_right;
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