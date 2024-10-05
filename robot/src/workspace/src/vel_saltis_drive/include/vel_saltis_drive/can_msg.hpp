#pragma once

#include <cstdint>

struct motor_msg_t
{
    int16_t speed_left;
    int16_t speed_right;
};

struct speed_msg_t
{
    int32_t speed_left;
    int32_t speed_right;
    int32_t distance_left;
    int32_t distance_right;
};