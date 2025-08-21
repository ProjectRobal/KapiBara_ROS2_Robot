#pragma once

#include <cstdint>

struct point
{
    // 2d coordinates space
    int32_t x;
    int32_t y;
    // stationary position
    uint32_t w;

    float emotion_state;
};