#pragma once

#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "datatypes.h"

class CanFrame
{
    packet_type_t type;
    uint16_t size;
    uint32_t packet_size;
    uint8_t* data;

    public:
    
    CanFrame(uint32_t packet_size,packet_type_t type);

    void read(const uint8_t* data,uint16_t size,uint16_t offset);

    template<typename T>
    const T* to() const
    {
        return (T*)this->data;
    }

    bool ready()
    {
        return this->size >= this->packet_size;
    }

    packet_type_t getType() const
    {
        return this->type;
    }

    ~CanFrame()
    {
        delete [] this->data;
    }
};