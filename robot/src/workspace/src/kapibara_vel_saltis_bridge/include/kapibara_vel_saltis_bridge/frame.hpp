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

    void overide_size(uint32_t size)
    {
        this->packet_size = size;
        this->size = 0;   
    }

    void overide_type(packet_type_t type)
    {
        this->type = type;
    }

    void read(const uint8_t* data,uint16_t size,uint16_t offset);

    template<typename T>
    const T* to() const
    {
        return (T*)this->data;
    }

    void dump(uint8_t buff[])
    {
        for(size_t i=0;i<this->packet_size;++i)
        {
            buff[i] = this->data[i];
        }
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