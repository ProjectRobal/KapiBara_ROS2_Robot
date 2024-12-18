#include "kapibara_twingo_bridge/frame.hpp"

CanFrame::CanFrame(uint32_t packet_size,packet_type_t type)
    {
        this->packet_size = packet_size;

        this->data = new uint8_t[this->packet_size];
        this->type = type;
        this->size=0;
    }

void CanFrame::read(const uint8_t* data,uint16_t size,uint16_t offset)
    {
        // when we recive first packet we reset buffer
        if( this->ready() || offset == 0 )
        {
            this->size = 0;
        }

        // memcpy(((uint8_t*)this->data)+offset,data,size)        

        for(size_t i=0;i<size;++i)
        {
            this->data[i+offset] = data[i];
            
            // if(this->type == packet_type_t::Encoder)
            // {  
            //     std::cout<<(int32_t)this->data[i+offset]<<" ";
            // }
        }

        this->size += size;
        
    }