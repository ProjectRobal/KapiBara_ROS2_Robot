#include "kapibara_vel_saltis_bridge/frame.hpp"

CanFrame::CanFrame(uint32_t packet_size,packet_type_t type)
    {
        this->packet_size = packet_size;

        if(type == packet_type_t::Encoder)
        {
            std::cout<<"Packet size: "<<this->packet_size<<std::endl;
        }


        this->data = new uint8_t[this->packet_size];
        this->type = type;
        this->size=0;
    }

void CanFrame::read(const uint8_t* data,uint16_t size,uint16_t offset)
    {
        if(this->ready())
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