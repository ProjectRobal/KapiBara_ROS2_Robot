#include "kapibara_vel_saltis_bridge/can.hpp"


bool CANBridge::start(const char* can_name)
    {
        if ((this->can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            std::cerr<<"Unable to open can socket: "<<can_name<<std::endl;
            return false;
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_name);
        ioctl(this->can_sock, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(this->can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            std::cerr<<"Unable to bind can socket: "<<can_name<<std::endl;
            return false;
        }

        return true;
    }

const CanFrame* CANBridge::recive()
    {
        int nbytes;
        struct can_frame frame;

        nbytes = read(this->can_sock, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            std::cerr<<"Nothing to read from CAN!"<<std::endl;
        }

        if(frame.can_dlc<=3)
        {
            return NULL;
        }

        uint8_t id = frame.data[0];

        if( id >= PACKET_TYPE_COUNT )
        {
            return NULL;
        }

        CanFrame* base = &this->frames[id];

        uint16_t offset = 0;

        memcpy((uint8_t*)&offset,frame.data+1,sizeof(uint16_t));

        base->read(frame.data+3,frame.can_dlc-3,offset);

        if( base->ready() )
        {
            return base;
        }

        return NULL;
    }

    /*
        data - data to send
        size - data size
        target - CAN bus id
        id - a device register bank id
        offset - register bank offset in bytes
    */
void CANBridge::send(uint8_t* data,uint32_t size,uint16_t target,uint16_t id,uint16_t offset)
    {

        uint32_t data_offset = 0;

        while(size>0)
        {
            struct can_frame frame;
            frame.can_id = target;

            frame.data[0] = id;
            frame.data[1] = offset & 0xFF;
            frame.data[2] = ( offset>>8 ) & 0xFF;

            uint8_t data_to_send = std::min(static_cast<uint32_t>(5),size);

            frame.can_dlc = data_to_send+3;

            for(uint8_t i=0;i<data_to_send;++i)
            {
                frame.data[3+i] = data[i+data_offset];
            }

            if (write(this->can_sock, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                std::cerr<<"Cannot send frame!"<<std::endl;
            }

            size -= data_to_send;
            offset += data_to_send;
            data_offset += data_to_send;

        }

    }