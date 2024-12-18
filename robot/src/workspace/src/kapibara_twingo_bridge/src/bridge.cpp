#include <iostream>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

extern "C"
{
    #include <unistd.h>
    #include <net/if.h>
    #include <sys/ioctl.h>
    #include <sys/socket.h>

    #include <linux/can.h>
    #include <linux/can/raw.h>

}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>


#include <kapibara_interfaces/msg/encoders_and_speed.hpp>

#include <kapibara_interfaces/msg/can_ping.hpp>
#include <kapibara_interfaces/msg/piezo_sense.hpp>

#include "kapibara_twingo_bridge/can.hpp"


// services include

#include <vel_saltis_services/srv/get_imu_cfg.hpp>
#include <vel_saltis_services/srv/set_imu_cfg.hpp>

#include <vel_saltis_services/srv/get_fusion_cfg.hpp>
#include <vel_saltis_services/srv/set_fusion_cfg.hpp>

#include <vel_saltis_services/srv/get_pidcfg.hpp>
#include <vel_saltis_services/srv/set_pidcfg.hpp>

#include <vel_saltis_services/srv/get_servo_cfg.hpp>
#include <vel_saltis_services/srv/set_servo_cfg.hpp>

#include <vel_saltis_services/srv/get_motor_cfg.hpp>
#include <vel_saltis_services/srv/set_motor_cfg.hpp>

#include <kapibara_twingo_bridge/event.hpp>
#include <kapibara_twingo_bridge/event_buffered.hpp>

#include <kapibara_twingo_bridge/common.hpp>


class BridgeNode : public rclcpp::Node
{
public:
    BridgeNode() : Node("twingo_bridge") {
        this->declare_parameter("device", "can0");
    }

    std::string deviceName()
    {
        return this->get_parameter("device").as_string();
    }

private:
};

CANBridge can;

std::mutex ack_filter_mux;
char ack_filter[2] = {0};

std::mutex service_lock_mux;

Event<ack_msg_t> ack_event;

EventBuffered<CONFIG_MAX_BUFFER_SIZE> cfg_event;

void set_ack_filter(char a,char b)
{
    ack_filter_mux.lock();

    ack_filter[0] = a;
    ack_filter[1] = b;

    ack_filter_mux.unlock();
}

void clear_ack_filter()
{
    ack_filter_mux.lock();

    ack_filter[0] = 0;
    ack_filter[1] = 0;

    ack_filter_mux.unlock();
}


// sepeare task for can reciving
void can_task(std::shared_ptr<BridgeNode> node)
{

    rclcpp::Publisher<kapibara_interfaces::msg::PiezoSense>::SharedPtr sense_publisher = node->create_publisher<kapibara_interfaces::msg::PiezoSense>("/sense",10);

    rclcpp::Publisher<kapibara_interfaces::msg::CanPing>::SharedPtr ping_publisher = node->create_publisher<kapibara_interfaces::msg::CanPing>("/ping",10);


    while(1)
    {
        const CanFrame* frame = can.recive();

        if( frame == NULL )
        {
            continue;
        }
        // printf("Packet id: %d\n",id);

        // std::cout<<"frame type: "<<frame->getType()<<std::endl;
        switch (frame->getType())
        {
            case PING:
                {
                    const ping_msg_t* ping = frame->to<ping_msg_t>();

                    // std::cout<<"Ping: "<<ping->msg[0]<<ping->msg[1]<<std::endl;
 
                    if( ping->msg[0]=='H' && ping->msg[1]=='I' )
                    {
                        auto _ping = kapibara_interfaces::msg::CanPing();

                        _ping.boardname = "twingo";

                        ping_publisher->publish(_ping);
                    }
                }
            break;
            case SENSE:
                {
                    const sense_t* sense = frame->to<sense_t>();

                    auto _sense = kapibara_interfaces::msg::PiezoSense();

                    _sense.id = sense->id;
                    _sense.frequency = sense->frequency;
                    _sense.power = sense->power;

                    sense_publisher->publish(_sense);

                }
            break;

            // used by configuration services
            case GENERAL_CFG_DATA:
                {
                    uint8_t data[CONFIG_MAX_BUFFER_SIZE];

                    frame->dump(data);

                    cfg_event.notify(data);
                }
            break;

            // used by configuration services
            case ACK:
                {
                    const ack_msg_t* enc = frame->to<ack_msg_t>();

                    ack_filter_mux.lock();

                    if( ack_filter[0] == enc->msg[0] && ack_filter[1] == enc->msg[1] )
                    {
                        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Got ACK! %x %x",enc->msg[0],enc->msg[1]);

                        ack_event.notify(*enc);

                        ack_filter[0] = 0;
                        ack_filter[1] = 0;
                    }

                    ack_filter_mux.unlock();
                }
            break;

            
            default:
                break;
        }

    }

}

// services


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BridgeNode>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting Twingo Can Bridge");


    if(! can.start(node->deviceName().c_str()) )
    {
        RCLCPP_ERROR(node->get_logger(),"Cannot start CAN!");
        rclcpp::shutdown();
        return -1;
    }

    std::thread can_thread(can_task,node);
    
    rclcpp::spin(node);
    can_thread.join();
    rclcpp::shutdown();

    return 0;
}
