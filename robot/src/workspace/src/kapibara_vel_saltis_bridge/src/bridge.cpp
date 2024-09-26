#include <iostream>

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

#include <kapibara_interfaces/msg/encoders_and_speed.hpp>

#include <kapibara_interfaces/msg/can_ping.hpp>

#include "kapibara_vel_saltis_bridge/can.hpp"



int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("vel_saltis_bridge");

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting Vel Saltis Can Bridge");

    CANBridge can;

    if(! can.start("can0") )
    {
        // std::cerr<<"Cannot start CAN!"<<std::endl;
        RCLCPP_ERROR(node->get_logger(),"Cannot start CAN!");
        rclcpp::shutdown();
        return -1;
    }
    

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu",10);

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tofs[8];

    std::string tof_topic="/distance_";

    for(uint8_t i=0;i<8;++i)
    {
        tofs[i] = node->create_publisher<sensor_msgs::msg::Range>(tof_topic+std::to_string(i),10);
    }

    rclcpp::Publisher<kapibara_interfaces::msg::EncodersAndSpeed>::SharedPtr encoders_publisher = node->create_publisher<kapibara_interfaces::msg::EncodersAndSpeed>("/encoders",10);

    rclcpp::Publisher<kapibara_interfaces::msg::CanPing>::SharedPtr ping_publisher = node->create_publisher<kapibara_interfaces::msg::CanPing>("/ping",10);


    while(1)
    {
        rclcpp::spin_some(node);
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

                        _ping.boardname = "vel saltis";

                        ping_publisher->publish(_ping);
                    }
                }
            break;

            case Orientation:
                {
                    const orientation_t* fusion = frame->to<orientation_t>();

                    auto _imu = sensor_msgs::msg::Imu();

                    _imu.orientation.x = fusion->x;
                    _imu.orientation.y = fusion->y;
                    _imu.orientation.z = fusion->z;
                    _imu.orientation.w = fusion->w;

                    _imu.angular_velocity.x = fusion->imu.gyroscope.x;
                    _imu.angular_velocity.y = fusion->imu.gyroscope.y;
                    _imu.angular_velocity.z = fusion->imu.gyroscope.z;

                    _imu.linear_acceleration.x = fusion->imu.accelerometer.x;
                    _imu.linear_acceleration.y = fusion->imu.accelerometer.y;
                    _imu.linear_acceleration.z = fusion->imu.accelerometer.z;

                    imu_publisher->publish(_imu);

                    // printf("Orientaion:\n   x:%f\n  y:%f\n  z:%f\n  w:%f\n",fusion->x,fusion->y,fusion->z,fusion->w);    
                    // printf("IMU:\n  Gyro:\n     x:%f\n      y:%f\n      z:%f\n  Acceleration:\n       x:%f\n      y:%f\n      z:%f\n",
                    // fusion->imu.gyroscope.x,fusion->imu.gyroscope.y,fusion->imu.gyroscope.z,fusion->imu.accelerometer.x,fusion->imu.accelerometer.y,fusion->imu.accelerometer.z);            
                }

            break;

            case TOF:
                {
                    const tof_t* tof = frame->to<tof_t>();

                    // printf("Tof:\n   id:%d\n  distance:%d\n",tof->id,tof->distance);

                    auto _range =  sensor_msgs::msg::Range();

                    _range.radiation_type = sensor_msgs::msg::Range::INFRARED;

                    _range.field_of_view = 0.2618;

                    _range.min_range = 0;
                    _range.max_range = 2;

                    if( tof->id < 8 )
                    {
                        _range.range = tof->distance;
                        tofs[tof->id]->publish(_range); 
                    }
                }
            break;

            case Encoder:
                {
                    const encoder_t* enc = frame->to<encoder_t>();

                    // std::cout<<"Encoder distance left "<<static_cast<int>(enc->distance_left)<<" distance right: "<<enc->distance_right<<std::endl;
                    // std::cout<<"Speed  left "<<static_cast<int>(enc->speed_left)<<" speed right: "<<enc->speed_right<<std::endl;

                    auto _encoders = kapibara_interfaces::msg::EncodersAndSpeed();

                    _encoders.distance_left = enc->distance_left;
                    _encoders.distance_right = enc->distance_right;

                    _encoders.speed_left = enc->speed_left;
                    _encoders.speed_right = enc->speed_right;

                    encoders_publisher->publish(_encoders);
                }
            break;

            // used by configuration services
            case ACK:
                {
                    const ack_msg_t* enc = frame->to<ack_msg_t>();
                }
            break;

            case FUSION_CFG:
                {
                    const fusion_cfg_t* enc = frame->to<fusion_cfg_t>();
                }
            break;

            case MOTOR_CFG:
                {
                    const motor_cfg_t* enc = frame->to<motor_cfg_t>();
                }
            break;

            case SERVO_CFG:
                {
                    const servo_cfg_t* enc = frame->to<servo_cfg_t>();
                }
            break;

            case PID_CFG:
                {
                    const pid_cfg_t* enc = frame->to<pid_cfg_t>();
                }
            break;

            case AUX_CFG:
                {
                    const aux_cfg_t* enc = frame->to<aux_cfg_t>();
                }
            break;

            case IMU_CFG:
                {
                    const imu_cfg_t* enc = frame->to<imu_cfg_t>();
                }
            break;
            
            default:
                break;
        }

    }

    return 0;
}
