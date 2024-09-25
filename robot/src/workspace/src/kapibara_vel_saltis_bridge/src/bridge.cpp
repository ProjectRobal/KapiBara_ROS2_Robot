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

#include "kapibara_vel_saltis_bridge/can.hpp"



int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    CANBridge can;

    if(! can.start("can0") )
    {
        std::cerr<<"Cannot start CAN!"<<std::endl;
        return -1;
    }

    auto node = std::make_shared<rclcpp::Node>("vel_saltis_can_bridge");

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu",10);

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tofs[8];

    std::string tof_topic="/distance_";

    for(uint8_t i=0;i<8;++i)
    {
        tofs[i] = node->create_publisher<sensor_msgs::msg::Range>(tof_topic+std::to_string(i),10);
    }

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

                }
            break;

            case IMU:
                
                {
                    const imu_raw_t* imu = frame->to<imu_raw_t>();


                    // printf("IMU:\n  Gyro:\n     x:%f\n      y:%f\n      z:%f\n  Acceleration:\n       x:%f\n      y:%f\n      z:%f\n",
                    // imu->gyroscope.x,imu->gyroscope.y,imu->gyroscope.z,imu->accelerometer.x,imu->accelerometer.y,imu->accelerometer.z);

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

                    _range.radiation_type = INFRARED;

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
                }
            break;
            
            default:
                break;
        }

    }

    return 0;
}
