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

#include <kapibara_interfaces/msg/encoders_and_speed.hpp>

#include <kapibara_interfaces/msg/can_ping.hpp>

#include "kapibara_vel_saltis_bridge/can.hpp"


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

#include <kapibara_vel_saltis_bridge/event.hpp>
#include <kapibara_vel_saltis_bridge/event_buffered.hpp>

#include <kapibara_vel_saltis_bridge/common.hpp>


class BridgeNode : public rclcpp::Node
{
public:
    BridgeNode() : Node("vel_saltis_bridge") {
        this->declare_parameter("device", "can0");
        this->declare_parameter("tof_count", 4);
    }

    std::string deviceName()
    {
        return this->get_parameter("device").as_string();
    }

    uint64_t tofCount()
    {
        uint64_t out = this->get_parameter("tof_count").as_int();

        out = std::min<uint64_t>(out,8);

        return out;
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
void can_task(std::shared_ptr<BridgeNode> node,uint64_t tofCount)
{

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>("/imu",10);

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tofs[tofCount];

    std::string tof_topic="/distance_";

    for(uint8_t i=0;i<tofCount;++i)
    {
        tofs[i] = node->create_publisher<sensor_msgs::msg::Range>(tof_topic+std::to_string(i),10);
    }

    rclcpp::Publisher<kapibara_interfaces::msg::EncodersAndSpeed>::SharedPtr encoders_publisher = node->create_publisher<kapibara_interfaces::msg::EncodersAndSpeed>("/encoders",10);

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

                    _encoders.raw_left = enc->raw_left;
                    _encoders.raw_right = enc->raw_right;

                    encoders_publisher->publish(_encoders);
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

void get_imu_cfg(std::shared_ptr<vel_saltis_services::srv::GetImuCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::GetImuCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 0;

    can.set_recive_size(sizeof(imu_cfg_t));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with IMU config request!");
    // send request
    can.send_id(VEL_SALTIS_ID,id);

    // wait for response from can bus, for about 60 seconds
    imu_cfg_t cfg = {0};

    if(!cfg_event.wait_for(60))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Vel Saltis get imu configuration timeouted!");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with IMU config request!");   
    }

    uint8_t* buff_cfg = (uint8_t*)&cfg;

    for(size_t i=0;i<sizeof(imu_cfg_t);++i)
    {
        buff_cfg[i] = cfg_event[i];
    }

    response->config.accelerometer_offset.x = cfg.accelerometer_offset.x;
    response->config.accelerometer_offset.y = cfg.accelerometer_offset.y;
    response->config.accelerometer_offset.z = cfg.accelerometer_offset.z;

    response->config.gyroscope_offset.x = cfg.gyroscope_offset.x;
    response->config.gyroscope_offset.y = cfg.gyroscope_offset.y;
    response->config.gyroscope_offset.z = cfg.gyroscope_offset.z;

    service_lock_mux.unlock();
}

void set_fusion_cfg(std::shared_ptr<vel_saltis_services::srv::SetFusionCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::SetFusionCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 5;

    fusion_cfg_t cfg;

    cfg.beta = request->config.beta;
    cfg.integral = request->config.integral;

    set_ack_filter('F','U');

    // send configuration to board
    can.send((uint8_t*)&cfg,sizeof(fusion_cfg_t),VEL_SALTIS_ID,id);

    // wait for ack from can bus, for about 60 seconds
    ack_msg_t msg;

    bool ok = ack_event.wait_for(msg,60);
    
    if( ok )
    {
        response->ok = msg.msg[0] == 'F' && msg.msg[1] == 'U';
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Vel Saltis couldn't get ACK in time!");
        response->ok = false;
    }

}


void get_fusion_cfg(std::shared_ptr<vel_saltis_services::srv::GetFusionCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::GetFusionCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 5;

    can.set_recive_size(sizeof(fusion_cfg_t));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Fusion config request!");
    // send request
    can.send_id(VEL_SALTIS_ID,id);

    // wait for response from can bus, for about 60 seconds
    fusion_cfg_t cfg = {0};

    if(!cfg_event.wait_for(60))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Vel Saltis get Fusion configuration timeouted!");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Fusion config request!");   
    }

    uint8_t* buff_cfg = (uint8_t*)&cfg;

    for(size_t i=0;i<sizeof(fusion_cfg_t);++i)
    {
        buff_cfg[i] = cfg_event[i];
    }

    response->config.beta = cfg.beta;
    response->config.integral = cfg.integral;
}

void set_imu_cfg(std::shared_ptr<vel_saltis_services::srv::SetImuCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::SetImuCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 0;

    imu_cfg_t cfg;

    cfg.accelerometer_offset.x = request->config.accelerometer_offset.x;
    cfg.accelerometer_offset.y = request->config.accelerometer_offset.y;
    cfg.accelerometer_offset.z = request->config.accelerometer_offset.z;

    cfg.gyroscope_offset.x = request->config.gyroscope_offset.x;
    cfg.gyroscope_offset.y = request->config.gyroscope_offset.y;
    cfg.gyroscope_offset.z = request->config.gyroscope_offset.z;

    set_ack_filter('I','U');

    // send configuration to board
    can.send((uint8_t*)&cfg,sizeof(imu_cfg_t),VEL_SALTIS_ID,id);

    // wait for ack from can bus, for about 60 seconds
    ack_msg_t msg;

    bool ok = ack_event.wait_for(msg,60);
    
    if( ok )
    {
        response->ok = msg.msg[0] == 'I' && msg.msg[1] == 'U';
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Vel Saltis couldn't get ACK in time!");
        response->ok = false;
    }

}

void set_pid_cfg(std::shared_ptr<vel_saltis_services::srv::SetPIDCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::SetPIDCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 2;

    pid_cfg_t cfg;

    cfg.pid_left.p = request->config.left.p;
    cfg.pid_left.i = request->config.left.i;
    cfg.pid_left.d = request->config.left.d;

    cfg.pid_right.p = request->config.right.p;
    cfg.pid_right.i = request->config.right.i ;
    cfg.pid_right.d = request->config.right.d;

    cfg.open = request->config.open;

    set_ack_filter('P','U');

    // send configuration to board
    can.send((uint8_t*)&cfg,25,VEL_SALTIS_ID,id);

    // wait for ack from can bus, for about 60 seconds
    ack_msg_t msg;

    bool ok = ack_event.wait_for(msg,60);
    
    if( ok )
    {
        response->ok = msg.msg[0] == 'P' && msg.msg[1] == 'U';
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Vel Saltis couldn't get ACK in time!");
        response->ok = false;
    }

}


void get_pid_cfg(std::shared_ptr<vel_saltis_services::srv::GetPIDCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::GetPIDCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 2;

    // can.set_recive_size(sizeof(pid_cfg_t));
    can.set_recive_size(25);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with PID config request!");
    // send request
    can.send_id(VEL_SALTIS_ID,id);

    // wait for response from can bus, for about 60 seconds
    pid_cfg_t cfg = {0};

    if(!cfg_event.wait_for(60))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Vel Saltis get PID configuration timeouted!");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with PID config request!");   
    }

    uint8_t* buff_cfg = (uint8_t*)&cfg;

    for(size_t i=0;i<sizeof(pid_cfg_t);++i)
    {
        buff_cfg[i] = cfg_event[i];
    }

    response->config.left.p = cfg.pid_left.p;
    response->config.left.i = cfg.pid_left.i;
    response->config.left.d = cfg.pid_left.d;

    response->config.right.p = cfg.pid_right.p;
    response->config.right.i = cfg.pid_right.i;
    response->config.right.d = cfg.pid_right.d;

    response->config.open = cfg.open;
}

void set_servo_cfg(std::shared_ptr<vel_saltis_services::srv::SetServoCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::SetServoCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 3;

    servo_cfg_t cfg;

    cfg.angel[0] = request->config.angel_left;
    cfg.angel[1] = request->config.angel_right;

    set_ack_filter('S','U');

    // send configuration to board
    can.send((uint8_t*)&cfg,sizeof(servo_cfg_t),VEL_SALTIS_ID,id);

    // wait for ack from can bus, for about 60 seconds
    ack_msg_t msg;

    bool ok = ack_event.wait_for(msg,60);
    
    if( ok )
    {
        response->ok = msg.msg[0] == 'S' && msg.msg[1] == 'U';
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Vel Saltis couldn't get ACK in time!");
        response->ok = false;
    }

}


void get_servo_cfg(std::shared_ptr<vel_saltis_services::srv::GetServoCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::GetServoCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 3;

    can.set_recive_size(sizeof(servo_cfg_t));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Servo config request!");
    // send request
    can.send_id(VEL_SALTIS_ID,id);

    // wait for response from can bus, for about 60 seconds
    servo_cfg_t cfg = {0};

    if(!cfg_event.wait_for(60))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Vel Saltis get Servo configuration timeouted!");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Servo config request!");   
    }

    uint8_t* buff_cfg = (uint8_t*)&cfg;

    for(size_t i=0;i<sizeof(servo_cfg_t);++i)
    {
        buff_cfg[i] = cfg_event[i];
    }

    response->config.angel_left = cfg.angel[0];
    response->config.angel_right = cfg.angel[1];
}

void set_motor_cfg(std::shared_ptr<vel_saltis_services::srv::SetMotorCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::SetMotorCFG::Response> response)
{
    // send request to get imu config through CAN Bus
    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 4;

    motor_cfg_t cfg;

    cfg.velocities[0] = request->config.velocities_left;
    cfg.velocities[1] = request->config.velocities_right;

    set_ack_filter('M','U');

    // send configuration to board
    can.send((uint8_t*)&cfg,sizeof(motor_cfg_t),VEL_SALTIS_ID,id);

    // wait for ack from can bus, for about 60 seconds
    ack_msg_t msg;

    bool ok = ack_event.wait_for(msg,60);
    
    if( ok )
    {
        response->ok = msg.msg[0] == 'M' && msg.msg[1] == 'U';
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Vel Saltis couldn't get ACK in time!");
        response->ok = false;
    }

}


void get_motor_cfg(std::shared_ptr<vel_saltis_services::srv::GetMotorCFG::Request> request,
std::shared_ptr<vel_saltis_services::srv::GetMotorCFG::Response> response)
{
    // send request to get imu config through CAN Bus

    std::lock_guard<std::mutex> guard(service_lock_mux);

    // imu id
    uint8_t id = 4;

    can.set_recive_size(sizeof(motor_cfg_t));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Motor config request!");
    // send request
    can.send_id(VEL_SALTIS_ID,id);

    // wait for response from can bus, for about 60 seconds
    motor_cfg_t cfg = {0};

    if(!cfg_event.wait_for(60))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Vel Saltis get Motor configuration timeouted!");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending id with Motor config request!");   
    }

    uint8_t* buff_cfg = (uint8_t*)&cfg;

    for(size_t i=0;i<sizeof(servo_cfg_t);++i)
    {
        buff_cfg[i] = cfg_event[i];
    }

    response->config.velocities_left = cfg.velocities[0];
    response->config.velocities_right = cfg.velocities[1];
}


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BridgeNode>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting Vel Saltis Can Bridge");


    if(! can.start(node->deviceName().c_str()) )
    {
        RCLCPP_ERROR(node->get_logger(),"Cannot start CAN!");
        rclcpp::shutdown();
        return -1;
    }


    rclcpp::Service<vel_saltis_services::srv::SetImuCFG>::SharedPtr set_imu_srv = node->create_service<vel_saltis_services::srv::SetImuCFG>("set_imu_cfg",&set_imu_cfg);
    rclcpp::Service<vel_saltis_services::srv::GetImuCFG>::SharedPtr get_imu_srv = node->create_service<vel_saltis_services::srv::GetImuCFG>("get_imu_cfg",&get_imu_cfg);

    rclcpp::Service<vel_saltis_services::srv::SetFusionCFG>::SharedPtr set_fusion_srv = node->create_service<vel_saltis_services::srv::SetFusionCFG>("set_fusion_cfg",&set_fusion_cfg);
    rclcpp::Service<vel_saltis_services::srv::GetFusionCFG>::SharedPtr get_fusion_srv = node->create_service<vel_saltis_services::srv::GetFusionCFG>("get_fusion_cfg",&get_fusion_cfg);

    rclcpp::Service<vel_saltis_services::srv::SetPIDCFG>::SharedPtr set_pid_srv = node->create_service<vel_saltis_services::srv::SetPIDCFG>("set_pid_cfg",&set_pid_cfg);
    rclcpp::Service<vel_saltis_services::srv::GetPIDCFG>::SharedPtr get_pid_srv = node->create_service<vel_saltis_services::srv::GetPIDCFG>("get_pid_cfg",&get_pid_cfg);

    rclcpp::Service<vel_saltis_services::srv::SetServoCFG>::SharedPtr set_servo_srv = node->create_service<vel_saltis_services::srv::SetServoCFG>("set_servo_cfg",&set_servo_cfg);
    rclcpp::Service<vel_saltis_services::srv::GetServoCFG>::SharedPtr get_servo_srv = node->create_service<vel_saltis_services::srv::GetServoCFG>("get_servo_cfg",&get_servo_cfg);

    rclcpp::Service<vel_saltis_services::srv::SetMotorCFG>::SharedPtr set_motor_srv = node->create_service<vel_saltis_services::srv::SetMotorCFG>("set_motor_cfg",&set_motor_cfg);
    rclcpp::Service<vel_saltis_services::srv::GetMotorCFG>::SharedPtr get_motor_srv = node->create_service<vel_saltis_services::srv::GetMotorCFG>("get_motor_cfg",&get_motor_cfg);

    std::thread can_thread(can_task,node,node->tofCount());
    
    rclcpp::spin(node);
    can_thread.join();
    rclcpp::shutdown();

    return 0;
}
