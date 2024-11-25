#pragma once

#include <cstring>
#include <vector>
#include <chrono>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "can_msg.hpp"

#include "common.hpp"

#include "Config.hpp"

#include "can.hpp"

#include "servo.hpp"


namespace vel_saltis_ears
{
    class VelSaltisEars : public hardware_interface::SystemInterface
    {  
        public:

        RCLCPP_SHARED_PTR_DEFINITIONS(VelSaltisEars);

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);

        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info);

        std::vector<hardware_interface::StateInterface> export_state_interfaces();

        std::vector<hardware_interface::CommandInterface> export_command_interfaces();

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period);

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period);

        private:

        Config cfg;

        speed_msg_t speed;

        CANBridge can;

        std::mutex can_mux;

        bool can_run;

        std::thread can_task;

        Servo left_servo;
        Servo right_servo;

        void read_from_can();

        void send_servo_msg()
        {
            servo_msg_t servos = {
                .left = static_cast<uint8_t>(this->left_servo.getAngle()),
                .right = static_cast<uint8_t>(this->right_servo.getAngle())
            };

            uint8_t* buff = (uint8_t*)&servos;

            this->can.send(buff,sizeof(servos),VEL_SALTIS_ID,SERVOS_ID);
        }

    };
}