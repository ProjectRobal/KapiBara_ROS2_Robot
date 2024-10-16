#pragma once

#include <cstring>
#include <vector>
#include <chrono>
#include <mutex>

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

#include "wheel.hpp"

#include "Config.hpp"

#include "can.hpp"


namespace vel_saltis_drive
{
    class VelSaltisDrive : public hardware_interface::SystemInterface
    {  
        public:

        RCLCPP_SHARED_PTR_DEFINITIONS(VelSaltisDrive);

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

        Wheel w_left;
        Wheel w_right;

        speed_msg_t speed;

        CANBridge can;

        std::mutex can_mux;

        bool can_run;

        std::thread can_task;

        void read_from_can();

        void send_motor_msg(int16_t speed_left,int16_t speed_right)
        {
            motor_msg_t motors = {
                .speed_left = speed_left,
                .speed_right = speed_right
            };

            uint8_t* buff = (uint8_t*)&motors;

            this->can.send(buff,sizeof(motors),VEL_SALTIS_ID,4);
        }

    };
}