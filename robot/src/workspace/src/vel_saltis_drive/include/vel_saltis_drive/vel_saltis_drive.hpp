#ifndef TB6612_DRIVE_HH
#define TB6612_DRIVE_HH

#include <cstring>
#include <vector>
#include <chrono>
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

#include "kapibara_interfaces/msg/diff_encoder.hpp" 

#include "vel_saltis_drive/wheel.hpp"
#include "vel_saltis_drive/PID.hpp"

#include "Config.hpp"

/*

    A hardware interface communicate with two joints.
    It takes arguments:
        GPIO numbers:
        - MOTORA1
        - MOTORA2
        - MOTORB1
        - MOTORB2

        Joints names
        
    Additional functions for encoders topic retrival

*/

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

        void encoder_callback(const kapibara_interfaces::msg::DiffEncoder::SharedPtr msg)
        {
            rclcpp::Time curr_time=this->external->now();

            RCLCPP_INFO(rclcpp::get_logger("TB6612Drive"), "Encoder left: '%d'", msg->motora);
            RCLCPP_INFO(rclcpp::get_logger("TB6612Drive"), "Encoder right: '%d'", msg->motorb);

            if(this->last_encoder_T>rclcpp::Time(0))
            {
                double dt=(curr_time.nanoseconds()-curr_time.nanoseconds())/pow(10,9);

                this->w_left.update(msg->motora,dt);
                this->w_right.update(msg->motorb,dt);
            }


        }

        Config cfg;

        Wheel w_left;
        Wheel w_right;

        PID<double> left_pid;
        PID<double> right_pid;

        rclcpp::Time last_encoder_T;

        rclcpp::Node::SharedPtr external;
    };
}

#endif