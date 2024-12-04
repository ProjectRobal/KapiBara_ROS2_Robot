#include <thread>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"
#include "vel_saltis_ears/vel_saltis_ears.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vel_saltis_ears/can_msg.hpp"

#include "vel_saltis_ears/frame.hpp"

#include "kapibara_interfaces/msg/encoders_and_speed.hpp"


namespace vel_saltis_ears
{

    hardware_interface::CallbackReturn VelSaltisEars::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {

        if ( hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }


        RCLCPP_INFO(rclcpp::get_logger("VelSaltisEars"),"Configuring...");

        if( info_.hardware_parameters.count("can_device") > 0 )
        {
            this->cfg.can_device = info_.hardware_parameters["can_device"];
        }


        if( info_.hardware_parameters.count("left_ear_name") > 0 )
        {
            this->cfg.left_ear_name = info_.hardware_parameters["left_ear_name"];
        }

        if( info_.hardware_parameters.count("right_ear_name") > 0 )
        {
            this->cfg.right_ear_name = info_.hardware_parameters["right_ear_name"];
        }


        this->cfg.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);

        // open can socket
        
        if( !this->can.start(this->cfg.can_device.c_str()) )
        {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        this->left_servo.name = this->cfg.left_ear_name;
        this->right_servo.name = this->cfg.right_ear_name;


        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisEars::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisEars::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisEars::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisEars::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisEars::on_error(const rclcpp_lifecycle::State &previous_state)
    {

        RCLCPP_ERROR(rclcpp::get_logger("VelSaltisEars"), "Error occured!!!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> VelSaltisEars::export_state_interfaces()
    {

        std::vector<hardware_interface::StateInterface> state_interfaces;

        // servos
        state_interfaces.emplace_back(hardware_interface::StateInterface(this->left_servo.name, hardware_interface::HW_IF_POSITION, &this->left_servo.angle));
        state_interfaces.emplace_back(hardware_interface::StateInterface(this->right_servo.name, hardware_interface::HW_IF_POSITION, &this->right_servo.angle));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> VelSaltisEars::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(this->left_servo.name, hardware_interface::HW_IF_POSITION, &this->left_servo.angle));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(this->right_servo.name, hardware_interface::HW_IF_POSITION, &this->right_servo.angle));

        return command_interfaces;

    }

    hardware_interface::return_type VelSaltisEars::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("VelSaltisEars"), "Reading from servos!");


        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VelSaltisEars::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // update pwm according to cmd value

        RCLCPP_DEBUG(rclcpp::get_logger("VelSaltisEars"), "Sending servo data!");


        this->send_servo_msg();
        
        return hardware_interface::return_type::OK;
    }

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vel_saltis_ears::VelSaltisEars, hardware_interface::SystemInterface)