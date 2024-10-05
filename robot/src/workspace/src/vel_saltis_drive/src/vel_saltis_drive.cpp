#include "pluginlib/class_list_macros.hpp"
#include "vel_saltis_drive/vel_saltis_drive.hpp"

#include "vel_saltis_drive/gpio_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vel_saltis_drive/can_msg.hpp"

namespace vel_saltis_drive
{

    hardware_interface::CallbackReturn VelSaltisDrive::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {

        if ( hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }


        RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"),"Configuring...");

        if( info_.hardware_parameters.counts("can_device") > 0 )
        {
            this->cfg.can_device = info_.hardware_parameters["can_device"];
        }

        this->cfg.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        this->cfg.right_wheel_name = info_.hardware_parameters["right_wheel_name"];


        this->cfg.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        this->cfg.encoder_resolution = std::stoi(info_.hardware_parameters["encoder_resolution"]);

        this->w_left.setup(this->cfg.left_wheel_name,this->cfg.encoder_resolution);
        this->w_right.setup(this->cfg.right_wheel_name,this->cfg.encoder_resolution);

        // open can socket
        

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        // close can socket

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        // stop motors

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_error(const rclcpp_lifecycle::State &previous_state)
    {

        RCLCPP_ERROR(rclcpp::get_logger("VelSaltisDrive"), "Error occured, stoping engine!!!");

        // stop motors

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> VelSaltisDrive::export_state_interfaces()
    {

        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(this->w_left.name, hardware_interface::HW_IF_VELOCITY, &this->w_left.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(this->w_left.name, hardware_interface::HW_IF_POSITION, &this->w_left.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(this->w_right.name, hardware_interface::HW_IF_VELOCITY, &this->w_right.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(this->w_right.name, hardware_interface::HW_IF_POSITION, &this->w_right.position));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> VelSaltisDrive::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(this->w_left.name, hardware_interface::HW_IF_VELOCITY, &this->w_left.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(this->w_right.name, hardware_interface::HW_IF_VELOCITY, &this->w_right.cmd));

        return command_interfaces;

    }

    hardware_interface::return_type VelSaltisDrive::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // read wheel distance and speed from board

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VelSaltisDrive::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // update pwm according to cmd value

        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Left value: %f", this->w_left.cmd);
        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Right value: %f", this->w_right.cmd);

        double left_target_vel=this->w_left.targetVelocity()/this->cfg.loop_rate;
        double right_target_vel=this->w_right.targetVelocity()/this->cfg.loop_rate;
        
        // send speed to motors

        return hardware_interface::return_type::OK;
    }

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vel_saltis_drive::VelSaltisDrive, hardware_interface::SystemInterface)