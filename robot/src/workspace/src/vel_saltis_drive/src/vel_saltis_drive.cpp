#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "vel_saltis_drive/vel_saltis_drive.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vel_saltis_drive/can_msg.hpp"

#include "vel_saltis_drive/frame.hpp"

namespace vel_saltis_drive
{
    void VelSaltisDrive::read_from_can()
    {
        while(this->can_run)
        {
            const CanFrame* frame = this->can.recive();

            if( frame != NULL )
            {
                const speed_msg_t *msg = frame->to<speed_msg_t>();

                this->can_mux.lock();
                // update speed

                this->speed = *msg;

                this->can_mux.unlock();
            }

        }
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {

        if ( hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }


        RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"),"Configuring...");

        this->can_run = true;

        if( info_.hardware_parameters.count("can_device") > 0 )
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
        
        if( !this->can.start(this->cfg.can_device.c_str()) )
        {
            return hardware_interface::CallbackReturn::FAILURE;
        }

        this->can_task = std::thread([this](){this->read_from_can();});

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        this->send_motor_msg(0,0);

        // close can socket

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        this->send_motor_msg(0,0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        // stop motors

        this->send_motor_msg(0,0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // stop motors

        this->send_motor_msg(0,0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_error(const rclcpp_lifecycle::State &previous_state)
    {

        RCLCPP_ERROR(rclcpp::get_logger("VelSaltisDrive"), "Error occured, stoping engine!!!");

        // stop motors

        this->send_motor_msg(0,0);

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
        this->can_mux.lock();

        // read wheel distance and speed from board

        this->w_left.velocity = speed.speed_left / this->w_left.EncoderToAngelRatio;
        this->w_right.velocity = speed.speed_right / this->w_right.EncoderToAngelRatio;

        this->w_left.position = speed.distance_left / this->w_left.EncoderToAngelRatio;
        this->w_right.position = speed.distance_right / this->w_right.EncoderToAngelRatio;

        this->can_mux.unlock();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VelSaltisDrive::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // update pwm according to cmd value

        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Left value: %f", this->w_left.cmd);
        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Right value: %f", this->w_right.cmd);

        // double left_target_vel=this->w_left.targetVelocity()/this->cfg.loop_rate;
        // double right_target_vel=this->w_right.targetVelocity()/this->cfg.loop_rate;

        int16_t left_target_vel = this->w_left.cmd*this->w_left.EncoderToAngelRatio;
        int16_t right_target_vel = this->w_right.cmd*this->w_right.EncoderToAngelRatio;

        this->send_motor_msg(left_target_vel,right_target_vel);
        
        // send speed to motors

        return hardware_interface::return_type::OK;
    }

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vel_saltis_drive::VelSaltisDrive, hardware_interface::SystemInterface)