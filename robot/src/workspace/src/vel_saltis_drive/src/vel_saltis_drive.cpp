#include "pluginlib/class_list_macros.hpp"
#include "vel_saltis_drive/vel_saltis_drive.hpp"

#include "vel_saltis_drive/gpio_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vel_saltis_drive
{

    hardware_interface::CallbackReturn VelSaltisDrive::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {

        if ( hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS )
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"),"Configuring...");

        this->last_encoder_T=rclcpp::Time(0);

        this->cfg.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        this->cfg.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        this->cfg.encoder_topic = info_.hardware_parameters["encoder_topic"];

        this->cfg.left_gpioA = info_.hardware_parameters["left_gpioA"];
        this->cfg.left_gpioB = info_.hardware_parameters["left_gpioB"];

        this->cfg.right_gpioA = info_.hardware_parameters["right_gpioA"];
        this->cfg.right_gpioB = info_.hardware_parameters["right_gpioB"];


        if(info_.hardware_parameters.count("left_P")!=0)
        {
            this->cfg.left_P = std::stof(info_.hardware_parameters["left_P"]);
        }

        if(info_.hardware_parameters.count("left_I")!=0)
        {
            this->cfg.left_I = std::stof(info_.hardware_parameters["left_I"]);
        }

        if(info_.hardware_parameters.count("left_D")!=0)
        {
            this->cfg.left_D = std::stof(info_.hardware_parameters["left_D"]);
        }


        if(info_.hardware_parameters.count("right_P")!=0)
        {
            this->cfg.right_P = std::stof(info_.hardware_parameters["right_P"]);
        }

        if(info_.hardware_parameters.count("right_I")!=0)
        {
            this->cfg.right_I = std::stof(info_.hardware_parameters["right_I"]);
        }

        if(info_.hardware_parameters.count("right_D")!=0)
        {
            this->cfg.right_D = std::stof(info_.hardware_parameters["right_D"]);
        }

        this->left_pid.setParams(this->cfg.left_P,this->cfg.left_I,this->cfg.left_D);
        this->right_pid.setParams(this->cfg.right_P,this->cfg.right_I,this->cfg.right_D);

        this->left_pid.setMax(1.f);
        this->left_pid.setMin(-1.f);

        this->right_pid.setMax(1.f);
        this->right_pid.setMin(-1.f);

        this->cfg.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        this->cfg.encoder_resolution = std::stoi(info_.hardware_parameters["encoder_resolution"]);

        this->w_left.setup(this->cfg.left_wheel_name,this->cfg.encoder_resolution,this->cfg.left_gpioA,this->cfg.left_gpioB,"0");
        this->w_right.setup(this->cfg.right_wheel_name,this->cfg.encoder_resolution,this->cfg.right_gpioA,this->cfg.right_gpioB,"1");
        
        // init topics to get messages from encoders, we will use simple Vector2 / Vector3 for getting encoder data through messages
        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });

        this->external = rclcpp::Node::make_shared("_", options);

        this->external->create_subscription<kapibara_interfaces::msg::DiffEncoder>(
      info_.hardware_parameters["encoder_topic"], rclcpp::SensorDataQoS(), 
      [this](const kapibara_interfaces::msg::DiffEncoder::SharedPtr msg)
      {
            this->encoder_callback(msg);
      });

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        // uninit pins 

        // uinit pwm

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {

        this->w_left.setPower(0);
        this->w_right.setPower(0);

        this->w_left.stop();
        this->w_right.stop();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        this->w_left.setPower(0);
        this->w_right.setPower(0);

        this->w_left.stop();
        this->w_right.stop();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_activate(const rclcpp_lifecycle::State &previous_state)
    {

        this->w_left.setPower(0);
        this->w_right.setPower(0);


        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VelSaltisDrive::on_error(const rclcpp_lifecycle::State &previous_state)
    {

        RCLCPP_ERROR(rclcpp::get_logger("VelSaltisDrive"), "Error occured, stoping engine!!!");

        this->w_left.setPower(0);
        this->w_right.setPower(0);

        this->w_left.stop();
        this->w_right.stop();

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> VelSaltisDrive::export_state_interfaces()
    {

        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(w_left.name, hardware_interface::HW_IF_VELOCITY, &w_left.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(w_left.name, hardware_interface::HW_IF_POSITION, &w_left.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(w_right.name, hardware_interface::HW_IF_VELOCITY, &w_right.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(w_right.name, hardware_interface::HW_IF_POSITION, &w_right.position));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> VelSaltisDrive::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(w_left.name, hardware_interface::HW_IF_VELOCITY, &w_left.cmd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(w_right.name, hardware_interface::HW_IF_VELOCITY, &w_right.cmd));

        return command_interfaces;

    }

    hardware_interface::return_type VelSaltisDrive::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (rclcpp::ok())
        {
            rclcpp::spin_some(this->external);
        }

        // calculate wheel position and velocity

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VelSaltisDrive::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // update pwm according to cmd value

        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Left value: %f", this->w_left.cmd);
        //RCLCPP_INFO(rclcpp::get_logger("VelSaltisDrive"), "CMD Right value: %f", this->w_right.cmd);

        double left_target_vel=this->w_left.targetVelocity()/this->cfg.loop_rate;
        double right_target_vel=this->w_right.targetVelocity()/this->cfg.loop_rate;

        double dt=period.seconds();
        
        // here goes PID:

        this->w_left.setPower(this->left_pid.step(left_target_vel-this->w_left.velocity,dt));
        this->w_right.setPower(this->left_pid.step(right_target_vel-this->w_right.velocity,dt));

        return hardware_interface::return_type::OK;
    }

}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vel_saltis_drive::VelSaltisDrive, hardware_interface::SystemInterface)