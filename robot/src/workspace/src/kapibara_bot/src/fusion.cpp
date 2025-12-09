#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kapibara_interfaces/msg/diff_encoder.hpp"

#include "kapibara/fusion.hpp"

using std::placeholders::_1;


FusionNode::FusionNode()
: Node("fusionNode")
{
    this->declare_parameter("encoders_topic", "/encoders");
    this->declare_parameter("imu_topic", "/imu");

    this->declare_parameter("output_topic", "/pose");

    // in us
    this->declare_parameter("post_time", 100000);

    std::string imu_topic=this->get_parameter("imu_topic").as_string();
    std::string encoders_topic=this->get_parameter("encoders_topic").as_string();

    int64_t pub_time=this->get_parameter("post_time").as_int();


    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 10, std::bind(&FusionNode::imu_callback, this, _1));

    this->encoders_sub = this->create_subscription<kapibara_interfaces::msg::DiffEncoder>(
    encoders_topic, 10, std::bind(&FusionNode::encoders_callback, this, _1));

    this->fusion_pub = this->create_publisher<geometry_msgs::msg::Pose>("topic", 10);
    this->timer_ = this->create_wall_timer(
      std::chrono::microseconds(pub_time), std::bind(&FusionNode::fusion_node, this));

}

void FusionNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // perform quaterion calculations 
}

void FusionNode::encoders_callback(const kapibara_interfaces::msg::DiffEncoder::SharedPtr msg)
{
    // perform delta distance calculation
}

void FusionNode::fusion_node()
{
    // perform sensor fusion
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}