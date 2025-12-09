#pragma once
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "kapibara_interfaces/msg/diff_encoder.hpp"


class FusionNode : public rclcpp::Node
{
  public:
    FusionNode();

  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void encoders_callback(const kapibara_interfaces::msg::DiffEncoder::SharedPtr msg);

    void fusion_node();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<kapibara_interfaces::msg::DiffEncoder>::SharedPtr encoders_sub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr fusion_pub;
};
