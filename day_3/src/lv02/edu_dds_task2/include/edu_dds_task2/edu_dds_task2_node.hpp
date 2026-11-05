#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class EduDdsTast2Node : public rclcpp::Node
{
public:
  EduDdsTast2Node();

private:
  void config_collback(const std_msgs::msg::String::SharedPtr msg);
  void stream_collback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_config;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_stream;
};
