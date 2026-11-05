#include "rclcpp/rclcpp.hpp"
#include "edu_dds_task2/edu_dds_task2_node.hpp"

EduDdsTast2Node::EduDdsTast2Node() : Node("edu_dds_task2_node")
{
  sub_config = create_subscription<std_msgs::msg::String>(
      "/bb_ae3e/config", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg)
      { config_collback(msg); });
  sub_stream = create_subscription<std_msgs::msg::String>(
      "/bb_ae3e/stream", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg)
      { stream_collback(msg); });
}

void EduDdsTast2Node::config_collback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "config: '%s'", msg->data.c_str());
}

void EduDdsTast2Node::stream_collback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "stream: '%s'", msg->data.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EduDdsTast2Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
