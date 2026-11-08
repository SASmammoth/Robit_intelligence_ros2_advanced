#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include "pingpong_lab_interfaces/msg/pingpongball.hpp"
#include "my_pong/my_pong_node.hpp"

my_pingpong::MyPongNode::MyPongNode() : Node("my_pong_node")
{
  sub_ = create_subscription<Boll>("ping", 10, [this](const Boll &msg_)
                                    { sub_collback(msg_); });
  pub_ = create_publisher<Boll>("pong", 10);
}

void my_pingpong::MyPongNode::sub_collback(const Boll &msg_)
{
  pub_->publish(msg_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "seq = %u", msg_.seq);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_pingpong::MyPongNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
