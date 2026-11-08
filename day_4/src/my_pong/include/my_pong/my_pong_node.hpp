#pragma once

#include <rclcpp/rclcpp.hpp>
#include "pingpong_lab_interfaces/msg/pingpongball.hpp"

namespace my_pingpong
{
  using Boll = pingpong_lab_interfaces::msg::Pingpongball;

  class MyPongNode : public rclcpp::Node
  {
  public:
    MyPongNode();

  private:
    rclcpp::Subscription<Boll>::SharedPtr sub_; // 받는자
    rclcpp::Publisher<Boll>::SharedPtr pub_;    // 주는자
    void sub_collback(const Boll &msg_);
  };
}
