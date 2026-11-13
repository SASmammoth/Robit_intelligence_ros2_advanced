#pragma once

#include <array>
#include <memory>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/pose.hpp"

class Calc_my_IK : public rclcpp::Node
{
public:
  Calc_my_IK();

private:
  void onTimer();

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::TimerBase::SharedPtr timer_;

  using JointArray = std::array<double, 6>;
  std::optional<JointArray> solveIK(const geometry_msgs::msg::Pose &goal) const;
  static constexpr double a_1 = 0.41686; // DH테이블
  static constexpr double a_2 = 0.42586;

  struct Limit
  {
    double lo, hi;
  };
  static constexpr Limit kLimit[6] = {
      {-1.570796, 1.570796}, // arm_joint1
      {-1.570796, 1.570796}, // arm_joint2
      {0.0, 3.141593},       // arm_joint3
      {-2.0, 2.0},           // arm_joint4
      {-1.7802, 2.0071},     // arm_joint5
      {-1.570796, 1.570796}, // arm_joint6
  };
};
