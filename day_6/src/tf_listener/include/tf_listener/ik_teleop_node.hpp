#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 키보드로 손목 중심(엔드포인트) 목표를 움직이고, IK 결과를 조인트로 발행
class IkTeleop : public rclcpp::Node
{
public:
  IkTeleop();
  ~IkTeleop() override;

private:
  using JointArray = std::array<double, 6>;

  // ───── IK ─────
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

  // ───── 노드 ─────
  void onTimer();
  bool readInitialPose();
  void handleKeys();
  void publishJoints(const JointArray &q);
  void printHelp() const;
  bool setupTerminal();
  void restoreTerminal();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_;
  std::string ee_frame_;
  double pos_step_{0.005};
  double rot_step_{0.0};

  JointArray q_init_{};
  JointArray q_cur_{};
  tf2::Vector3 target_pos_, init_pos_;
  tf2::Quaternion target_rot_, init_rot_;

  bool running_{false};
  rclcpp::Time start_time_;

  termios term_orig_{};
  bool term_ok_{false};
};
