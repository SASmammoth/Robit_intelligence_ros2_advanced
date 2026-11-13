#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 시작 시 현재 엔드포인트 저장 → 원점 이동 → 대기 → IK 결과로 이동
class IkDemo : public rclcpp::Node
{
public:
  IkDemo();

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
  enum class Phase
  {
    CAPTURE, // 현재 엔드포인트·관절 읽기 → 원점으로 순간 이동
    HOLD,    // 원점에서 대기 → IK 결과로 순간 이동
    DONE     // 결과 비교 출력
  };

  void onTimer();
  void onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  bool lookupPose(geometry_msgs::msg::Pose &pose);
  void publishJoints(const JointArray &q);
  void enter(Phase p);
  double elapsed() const;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string base_frame_;
  std::string ee_frame_;
  double hold_time_{2.0};

  Phase phase_{Phase::CAPTURE};
  rclcpp::Time phase_start_;

  bool have_js_{false};
  JointArray q_start_{}; // 시작할 때 관절 (비교용)
  JointArray q_ik_{};    // IK 결과
  geometry_msgs::msg::Pose goal_;
  bool reported_{false};
};
