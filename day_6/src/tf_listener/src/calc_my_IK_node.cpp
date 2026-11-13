#include "tf_listener/calc_my_IK_node.hpp"

#include <chrono>
#include "tf2/exceptions.hpp"
#include "tf2/utils.hpp"

#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

#include <cmath>
#include <algorithm>

namespace
{
  double wrapPi(double a) // [-π, π]로 정규화
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  // a를 [lo, hi] 안에 넣을 수 있으면 true (2π 감기 + 1° 여유, 결과는 clamp)
  bool fitLimit(double &a, double lo, double hi, double tol = 1.0 * M_PI / 180.0)
  {
    a = wrapPi(a);
    for (double cand : {a, a + 2 * M_PI, a - 2 * M_PI})
    {
      if (cand >= lo - tol && cand <= hi + tol)
      {
        a = std::clamp(cand, lo, hi);
        return true;
      }
    }
    return false;
  }
}

std::optional<Calc_my_IK::JointArray>
Calc_my_IK::solveIK(const geometry_msgs::msg::Pose &goal) const
{
  // ========== 포지션 계산

  JointArray q{};
  const double x_ = goal.position.x;
  const double y_ = goal.position.y;
  const double z_ = goal.position.z;

  /// 평면 제조 + 세타0 계산 : 코드(1)
  const double r_h = std::hypot(x_, z_);
  q[0] = (r_h < 1e-3) ? 0.0 : std::atan2(z_, x_);

  if (!fitLimit(q[0], kLimit[0].lo, kLimit[0].hi))
  {
    q[0] += M_PI; // 뒤로 뻗기 : 반대쪽을 보고 팔을 뒤로 젖힘
    if (!fitLimit(q[0], kLimit[0].lo, kLimit[0].hi))
      return std::nullopt;
  }

  tf2::Vector3 p(x_, y_, z_);
  tf2::Quaternion r;
  r.setRotation(tf2::Vector3(0, 1, 0), q[0]);
  tf2::Vector3 pp = tf2::quatRotate(r, p);


  // 포지션 계산 (세타 1,2) : 코드(2)
  const double x_plane = pp.x(); // 수평 거리 (≥ 0)
  const double y_plane = pp.y(); // 높이 (위쪽이 −)

  const double off_[2] = {-1.5059748, -0.1286831};
  const double C_2 = (x_plane * x_plane + y_plane * y_plane - a_1 * a_1 - a_2 * a_2) / (2 * a_1 * a_2);
  if (C_2 < -1.0 || C_2 > 1.0)
  {
    return std::nullopt; // 닿지 않는 위치
  }

  const double S_2[2] = {sqrt(1 - C_2 * C_2), -sqrt(1 - C_2 * C_2)};
  double th2[2];
  double th1[2];
  for (int i = 0; i < 2; i++)
  {
    th2[i] = atan2(S_2[i], C_2);
    th1[i] = atan2(y_plane, x_plane) - atan2(a_2 * S_2[i], a_1 + a_2 * C_2);
    th1[i] -= off_[0];
    th2[i] -= off_[1];
  }
  bool found_q23 = false;
  for (int i = 0; i < 2; i++)
  {
    double j2 = th1[i];
    double j3 = th2[i];
    if (!fitLimit(j2, kLimit[1].lo, kLimit[1].hi))
      continue;
    if (!fitLimit(j3, kLimit[2].lo, kLimit[2].hi))
      continue;

    q[1] = j2;
    q[2] = j3;
    found_q23 = true;
    break; // 첫 번째로 가능한 해 사용
  }
  if (!found_q23)
  {
    return std::nullopt; // 둘 다 한계 밖
  }






  // ===== 손목 계산

  // 목표
  const tf2::Matrix3x3 R_goal(tf2::Quaternion(goal.orientation.x, goal.orientation.y,
                                         goal.orientation.z, goal.orientation.w));

  // 팔 회전 행렬
  tf2::Quaternion q_j1, q_j2, q_j3;
  q_j1.setRotation(tf2::Vector3(0, 1, 0), -q[0]);        // joint1 : −y축
  q_j2.setRotation(tf2::Vector3(0, 0, 1), q[1]); // joint2  : z축
  q_j3.setRotation(tf2::Vector3(0, 0, 1), q[2]);
  const tf2::Matrix3x3 R_arm(q_j1 * q_j2 * q_j3);

  // 손목 회전행렬
  const tf2::Matrix3x3 R_wrist = R_arm.transposeTimes(R_goal);

  // j3,4,5 계산
  q[4] = atan2(sqrt(R_wrist[0][1] * R_wrist[0][1] + R_wrist[2][1] * R_wrist[2][1]), R_wrist[1][1]);
  q[3] = -atan2(R_wrist[2][1], -R_wrist[0][1]);
  q[5] = -atan2(R_wrist[1][2], R_wrist[1][0]);

  return q;
}

Calc_my_IK::Calc_my_IK() : Node("Calc_my_IK_node")
{
  tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]()
                             { onTimer(); });
}

void Calc_my_IK::onTimer()
{
  geometry_msgs::msg::TransformStamped tB2G;
  geometry_msgs::msg::TransformStamped tB2link_3F;
  geometry_msgs::msg::TransformStamped tlink_3F2G;
  geometry_msgs::msg::TransformStamped tB2Grot;

  try
  {
    tB2G = tf_buffer->lookupTransform(
        "mani_base_link_MyFAKE",
        "gripper_base_link_MyFAKE",
        tf2::TimePointZero);
    tB2link_3F = tf_buffer->lookupTransform(
        "mani_base_link_MyFAKE",
        "link3_1_FAKE",
        tf2::TimePointZero);
    tlink_3F2G = tf_buffer->lookupTransform(
        "link3_1_FAKE",
        "gripper_base_link_MyFAKE",
        tf2::TimePointZero);
    tB2Grot = tf_buffer->lookupTransform(
        "mani_base_link_MyFAKE",
        "gripper_base_link_MyFAKE_for_rot",
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF 없음: %s", e.what());
    return;
  }

  const auto &p_tB2G = tB2G.transform.translation;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "tB2G : x=%.6f y=%.6f z=%.6f",
                       p_tB2G.x, p_tB2G.y, p_tB2G.z);
  /*
  const auto &p_tB2link_3F = tB2link_3F.transform.translation;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "tB2link_6F : x=%.6f y=%.6f z=%.6f",
                       p_tB2link_3F.x, p_tB2link_3F.y, p_tB2link_3F.z);

  const auto &p_tlink_3F2G = tlink_3F2G.transform.translation;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "tlink_3F2G : x=%.6f y=%.6f z=%.6f",
                       p_tlink_3F2G.x, p_tlink_3F2G.y, p_tlink_3F2G.z);
                       */

  geometry_msgs::msg::Pose goal;
  goal.position.x = tB2Grot.transform.translation.x;
  goal.position.y = tB2Grot.transform.translation.y;
  goal.position.z = tB2Grot.transform.translation.z;
  goal.orientation = tB2Grot.transform.rotation;

  if (auto q = solveIK(goal))
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK deg: j0=%.1f j1=%.1f j2=%.1f\n[INFO] [0000000000.000000000] [               ]: IK deg: j3=%.1f j4=%.1f j5=%.1f",
                         (*q)[0] * 180 / M_PI, (*q)[1] * 180 / M_PI, (*q)[2] * 180 / M_PI,
                         (*q)[3] * 180 / M_PI, (*q)[4] * 180 / M_PI, (*q)[5] * 180 / M_PI);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK rad: j0=%.3f j1=%.3f j2=%.3f\n[INFO] [0000000000.000000000] [               ]: IK rad: j3=%.3f j4=%.3f j5=%.3f\n\n",
                         (*q)[0], (*q)[1], (*q)[2],
                         (*q)[3], (*q)[4], (*q)[5]);
  }
  else
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IK 해 없음");
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Calc_my_IK>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
