#include "tf_listener/ik_demo_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"

namespace
{
  const char *const kJointNames[6] = {"arm_joint1", "arm_joint2", "arm_joint3",
                                      "arm_joint4", "arm_joint5", "arm_joint6"};

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

  double deg(double rad) { return rad * 180.0 / M_PI; }
} // namespace

// =====================================================================
//  IK  (calc_my_IK_node 의 solveIK 그대로)
// =====================================================================
std::optional<IkDemo::JointArray>
IkDemo::solveIK(const geometry_msgs::msg::Pose &goal) const
{
  // ========== 포지션 계산

  JointArray q{};
  const double x_ = goal.position.x;
  const double y_ = goal.position.y;
  const double z_ = goal.position.z;

  // 평면 제조 + 세타0 계산 : 코드(1)
  const double r_h = std::hypot(x_, z_);
  q[0] = (r_h < 1e-3) ? 0.0 : std::atan2(z_, x_);

  if (!fitLimit(q[0], kLimit[0].lo, kLimit[0].hi))
  {
    q[0] += M_PI; // 뒤로 뻗기 : 반대쪽을 보고 팔을 뒤로 젖힘
    if (!fitLimit(q[0], kLimit[0].lo, kLimit[0].hi))
      return std::nullopt;
  }

  if (!fitLimit(q[0], kLimit[0].lo, kLimit[0].hi))
    return std::nullopt;

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
  q_j1.setRotation(tf2::Vector3(0, 1, 0), -q[0]); // joint1 : −y축
  q_j2.setRotation(tf2::Vector3(0, 0, 1), q[1]);  // joint2  : z축
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

// =====================================================================
//  노드
// =====================================================================
IkDemo::IkDemo() : Node("ik_demo_node")
{
  const auto topic = declare_parameter<std::string>("joint_topic", "/ik/joint_states");
  base_frame_ = declare_parameter<std::string>("base_frame", "mani_base_link_MyFAKE");
  ee_frame_ = declare_parameter<std::string>("ee_frame", "gripper_base_link_MyFAKE_for_rot");
  hold_time_ = declare_parameter<double>("hold_time", 2.0); // 원점에서 대기 [s]
  const double rate_hz = declare_parameter<double>("rate_hz", 50.0);

  js_pub_ = create_publisher<sensor_msgs::msg::JointState>(topic, 10);
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&IkDemo::onJointState, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
                             [this]()
                             { onTimer(); });

  enter(Phase::CAPTURE);
  RCLCPP_INFO(get_logger(), "현재 엔드포인트 읽는 중... (%s 로 발행)", topic.c_str());
}

void IkDemo::onJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  if (have_js_)
    return;

  int found = 0;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
  {
    for (int j = 0; j < 6; j++)
    {
      if (msg->name[i] == kJointNames[j])
      {
        q_start_[j] = msg->position[i];
        found++;
      }
    }
  }
  have_js_ = (found == 6);
}

void IkDemo::onTimer()
{
  switch (phase_)
  {
  case Phase::CAPTURE:
  {
    if (!have_js_ || !lookupPose(goal_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/joint_states 와 TF 대기 중...");
      return;
    }
    js_sub_.reset(); // 시작 관절만 필요

    RCLCPP_INFO(get_logger(),
                "\n  저장한 엔드포인트 pos[mm]=(%.1f, %.1f, %.1f) quat=(%.3f, %.3f, %.3f, %.3f)"
                "\n  그때 관절[deg]       = %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f",
                goal_.position.x * 1000, goal_.position.y * 1000, goal_.position.z * 1000,
                goal_.orientation.x, goal_.orientation.y, goal_.orientation.z, goal_.orientation.w,
                deg(q_start_[0]), deg(q_start_[1]), deg(q_start_[2]),
                deg(q_start_[3]), deg(q_start_[4]), deg(q_start_[5]));
    RCLCPP_INFO(get_logger(), "원점으로 이동 → %.1f 초 대기", hold_time_);
    publishJoints(JointArray{}); // 원점으로 순간 이동
    enter(Phase::HOLD);
    return;
  }

  case Phase::HOLD:
  {
    publishJoints(JointArray{}); // 원점 유지
    if (elapsed() < hold_time_)
      return;

    const auto q = solveIK(goal_);
    if (!q)
    {
      RCLCPP_ERROR(get_logger(), "IK 해 없음 → 원점에서 멈춤");
      phase_ = Phase::DONE;
      reported_ = true;
      return;
    }
    q_ik_ = *q;
    RCLCPP_INFO(get_logger(), "IK 결과로 이동");
    publishJoints(q_ik_); // IK 결과로 순간 이동
    enter(Phase::DONE);
    return;
  }

  case Phase::DONE:
  {
    if (reported_)
      return;
    publishJoints(q_ik_);
    if (elapsed() < 0.5) // TF 반영 대기
      return;

    geometry_msgs::msg::Pose now_pose;
    if (!lookupPose(now_pose))
      return;

    const tf2::Vector3 p0(goal_.position.x, goal_.position.y, goal_.position.z);
    const tf2::Vector3 p1(now_pose.position.x, now_pose.position.y, now_pose.position.z);
    const tf2::Quaternion r0(goal_.orientation.x, goal_.orientation.y, goal_.orientation.z, goal_.orientation.w);
    const tf2::Quaternion r1(now_pose.orientation.x, now_pose.orientation.y, now_pose.orientation.z, now_pose.orientation.w);

    RCLCPP_INFO(get_logger(),
                "\n  ===== 결과 ====="
                "\n  원래 관절[deg] = %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f"
                "\n  IK   관절[deg] = %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f"
                "\n  엔드포인트 오차 : 위치 %.3f mm, 회전 %.3f deg",
                deg(q_start_[0]), deg(q_start_[1]), deg(q_start_[2]),
                deg(q_start_[3]), deg(q_start_[4]), deg(q_start_[5]),
                deg(q_ik_[0]), deg(q_ik_[1]), deg(q_ik_[2]),
                deg(q_ik_[3]), deg(q_ik_[4]), deg(q_ik_[5]),
                p0.distance(p1) * 1000.0, deg(r0.angleShortestPath(r1)));
    reported_ = true;
    return;
  }
  }
}

bool IkDemo::lookupPose(geometry_msgs::msg::Pose &pose)
{
  geometry_msgs::msg::TransformStamped t;
  try
  {
    t = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &)
  {
    return false;
  }
  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;
  return true;
}

void IkDemo::publishJoints(const JointArray &q)
{
  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  for (int i = 0; i < 6; i++)
  {
    js.name.emplace_back(kJointNames[i]);
    js.position.push_back(q[i]);
  }
  js_pub_->publish(js);
}

void IkDemo::enter(Phase p)
{
  phase_ = p;
  phase_start_ = now();
}

double IkDemo::elapsed() const
{
  return (now() - phase_start_).seconds();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkDemo>());
  rclcpp::shutdown();
  return 0;
}
