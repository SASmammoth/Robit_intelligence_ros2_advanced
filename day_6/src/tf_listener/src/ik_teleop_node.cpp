#include "tf_listener/ik_teleop_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <vector>

#include <unistd.h>

#include "tf2/exceptions.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

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
std::optional<IkTeleop::JointArray>
IkTeleop::solveIK(const geometry_msgs::msg::Pose &goal) const
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
IkTeleop::IkTeleop() : Node("ik_teleop_node")
{
  const auto topic = declare_parameter<std::string>("joint_topic", "/ik/joint_states");
  base_frame_ = declare_parameter<std::string>("base_frame", "mani_base_link_MyFAKE");
  ee_frame_ = declare_parameter<std::string>("ee_frame", "gripper_base_link_MyFAKE_for_rot");
  pos_step_ = declare_parameter<double>("pos_step", 0.005);                     // 한 번 누를 때 [m]
  rot_step_ = declare_parameter<double>("rot_step_deg", 2.0) * M_PI / 180.0;    // 한 번 누를 때 [deg]
  const double rate_hz = declare_parameter<double>("rate_hz", 30.0);
  const auto init = declare_parameter<std::vector<double>>(
      "init_joints", std::vector<double>{0.0, 0.3, 1.2, 0.0, 0.6, 0.0});         // 시작 자세 [rad]

  for (size_t i = 0; i < 6 && i < init.size(); i++)
    q_init_[i] = init[i];
  q_cur_ = q_init_;

  js_pub_ = create_publisher<sensor_msgs::msg::JointState>(topic, 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!setupTerminal())
    RCLCPP_WARN(get_logger(), "터미널(tty)이 아니라 키 입력을 받을 수 없음 → ros2 run 으로 직접 실행하세요");

  start_time_ = now();
  timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
                             [this]()
                             { onTimer(); });

  RCLCPP_INFO(get_logger(), "초기 자세로 이동 중... (%s 로 발행)", topic.c_str());
}

IkTeleop::~IkTeleop()
{
  restoreTerminal();
}

void IkTeleop::onTimer()
{
  if (!running_)
  {
    // 시작 자세를 발행하고, TF에 반영되면 그 엔드포인트를 목표 초기값으로 사용
    publishJoints(q_init_);
    if ((now() - start_time_).seconds() < 1.0)
      return;
    if (!readInitialPose())
      return;
    running_ = true;
    printHelp();
    return;
  }

  handleKeys();
  publishJoints(q_cur_);
}

bool IkTeleop::readInitialPose()
{
  geometry_msgs::msg::TransformStamped t;
  try
  {
    t = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException &e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 대기 중: %s", e.what());
    return false;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  // TF가 정말 시작 자세를 반영하는지 IK로 확인 (source_list 설정이 안 되어 있으면 여기서 걸림)
  const auto q = solveIK(pose);
  double diff = 1e9;
  if (q)
  {
    diff = 0.0;
    for (int j = 0; j < 6; j++)
      diff = std::max(diff, std::abs(wrapPi((*q)[j] - q_init_[j])));
  }
  if (diff > 2.0 * M_PI / 180.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF가 시작 자세와 다름 → joint_state_publisher 의 source_list 설정을 확인하세요");
    return false;
  }

  init_pos_ = target_pos_ = tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
  init_rot_ = target_rot_ = tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                                            pose.orientation.z, pose.orientation.w);
  return true;
}

void IkTeleop::handleKeys()
{
  if (!term_ok_)
    return;

  tf2::Vector3 pos = target_pos_;
  tf2::Quaternion rot = target_rot_;
  bool changed = false;

  const tf2::Vector3 ex(1, 0, 0), ey(0, 1, 0), ez(0, 0, 1);
  char c;
  while (::read(STDIN_FILENO, &c, 1) == 1)
  {
    switch (std::tolower(static_cast<unsigned char>(c)))
    {
    // 위치 : base_frame(mani_base_link_MyFAKE) 기준 → x 앞, y 아래, z 왼쪽
    case 'w': pos += ex * pos_step_; break; // 앞
    case 's': pos -= ex * pos_step_; break; // 뒤
    case 'a': pos += ez * pos_step_; break; // 왼쪽
    case 'd': pos -= ez * pos_step_; break; // 오른쪽
    case 'q': pos -= ey * pos_step_; break; // 위
    case 'e': pos += ey * pos_step_; break; // 아래

        // 회전 : 베이스(mani_base_link_MyFAKE) 축 기준 (왼쪽 곱)
    //   x = 앞, y = 아래, z = 왼쪽
    case 'i': rot = tf2::Quaternion(ez, +rot_step_) * rot; break; // 왼쪽 축(z) 회전
    case 'k': rot = tf2::Quaternion(ez, -rot_step_) * rot; break;
    case 'j': rot = tf2::Quaternion(ey, -rot_step_) * rot; break; // 수직 축 회전 (위에서 볼 때 반시계)
    case 'l': rot = tf2::Quaternion(ey, +rot_step_) * rot; break;
    case 'u': rot = tf2::Quaternion(ex, +rot_step_) * rot; break; // 앞 축(x) 회전
    case 'o': rot = tf2::Quaternion(ex, -rot_step_) * rot; break;

    case 'r': pos = init_pos_; rot = init_rot_; break;            // 시작 자세로
    case 'h': printHelp(); continue;
    default: continue;
    }
    changed = true;
  }
  if (!changed)
    return;

  rot.normalize();
  geometry_msgs::msg::Pose goal;
  goal.position.x = pos.x();
  goal.position.y = pos.y();
  goal.position.z = pos.z();
  goal.orientation.x = rot.x();
  goal.orientation.y = rot.y();
  goal.orientation.z = rot.z();
  goal.orientation.w = rot.w();

  const auto q = solveIK(goal);
  if (!q)
  {
    RCLCPP_WARN(get_logger(), "IK 해 없음 → 이동 무시 (작업 공간 또는 관절 한계 밖)");
    return; // 목표를 바꾸지 않음 → 마지막 가능한 자세 유지
  }

  target_pos_ = pos;
  target_rot_ = rot;
  q_cur_ = *q;

  RCLCPP_INFO(get_logger(),
              "pos[mm]=(%.0f, %.0f, %.0f) | q[deg]= %.1f %.1f %.1f | %.1f %.1f %.1f",
              pos.x() * 1000, pos.y() * 1000, pos.z() * 1000,
              deg(q_cur_[0]), deg(q_cur_[1]), deg(q_cur_[2]),
              deg(q_cur_[3]), deg(q_cur_[4]), deg(q_cur_[5]));
}

void IkTeleop::publishJoints(const JointArray &q)
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

void IkTeleop::printHelp() const
{
  RCLCPP_INFO(get_logger(),
              "\n"
              "  ── 위치 (%.0f mm/키) ──────────────  ── 회전 (%.1f deg/키) ─────\n"
              "    w / s : 앞 / 뒤                    i / k : pitch\n"
              "    a / d : 왼쪽 / 오른쪽              j / l : yaw\n"
              "    q / e : 위 / 아래                  u / o : roll\n"
              "    r : 시작 자세로    h : 도움말    Ctrl+C : 종료\n",
              pos_step_ * 1000, deg(rot_step_));
}

bool IkTeleop::setupTerminal()
{
  if (!isatty(STDIN_FILENO))
    return false;
  if (tcgetattr(STDIN_FILENO, &term_orig_) != 0)
    return false;

  termios raw = term_orig_;
  raw.c_lflag &= ~(ICANON | ECHO); // 엔터 없이 한 글자씩, 화면에 안 찍힘 (Ctrl+C는 그대로 동작)
  raw.c_cc[VMIN] = 0;              // read()가 기다리지 않음
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
    return false;

  term_ok_ = true;
  return true;
}

void IkTeleop::restoreTerminal()
{
  if (term_ok_)
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &term_orig_);
    term_ok_ = false;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<IkTeleop>();
    rclcpp::spin(node);
  } // 소멸자에서 터미널 복구
  rclcpp::shutdown();
  return 0;
}
