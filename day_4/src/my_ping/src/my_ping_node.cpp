#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include "pingpong_lab_interfaces/msg/pingpongball.hpp"
#include "my_ping/my_ping_node.hpp"

my_pingpong::MyPingNode::MyPingNode() : Node("my_ping_node")
{
  N_ = declare_parameter<int>("N", 200);
  hz_ = declare_parameter<double>("hz", 10.0);
  size_ = declare_parameter<int>("size", 1024);

  samples_.resize(N_); // 결과 벡터 크기 늘리기
  msg_.size = size_;
  msg_.data.resize(size_);

  pub_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz_), [this]
                                 { onPingTimer(); });
  boll_sub_ = create_subscription<Ball>("pong", 10, [this](const Ball &m)
                                        { boll_callback(m); });
  boll_pub_ = create_publisher<Ball>("ping", 10);
}

void my_pingpong::MyPingNode::finish()
{
  finish_timer_->cancel();

  std::vector<double> rtts;
  size_t lost = 0;
  for (const auto &s : samples_)
  {
    if (s.received())
      rtts.push_back(s.rtt_ms());
    else
      ++lost;
  }
  std::sort(rtts.begin(), rtts.end());
  auto pct = [&](double p)
  {
    return rtts.empty() ? 0.0 : rtts[static_cast<size_t>(p * (rtts.size() - 1))];
  };

  RCLCPP_INFO(get_logger(),
              "size=%d sent=%zu lost=%zu (%.2f%%) | RTT ms p50=%.2f p95=%.2f p99=%.2f max=%.2f",
              size_, samples_.size(), lost, 100.0 * lost / samples_.size(),
              pct(0.50), pct(0.95), pct(0.99), rtts.empty() ? 0.0 : rtts.back());

  rclcpp::shutdown();
}

void my_pingpong::MyPingNode::onPingTimer()
{
  // /ping을 pong이 구독하고, /pong을 pong이 발행할 때까지 대기
  if (boll_pub_->get_subscription_count() == 0 || boll_sub_->get_publisher_count() == 0)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "pong 연결 대기 중...");
    return;
  }

  if (count >= static_cast<uint32_t>(N_))
  {
    pub_timer_->cancel();
    finish_timer_ = create_wall_timer(std::chrono::seconds(2), [this]
                                      { finish(); });
    return;
  }

  // 메시지 생성해서 전송하기
  msg_.header.stamp = now(); // 현재 시간 기입
  msg_.seq = count++; // 시퀀스 번호 기입 및 시간 추가
  samples_[msg_.seq] = {msg_.seq, static_cast<uint32_t>(size_), nowNs(), -1};
  boll_pub_->publish(msg_);
}

void my_pingpong::MyPingNode::boll_callback(const Ball &msg)
{
  const int64_t t = nowNs();
  if (msg.seq >= samples_.size())
    return; // 이상한 seq 방어
  auto &s = samples_[msg.seq];
  if (s.received())
    return; // 중복 수신 방어
  s.t_recv_ns = t;
}

int64_t my_pingpong::MyPingNode::nowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_pingpong::MyPingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
