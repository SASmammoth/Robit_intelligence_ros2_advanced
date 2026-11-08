#pragma once

#include <chrono>
#include <cstdint>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "pingpong_lab_interfaces/msg/pingpongball.hpp"

namespace my_pingpong
{
  struct Sample
  {
    uint32_t seq;
    uint32_t size;     // payload 크기 (bytes)
    int64_t t_send_ns = -1; // 보낸 시각 (steady_clock)
    int64_t t_recv_ns = -1; // pong 받은 시각, 못 받았으면 -1
    bool received() const { return t_recv_ns >= 0; }
    double rtt_ms() const { return (t_recv_ns - t_send_ns) / 1e6; }
  };

  using Ball = pingpong_lab_interfaces::msg::Pingpongball;

  class MyPingNode : public rclcpp::Node
  {
  public:
    MyPingNode();

  private:
    rclcpp::Publisher<Ball>::SharedPtr boll_pub_;                        // 발행자 생성
    rclcpp::Subscription<Ball>::SharedPtr boll_sub_; // 구독자 생성
    void boll_callback(const Ball &msg);             // 구독자 콜백

    rclcpp::TimerBase::SharedPtr pub_timer_;
    void onPingTimer();
    uint32_t count = 0; // 몇회 전송했는지 카운터

    std::vector<Sample> samples_; // 결과를 저장하는 창

    rclcpp::TimerBase::SharedPtr finish_timer_;
    void finish();

    static int64_t nowNs();

    Ball msg_;// 전송하는 메시지

    // 파라미터 변수
    int N_;     // 전송
    double hz_; // 전송 주기
    int size_; // 전송하는 패킷 크기
  };
}
