#pragma once

// rl_nav_tuner 대역 (faker, ml:=fake 또는 rl_nav_tuner:=fake) — 도커·CI에는 GPU·학습 모델이 없어 같은 노드 이름·토픽으로 대신 돈다.
//   ✉️ /speed_limit — 프리셋 A(100%) 고정, controller_server 파라미터는 안 건드림. 진짜: ml/rl_nav_tuner
// /diagnostics에 "대역(fake)"으로 상태를 낸다.
#include <chrono>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faker {

class FakeRlNavTuner : public rclcpp::Node {
public:
  FakeRlNavTuner();

private:
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace faker
