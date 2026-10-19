#pragma once

// rl_nav_correction 대역 (faker, ml:=fake 또는 rl_nav_correction:=fake) — 도커·CI에는 GPU·학습 모델이 없어 같은 노드 이름·토픽으로 대신 돈다.
//   ✉️ /cmd_vel_nav → /cmd_vel_corrected — Δ=0, /speed_limit 상한으로만 자름. 진짜: ml/rl_nav_correction
// /diagnostics에 "대역(fake)"으로 상태를 낸다.
#include <algorithm>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faker {

class FakeRlNavCorrection : public rclcpp::Node {
public:
  FakeRlNavCorrection();

private:
  void on_cmd(const geometry_msgs::msg::TwistStamped & in);

  double max_v_, limit_v_;
  size_t passed_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace faker
