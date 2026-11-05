#pragma once

// rl_nav_correction (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/06-base.md, 11-learn.md
//   입력: /cmd_vel_nav (Nav2, 20Hz), /plan (경로 오차), /odom·/imu (실제 움직임), /speed_limit (재클리핑 상한)
//   출력: /cmd_vel_corrected (geometry_msgs/TwistStamped) → velocity_smoother
// 규칙: Δ는 선속도 ±0.05m/s · 각속도 ±0.10rad/s 로 제한, 더한 뒤 min(프리셋 상한, /speed_limit)으로 다시 자른다.
//       센서 입력이 timeout_s(0.2초) 넘게 끊기거나 enabled:=false 면 Δ=0 (통과만).
// 지금 Δ = 0 (Nav2용 보정 정책은 아직 없음) — 속도 상한(/speed_limit) 재클리핑만. 상자 앞 도킹 · 출발 구역 주차 같은
//   정밀 주행의 학습 보정은 skill_approach 노드가 자기 정책(model/skills/dock_box …)으로 직접 계산해 /cmd_vel_nav로 냄 (2026-11-04)
#include <algorithm>
#include <cmath>

#include <map>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ml {

using geometry_msgs::msg::TwistStamped;

class RlNavCorrection : public rclcpp::Node {
public:
  RlNavCorrection();

private:
  void on_cmd(const TwistStamped & in);

  double max_v_, max_w_, timeout_s_, limit_v_;
  rclcpp::Time last_sensor_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path::ConstSharedPtr plan_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_sub_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_;
  size_t corrected_ = 0;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace ml
