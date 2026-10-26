#pragma once

// rl_nav_tuner (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/11-learn.md "rl_nav_tuner 프리셋"
//   입력: /local_costmap/costmap (통로 폭·최근접 장애물), /odometry/filtered (현재 속도), /plan (남은 거리)
//   출력: ✉️ /speed_limit (nav2_msgs/SpeedLimit) → controller_server, rl_nav_correction
//         📞 /controller_server/set_parameters (DWB kinematic: max_vel_theta·acc_lim_x·decel_lim_x)
// 1~2초마다 프리셋 A(넓은 공간)/B(좁은 통로)/C(목표 직전)를 고르고, 바뀔 때만 내보낸다.
// enabled:=false → 프리셋 A 고정 (학습 노드는 끌 수 있어야 한다).
// TODO(송정혁): choose()를 규칙으로 → 필요하면 RL 정책으로
#include <chrono>
#include <cmath>
#include <map>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ml {

class RlNavTuner : public rclcpp::Node {
public:
  RlNavTuner();

private:
  char choose() const;

  void tick();

  bool enabled_;
  char current_ = 0;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  nav_msgs::msg::Path::ConstSharedPtr plan_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_pub_;
  rclcpp::AsyncParametersClient::SharedPtr params_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace ml
