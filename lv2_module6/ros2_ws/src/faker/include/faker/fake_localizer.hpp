#pragma once

// fake_localizer (faker 대역) — 실기에서 RTAB-Map이 하는 map → odom 보정을 시뮬레이터 진짜 위치로 대신한다 (map:=course · fake_opencr)
//   입력: ✉️ /fake_opencr/ground_truth (PoseStamped, map — 미끄럼·충돌까지 반영된 진짜 위치) · TF odom → base_footprint (EKF)
//   출력: TF map → odom = 진짜 위치 × (odom 위치)⁻¹ — 바퀴가 헛돌거나 벽·박스에 걸려 odom이 틀어져도 로봇이 아는 map 위치는 진짜
//   (예전 map → odom 항등이면 한 번 걸린 뒤 위치가 2 m 넘게 틀어져 Nav2가 출발점부터 못 찾았음 — 2026-10-28 주사위 맵 시험)
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace faker {

class FakeLocalizer : public rclcpp::Node {
public:
  FakeLocalizer();

private:
  void on_truth(const geometry_msgs::msg::PoseStamped & truth);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

}  // namespace faker
