#pragma once

// fake_nvblox (faker 대역) — 실기에서 nvblox가 내는 2D ESDF slice를 코스 지도로 대신한다 (map:=course)
//   진짜 nvblox: depth 영상으로 TSDF를 쌓아, 높이 slice의 voxel마다 "가장 가까운 장애물까지 거리"(ESDF)를
//     intensity에 넣은 점구름 /nvblox_node/combined_esdf_pointcloud (전역 프레임)를 낸다.
//   대역: 시뮬레이터가 코스를 알고 있으므로(fake_cuvslam이 진짜 위치를 쓰듯) /map(코스 지도)에서 같은 ESDF를 계산한다.
//   입력: ✉️ /map (OccupancyGrid, transient_local — faker map_server) · TF map → base_footprint (로봇 근처만 냄)
//   출력: ✉️ ~/combined_esdf_pointcloud (PointCloud2, frame = map) — 점 = 칸 중심 (x, y, z_m), intensity = 장애물까지 거리 [m]
//     장애물 칸은 0, 알 수 없는 칸(-1)은 안 냄(nvblox도 관측 안 한 voxel은 없음). 로봇에서 radius_m 안의 칸만.
//   (예전 대역은 "빈 지도"라 모든 점이 free → rl_nav_controller 가상 스캔이 늘 비어 정책이 벽을 못 봤음 — 2026-11-07 A/B)
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace faker {

// 점유 격자 → 칸마다 가장 가까운 장애물 칸까지 거리 [칸] (8방향 chamfer 1·√2 — 정확한 유클리드와 최대 약 8% 차이)
//   occupied: true = 장애물. 장애물이 하나도 없으면 전부 +inf.
std::vector<float> chamfer_distance(const std::vector<bool> & occupied, int width, int height);

class FakeNvblox : public rclcpp::Node {
public:
  FakeNvblox();

private:
  void on_map(const nav_msgs::msg::OccupancyGrid & map);
  void on_timer();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid map_;  // info만 쓰고 data는 아래 두 배열로
  std::vector<float> esdf_m_;         // 칸마다 장애물까지 거리 [m]
  std::vector<bool> known_;           // 관측된 칸 (OccupancyGrid -1이 아님)
  bool have_map_ = false;

  std::string base_frame_ = "base_footprint";
  double radius_m_ = 4.0;       // 로봇에서 이 반경 안 칸만 [m] — rl_nav_controller lidar_max(3.5)보다 넉넉히
  double z_m_ = 0.1;            // slice 높이 [m] (map 기준 — nvblox slice 0.03~0.30)
  double max_distance_m_ = 2.0;  // ESDF 상한 [m] — nvblox max_distance처럼 이보다 먼 거리는 이 값
  int occupied_thresh_ = 65;    // OccupancyGrid 값이 이 이상이면 장애물 (map yaml occupied_thresh 0.65)
};

}  // namespace faker
