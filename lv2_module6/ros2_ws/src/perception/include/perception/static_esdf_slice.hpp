#pragma once

// static_esdf_slice — map_backend:=static_2d(nvblox 없음)에서 nvblox 2D ESDF slice 대역.
//   RL 주행 controller(ml::RlNavController, scan_source=points)는 /nvblox_node/combined_esdf_pointcloud를
//   가상 LiDAR로 바꿔 정책 입력으로 쓴다. nvblox가 없으면 이 토픽이 끊겨 0.5 s 뒤 바이패스(MPPI)로만 달린다.
//   이 노드가 같은 토픽·같은 형식으로 낸다:
//     점 = 칸 중심 (x, y, z = 로봇 높이 + z_m) · intensity = 가장 가까운 장애물까지 거리 [m] · frame = map
//     로봇에서 radius_m 안 · 거리 max_distance_m 이하 칸만 (controller는 esdf_max_distance 0.20 이하만 쓴다)
//   장애물 = 정적 /map 점유 칸 (map_server) + 최근 points_memory_s 동안 본 점구름(body_filter가 몸 뺀 것) 중
//            로봇 바닥 기준 slice_min_z~slice_max_z 높이 점 (nvblox.yaml esdf_slice_min/max_height 0.08~0.30과 같게)
//   거리 = OpenCV distanceTransform(L2 정밀) — nvblox ESDF처럼 칸 중심에서 장애물 칸 중심까지 유클리드 거리
#include <deque>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception {

struct EsdfSliceParams {
  double radius_m = 4.0;        // 로봇에서 이 반경 안 칸만 — controller lidar_max 3.5보다 넉넉히 (fake_nvblox와 같음)
  double max_distance_m = 0.3;  // 이보다 먼 칸은 안 냄 — controller esdf_max_distance(0.20) 이상이어야 함
  double z_m = 0.1;             // 점 높이 = 로봇 바닥 + z_m (nvblox esdf_slice_height 0.1 · controller z 0.03~0.30 안)
  int occupied_thresh = 65;     // OccupancyGrid 값이 이 이상이면 장애물 (map yaml occupied_thresh 0.65)
};

// 지도 + 실시간 장애물 점(map 프레임 x,y 쌍) + 로봇 위치(map) → 점 (x, y, z, intensity) 4개씩 이어 붙인 배열
std::vector<float> esdf_slice(const nav_msgs::msg::OccupancyGrid & map, const std::vector<float> & live_xy,
                              double rx, double ry, double rz, const EsdfSliceParams & p);

class StaticEsdfSlice : public rclcpp::Node {
public:
  explicit StaticEsdfSlice(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2 & cloud);
  void on_timer();

  EsdfSliceParams p_;
  std::string base_frame_;
  double slice_min_z_, slice_max_z_, memory_s_;
  int stride_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::deque<std::pair<rclcpp::Time, std::vector<float>>> live_;  // 점구름마다 slice 높이 점 x,y (map)
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perception
