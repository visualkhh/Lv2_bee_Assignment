#pragma once

// fake_nvblox (faker 대역) — 테스트베드 SLAM + ESDF.
//   MAPPING: /pole_front_cam/depth/color/points + TF(map←camera)로 점유 격자를 쌓아 /map(transient_local)을 내고
//            <map_dir>/<map_name>/nvblox.nvblx(FAKESLAM1)에 저장한다. 코스 지도를 미리 쓰지 않는다.
//   LOCALIZATION: 저장한 nvblox.nvblx를 읽어 /map을 내고, 그 지도에서 ESDF를 계산한다.
//   ESDF: 실기 nvblox의 /nvblox_node/combined_esdf_pointcloud와 같은 형식
//     (점 = 칸 중심 (x, y, z_m), intensity = 장애물까지 거리 [m], frame = map, 로봇에서 radius_m 안).
//   지도 파일 FAKESLAM1: magic 9B + int32 width·height + double resolution·origin_x·origin_y + int8 data[].
//     진짜 nvblox(.nvblx) 파일이 아니라 테스트베드 전용이다 — state_manager 지도 목록·map:=saved가 이 파일을 본다.
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace faker {

// 점유 격자 → 칸마다 가장 가까운 장애물 칸까지 거리 [칸] (8방향 chamfer 1·√2 — 정확한 유클리드와 최대 약 8% 차이)
//   occupied: true = 장애물. 장애물이 하나도 없으면 전부 +inf.
std::vector<float> chamfer_distance(const std::vector<bool> & occupied, int width, int height);

class FakeNvblox : public rclcpp::Node {
public:
  FakeNvblox();
  ~FakeNvblox() override;

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2 & cloud);
  void on_timer();
  void init_grid();
  void publish_map();
  void publish_esdf();
  void rebuild_esdf();
  bool load_map_file();
  bool save_map_file() const;
  bool save_preview_files() const;  // map.pgm · map.yaml (사람·도구용 미리보기 — 정본은 nvblox.nvblx)
  int cell_index(double x, double y) const;
  void mark_free(double x, double y);
  void mark_occupied(double x, double y);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid map_;  // info + data (MAPPING/LOCALIZATION이면 이 노드가 /map 발행)
  std::vector<float> esdf_m_;         // 칸마다 장애물까지 거리 [m]
  std::vector<bool> known_;           // 관측된 칸 (OccupancyGrid -1이 아님)
  bool have_map_ = false;             // ESDF를 낼 지도가 있음
  bool mapping_ = false;
  bool map_dirty_ = false;
  bool map_published_ = false;
  double last_map_pub_ = 0.0;
  double last_save_ = 0.0;

  std::string base_frame_ = "base_footprint";
  double radius_m_ = 4.0;       // 로봇에서 이 반경 안 칸만 [m] — rl_nav_controller lidar_max(3.5)보다 넉넉히
  double z_m_ = 0.1;            // slice 높이 [m] (map 기준 — nvblox slice 0.03~0.30)
  double max_distance_m_ = 2.0;  // ESDF 상한 [m] — nvblox max_distance처럼 이보다 먼 거리는 이 값
  int occupied_thresh_ = 65;    // OccupancyGrid 값이 이 이상이면 장애물 (map yaml occupied_thresh 0.65)

  std::string map_dir_;         // 지도 폴더 (MAPPING 저장 · LOCALIZATION 로드)
  std::string map_name_;        // 지도 폴더 이름
  std::string map_mode_;        // MAPPING | LOCALIZATION
};

}  // namespace faker
