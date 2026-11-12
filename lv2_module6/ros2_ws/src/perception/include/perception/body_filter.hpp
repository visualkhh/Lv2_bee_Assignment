#pragma once

// body_filter (🛠 직접 작성, 담당: 전승혜) — 카메라 점구름에서 로봇 자기 몸(팔·차체·바구니)에 찍힌 점을 뺀다
//   pole_front_cam은 차체 뒤 폴 위에서 앞을 내려다봐서 팔이 화면에 들어온다 → 빼지 않으면 Nav2 장애물 지도·collision_monitor가
//   자기 팔을 장애물로 보고 멈춘다. Lyrical에는 robot_body_filter가 없고 pcl_ros CropBox는 고정 상자 하나뿐이라 직접 작성.
//   입력: input_topic (기본 /pole_front_cam/depth/color/points) · /robot_description (URDF, transient_local) · TF
//   출력: output_topic (기본 /pole_front_cam/depth/color/points_filtered) — Nav2 costmap·collision_monitor가 이걸 본다
//         no_floor_topic (기본 /pole_front_cam/depth/color/points_no_floor) — 위에서 바닥 점(floor_frame 기준 높이 floor_z 이하)까지 뺀 것
//           → MoveIt octomap 센서(ros_system/config/moveit/sensors_3d.yaml)가 본다. 바닥이 장애물 칸이 되면 팔이 바닥 퍽을
//             집으러 내려가지 못함 (이슈 #126). floor_z 2cm = depth 잡음(약 1cm)만 빼는 바닥 여유 (그보다 높은 물체는 남김 — 누운 퍽 30mm도 윗부분이 장애물).
//             퍽(서 있음 60mm · 누움 30mm)의 윗부분은 남음 — 집을 퍽을 장애물에서 빼는 건 skill_pick 쪽 (CollisionObject)
//             받는 쪽이 있을 때만 계산 (없으면 CPU 안 씀)
// 판단: URDF <collision> 모양을 링크마다 TF로 카메라 프레임에 옮겨, 그 상자(+ padding) 안의 점을 뺀다.
//   box는 그대로 · cylinder·sphere는 맞춤 상자 · mesh는 STL 경계 상자 (package:// → share 경로).
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception {

using sensor_msgs::msg::PointCloud2;

// 링크에 붙은 상자 하나 — origin = 링크 좌표의 상자 중심·방향, half = 반 크기 [m]
struct BodyBox {
  std::string link;
  tf2::Transform origin;
  tf2::Vector3 half;
};

// URDF 문자열 → 충돌 상자들 (mesh STL을 못 찾으면 그 모양은 건너뜀)
std::vector<BodyBox> parse_collision_boxes(const std::string & urdf_xml);

class BodyFilter : public rclcpp::Node {
public:
  explicit BodyFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_cloud(const PointCloud2 & in);

  double padding_;
  int stride_{1};
  std::string floor_frame_;  // 바닥 높이를 잴 프레임 (z = 0이 바닥)
  double floor_z_;           // 이 높이 이하 점은 no_floor 출력에서 뺌 [m]
  std::vector<BodyBox> boxes_;
  size_t removed_ = 0, clouds_ = 0, floor_removed_ = 0;
  std::string last_error_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr no_floor_pub_;  // 몸 + 바닥 뺀 점구름 (MoveIt octomap용)
  diagnostic_updater::Updater diag_{this};
};

}  // namespace perception
