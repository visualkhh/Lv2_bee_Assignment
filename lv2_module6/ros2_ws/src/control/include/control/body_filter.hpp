#pragma once

// body_filter (🛠 직접 작성, 담당: 김현하) — 카메라 점구름에서 로봇 자기 몸(팔·차체·바구니)에 찍힌 점을 뺀다
//   pole_cam은 차체 뒤 폴 위에서 앞을 내려다봐서 팔이 화면에 들어온다 → 빼지 않으면 Nav2 장애물 지도·collision_monitor가
//   자기 팔을 장애물로 보고 멈춘다. Lyrical에는 robot_body_filter가 없고 pcl_ros CropBox는 고정 상자 하나뿐이라 직접 작성.
//   입력: input_topic (기본 /pole_cam/depth/color/points) · /robot_description (URDF, transient_local) · TF
//   출력: output_topic (기본 /pole_cam/depth/color/points_filtered) — Nav2 costmap·collision_monitor가 이걸 본다
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

namespace control {

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
  std::vector<BodyBox> boxes_;
  size_t removed_ = 0, clouds_ = 0;
  std::string last_error_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace control
