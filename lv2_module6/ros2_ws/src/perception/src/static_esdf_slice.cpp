// static_esdf_slice 구현 — 설명은 include/perception/static_esdf_slice.hpp
#include "perception/static_esdf_slice.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <opencv2/imgproc.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace perception {

std::vector<float> esdf_slice(const nav_msgs::msg::OccupancyGrid & map, const std::vector<float> & live_xy,
                              double rx, double ry, double rz, const EsdfSliceParams & p) {
  std::vector<float> out;
  const auto & info = map.info;
  const double res = info.resolution;
  const int w = int(info.width), h = int(info.height);
  if (res <= 0.0 || w <= 0 || h <= 0 || map.data.size() != size_t(w) * h) return out;
  // 지도 격자 좌표 (map_server origin yaw 포함)
  const auto & q = info.origin.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw), s = std::sin(yaw), ox = info.origin.position.x, oy = info.origin.position.y;
  auto to_grid = [&](double x, double y, double & gx, double & gy) {
    const double dx = x - ox, dy = y - oy;
    gx = (c * dx + s * dy) / res;
    gy = (-s * dx + c * dy) / res;
  };
  // 로봇 둘레 창 — 반경 + max_distance (창 밖 장애물이 창 안 거리에 영향 주는 몫까지)
  double gx, gy;
  to_grid(rx, ry, gx, gy);
  const int half = int(std::ceil((p.radius_m + p.max_distance_m) / res)) + 1;
  const int n = 2 * half + 1, c0 = int(std::floor(gx)) - half, r0 = int(std::floor(gy)) - half;
  cv::Mat freecells(n, n, CV_8U, cv::Scalar(255));  // distanceTransform: 0 칸까지 거리
  for (int r = 0; r < n; ++r) {
    const int mr = r0 + r;
    if (mr < 0 || mr >= h) continue;
    for (int col = 0; col < n; ++col) {
      const int mc = c0 + col;
      if (mc >= 0 && mc < w && map.data[size_t(mr) * w + mc] >= p.occupied_thresh) freecells.at<uint8_t>(r, col) = 0;
    }
  }
  for (size_t i = 0; i + 1 < live_xy.size(); i += 2) {
    double lx, ly;
    to_grid(live_xy[i], live_xy[i + 1], lx, ly);
    const int col = int(std::floor(lx)) - c0, r = int(std::floor(ly)) - r0;
    if (col >= 0 && col < n && r >= 0 && r < n) freecells.at<uint8_t>(r, col) = 0;
  }
  cv::Mat dist;
  cv::distanceTransform(freecells, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);  // [칸]

  const double rad2 = (p.radius_m / res) * (p.radius_m / res);
  const float z = float(rz + p.z_m);
  for (int r = 0; r < n; ++r) {
    const double cy = r0 + r + 0.5;
    for (int col = 0; col < n; ++col) {
      const double cx = c0 + col + 0.5;
      if ((cx - gx) * (cx - gx) + (cy - gy) * (cy - gy) > rad2) continue;
      const double d = dist.at<float>(r, col) * res;
      if (d > p.max_distance_m) continue;
      const double px = ox + (c * cx - s * cy) * res, py = oy + (s * cx + c * cy) * res;
      out.insert(out.end(), {float(px), float(py), z, float(d)});
    }
  }
  // 빈 점구름이면 controller가 stamp를 안 갱신해 0.5 s 뒤 바이패스 — 트인 곳에서도 로봇 칸 하나는 낸다
  //   (거리 > esdf_max_distance라 controller 가상 스캔에는 안 들어감 · nvblox도 빈 칸 거리를 낸다)
  if (out.empty()) out = {float(rx), float(ry), z, float(dist.at<float>(half, half) * res)};
  return out;
}

StaticEsdfSlice::StaticEsdfSlice(const rclcpp::NodeOptions & options)
: rclcpp::Node("static_esdf_slice", options) {
  const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
  const auto points_topic = declare_parameter<std::string>("points_topic", "/pole_front_cam/depth/color/points_filtered");
  const auto out_topic = declare_parameter<std::string>("output_topic", "/nvblox_node/combined_esdf_pointcloud");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  p_.radius_m = declare_parameter<double>("radius_m", p_.radius_m);
  p_.max_distance_m = declare_parameter<double>("max_distance_m", p_.max_distance_m);
  p_.z_m = declare_parameter<double>("z_m", p_.z_m);
  p_.occupied_thresh = int(declare_parameter<int>("occupied_thresh", p_.occupied_thresh));
  slice_min_z_ = declare_parameter<double>("slice_min_z", 0.08);  // nvblox.yaml esdf_slice_min_height
  slice_max_z_ = declare_parameter<double>("slice_max_z", 0.30);  // nvblox.yaml esdf_slice_max_height
  memory_s_ = declare_parameter<double>("points_memory_s", 1.0);  // 카메라가 앞만 봐서 지나친 장애물을 잠깐 기억
  stride_ = std::max<int>(1, int(declare_parameter<int>("points_stride", 2)));
  const double rate = declare_parameter<double>("rate_hz", 10.0);  // controller cloud_timeout 0.5 s 보다 충분히 자주

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::SensorDataQoS().keep_last(1));
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { map_ = m; });
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_topic, rclcpp::SensorDataQoS().keep_last(1),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate)), [this]() { on_timer(); });
  RCLCPP_INFO(get_logger(), "%s + %s → %s (static_2d ESDF slice 대역)", map_topic.c_str(), points_topic.c_str(),
              out_topic.c_str());
}

void StaticEsdfSlice::on_cloud(const sensor_msgs::msg::PointCloud2 & cloud) {
  if (cloud.width * cloud.height == 0) return;
  const std::string frame = map_ ? map_->header.frame_id : std::string("map");
  tf2::Transform T;
  double base_z;
  try {
    tf2::fromMsg(tf_->lookupTransform(frame, cloud.header.frame_id, tf2::TimePointZero).transform, T);
    base_z = tf_->lookupTransform(frame, base_frame_, tf2::TimePointZero).transform.translation.z;
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF 없음 — 점구름 건너뜀: %s", e.what());
    return;
  }
  std::vector<float> xy;
  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  const size_t total = size_t(cloud.width) * cloud.height;
  for (size_t i = 0; i < total; ++i, ++ix, ++iy, ++iz) {
    if (i % stride_ != 0 || !std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
    const tf2::Vector3 v = T * tf2::Vector3(*ix, *iy, *iz);
    const double hz = v.z() - base_z;  // 로봇 바닥 기준 높이
    if (hz < slice_min_z_ || hz > slice_max_z_) continue;
    xy.push_back(float(v.x()));
    xy.push_back(float(v.y()));
  }
  live_.emplace_back(now(), std::move(xy));
}

void StaticEsdfSlice::on_timer() {
  const auto t = now();
  while (!live_.empty() && (t - live_.front().first).seconds() > memory_s_) live_.pop_front();
  if (!map_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/map 대기 — 아직 안 냄");
    return;
  }
  geometry_msgs::msg::TransformStamped base;
  try {
    base = tf_->lookupTransform(map_->header.frame_id, base_frame_, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF 없음 — 안 냄: %s", e.what());
    return;
  }
  std::vector<float> live;
  for (const auto & e : live_) live.insert(live.end(), e.second.begin(), e.second.end());
  const auto & b = base.transform.translation;
  const auto pts = esdf_slice(*map_, live, b.x, b.y, b.z, p_);

  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = t;
  m.header.frame_id = map_->header.frame_id;
  m.height = 1;
  m.width = uint32_t(pts.size() / 4);
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (uint32_t k = 0; k < 4; ++k) {
    sensor_msgs::msg::PointField f;
    f.name = names[k];
    f.offset = 4 * k;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    m.fields.push_back(f);
  }
  m.point_step = 16;
  m.row_step = m.point_step * m.width;
  m.is_dense = true;
  m.data.resize(pts.size() * sizeof(float));
  if (!pts.empty()) std::memcpy(m.data.data(), pts.data(), m.data.size());
  pub_->publish(m);
}

}  // namespace perception
