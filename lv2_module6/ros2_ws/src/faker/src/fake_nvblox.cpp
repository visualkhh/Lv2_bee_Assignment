// fake_nvblox 구현 — 설명은 include/faker/fake_nvblox.hpp
#include "faker/fake_nvblox.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace faker {

std::vector<float> chamfer_distance(const std::vector<bool> & occupied, int width, int height) {
  const float inf = std::numeric_limits<float>::infinity();
  const float diag = std::sqrt(2.0f);
  std::vector<float> d(size_t(width) * height, inf);
  for (size_t i = 0; i < d.size(); ++i) {
    if (occupied[i]) d[i] = 0.0f;
  }
  auto at = [&](int x, int y) -> float & { return d[size_t(y) * width + x]; };
  // 앞으로 한 번 (왼쪽·위), 뒤로 한 번 (오른쪽·아래)
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float & v = at(x, y);
      if (x > 0) v = std::min(v, at(x - 1, y) + 1.0f);
      if (y > 0) {
        v = std::min(v, at(x, y - 1) + 1.0f);
        if (x > 0) v = std::min(v, at(x - 1, y - 1) + diag);
        if (x + 1 < width) v = std::min(v, at(x + 1, y - 1) + diag);
      }
    }
  }
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      float & v = at(x, y);
      if (x + 1 < width) v = std::min(v, at(x + 1, y) + 1.0f);
      if (y + 1 < height) {
        v = std::min(v, at(x, y + 1) + 1.0f);
        if (x + 1 < width) v = std::min(v, at(x + 1, y + 1) + diag);
        if (x > 0) v = std::min(v, at(x - 1, y + 1) + diag);
      }
    }
  }
  return d;
}

FakeNvblox::FakeNvblox()
: rclcpp::Node("nvblox_node") {
  base_frame_ = declare_parameter("base_frame", base_frame_);
  radius_m_ = declare_parameter("radius_m", radius_m_);
  z_m_ = declare_parameter("z_m", z_m_);
  max_distance_m_ = declare_parameter("max_distance_m", max_distance_m_);
  occupied_thresh_ = static_cast<int>(declare_parameter("occupied_thresh", occupied_thresh_));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/combined_esdf_pointcloud", rclcpp::SensorDataQoS());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { on_map(*m); });
  timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { on_timer(); });  // nvblox ESDF 갱신 ~5Hz
}

void FakeNvblox::on_map(const nav_msgs::msg::OccupancyGrid & map) {
  const int w = int(map.info.width), h = int(map.info.height);
  if (w <= 0 || h <= 0 || map.data.size() != size_t(w) * h) return;
  std::vector<bool> occ(map.data.size());
  known_.assign(map.data.size(), false);
  for (size_t i = 0; i < map.data.size(); ++i) {
    occ[i] = map.data[i] >= occupied_thresh_;
    known_[i] = map.data[i] >= 0;
  }
  const auto cells = chamfer_distance(occ, w, h);
  const float res = float(map.info.resolution);
  esdf_m_.resize(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) esdf_m_[i] = std::min(cells[i] * res, float(max_distance_m_));
  map_.header = map.header;
  map_.info = map.info;
  have_map_ = true;
  RCLCPP_INFO(get_logger(), "코스 지도 %dx%d @ %.3f m → ESDF (frame %s)", w, h, res, map.header.frame_id.c_str());
}

void FakeNvblox::on_timer() {
  if (!have_map_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/map 대기 중 — ESDF 안 냄 (map_server active인지 확인)");
    return;
  }
  const std::string & frame = map_.header.frame_id.empty() ? std::string("map") : map_.header.frame_id;
  double rx, ry;
  try {
    const auto t = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
    rx = t.transform.translation.x;
    ry = t.transform.translation.y;
  } catch (const std::exception &) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s→%s 없음 — ESDF 안 냄", frame.c_str(),
                         base_frame_.c_str());
    return;
  }
  const auto & info = map_.info;
  const double res = info.resolution, ox = info.origin.position.x, oy = info.origin.position.y;
  const int w = int(info.width), h = int(info.height);
  const int cx = int(std::floor((rx - ox) / res)), cy = int(std::floor((ry - oy) / res));
  const int rc = int(std::ceil(radius_m_ / res));
  const double r2 = radius_m_ * radius_m_;

  std::vector<float> pts;
  pts.reserve(size_t(4) * (2 * rc + 1) * (2 * rc + 1));
  for (int y = std::max(0, cy - rc); y <= std::min(h - 1, cy + rc); ++y) {
    for (int x = std::max(0, cx - rc); x <= std::min(w - 1, cx + rc); ++x) {
      const size_t i = size_t(y) * w + x;
      if (!known_[i]) continue;
      const double px = ox + (x + 0.5) * res, py = oy + (y + 0.5) * res;  // 지도 원점 회전은 없다고 봄 (map_server 코스 지도)
      if ((px - rx) * (px - rx) + (py - ry) * (py - ry) > r2) continue;
      pts.insert(pts.end(), {float(px), float(py), float(z_m_), esdf_m_[i]});
    }
  }

  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = now();
  m.header.frame_id = frame;
  m.height = 1;
  m.width = static_cast<uint32_t>(pts.size() / 4);
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (int k = 0; k < 4; ++k) {
    sensor_msgs::msg::PointField f;
    f.name = names[k];
    f.offset = uint32_t(4 * k);
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    m.fields.push_back(f);
  }
  m.is_bigendian = false;
  m.point_step = 16;
  m.row_step = m.point_step * m.width;
  m.is_dense = true;
  m.data.resize(pts.size() * sizeof(float));
  if (!pts.empty()) std::memcpy(m.data.data(), pts.data(), m.data.size());
  cloud_pub_->publish(m);
}

}  // namespace faker
