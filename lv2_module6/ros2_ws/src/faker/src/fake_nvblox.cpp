// fake_nvblox 구현 — 설명은 include/faker/fake_nvblox.hpp
#include "faker/fake_nvblox.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>

#include "tf2/LinearMath/Transform.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace faker {

namespace {

// 테스트베드 지도 격자 — map 프레임 원점(출발 칸) 기준 16m×16m (-8..8). 두 코스를 다 덮는다:
//   isaac_maze ≈ x -2.7..3.2 · y -6.1..1.4 (예전 12m·-3 시작이면 y<-3이 잘려 미로 아래쪽 벽이 지도에 없었음) · dice_maze ≈ x -1..4 · y -1.2..5.6
constexpr char kMagic[9] = {'F', 'A', 'K', 'E', 'S', 'L', 'A', 'M', '1'};
constexpr int kMapW = 320;
constexpr int kMapH = 320;
constexpr double kMapRes = 0.05;
constexpr double kMapOriginX = -8.0;
constexpr double kMapOriginY = -8.0;

}  // namespace

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
  map_dir_ = declare_parameter("map_dir", map_dir_);
  map_name_ = declare_parameter("map_name", map_name_);
  map_mode_ = declare_parameter("map_mode", map_mode_);
  mapping_ = map_mode_ == "MAPPING";

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/combined_esdf_pointcloud", rclcpp::SensorDataQoS());
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local());

  if (mapping_) {
    init_grid();
    // 코스 지도가 아니라 카메라가 실제로 본 점구름으로 지도를 쌓는다 (fake_camera depth → points).
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pole_front_cam/depth/color/points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
    RCLCPP_INFO(get_logger(), "가짜 매핑 — point cloud → /map · %s/%s/nvblox.nvblx",
                map_dir_.c_str(), map_name_.c_str());
  } else if (map_mode_ == "LOCALIZATION" && load_map_file()) {
    RCLCPP_INFO(get_logger(), "저장 지도 %s/%s/nvblox.nvblx → /map (%dx%d @ %.3f m)",
                map_dir_.c_str(), map_name_.c_str(), int(map_.info.width), int(map_.info.height),
                map_.info.resolution);
    save_preview_files();  // 지도 폴더에 map.pgm·map.yaml도 있게
  } else {
    // 저장 지도가 없거나 형식이 깨짐 — 빈 지도로라도 스택은 뜨게 한다 (setup_map이 nvblox.nvblx를 확인하지만 방어)
    init_grid();
    RCLCPP_ERROR(get_logger(), "저장 지도 없음/형식 오류 — 빈 /map으로 시작: %s/%s/nvblox.nvblx",
                 map_dir_.c_str(), map_name_.c_str());
  }

  timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { on_timer(); });  // ESDF ~5Hz
}

FakeNvblox::~FakeNvblox() {
  if (mapping_) {
    save_map_file();       // 마지막 정본 — 2초 주기 저장의 마지막 틈을 메운다
    save_preview_files();  // map.pgm · map.yaml
  }
}

void FakeNvblox::init_grid() {
  map_.header.frame_id = "map";
  map_.info.resolution = kMapRes;
  map_.info.width = kMapW;
  map_.info.height = kMapH;
  map_.info.origin.position.x = kMapOriginX;
  map_.info.origin.position.y = kMapOriginY;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.assign(size_t(kMapW) * kMapH, -1);
  known_.assign(size_t(kMapW) * kMapH, false);
  have_map_ = true;
}

int FakeNvblox::cell_index(double x, double y) const {
  const double res = map_.info.resolution;
  if (res <= 0.0) return -1;
  const int w = int(map_.info.width), h = int(map_.info.height);
  const int col = int(std::floor((x - map_.info.origin.position.x) / res));
  const int row = int(std::floor((y - map_.info.origin.position.y) / res));
  if (col < 0 || col >= w || row < 0 || row >= h) return -1;
  return row * w + col;
}

void FakeNvblox::mark_free(double x, double y) {
  const int i = cell_index(x, y);
  if (i >= 0 && map_.data[i] < 0) map_.data[i] = 0;  // 장애물은 안 지운다
}

void FakeNvblox::mark_occupied(double x, double y) {
  const int i = cell_index(x, y);
  if (i >= 0) map_.data[i] = 100;
}

void FakeNvblox::on_cloud(const sensor_msgs::msg::PointCloud2 & cloud) {
  if (!mapping_ || cloud.width == 0 || cloud.height == 0) return;
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("map", cloud.header.frame_id, cloud.header.stamp,
                                     tf2::durationFromSec(0.1));
  } catch (const std::exception &) {
    try {
      tf = tf_buffer_->lookupTransform("map", cloud.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &) {
      return;  // 아직 map→camera TF 없음
    }
  }
  tf2::Transform T;
  tf2::fromMsg(tf.transform, T);
  const tf2::Vector3 cam = T.getOrigin();

  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
  size_t counter = 0;
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    if ((counter++ & 1) != 0) continue;  // 절반만 — 5Hz 19k 점을 다 쌓을 필요는 없다
    const float x = *ix, y = *iy, z = *iz;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0f) continue;
    const tf2::Vector3 q = T * tf2::Vector3(x, y, z);
    if (q.z() < 0.06 || q.z() > 0.9) continue;  // 바닥·천장 제외 (벽/장애물 점만)
    const double dx = q.x() - cam.x(), dy = q.y() - cam.y();
    const double d = std::hypot(dx, dy);
    if (d < 0.30 || d > 8.0) continue;  // 자기 몸·너무 먼 점 제외
    const int steps = std::max(1, std::min(80, int(d / (kMapRes * 2.0))));
    for (int s = 1; s < steps; ++s) {  // 광선 사이 = 빈 칸
      const double t = double(s) / double(steps);
      mark_free(cam.x() + dx * t, cam.y() + dy * t);
    }
    mark_occupied(q.x(), q.y());
    map_dirty_ = true;
  }
}

void FakeNvblox::rebuild_esdf() {
  const int w = int(map_.info.width), h = int(map_.info.height);
  if (w <= 0 || h <= 0 || map_.data.size() != size_t(w) * h) return;
  std::vector<bool> occ(map_.data.size());
  known_.assign(map_.data.size(), false);
  for (size_t i = 0; i < map_.data.size(); ++i) {
    occ[i] = map_.data[i] >= occupied_thresh_;
    known_[i] = map_.data[i] >= 0;
  }
  const auto cells = chamfer_distance(occ, w, h);
  const float res = float(map_.info.resolution);
  esdf_m_.resize(cells.size());
  for (size_t i = 0; i < cells.size(); ++i) esdf_m_[i] = std::min(cells[i] * res, float(max_distance_m_));
  have_map_ = true;
}

void FakeNvblox::publish_map() {
  map_.header.stamp = now();
  map_.header.frame_id = "map";
  map_pub_->publish(map_);
}

void FakeNvblox::on_timer() {
  const double t = now().seconds();
  if (mapping_) {
    if (map_dirty_) {
      rebuild_esdf();
      map_dirty_ = false;
      if (t - last_map_pub_ >= 1.0) {
        publish_map();
        last_map_pub_ = t;
      }
      if (t - last_save_ >= 2.0) {
        save_map_file();
        save_preview_files();
        last_save_ = t;
      }
    }
  } else if (!map_published_) {
    publish_map();
    map_published_ = true;
  }
  if (have_map_) publish_esdf();
}

bool FakeNvblox::save_map_file() const {
  if (map_dir_.empty() || map_name_.empty()) return false;
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::path(map_dir_) / map_name_;
  fs::create_directories(dir, ec);
  const fs::path file = dir / "nvblox.nvblx";
  std::ofstream out(file, std::ios::binary | std::ios::trunc);
  if (!out) {
    RCLCPP_WARN(get_logger(), "가짜 매핑 — 지도 파일을 못 씀: %s", file.c_str());
    return false;
  }
  out.write(kMagic, sizeof(kMagic));
  const int32_t w = int32_t(map_.info.width), h = int32_t(map_.info.height);
  const double res = map_.info.resolution;
  const double ox = map_.info.origin.position.x, oy = map_.info.origin.position.y;
  out.write(reinterpret_cast<const char *>(&w), sizeof(w));
  out.write(reinterpret_cast<const char *>(&h), sizeof(h));
  out.write(reinterpret_cast<const char *>(&res), sizeof(res));
  out.write(reinterpret_cast<const char *>(&ox), sizeof(ox));
  out.write(reinterpret_cast<const char *>(&oy), sizeof(oy));
  out.write(reinterpret_cast<const char *>(map_.data.data()), std::streamsize(map_.data.size()));
  return bool(out);
}

bool FakeNvblox::save_preview_files() const {
  if (map_dir_.empty() || map_name_.empty()) return false;
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::path(map_dir_) / map_name_;
  fs::create_directories(dir, ec);
  const int w = int(map_.info.width), h = int(map_.info.height);
  if (w <= 0 || h <= 0 || map_.data.size() != size_t(w) * size_t(h)) return false;

  const fs::path pgm_path = dir / "map.pgm";
  std::ofstream pgm(pgm_path, std::ios::binary | std::ios::trunc);
  if (!pgm) {
    RCLCPP_WARN(get_logger(), "지도 미리보기 PGM을 못 씀: %s", pgm_path.c_str());
    return false;
  }
  pgm << "P5\n" << w << " " << h << "\n255\n";
  std::vector<uint8_t> row(size_t(w), 0);
  for (int r = h - 1; r >= 0; --r) {  // PGM 0행 = 큰 y (OccupancyGrid는 아래가 0행)
    for (int c = 0; c < w; ++c) {
      const int8_t v = map_.data[size_t(r) * w + c];
      row[size_t(c)] = uint8_t(v < 0 ? 205 : (v >= occupied_thresh_ ? 0 : 254));  // trinary: unknown 205 · free 254 · occupied 0
    }
    pgm.write(reinterpret_cast<const char *>(row.data()), std::streamsize(row.size()));
  }

  const fs::path yaml_path = dir / "map.yaml";
  std::ofstream yaml(yaml_path, std::ios::trunc);
  if (!yaml) {
    RCLCPP_WARN(get_logger(), "지도 미리보기 YAML을 못 씀: %s", yaml_path.c_str());
    return false;
  }
  yaml << "image: map.pgm\n"
       << "resolution: " << map_.info.resolution << "\n"
       << "origin: [" << map_.info.origin.position.x << ", " << map_.info.origin.position.y << ", 0.0]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.25\n";
  return bool(pgm) && bool(yaml);
}

bool FakeNvblox::load_map_file() {
  if (map_dir_.empty() || map_name_.empty()) return false;
  namespace fs = std::filesystem;
  const fs::path file = fs::path(map_dir_) / map_name_ / "nvblox.nvblx";
  std::ifstream in(file, std::ios::binary);
  if (!in) return false;
  char magic[sizeof(kMagic)]{};
  in.read(magic, sizeof(magic));
  if (!in || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    RCLCPP_WARN(get_logger(), "저장 지도 형식이 아님 (옛 자리 파일?) — /map 대기: %s", file.c_str());
    return false;
  }
  int32_t w = 0, h = 0;
  double res = 0.0, ox = 0.0, oy = 0.0;
  in.read(reinterpret_cast<char *>(&w), sizeof(w));
  in.read(reinterpret_cast<char *>(&h), sizeof(h));
  in.read(reinterpret_cast<char *>(&res), sizeof(res));
  in.read(reinterpret_cast<char *>(&ox), sizeof(ox));
  in.read(reinterpret_cast<char *>(&oy), sizeof(oy));
  if (!in || w <= 0 || h <= 0 || res <= 0.0) return false;
  map_.header.frame_id = "map";
  map_.info.resolution = res;
  map_.info.width = uint32_t(w);
  map_.info.height = uint32_t(h);
  map_.info.origin.position.x = ox;
  map_.info.origin.position.y = oy;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;
  map_.data.resize(size_t(w) * size_t(h));
  in.read(reinterpret_cast<char *>(map_.data.data()), std::streamsize(map_.data.size()));
  if (!in) return false;
  have_map_ = true;
  rebuild_esdf();
  return true;
}

void FakeNvblox::publish_esdf() {
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
      if (i >= known_.size() || !known_[i]) continue;
      const double px = ox + (x + 0.5) * res, py = oy + (y + 0.5) * res;
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
