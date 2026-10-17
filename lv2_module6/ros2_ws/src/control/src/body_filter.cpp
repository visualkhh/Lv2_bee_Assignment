// body_filter 구현 — 설명·선언은 include/control/body_filter.hpp, main은 body_filter_main.cpp
#include "control/body_filter.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "urdf/model.hpp"

namespace control {

using diagnostic_msgs::msg::DiagnosticStatus;

namespace {

// package://<패키지>/<경로> → 실제 경로 (못 찾으면 빈 문자열)
std::string resolve_package(const std::string & uri) {
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0) return uri.compare(0, 7, "file://") == 0 ? uri.substr(7) : uri;
  const auto slash = uri.find('/', prefix.size());
  if (slash == std::string::npos) return "";
  try {
    return ament_index_cpp::get_package_share_path(uri.substr(prefix.size(), slash - prefix.size())).string() + uri.substr(slash);
  } catch (const std::exception &) {
    return "";
  }
}

// 바이너리 STL 경계 상자 (scale 반영) → false면 못 읽음
bool stl_bbox(const std::string & path, const urdf::Vector3 & scale, tf2::Vector3 & lo, tf2::Vector3 & hi) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  char header[80];
  uint32_t n = 0;
  if (!f.read(header, 80) || !f.read(reinterpret_cast<char *>(&n), 4) || n == 0) return false;
  const double inf = std::numeric_limits<double>::infinity();
  lo = tf2::Vector3(inf, inf, inf);
  hi = -lo;
  char rec[50];
  for (uint32_t i = 0; i < n && f.read(rec, 50); ++i) {
    for (int v = 0; v < 3; ++v) {
      float xyz[3];
      std::memcpy(xyz, rec + 12 + v * 12, 12);
      const tf2::Vector3 p(xyz[0] * scale.x, xyz[1] * scale.y, xyz[2] * scale.z);
      lo.setMin(p);
      hi.setMax(p);
    }
  }
  return lo.x() <= hi.x();
}

tf2::Transform to_tf(const urdf::Pose & p) {
  return tf2::Transform(tf2::Quaternion(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w),
                        tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

}  // namespace

std::vector<BodyBox> parse_collision_boxes(const std::string & urdf_xml) {
  std::vector<BodyBox> out;
  urdf::Model model;
  if (!model.initString(urdf_xml)) return out;
  for (const auto & [name, link] : model.links_) {
    for (const auto & col : link->collision_array) {
      if (!col || !col->geometry) continue;
      BodyBox b{name, to_tf(col->origin), {}};
      const auto & g = col->geometry;
      if (g->type == urdf::Geometry::BOX) {
        const auto & d = std::static_pointer_cast<urdf::Box>(g)->dim;
        b.half = tf2::Vector3(d.x / 2, d.y / 2, d.z / 2);
      } else if (g->type == urdf::Geometry::CYLINDER) {
        const auto c = std::static_pointer_cast<urdf::Cylinder>(g);
        b.half = tf2::Vector3(c->radius, c->radius, c->length / 2);
      } else if (g->type == urdf::Geometry::SPHERE) {
        const double r = std::static_pointer_cast<urdf::Sphere>(g)->radius;
        b.half = tf2::Vector3(r, r, r);
      } else if (g->type == urdf::Geometry::MESH) {
        const auto m = std::static_pointer_cast<urdf::Mesh>(g);
        tf2::Vector3 lo, hi;
        if (!stl_bbox(resolve_package(m->filename), m->scale, lo, hi)) continue;
        b.origin = b.origin * tf2::Transform(tf2::Quaternion::getIdentity(), (lo + hi) / 2);
        b.half = (hi - lo) / 2;
      } else {
        continue;
      }
      out.push_back(b);
    }
  }
  return out;
}

BodyFilter::BodyFilter(const rclcpp::NodeOptions & options)
: rclcpp::Node("body_filter", options) {
  const auto in = declare_parameter<std::string>("input_topic", "/pole_cam/depth/color/points");
  const auto out = declare_parameter<std::string>("output_topic", "/pole_cam/depth/color/points_filtered");
  padding_ = declare_parameter<double>("padding", 0.03);
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  urdf_sub_ = create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      boxes_ = parse_collision_boxes(m->data);
      RCLCPP_INFO(get_logger(), "자기 몸 충돌 상자 %zu개 (URDF <collision>)", boxes_.size());
    });
  pub_ = create_publisher<PointCloud2>(out, rclcpp::SensorDataQoS());
  cloud_sub_ = create_subscription<PointCloud2>(in, rclcpp::SensorDataQoS(),
    [this](PointCloud2::ConstSharedPtr m) { on_cloud(*m); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (boxes_.empty()) s.summary(DiagnosticStatus::WARN, "URDF 대기 — 거르지 않고 그대로 통과");
    else if (!last_error_.empty()) s.summary(DiagnosticStatus::WARN, "TF 없음 — " + last_error_);
    else s.summary(DiagnosticStatus::OK, "자기 몸 점 제거 중");
    s.add("충돌 상자", boxes_.size());
    s.add("처리한 점구름", clouds_);
    s.add("뺀 점 (누적)", removed_);
  });
  RCLCPP_INFO(get_logger(), "%s → %s (padding %.2fm)", in.c_str(), out.c_str(), padding_);
}

void BodyFilter::on_cloud(const PointCloud2 & in) {
  ++clouds_;
  int ox = -1, oy = -1, oz = -1;
  for (const auto & f : in.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
    if (f.name == "x") ox = static_cast<int>(f.offset);
    if (f.name == "y") oy = static_cast<int>(f.offset);
    if (f.name == "z") oz = static_cast<int>(f.offset);
  }
  if (boxes_.empty() || ox < 0 || oy < 0 || oz < 0) {
    pub_->publish(in);  // URDF 전이거나 xyz가 없으면 그대로 (Nav2가 센서를 잃지 않게)
    return;
  }
  // 상자마다 카메라 → 상자 좌표 변환 (촬영 시각 TF, 없으면 최신)
  std::vector<tf2::Transform> to_box;
  std::vector<tf2::Vector3> half;
  to_box.reserve(boxes_.size());
  last_error_.clear();
  for (const auto & b : boxes_) {
    geometry_msgs::msg::TransformStamped ts;
    try {
      ts = tf_->lookupTransform(in.header.frame_id, b.link, in.header.stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException &) {
      try {
        ts = tf_->lookupTransform(in.header.frame_id, b.link, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        last_error_ = e.what();
        continue;
      }
    }
    tf2::Transform cam_link;
    tf2::fromMsg(ts.transform, cam_link);
    to_box.push_back((cam_link * b.origin).inverse());
    half.push_back(b.half + tf2::Vector3(padding_, padding_, padding_));
  }
  PointCloud2 out;
  out.header = in.header;
  out.height = 1;
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.is_dense = in.is_dense;
  out.data.reserve(in.data.size());
  const size_t n = static_cast<size_t>(in.width) * in.height;
  size_t kept = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = &in.data[i * in.point_step];
    float x, y, z;
    std::memcpy(&x, p + ox, 4);
    std::memcpy(&y, p + oy, 4);
    std::memcpy(&z, p + oz, 4);
    const tf2::Vector3 pt(x, y, z);
    bool body = false;
    for (size_t k = 0; k < to_box.size() && !body; ++k) {
      const tf2::Vector3 q = to_box[k] * pt;
      body = std::abs(q.x()) <= half[k].x() && std::abs(q.y()) <= half[k].y() && std::abs(q.z()) <= half[k].z();
    }
    if (body) continue;
    out.data.insert(out.data.end(), p, p + in.point_step);
    ++kept;
  }
  removed_ += n - kept;
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.point_step * out.width;
  pub_->publish(out);
}

}  // namespace control
