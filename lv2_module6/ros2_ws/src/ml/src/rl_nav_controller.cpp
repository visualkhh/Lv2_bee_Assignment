// RlNavController — Nav2 controller 플러그인. 학습 정책(ONNX)으로 경로를 따라가고,
//   정책이 없거나 끄면(기본) 안에 품은 기본 controller(DWB 등)로 그대로 넘긴다.
//
// 관측 51 (학습 env와 같은 순서·단위 — 바꾸면 학습도 같이):
//   [웨이포인트 K=5 × (x, y)/wp_scale] (10)
//   [가상 스캔 R=36 방향 거리/lidar_max] (36)
//   [v/max_lin, w/max_ang, gyro/max_ang, 이전 행동 a0, a1] (5)
//   → 합 51
// 가상 스캔: LiDAR가 없어서 /pole_front_cam/depth/color/points_filtered (PointCloud2)를
//   base_footprint로 변환, 높이 z_min~z_max 점만 36방향 빈에 최소거리. 점구름이 없으면 기본 controller로 바이패스.
//   (scan_source=scan 이면 /scan LaserScan 사용 — 실제 LiDAR 붙였을 때)
// 행동 [-1,1]² → v = a0 × max_lin (후진은 v_min_ratio까지) · w = a1 × max_ang
#include "ml/rl_nav_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ml {

template<typename T>
static T param(const nav2::LifecycleNode::SharedPtr & node, const std::string & name, const T & def) {
  nav2::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(def));
  T v = def;
  node->get_parameter(name, v);
  return v;
}

void RlNavController::configure(
  const nav2::LifecycleNode::WeakPtr & parent, std::string name,
  nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  logger_ = node->get_logger();

  const auto inner = param<std::string>(node, name + ".inner_plugin", "dwb_core::DWBLocalPlanner");
  const auto model = param<std::string>(node, name + ".model_path", "nav_controller/policy.onnx");
  enable_ = param<bool>(node, name + ".enable", true);
  const auto scan_source = param<std::string>(node, name + ".scan_source", "points");
  const auto points_topic = param<std::string>(
    node, name + ".points_topic", "/pole_front_cam/depth/color/points_filtered");
  const auto scan_topic = param<std::string>(node, name + ".scan_topic", "scan");
  const auto imu_topic = param<std::string>(node, name + ".imu_topic", "imu");
  lookahead_ = param<int>(node, name + ".lookahead", lookahead_);
  wp_step_ = param<double>(node, name + ".wp_step", wp_step_);
  wp_scale_ = param<double>(node, name + ".wp_scale", wp_scale_);
  lidar_rays_ = param<int>(node, name + ".lidar_rays", lidar_rays_);
  lidar_max_ = param<double>(node, name + ".lidar_max", lidar_max_);
  max_lin_ = param<double>(node, name + ".max_lin", max_lin_);
  max_ang_ = param<double>(node, name + ".max_ang", max_ang_);
  v_min_ratio_ = param<double>(node, name + ".v_min_ratio", v_min_ratio_);
  extra_lin_vel_ = param<double>(node, name + ".extra_lin_vel", extra_lin_vel_);
  extra_ang_vel_ = param<double>(node, name + ".extra_ang_vel", extra_ang_vel_);
  points_stride_ = param<int>(node, name + ".points_stride", points_stride_);
  z_min_ = param<double>(node, name + ".z_min", z_min_);
  z_max_ = param<double>(node, name + ".z_max", z_max_);
  cloud_timeout_ = param<double>(node, name + ".cloud_timeout", cloud_timeout_);
  ranges_.assign(lidar_rays_, float(lidar_max_));

  inner_ = loader_.createSharedInstance(inner);
  inner_->configure(parent, name + ".inner", tf, costmap_ros);
  load_model(model);
  use_scan_ = (scan_source == "scan");
  if (enable_ && !policy_.loaded()) {
    RCLCPP_WARN(logger_, "[%s] enable인데 정책 없음 — 바이패스(%s)", name_.c_str(), inner.c_str());
    enable_ = false;
  }

  if (use_scan_) {
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        scan_ = m;
      }, nav2::qos::SensorDataQoS());
  } else {
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic, [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { on_cloud(m); },
      nav2::qos::SensorDataQoS());
  }
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, [this](sensor_msgs::msg::Imu::SharedPtr m) {
      std::lock_guard<std::mutex> lk(mtx_);
      gyro_z_ = m->angular_velocity.z;
      gyro_seen_ = true;
    }, nav2::qos::SensorDataQoS());
  debug_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("~/" + name + "/debug", 10);
  toggle_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "~/" + name + "/toggle",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      std::lock_guard<std::mutex> lk(mtx_);
      if (req->data && !policy_.loaded()) {
        res->success = false;
        res->message = "정책 없음 (model_path) — 바이패스 유지";
        return;
      }
      if (enable_ && !req->data) inner_->reset();
      enable_ = req->data;
      prev_a_[0] = prev_a_[1] = 0.0f;
      res->success = true;
      res->message = std::string("enable=") + (enable_ ? "true (정책)" : "false (바이패스)") +
                     " · model=" + (policy_.loaded() ? "loaded" : "none");
      RCLCPP_INFO(logger_, "[%s] toggle: %s", name_.c_str(), res->message.c_str());
    });
  RCLCPP_INFO(logger_, "[%s] RlNavController: inner=%s · model=%s · enable=%s · scan=%s · toggle=~/%s/toggle",
              name_.c_str(), inner.c_str(), policy_.loaded() ? model.c_str() : "(없음)",
              enable_ ? "true" : "false", use_scan_ ? "scan" : points_topic.c_str(), name_.c_str());
}

void RlNavController::load_model(const std::string & path) {
  if (path.empty()) return;
  std::string resolved = path;
  namespace fs = std::filesystem;
  const fs::path p(path);
  if (!p.is_absolute()) {  // 상대 경로: share/ml/model/<경로> 하나만 본다.
    //   "nav_controller/policy.onnx"처럼 노드별 폴더 포함 — 노드마다 onnx를 가져와도 이름이 안 겹친다.
    try {
      const fs::path share = fs::path(ament_index_cpp::get_package_share_directory("ml")) /
                             "model" / p;
      if (fs::exists(share)) resolved = share.string();
    } catch (const std::exception &) {
    }
  }
  try {
    OnnxPolicy m;
    m.load(resolved);
    const int64_t want = 2 * lookahead_ + lidar_rays_ + 5;
    if (m.inputs() != want || m.outputs() < 2) {
      RCLCPP_ERROR(logger_, "정책 입력 %ld · 출력 %ld — 기대 %ld · 2 (lookahead·lidar_rays 확인): %s",
                   long(m.inputs()), long(m.outputs()), long(want), resolved.c_str());
      return;
    }
    policy_ = std::move(m);
    RCLCPP_INFO(logger_, "ONNX loaded: %s (obs %ld → %ld)", resolved.c_str(),
                long(m.inputs()), long(m.outputs()));
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "%s — 바이패스", e.what());
  }
}

void RlNavController::on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  if (cloud->width * cloud->height == 0) return;
  // 카메라 → base_footprint (고정 마운트라 최신 TF로 충분)
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(costmap_ros_->getBaseFrameID(), cloud->header.frame_id, tf2::TimePointZero);
  } catch (const std::exception &) {
    ++cloud_tf_fail_;
    return;
  }
  const double qx = tf.transform.rotation.x, qy = tf.transform.rotation.y;
  const double qz = tf.transform.rotation.z, qw = tf.transform.rotation.w;
  const double r00 = 1 - 2 * (qy * qy + qz * qz), r01 = 2 * (qx * qy - qz * qw), r02 = 2 * (qx * qz + qy * qw);
  const double r10 = 2 * (qx * qy + qz * qw), r11 = 1 - 2 * (qx * qx + qz * qz), r12 = 2 * (qy * qz - qx * qw);
  const double r20 = 2 * (qx * qz - qy * qw), r21 = 2 * (qy * qz + qx * qw), r22 = 1 - 2 * (qx * qx + qy * qy);
  const double tx = tf.transform.translation.x, ty = tf.transform.translation.y, tz = tf.transform.translation.z;

  std::vector<float> best(lidar_rays_, float(lidar_max_));
  const int stride = std::max(1, points_stride_);
  const size_t total = size_t(cloud->width) * cloud->height;
  try {
    sensor_msgs::PointCloud2ConstIterator<float> ix(*cloud, "x"), iy(*cloud, "y"), iz(*cloud, "z");
    size_t n = 0;
    for (size_t i = 0; i < total && n < 200000; ++i, ++ix, ++iy, ++iz, ++n) {
      if ((i % stride) != 0) continue;
      const double px = *ix, py = *iy, pz = *iz;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
      const double bx = r00 * px + r01 * py + r02 * pz + tx;
      const double by = r10 * px + r11 * py + r12 * pz + ty;
      const double bz = r20 * px + r21 * py + r22 * pz + tz;
      if (bz < z_min_ || bz > z_max_) continue;  // 바닥·천장 제외
      const double r = std::hypot(bx, by);
      if (r > lidar_max_ || r < 0.05) continue;
      int bin = int(std::floor((std::atan2(by, bx) + M_PI) / (2 * M_PI) * lidar_rays_)) % lidar_rays_;
      if (bin < 0) bin += lidar_rays_;
      if (r < best[bin]) best[bin] = float(r);
    }
  } catch (const std::exception &) {
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  ranges_.swap(best);
  cloud_stamp_ = node_.lock()->now();
  cloud_ok_ = true;
}

std::vector<float> RlNavController::waypoints(
  const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & plan) const {
  std::vector<float> out(2 * lookahead_, 0.0f);
  const auto & ps = plan.poses;
  if (ps.empty()) return out;
  const double px = pose.pose.position.x, py = pose.pose.position.y, yaw = tf2::getYaw(pose.pose.orientation);
  size_t near = 0;
  double best = 1e18;
  for (size_t i = 0; i < ps.size(); ++i) {
    const double d = std::hypot(ps[i].pose.position.x - px, ps[i].pose.position.y - py);
    if (d < best) {
      best = d;
      near = i;
    }
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  double acc = 0.0, want = wp_step_;
  size_t i = near;
  int k = 0;
  auto emit = [&](double x, double y) {
    const double dx = x - px, dy = y - py;
    out[2 * k] = float((c * dx + s * dy) / wp_scale_);
    out[2 * k + 1] = float((-s * dx + c * dy) / wp_scale_);
    ++k;
  };
  while (k < lookahead_ && i + 1 < ps.size()) {
    const auto & a = ps[i].pose.position, & b = ps[i + 1].pose.position;
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    while (k < lookahead_ && acc + seg >= want) {
      const double u = seg > 1e-9 ? (want - acc) / seg : 0.0;
      emit(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y));
      want += wp_step_;
    }
    acc += seg;
    ++i;
  }
  while (k < lookahead_) emit(ps.back().pose.position.x, ps.back().pose.position.y);
  return out;
}

std::vector<float> RlNavController::scan_rays(const sensor_msgs::msg::LaserScan & scan, double laser_yaw) const {
  std::vector<float> out(lidar_rays_, 1.0f);
  const int n = int(scan.ranges.size());
  if (n == 0 || scan.angle_increment == 0.0f) return out;
  for (int r = 0; r < lidar_rays_; ++r) {
    double a = 2.0 * M_PI * r / lidar_rays_ - laser_yaw - scan.angle_min;
    a = std::fmod(std::fmod(a, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
    const int idx = int(std::lround(a / scan.angle_increment));
    double best = lidar_max_;
    for (int j = idx - 1; j <= idx + 1; ++j) {
      const float v = scan.ranges[((j % n) + n) % n];
      if (std::isfinite(v) && v >= scan.range_min && v <= scan.range_max) best = std::min(best, double(v));
    }
    out[r] = float(std::clamp(best, 0.0, lidar_max_) / lidar_max_);
  }
  return out;
}

geometry_msgs::msg::TwistStamped RlNavController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
  const geometry_msgs::msg::PoseStamped & global_goal) {
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  std::vector<float> ranges;
  bool on = false, cloud_ok = false;
  double gyro = 0.0, cloud_age = 0.0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    on = enable_ && policy_.loaded();
    scan = scan_;
    ranges = ranges_;
    cloud_ok = cloud_ok_;
    gyro = gyro_seen_ ? gyro_z_ : velocity.angular.z;
    if (cloud_stamp_.nanoseconds() != 0) cloud_age = (node_.lock()->now() - cloud_stamp_).seconds();
  }
  const bool sensor_ok = use_scan_ ? bool(scan) : (cloud_ok && cloud_age <= cloud_timeout_);
  if (!on || !sensor_ok) {
    if (on) {
      RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000,
                           "[%s] %s 없음 — 바이패스", name_.c_str(), use_scan_ ? "/scan" : "점구름");
    }
    return inner_->computeVelocityCommands(pose, velocity, goal_checker, transformed_global_plan, global_goal);
  }
  std::vector<float> obs = waypoints(pose, transformed_global_plan);
  if (use_scan_) {
    if (!laser_yaw_known_) {
      try {
        const auto t = tf_->lookupTransform(costmap_ros_->getBaseFrameID(), scan->header.frame_id, tf2::TimePointZero);
        laser_yaw_ = tf2::getYaw(t.transform.rotation);
        laser_yaw_known_ = true;
      } catch (const std::exception &) {
      }
    }
    const auto rays = scan_rays(*scan, laser_yaw_);
    obs.insert(obs.end(), rays.begin(), rays.end());
  } else {
    for (float r : ranges) obs.push_back(std::clamp(r, 0.0f, float(lidar_max_)) / float(lidar_max_));
  }
  obs.push_back(float(velocity.linear.x / max_lin_));
  obs.push_back(float(velocity.angular.z / max_ang_));
  obs.push_back(float(gyro / max_ang_));
  obs.push_back(prev_a_[0]);
  obs.push_back(prev_a_[1]);

  std::vector<float> a;
  try {
    a = policy_.run(obs);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 5000, "정책 추론 실패: %s — 바이패스", e.what());
    return inner_->computeVelocityCommands(pose, velocity, goal_checker, transformed_global_plan, global_goal);
  }
  float scan_min = float(lidar_max_);
  for (size_t i = 0; i < ranges.size(); ++i) scan_min = std::min(scan_min, ranges[i]);
  const float a0 = std::clamp(a[0], -1.0f, 1.0f), a1 = std::clamp(a[1], -1.0f, 1.0f);
  prev_a_[0] = a0;
  prev_a_[1] = a1;

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = pose.header.stamp;
  cmd.twist.linear.x = std::clamp(std::max(double(a0) * max_lin_, v_min_ratio_ * max_lin_) * speed_ratio_ + extra_lin_vel_,
                                   v_min_ratio_ * max_lin_, max_lin_);
  cmd.twist.angular.z = std::clamp(double(a1) * max_ang_ + extra_ang_vel_, -max_ang_, max_ang_);
  if (debug_pub_) {  // A/B 분석용: [a0, a1, v_out, w_out, scan_min, v_odom, w_odom, on]
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {a0, a1, static_cast<float>(cmd.twist.linear.x), static_cast<float>(cmd.twist.angular.z),
                scan_min, static_cast<float>(velocity.linear.x), static_cast<float>(velocity.angular.z), 1.0f};
    debug_pub_->publish(dbg);
  }
  return cmd;
}

void RlNavController::cleanup() {
  inner_->cleanup();
  cloud_sub_.reset();
  scan_sub_.reset();
  imu_sub_.reset();
  toggle_srv_.reset();
}
void RlNavController::activate() { inner_->activate(); }
void RlNavController::deactivate() { inner_->deactivate(); }
void RlNavController::newPathReceived(const nav_msgs::msg::Path & raw) { inner_->newPathReceived(raw); }
bool RlNavController::cancel() { return inner_->cancel(); }
void RlNavController::reset() {
  inner_->reset();
  prev_a_[0] = prev_a_[1] = 0.0f;
}
void RlNavController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  inner_->setSpeedLimit(speed_limit, percentage);
  speed_ratio_ = percentage ? speed_limit / 100.0 : speed_limit / max_lin_;
  speed_ratio_ = (speed_limit <= 0.0) ? 1.0 : std::clamp(speed_ratio_, 0.0, 1.0);
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlNavController, nav2_core::Controller)
