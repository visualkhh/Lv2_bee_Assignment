// RlNavController — Nav2 controller 플러그인. 학습 정책(ONNX)으로 경로를 따라가고,
//   정책이 없거나 끄면(기본) 안에 품은 기본 controller(DWB 등)로 그대로 넘긴다.
//
// 관측 (학습 env와 같은 순서·단위 — 바꾸면 학습도 같이):
//   [웨이포인트 K=5 × (x, y)/wp_scale] (10)
//   [가상 스캔 R=lidar_rays 방향 거리/lidar_max] (기본 360)
//   [v/max_lin, w/max_ang, gyro/max_ang, 이전 행동 a0, a1] (5)
//   → 합 2*K + R + 5
// 가상 스캔: LiDAR가 없어서 nvblox 2D ESDF height slice(PointCloud2, 0.03~0.30 m)를
//   base_footprint로 변환, 그 점들을 R방향 빈의 최소거리로. 점구름이 없으면 기본 controller로 바이패스.
//   (scan_source=scan 이면 /scan LaserScan 사용 — 실제 LiDAR 붙였을 때)
// 행동 [-1,1]² → v = a0 × max_lin (후진은 v_min_ratio까지) · w = a1 × max_ang
#include "ml/rl_nav_controller.hpp"
#include "ml/nav_obs.hpp"  // heading_speed_cap

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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

  // 바이패스(inner) controller — fallback_controller: dwb | mppi | rpp | rotation_shim (또는 전체 pluginlib 클래스명)
  //   각 inner 의 파라미터 공간은 <name>.inner_<별칭> (include/ml/fallback_controller.hpp) — nav2_params.yaml 에 블록별로 둔다.
  //   실행 중에는 서비스 ~/<name>/set_fallback 으로 바꾼다 (switch_inner).
  std::string want = param<std::string>(node, name + ".fallback_controller", "");
  if (want.empty()) want = param<std::string>(node, name + ".inner_plugin", "mppi");  // 예전 이름 호환 · 둘 다 없으면 MPPI
  Fallback fb;
  resolve_fallback(want, fb);
  RCLCPP_INFO(logger_, "[%s] fallback_controller=%s → %s (파라미터 %s.*)", name_.c_str(), want.c_str(), fb.cls.c_str(),
              inner_namespace(name, fb).c_str());
  inner_kind_ = fb.kind;
  inner_key_ = fb.key;
  const std::string inner = fb.cls;
  const auto model = param<std::string>(node, name + ".model_path", "rl_nav_controller/policy.onnx");
  enable_requested_ = param<bool>(node, name + ".enable", true);
  enable_ = enable_requested_;
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
  lidar_inflation_ = std::max(0.0, param<double>(node, name + ".lidar_inflation", lidar_inflation_));
  max_lin_ = param<double>(node, name + ".max_lin", max_lin_);
  max_ang_ = param<double>(node, name + ".max_ang", max_ang_);
  v_min_ratio_ = param<double>(node, name + ".v_min_ratio", v_min_ratio_);
  extra_lin_vel_ = param<double>(node, name + ".extra_lin_vel", extra_lin_vel_);
  heading_cap_ = param<bool>(node, name + ".heading_speed_cap", heading_cap_);  // 경로와 어긋나면 전진 줄임 (ml::heading_speed_cap)
  extra_ang_vel_ = param<double>(node, name + ".extra_ang_vel", extra_ang_vel_);
  points_stride_ = param<int>(node, name + ".points_stride", points_stride_);
  z_min_ = param<double>(node, name + ".z_min", z_min_);
  z_max_ = param<double>(node, name + ".z_max", z_max_);
  esdf_max_distance_ = param<double>(node, name + ".esdf_max_distance", esdf_max_distance_);
  cloud_timeout_ = param<double>(node, name + ".cloud_timeout", cloud_timeout_);
  ranges_.assign(lidar_rays_, float(lidar_max_));

  inner_ = loader_.createSharedInstance(fb.cls);
  inner_->configure(parent, inner_namespace(name, fb), tf, costmap_ros);
  load_model(model);
  // CLI/파라미터 페이지에서 FollowPath.enable을 바꾸면 toggle 서비스 없이도 즉시 적용한다.
  param_cb_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      for (const auto & p : params) {
        if (p.get_name() != name_ + ".enable") continue;
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          r.successful = false;
          r.reason = name_ + ".enable은 bool이어야 함";
          continue;
        }
        const bool on = p.as_bool();
        std::lock_guard<std::mutex> lk(mtx_);
        if (on && !policy_.loaded()) {
          r.successful = false;
          r.reason = "정책 없음 (model_path) — 바이패스 유지";
          continue;
        }
        enable_requested_ = on;
        if (enable_ && !on) {
          std::lock_guard<std::mutex> il(inner_mtx_);
          inner_->reset();
        }
        enable_ = on;
        prev_a_[0] = prev_a_[1] = 0.0f;
        RCLCPP_INFO(logger_, "[%s] %s.enable=%s (파라미터 즉시 적용)",
                    name_.c_str(), name_.c_str(), on ? "true" : "false");
      }
      return r;
    });
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
  status_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("~/" + name + "/status", 10);
  diag_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = node->create_wall_timer(std::chrono::seconds(1), [this]() { publish_diagnostics(); });
  toggle_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "~/" + name + "/toggle",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      std::lock_guard<std::mutex> lk(mtx_);
      enable_requested_ = req->data;  // 실패해도 '켜려 했다'를 남겨 상태 이유를 모델 없음으로
      if (req->data && !policy_.loaded()) {
        res->success = false;
        res->message = "정책 없음 (model_path) — 바이패스 유지";
        return;
      }
      if (enable_ && !req->data) {
        std::lock_guard<std::mutex> il(inner_mtx_);
        inner_->reset();
      }
      enable_ = req->data;
      prev_a_[0] = prev_a_[1] = 0.0f;
      res->success = true;
      res->message = std::string("enable=") + (enable_ ? "true (정책)" : "false (바이패스)") +
                     " · model=" + (policy_.loaded() ? "loaded" : "none") +
                     " · device=" + policy_.device();
      RCLCPP_INFO(logger_, "[%s] toggle: %s", name_.c_str(), res->message.c_str());
    });
  fallback_srv_ = node->create_service<ml::srv::SetFallback>(
    "~/" + name + "/set_fallback",
    [this](const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<ml::srv::SetFallback::Request> req,
           std::shared_ptr<ml::srv::SetFallback::Response> res) {
      if (req->controller.empty()) {  // 조회만
        std::lock_guard<std::mutex> il(inner_mtx_);
        res->success = true;
        res->active = inner_key_;
        res->message = "지금 inner=" + inner_key_;
        return;
      }
      const auto r = switch_inner(req->controller);
      res->success = r.first;
      res->message = r.second;
      std::lock_guard<std::mutex> il(inner_mtx_);
      res->active = inner_key_;
      RCLCPP_INFO(logger_, "[%s] set_fallback(%s): %s", name_.c_str(), req->controller.c_str(), r.second.c_str());
    });
  RCLCPP_INFO(logger_, "[%s] RlNavController: inner=%s · model=%s · device=%s · enable=%s · scan=%s "
              "· toggle=~/%s/toggle · set_fallback=~/%s/set_fallback · status=~/%s/status · diagnostics=/diagnostics",
              name_.c_str(), inner.c_str(), policy_.loaded() ? model.c_str() : "(없음)",
              policy_.device().c_str(), enable_ ? "true" : "false",
              use_scan_ ? "scan" : points_topic.c_str(), name_.c_str(), name_.c_str(), name_.c_str());
}

void RlNavController::load_model(const std::string & path) {
  if (path.empty()) return;
  std::string resolved = path;
  namespace fs = std::filesystem;
  const fs::path p(path);
  if (!p.is_absolute()) {  // 상대 경로: share/ml/model/<경로> 하나만 본다.
    //   "rl_nav_controller/policy.onnx"처럼 노드별 폴더 포함 — 노드마다 onnx를 가져와도 이름이 안 겹친다.
    try {
      const fs::path share = fs::path(ament_index_cpp::get_package_share_directory("ml")) /
                             "model" / p;
      if (fs::exists(share)) resolved = share.string();
    } catch (const std::exception &) {
    }
  }
  try {
    OnnxPolicy m;
    m.load(resolved, true);  // CUDA EP가 있으면 먼저 시도 (#92)
    const int64_t want = 2 * lookahead_ + lidar_rays_ + 5;
    if (m.inputs() != want || m.outputs() < 2) {
      RCLCPP_ERROR(logger_, "정책 입력 %ld · 출력 %ld — 기대 %ld · 2 (lookahead·lidar_rays 확인): %s",
                   long(m.inputs()), long(m.outputs()), long(want), resolved.c_str());
      return;
    }
    policy_ = std::move(m);
    if (!policy_.device_note().empty()) {
      RCLCPP_WARN(logger_, "ONNX CUDA EP 실패 — CPU로 폴백: %s", policy_.device_note().c_str());
    }
    RCLCPP_INFO(logger_, "ONNX loaded: %s (obs %ld → %ld · device=%s)", resolved.c_str(),
                long(policy_.inputs()), long(policy_.outputs()), policy_.device().c_str());
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
    sensor_msgs::PointCloud2ConstIterator<float> ii(*cloud, "intensity");
    size_t n = 0;
    for (size_t i = 0; i < total && n < 200000; ++i, ++ix, ++iy, ++iz, ++ii, ++n) {
      if ((i % stride) != 0) continue;
      const double px = *ix, py = *iy, pz = *iz;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
      const double esdf = *ii;  // nvblox ESDF intensity = 장애물까지 거리 [m]
      if (!std::isfinite(esdf) || esdf < 0.0 || esdf > esdf_max_distance_) continue;
      const double bx = r00 * px + r01 * py + r02 * pz + tx;
      const double by = r10 * px + r11 * py + r12 * pz + ty;
      const double bz = r20 * px + r21 * py + r22 * pz + tz;
      if (bz < z_min_ || bz > z_max_) continue;  // 바닥·천장 제외
      const double r = std::max(0.0, std::hypot(bx, by) - esdf - lidar_inflation_);  // ESDF 점 → 장애물 표면 거리 − 사용자 여유
      if (r > lidar_max_) continue;
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

void RlNavController::publish_status(
  int rl_active, int reason, const geometry_msgs::msg::TwistStamped & cmd) {
  if (!status_pub_) return;
  bool enable = false, cloud_ok = false;
  float cloud_age = -1.0f, scan_min = float(lidar_max_);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    enable = enable_;
    cloud_ok = cloud_ok_;
    if (cloud_stamp_.nanoseconds() != 0) {
      const auto node = node_.lock();
      if (node) cloud_age = float((node->now() - cloud_stamp_).seconds());
    }
    if (use_scan_) {
      if (scan_) {
        const auto node = node_.lock();
        if (node && (scan_->header.stamp.sec != 0 || scan_->header.stamp.nanosec != 0)) {
          cloud_age = float((node->now() - rclcpp::Time(scan_->header.stamp)).seconds());
        }
        for (float r : scan_->ranges) {
          if (std::isfinite(r) && r >= scan_->range_min && r <= scan_->range_max) {
            scan_min = std::min(scan_min, r);
          }
        }
      }
    } else {
      for (float r : ranges_) scan_min = std::min(scan_min, r);
    }
    last_rl_active_ = rl_active;
    last_reason_ = reason;
    last_v_ = float(cmd.twist.linear.x);
    last_w_ = float(cmd.twist.angular.z);
    has_cmd_ = true;
    if (auto node = node_.lock()) last_cmd_stamp_ = node->now();
  }
  std_msgs::msg::Float32MultiArray s;
  s.data = {float(rl_active), float(reason), policy_.loaded() ? 1.0f : 0.0f,
            enable ? 1.0f : 0.0f, cloud_ok ? 1.0f : 0.0f, cloud_age, scan_min,
            float(cmd.twist.linear.x), float(cmd.twist.angular.z), float(inner_kind_)};
  status_pub_->publish(s);
}

void RlNavController::publish_diagnostics() {
  if (!diag_pub_) return;
  const auto node = node_.lock();
  if (!node) return;

  bool enable = false, requested = false, use_scan = false, has_cmd = false, sensor_ok = false;
  int reason = 2, rl_active = 0, inner_kind = 0;
  float v = 0.0f, w = 0.0f, scan_min = float(lidar_max_);
  double cloud_age = -1.0;
  rclcpp::Time last_cmd{0, 0, RCL_ROS_TIME};
  {
    std::lock_guard<std::mutex> lk(mtx_);
    enable = enable_;
    requested = enable_requested_;
    use_scan = use_scan_;
    has_cmd = has_cmd_;
    last_cmd = last_cmd_stamp_;
    reason = last_reason_;
    rl_active = last_rl_active_;
    inner_kind = inner_kind_;
    v = last_v_;
    w = last_w_;
    if (use_scan) {
      if (scan_) {
        sensor_ok = true;
        if (scan_->header.stamp.sec != 0 || scan_->header.stamp.nanosec != 0) {
          cloud_age = (node->now() - rclcpp::Time(scan_->header.stamp)).seconds();
        }
        for (float r : scan_->ranges) {
          if (std::isfinite(r) && r >= scan_->range_min && r <= scan_->range_max) {
            scan_min = std::min(scan_min, r);
          }
        }
      }
    } else {
      sensor_ok = cloud_ok_;
      if (cloud_stamp_.nanoseconds() != 0) {
        cloud_age = (node->now() - cloud_stamp_).seconds();
      }
      for (float r : ranges_) scan_min = std::min(scan_min, r);
    }
  }
  if (sensor_ok && cloud_age > cloud_timeout_) sensor_ok = false;
  const bool recent = has_cmd && last_cmd.nanoseconds() != 0 &&
                      (node->now() - last_cmd).seconds() <= 1.0;

  const bool loaded = policy_.loaded();
  const bool active = recent && rl_active >= 0.5;
  const char * inner = "기본 제어기";
  if (inner_kind == 1) inner = "DWB";
  else if (inner_kind == 2) inner = "MPPI";
  else if (inner_kind == 3) inner = "RPP";
  else if (inner_kind == 4) inner = "RotationShim";
  const char * why = !recent ? "idle" : "RL";
  if (recent && reason == 1) why = "모델 없음/로드 실패";
  else if (recent && reason == 2) why = "enable=false";
  else if (recent && reason == 3) why = "센서 없음/stale";
  else if (recent && reason == 4) why = "추론 실패";

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "controller_server: FollowPath";
  st.hardware_id = "onnxruntime";
  st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (requested && !loaded) st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  else if (recent && requested && enable && !sensor_ok) st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  else if (recent && reason == 4) st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

  const std::string en = requested ? "enable=on" : "enable=off";
  if (requested && !loaded) {
    st.message = std::string("policy missing — ") + inner + " 바이패스 · " + en;
  } else if (!recent) {
    st.message = loaded ? ("idle · " + en + " · " + policy_.device() +
                           " · ONNX Runtime " + OnnxPolicy::runtime_version() +
                           " · fallback=" + inner)
                        : ("idle · " + en + " · policy none · fallback=" + inner);
  } else if (active) {
    st.message = "RL active · " + en + " · " + policy_.device() +
                 " · ONNX Runtime " + OnnxPolicy::runtime_version() +
                 " · fallback=" + inner;
  } else if (enable && !sensor_ok) {
    st.message = std::string(inner) + " 바이패스 · " + en + " · 센서 없음/stale";
  } else {
    st.message = std::string(inner) + " 바이패스 · " + en + " · policy " +
                 (loaded ? "loaded" : "none") + " · " + policy_.device();
  }

  auto kv = [&st](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue e;
    e.key = key;
    e.value = value;
    st.values.push_back(e);
  };
  char buf[96];
  kv("policy", loaded ? "loaded" : "none");
  kv("enable", requested ? "true" : "false");
  kv("active", active ? "true" : "false");
  kv("mode", !recent ? "idle" : (active ? "RL" : inner));
  kv("device", policy_.device());
  kv("provider", policy_.provider());
  kv("runtime", loaded ? "ONNX Runtime " + OnnxPolicy::runtime_version() : "none");
  kv("model", policy_.path());
  kv("inner", inner);
  kv("sensor", sensor_ok ? "ok" : "none");
  snprintf(buf, sizeof(buf), "%.2f", cloud_age);
  kv("sensor_age", cloud_age >= 0.0 ? buf : "n/a");
  snprintf(buf, sizeof(buf), "%.2f", scan_min);
  kv("scan_min", buf);
  kv("reason", why);
  snprintf(buf, sizeof(buf), "v=%.2f w=%.2f", v, w);
  kv("cmd", buf);
  if (!policy_.device_note().empty()) kv("device_note", policy_.device_note());

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = node->now();
  arr.status.push_back(st);
  diag_pub_->publish(arr);
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
  bool on = false, cloud_ok = false, enabled = false, requested = false;
  double gyro = 0.0, cloud_age = 0.0;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    enabled = enable_;
    requested = enable_requested_;
    on = enable_ && policy_.loaded();
    scan = scan_;
    ranges = ranges_;
    cloud_ok = cloud_ok_;
    gyro = gyro_seen_ ? gyro_z_ : velocity.angular.z;
    if (cloud_stamp_.nanoseconds() != 0) cloud_age = (node_.lock()->now() - cloud_stamp_).seconds();
  }
  const bool sensor_ok = use_scan_ ? bool(scan) : (cloud_ok && cloud_age <= cloud_timeout_);
  auto fallback = [&](int reason) -> geometry_msgs::msg::TwistStamped {
    geometry_msgs::msg::TwistStamped cmd;
    {
      std::lock_guard<std::mutex> il(inner_mtx_);  // 제어 주기 중에는 inner 를 바꾸지 못하게
      cmd = inner_->computeVelocityCommands(pose, velocity, goal_checker, transformed_global_plan, global_goal);
    }
    publish_status(0, reason, cmd);
    return cmd;
  };
  if (!on || !sensor_ok) {
    if (on) {
      RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 2000,
                           "[%s] %s 없음 — 바이패스", name_.c_str(), use_scan_ ? "/scan" : "점구름");
    }
    int reason = 2;  // enable=false
    if (requested && !policy_.loaded()) reason = 1;  // 모델 없음/로드 실패
    else if (enabled) reason = 3;                    // 센서 없음/stale
    return fallback(reason);
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
    return fallback(4);
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
  if (heading_cap_)  // obs 앞 두 칸 = 첫 웨이포인트(wp_step 앞, 로봇 좌표)
    cmd.twist.linear.x = heading_speed_cap(cmd.twist.linear.x, max_lin_, obs[0], obs[1]);
  if (debug_pub_) {  // A/B 분석용: [a0, a1, v_out, w_out, scan_min, v_odom, w_odom, on]
    std_msgs::msg::Float32MultiArray dbg;
    dbg.data = {a0, a1, static_cast<float>(cmd.twist.linear.x), static_cast<float>(cmd.twist.angular.z),
                scan_min, static_cast<float>(velocity.linear.x), static_cast<float>(velocity.angular.z), 1.0f};
    debug_pub_->publish(dbg);
  }
  publish_status(1, 0, cmd);
  return cmd;
}

void RlNavController::cleanup() {
  if (auto node = node_.lock(); node && param_cb_) {
    node->remove_on_set_parameters_callback(param_cb_.get());
  }
  param_cb_.reset();
  {
    std::lock_guard<std::mutex> il(inner_mtx_);
    inner_->cleanup();
  }
  cloud_sub_.reset();
  scan_sub_.reset();
  imu_sub_.reset();
  toggle_srv_.reset();
  fallback_srv_.reset();
  debug_pub_.reset();
  status_pub_.reset();
  diag_timer_.reset();
  diag_pub_.reset();
}
void RlNavController::activate() {
  std::lock_guard<std::mutex> il(inner_mtx_);
  active_ = true;
  inner_->activate();
}
void RlNavController::deactivate() {
  std::lock_guard<std::mutex> il(inner_mtx_);
  active_ = false;
  inner_->deactivate();
}
void RlNavController::newPathReceived(const nav_msgs::msg::Path & raw) {
  std::lock_guard<std::mutex> il(inner_mtx_);
  last_path_ = raw;  // inner 를 바꿀 때 새 inner 에 다시 넘긴다
  inner_->newPathReceived(raw);
}
bool RlNavController::cancel() {
  std::lock_guard<std::mutex> il(inner_mtx_);
  return inner_->cancel();
}
void RlNavController::reset() {
  {
    std::lock_guard<std::mutex> il(inner_mtx_);
    inner_->reset();
  }
  prev_a_[0] = prev_a_[1] = 0.0f;
}
void RlNavController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
  {
    std::lock_guard<std::mutex> il(inner_mtx_);
    last_speed_limit_ = speed_limit;
    last_speed_pct_ = percentage;
    have_speed_limit_ = true;
    inner_->setSpeedLimit(speed_limit, percentage);
  }
  speed_ratio_ = percentage ? speed_limit / 100.0 : speed_limit / max_lin_;
  speed_ratio_ = (speed_limit <= 0.0) ? 1.0 : std::clamp(speed_ratio_, 0.0, 1.0);
}

std::pair<bool, std::string> RlNavController::switch_inner(const std::string & want) {
  Fallback fb;
  if (!resolve_fallback(want, fb)) return {false, "controller 이름이 비었음 (dwb | mppi | rpp | rotation_shim | 클래스명)"};
  {
    std::lock_guard<std::mutex> il(inner_mtx_);
    if (fb.kind != 0 && fb.key == inner_key_) return {true, "이미 " + fb.key + " 사용 중 — 바꾸지 않음"};
  }
  // 새 inner 를 먼저 만들고 설정한다 — 파라미터는 yaml 의 <name>.inner_<별칭>.* (없으면 그 plugin 의 기본값)
  nav2_core::Controller::Ptr next;
  try {
    next = loader_.createSharedInstance(fb.cls);
    next->configure(node_, inner_namespace(name_, fb), tf_, costmap_ros_);
  } catch (const std::exception & e) {
    return {false, std::string("로드·설정 실패 — 이전 inner 유지 (패키지 설치·파라미터 확인): ") + e.what()};
  }
  nav2_core::Controller::Ptr old;
  std::string old_key;
  {
    std::lock_guard<std::mutex> il(inner_mtx_);  // 제어 주기가 끝난 뒤 교체 (compute 가 같은 잠금을 쥠)
    try {
      if (active_) {
        next->activate();
        if (!last_path_.poses.empty()) next->newPathReceived(last_path_);
        if (have_speed_limit_) next->setSpeedLimit(last_speed_limit_, last_speed_pct_);
      }
    } catch (const std::exception & e) {
      try { next->cleanup(); } catch (...) {}
      return {false, std::string("활성화 실패 — 이전 inner 유지: ") + e.what()};
    }
    old = inner_;
    old_key = inner_key_;
    inner_ = next;
    inner_key_ = fb.key;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    inner_kind_ = fb.kind;
  }
  try {
    if (active_) old->deactivate();
    old->cleanup();
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "[%s] 이전 inner(%s) 정리 중 예외: %s", name_.c_str(), old_key.c_str(), e.what());
  }
  return {true, old_key + " → " + fb.key + " (" + fb.cls + ") · 파라미터 " + inner_namespace(name_, fb) + ".*"};
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlNavController, nav2_core::Controller)
