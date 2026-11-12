// perception_node 구현 — 설명·선언은 include/perception/perception_node.hpp, main은 perception_node_main.cpp
//   함수 이름·순서·규칙은 파이썬 scripts/perception_node.py와 같게 맞춤 (같은 곳을 찾아 비교하기 쉽게)
#include "perception/perception_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <functional>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer_interface.hpp"

#if __has_include(<ZXing/ReadBarcode.h>)
#include <ZXing/ReadBarcode.h>
#define PERCEPTION_HAVE_ZXING 1
#endif

namespace perception {

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double QR_GAP_S = 10.0;  // 바코드가 이만큼 안 보였다가 다시 읽히면 같은 내용도 /mission/qr 재발행 [s] (barcode_once 끔일 때)
// 모델 없을 때의 기본 클래스 (모델이 있으면 모델 안 이름)
const std::vector<std::string> CLASSES = {"puck", "puck_lying", "goal_zone", "station_zone", "qr_board", "obstacle", "person",
                                          "traffic_light"};
const std::set<std::string> DYNAMIC_CLASSES = {"person"};  // 스스로 움직이는 물체 — 예측 잡음 process_noise_dynamic

double wall_s() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
double mono_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double stamp_s(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }

rclcpp::QoS sensor_qos() {
  // 카메라 구독 (best_effort) · depth 5: 컬러·depth 동기화가 한쪽 프레임을 기다릴 여유
  return rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
}

cv::Matx33d quat_to_rot(const geometry_msgs::msg::Quaternion & q) {
  tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
  return {m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]};
}

std::string fmt(const char * f, double v) {
  char b[64];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

// ROS Image ↔ cv::Mat — cv_bridge를 안 씀: 젯슨은 ROS cv_bridge가 OpenCV 4.6, 이 노드는 JetPack OpenCV 4.8이라
//   cv_bridge를 링크하면 두 버전 라이브러리가 한 프로그램에 같이 올라감 (빌드 경고 "libopencv_* 406 may conflict with 408")
cv::Mat image_to_mat(const sensor_msgs::msg::Image & m, bool color) {
  const std::string & e = m.encoding;
  int type = -1;
  if (e == "bgr8" || e == "rgb8") type = CV_8UC3;
  else if (e == "bgra8" || e == "rgba8") type = CV_8UC4;
  else if (e == "mono8" || e == "8UC1") type = CV_8UC1;
  else if (e == "16UC1" || e == "mono16") type = CV_16UC1;
  else if (e == "32FC1") type = CV_32FC1;
  if (type < 0 || m.height == 0 || m.width == 0 || m.data.size() < static_cast<size_t>(m.step) * m.height)
    throw std::runtime_error("영상 형식 못 읽음: " + e);
  const cv::Mat view(static_cast<int>(m.height), static_cast<int>(m.width), type, const_cast<uint8_t *>(m.data.data()), m.step);
  cv::Mat out;
  if (!color) return view.clone();  // depth: 그대로 (16UC1 [mm] · 32FC1 [m])
  if (e == "bgr8") out = view.clone();
  else if (e == "rgb8") cv::cvtColor(view, out, cv::COLOR_RGB2BGR);
  else if (e == "bgra8") cv::cvtColor(view, out, cv::COLOR_BGRA2BGR);
  else if (e == "rgba8") cv::cvtColor(view, out, cv::COLOR_RGBA2BGR);
  else if (type == CV_8UC1) cv::cvtColor(view, out, cv::COLOR_GRAY2BGR);
  else throw std::runtime_error("컬러 영상이 아님: " + e);
  return out;
}

sensor_msgs::msg::Image mat_to_image(const cv::Mat & bgr, const std_msgs::msg::Header & hd) {
  sensor_msgs::msg::Image m;
  m.header = hd;
  m.height = static_cast<uint32_t>(bgr.rows);
  m.width = static_cast<uint32_t>(bgr.cols);
  m.encoding = "bgr8";
  m.is_bigendian = 0;
  m.step = static_cast<uint32_t>(bgr.cols * 3);
  m.data.resize(static_cast<size_t>(m.step) * m.height);
  const cv::Mat c = bgr.isContinuous() ? bgr : bgr.clone();
  std::memcpy(m.data.data(), c.data, m.data.size());
  return m;
}

}  // namespace

// ---------- 바코드 (zxing-cpp C++) — 1D(줄무늬)만 · 빌드에 라이브러리가 없으면 없음
struct PerceptionNode::Barcode {
  struct Hit { std::string text; std::array<cv::Point2d, 4> pts; };
  std::vector<Hit> read(const cv::Mat & bgr) {
    std::vector<Hit> out;
#ifdef PERCEPTION_HAVE_ZXING
    ZXing::ImageView iv(bgr.data, bgr.cols, bgr.rows, ZXing::ImageFormat::BGR, static_cast<int>(bgr.step));
    auto opts = ZXing::ReaderOptions().setFormats(ZXing::BarcodeFormat::LinearCodes);  // 발제 바코드는 Code128 (2D 검색은 뺌)
    for (const auto & r : ZXing::ReadBarcodes(iv, opts)) {
      if (r.text().empty()) continue;
      const auto & p = r.position();
      out.push_back({r.text(), {cv::Point2d(p.topLeft().x, p.topLeft().y), cv::Point2d(p.topRight().x, p.topRight().y),
                                cv::Point2d(p.bottomRight().x, p.bottomRight().y),
                                cv::Point2d(p.bottomLeft().x, p.bottomLeft().y)}});
    }
#else
    (void)bgr;
#endif
    return out;
  }
};

// ---------- 파라미터 읽기 (dynamic_typing — yaml에 정수·실수가 섞여도 받음)
namespace {
rcl_interfaces::msg::ParameterDescriptor dyn() {
  rcl_interfaces::msg::ParameterDescriptor d;
  d.dynamic_typing = true;
  return d;
}
double as_d(const rclcpp::Parameter & p) {
  return p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? static_cast<double>(p.as_int()) : p.as_double();
}
int as_i(const rclcpp::Parameter & p) {
  return p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ? static_cast<int>(p.as_double()) : static_cast<int>(p.as_int());
}
std::vector<double> as_dv(const rclcpp::Parameter & p) {
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    std::vector<double> o;
    for (auto v : p.as_integer_array()) o.push_back(static_cast<double>(v));
    return o;
  }
  return p.as_double_array();
}
}  // namespace

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options) : rclcpp::Node("perception", options) {
  // 파라미터 — 이름·기본값·뜻은 파이썬 PARAMS와 같음 (config/perception.yaml)
  auto pd = [this](const std::string & n, double v) {
    declare_parameter(n, rclcpp::ParameterValue(v), dyn());
    return as_d(get_parameter(n));
  };
  auto pi = [this](const std::string & n, int v) {
    declare_parameter(n, rclcpp::ParameterValue(v), dyn());
    return as_i(get_parameter(n));
  };
  auto pb = [this](const std::string & n, bool v) { return declare_parameter(n, v); };
  auto ps = [this](const std::string & n, const std::string & v) { return declare_parameter(n, v); };
  auto pdv = [this](const std::string & n, const std::vector<double> & v) {
    declare_parameter(n, rclcpp::ParameterValue(v), dyn());
    return as_dv(get_parameter(n));
  };
  rate_hz_ = pd("rate_hz", 10.0);                       // /tracked_objects 발행 주기 [Hz]
  const std::string model_path = ps("model_path", "");  // YOLO 가중치 ('' = 설치 폴더 model.pt · 이름만 = 설치 폴더에서 · 경로) — .onnx
  conf_th_ = pd("confidence", 0.4);                     // YOLO 신뢰도 하한
  imgsz_ = pi("imgsz", 416);                            // 입력 크기 [px] — .onnx가 고정 크기면 그 크기를 씀
  device_ = ps("device", "cuda");                       // cuda (젯슨 GPU — ONNX Runtime CUDA, 안 되면 저절로 CPU) · cpu
                                                        //   bringup은 device를 안 넘겨서 기본값이 쓰임 → GPU부터 시도
  cpu_threads_ = pi("cpu_threads", 2);                  // CPU 추론 스레드 수 (제어·Nav2가 밀리지 않게)
  sync_slop_ = pd("sync_slop", 0.05);                   // 컬러·depth 시각 차 허용 [s]
  match_dist_ = pd("match_dist", 0.4);                  // 검출을 트랙에 붙이는 최대 거리 [m]
  puck_center_offset_ = pd("puck_center_offset", 0.015);  // 퍽 표면 → 중심 [m] (0 = 끔)
  puck_center_full_deg_ = pd("puck_center_full_deg", 45.0);
  tag_center_ = pb("tag_center", true);                 // 태그를 읽은 서 있는 퍽은 태그 가운데로 중심
  det_merge_iou_ = pd("det_merge_iou", 0.5);            // 한 프레임 겹친 같은 퍽 박스를 하나로 (0 = 끔)
  track_merge_m_ = pd("track_merge_m", 0.03);           // 이 안의 같은 퍽 트랙 둘을 하나로 [m] (0 = 끔)
  pixel_sigma_ = pd("pixel_sigma", 4.0);                // 화면 위치 오차 [px]
  depth_sigma0_ = pd("depth_sigma0", 0.005);            // depth 오차 상수항 [m]
  depth_sigma_k_ = pd("depth_sigma_k", 0.01);           // depth 오차 거리² 계수
  q_ = pd("process_noise", 0.01);                       // 정지 물체 예측 잡음 [m²/s]
  q_dyn_ = pd("process_noise_dynamic", 1.0);            // 사람 예측 잡음
  track_frame_ = ps("track_frame", "odom");             // 추적·융합 좌표계 — 발행은 map
  self_filter_ = pb("self_filter", true);               // 로봇 자기 몸 검출을 버림
  base_frame_ = ps("base_frame", "base_footprint");
  self_links_ = declare_parameter("self_links", std::vector<std::string>{"shoulder_link", "upper_arm_link", "forearm_link",
                                                                         "wrist_link", "gripper_link", "end_effector_link"});
  self_radius_ = pd("self_radius", 0.06);               // 팔 선분에서 이 안이면 자기 팔 [m]
  body_radius_ = pd("body_radius", 0.2);                // 차체 원점에서 수평 이 안이면 차체 [m] (body_box 없을 때)
  body_height_ = pd("body_height", 0.6);                // 차체 높이 상한 [m]
  body_box_ = pdv("body_box", {0.0});                   // [x_min, x_max, y_half] base_frame 직사각형 — 3개면 body_radius 대신
  if (body_box_.size() != 3) body_box_.clear();
  self_arm_hits_ = pi("self_arm_hits", 3);              // 퍽이 아닌 박스 안 팔 투영점이 이만큼이면 자기 팔 (0 = 안 봄)
  agnostic_nms_ = pb("agnostic_nms", false);
  trust_model_posture_ = pb("trust_model_posture", false);  // 퍽 자세를 모델 이름(…_lying)으로
  yaw_min_aspect_ = pd("yaw_min_aspect", 1.3);          // (OBB 전용 — C++판은 회전 박스를 안 씀, 이름만 받음)
  fine_min_px_ = pd("fine_min_px", 30);                 // 퍽 박스 긴 변이 이보다 작으면 모양·모델 자세는 안 냄 [px]
  judge_.puck_colors = declare_parameter("puck_colors", std::vector<std::string>{"red", "green"});
  hue_tol_ = pi("hue_tol", 10);
  judge_.check_model_color = pb("check_model_color", true);
  judge_.long_min = pd("size_long_min_cm", 2.5);
  judge_.long_max = pd("size_long_max_cm", 10.0);
  judge_.short_min = pd("size_short_min_cm", 2.2);
  judge_.square_long = pd("square_long_cm", 4.5);
  {
    const auto r = pdv("square_ratio", {0.8, 1.25});
    judge_.square_ratio = {r.at(0), r.at(1)};
  }
  judge_.stand_ratio = pd("stand_ratio", 0.8);
  edge_px_ = pi("edge_px", 5);                          // 화면 가장자리 이 안에 닿으면 잘린 퍽 [px]
  near_px_ = pi("near_px", 70);                         // 깊이 없는데 박스 짧은 변이 이 이상이면 너무 가까운 퍽 [px]
  depth_min_valid_ = pd("depth_min_valid", 0.5);
  {
    // 값을 변수에 먼저 받아 둠 — get_parameter(...)가 돌려준 임시 객체의 배열을 바로 돌면 그 객체가 먼저 사라짐
    const auto puck_tags = declare_parameter("puck_tags", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
    for (auto v : puck_tags) puck_tag_ids_.insert(static_cast<int>(v));
    const auto loc_tags = declare_parameter("location_tags", std::vector<int64_t>{12, 13, 14});
    for (auto v : loc_tags) location_tags_.insert(static_cast<int>(v));
  }
  puck_tag_size_ = pd("puck_tag_size_m", 0.016);
  yaw_min_points_ = pi("yaw_min_points", 30);
  yaw_min_elong_ = pd("yaw_min_elong", 1.8);
  location_tag_size_ = pd("location_tag_size_m", 0.048);
  box_detect_ = pb("box_detect", true);                 // depth로 상자 윗면 찾기
  {
    const auto h = pdv("box_top_height_m", {0.05, 0.09});
    box_h_ = {h.at(0), h.at(1)};
  }
  box_min_side_ = pd("box_min_side_m", 0.17);
  box_max_side_ = pd("box_max_side_m", 1.0);
  box_cell_ = pd("box_cell_m", 0.02);
  box_tag_margin_ = pd("box_tag_margin_m", 0.08);
  box_edge_px_ = pd("box_edge_px", 0);
  {
    const auto l = pdv("location_box_size", {0.205, 0.205, 0.07});
    location_box_size_ = {l.at(0), l.at(1), l.at(2)};
  }
  const std::string box_model_path = ps("box_model_path", "");  // 상자 전용 YOLO (.onnx) — 있으면 그 네모 안에서만 상자를 찾음
  box_conf_ = pd("box_conf", 0.5);
  box_roi_margin_ = pd("box_roi_margin", 0.1);
  dice_px_ = pi("dice_image_px", 480);
  dice_margin_ = pd("dice_image_margin", 0.1);
  tag_gamma_ = pd("tag_gamma", 2.5);                    // AprilTag 감마 영상 (0 = 끔)
  tag_half_ = pb("tag_half", true);                     // 반 크기 영상도
  tag_min_px_ = pd("tag_min_perimeter_px", 16.0);       // 태그 후보 최소 둘레 [px] (CPU 절약 — 실물·Isaac 955장 같은 태그)
  tag_half_min_px_ = pd("tag_half_min_perimeter_px", 24.0);
  barcode_once_ = pb("barcode_once", true);             // 바코드를 한 번 읽으면 더 안 찾고 읽은 내용만 0.5초마다 다시 냄

  // ---- 모델
  std::string model_dir;
  try {
    model_dir = (ament_index_cpp::get_package_share_path("perception") / "model").string();
  } catch (const std::exception &) {
  }
  model_path_ = resolve_model_path(model_path, model_dir);
  for (size_t i = 0; i < CLASSES.size(); ++i) names_[static_cast<int>(i)] = CLASSES[i];
  auto load = [&](const std::string & path, const char * what) -> std::unique_ptr<YoloOnnx> {
    if (path.size() < 5 || path.substr(path.size() - 5) != ".onnx") {
      RCLCPP_WARN(get_logger(), "%s: .onnx만 읽음 (C++판) — %s", what, path.c_str());
      return nullptr;
    }
    auto m = std::make_unique<YoloOnnx>(path, device_, cpu_threads_, imgsz_);
    if (!m->ok()) {
      RCLCPP_WARN(get_logger(), "%s 로드 실패 (%s)", what, m->error().c_str());
      return nullptr;
    }
    RCLCPP_INFO(get_logger(), "%s 로드: %s (%s)", what, path.c_str(), m->provider().c_str());
    if (!m->error().empty()) RCLCPP_WARN(get_logger(), "%s: %s", what, m->error().c_str());  // GPU를 원했는데 CPU로 넘어간 이유
    return m;
  };
  if (!model_path_.empty()) {
    model_ = load(model_path_, "YOLO 모델");
    if (model_) names_ = model_->names();
    else RCLCPP_WARN(get_logger(), "YOLO 없음 — 빈 목록 발행");
  } else {
    RCLCPP_WARN(get_logger(), "model_path 없음 — 빈 목록 발행 (기동 검사용)");
  }
  if (!box_model_path.empty()) box_model_ = load(resolve_model_path(box_model_path, model_dir), "상자 YOLO 모델");
#ifdef PERCEPTION_HAVE_ZXING
  barcode_ = std::make_shared<Barcode>();
#endif

  // ---- TF · 발행 · 구독 · 타이머
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  dice_pub_ = create_publisher<Image>("/mission/dice_image", latched);
  dice_raw_pub_ = create_publisher<Image>("/mission/dice_image_raw", latched);
  dice_sub_ = create_subscription<std_msgs::msg::Int32>("/mission/capture_dice", 10,
    [this](std_msgs::msg::Int32::ConstSharedPtr m) { on_capture_dice(*m); });
  qr_pub_ = create_publisher<std_msgs::msg::String>("/mission/qr", latched);
  for (const std::string cam : {"pole_front_cam", "gripper_cam"}) {
    // 컬러·정렬 depth는 시각 맞춘 쌍으로만 (다른 순간의 컬러·depth를 섞지 않게) · camera_info는 따로
    CamSync cs;
    cs.color = std::make_shared<message_filters::Subscriber<Image>>(this, "/" + cam + "/color/image_raw", sensor_qos());
    cs.depth = std::make_shared<message_filters::Subscriber<Image>>(this, "/" + cam + "/aligned_depth_to_color/image_raw",
                                                                    sensor_qos());
    SyncPolicy policy(5);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_));
    // 정책(시각 차 한도)만 넣어 만들고 입력은 따로 연결 (정책 + 입력을 같이 넘기면 입력만 받는 생성자로 골라짐)
    cs.sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(static_cast<const SyncPolicy &>(policy));
    cs.sync->connectInput(*cs.color, *cs.depth);
    std::function<void(const Image::ConstSharedPtr &, const Image::ConstSharedPtr &)> cb =
      [this, cam](const Image::ConstSharedPtr & c, const Image::ConstSharedPtr & d) { on_frame(cam, c, d); };
    cs.sync->registerCallback(cb);
    syncs_.push_back(cs);
    info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>("/" + cam + "/color/camera_info", sensor_qos(),
      [this, cam](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { on_info(cam, *m); }));
    det_pubs_[cam] = create_publisher<TrackedObjectArray>("/perception/" + cam + "/detections", 5);
    debug_pubs_[cam] = create_publisher<Image>("/perception/" + cam + "/debug_image", sensor_qos());
  }
  pub_ = create_publisher<TrackedObjectArray>("/tracked_objects", 5);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
  using std::chrono::duration;
  timers_.push_back(create_wall_timer(duration<double>(1.0 / std::max(rate_hz_, 0.1)), [this]() { on_publish(); }));
  timers_.push_back(create_wall_timer(duration<double>(0.25), [this]() { on_infer(); }));  // 카메라당 최대 4Hz 추론
  timers_.push_back(create_wall_timer(duration<double>(1.0), [this]() { on_diag(); }));
  timers_.push_back(create_wall_timer(duration<double>(0.5), [this]() { read_barcodes(); }));  // 바코드 2Hz
}

// ---------- 입력
void PerceptionNode::on_frame(const std::string & cam, const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth) {
  cv::Mat bgr, d;
  try {
    bgr = image_to_mat(*color, true);
    d = image_to_mat(*depth, false);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 변환 실패: %s", cam.c_str(), e.what());
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  Frame & s = frames_[cam];
  s.bgr = bgr;
  s.depth_mm = d;
  s.stamp = color->header.stamp;
  s.frame_id = color->header.frame_id.empty() ? cam + "_color_optical_frame" : color->header.frame_id;
  ++synced_[cam];
}

void PerceptionNode::on_info(const std::string & cam, const sensor_msgs::msg::CameraInfo & msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  Frame & s = frames_[cam];
  s.fx = msg.k[0]; s.fy = msg.k[4]; s.cx = msg.k[2]; s.cy = msg.k[5];
}

static cv::Mat1f depth_meters(const cv::Mat & d) {
  cv::Mat1f out;
  if (d.type() == CV_16UC1) d.convertTo(out, CV_32F, 1.0 / 1000.0);  // [mm] → [m]
  else d.convertTo(out, CV_32F);                                      // 32FC1은 이미 [m]
  return out;
}

void PerceptionNode::prof_add(const std::string & cam, const std::string & key, double t0, std::optional<double> t1) {
  const double ms = ((t1 ? *t1 : mono_s()) - t0) * 1000.0;
  std::lock_guard<std::mutex> lk(mutex_);
  auto & p = prof_[{cam, key}];
  p[0] += ms; p[1] += 1; p[2] = std::max(p[2], ms);
}

// ---------- 추론 (타이머 4Hz, 카메라당 최대 4Hz)
void PerceptionNode::on_infer() {
  if (!model_) return;
  std::vector<std::pair<std::string, Frame>> cams;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto & [c, s] : frames_)
      if (!s.bgr.empty() && !s.depth_mm.empty() && s.fx > 0 && wall_s() - s.infer_t > 0.2) cams.emplace_back(c, s);
  }
  for (auto & [cam, s] : cams) {
    const double t_start = wall_s(), t0 = mono_s();
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (prof_last_.count(cam)) {
        auto & p = prof_[{cam, "간격"}];  // 직전 추론 시작부터 = 추론 간격 (0.25초 목표)
        const double ms = (t0 - prof_last_[cam]) * 1000.0;
        p[0] += ms; p[1] += 1; p[2] = std::max(p[2], ms);
      }
      prof_last_[cam] = t0;
    }
    std::vector<YoloBox> res;
    try {
      res = model_->predict(s.bgr, static_cast<float>(conf_th_), agnostic_nms_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "추론 실패: %s", e.what());
      continue;
    }
    prof_add(cam, "YOLO", t0);
    {
      std::lock_guard<std::mutex> lk(mutex_);
      frames_[cam].infer_t = t_start;
    }
    update_tracks(cam, s, res);
    prof_add(cam, "전체", t0);
  }
}

void PerceptionNode::update_tracks(const std::string & cam, Frame & s, const std::vector<YoloBox> & res) {
  // 한 카메라 한 프레임: YOLO 결과 · 태그 · depth 상자 → 검출 → 트랙에 융합 · 정리 → /perception/<cam>/detections
  const int h = s.bgr.rows, w = s.bgr.cols;
  const cv::Mat1f depth_m = depth_meters(s.depth_mm);
  const auto now_stamp = s.stamp;
  const auto tf = lookup_tf(s.frame_id, now_stamp);
  if (!tf) return;  // 위치를 못 구함
  const auto [rot, trans] = *tf;
  const double t_tag = mono_s();
  const auto tags = detect_tags(&tag_det_, s.bgr, tag_gamma_, tag_half_, tag_min_px_, tag_half_min_px_);
  prof_add(cam, "AprilTag", t_tag);
  // 로봇 몸(차체·팔) — 자기 몸 필터와 상자 찾기에 같이 씀
  const auto body = self_filter_ && (!res.empty() || box_detect_) ? robot_body(now_stamp) : std::nullopt;
  const auto arm_all = body ? arm_pixels(body->chain, rot, trans, s.fx, s.fy, s.cx, s.cy) : std::vector<cv::Vec3d>{};
  const auto arm_px = self_arm_hits_ > 0 ? arm_all : std::vector<cv::Vec3d>{};
  const cv::Mat1b arm_mask = arm_px.empty() ? cv::Mat1b() : arm_mask_image(arm_px, depth_m.size());
  std::vector<Det> dets;
  // 바닥 높이 = base_footprint 높이 (없으면 0) — 퍽 자세(높이)와 상자 윗면 찾기에 같이 씀
  const auto base_now = lookup_tf(base_frame_, now_stamp, true);
  const double floor_z = base_now ? base_now->second[2] : 0.0;
  const bool side_view = view_from_side(rot);
  s.drops.clear();
  std::vector<Box> puck_boxes;  // 태그를 겹친 퍽 중 어디에 붙일지
  for (const auto & b : res) {
    const auto it = names_.find(b.cls);
    if (it != names_.end() && split_class(it->second).first == "puck") puck_boxes.push_back(clip_box(b.x0, b.y0, b.x1, b.y1, w, h));
  }
  for (const auto & b : res) {
    std::optional<int> tag;
    std::optional<cv::Vec3d> tag_c;
    std::string shape;
    std::optional<double> yaw;
    double yaw_period = 0.0;
    const auto it = names_.find(b.cls);
    if (it == names_.end()) continue;
    const std::string name = it->second;
    auto [cls, model_color] = split_class(name);
    if (cls == "box") continue;  // 상자는 depth 윗면으로 따로 찾음
    const Box bx = clip_box(b.x0, b.y0, b.x1, b.y1, w, h);
    if (bx.x1 <= bx.x0 || bx.y1 <= bx.y0) continue;
    std::string posture, color;
    double zm = 0.0;
    if (cls == "puck") {
      zm = box_depth_m(depth_m, bx, depth_min_valid_, arm_mask);
      if (zm <= 0 && tag_center_) {  // 너무 가까워 depth 없음 → 퍽 태그가 보이면 그 거리(solvePnP)로
        const auto tc = tag_in_box(tags, bx, puck_tag_ids_, puck_boxes);
        const auto pnp = tc ? tag_pnp(tc->second, puck_tag_size_, s.fx, s.fy, s.cx, s.cy) : std::nullopt;
        if (pnp && pnp->second[2] > 0) zm = pnp->second[2];
      }
      const cv::Mat roi = s.bgr(cv::Range(bx.y0, bx.y1), cv::Range(bx.x0, bx.x1));
      const auto pc = puck_color(roi, judge_.puck_colors, hue_tol_).first;
      const auto [col, why_c] = settle_puck_color(roi, pc, model_color, judge_.puck_colors, judge_.check_model_color);
      if (!col) {
        s.drops.emplace_back(bx, "color");
        continue;
      }
      color = *col;
      const int m = edge_px_;
      const bool clipped = bx.x0 <= m || bx.y0 <= m || bx.x1 >= w - 1 - m || bx.y1 >= h - 1 - m;
      const bool near = std::min(bx.x1 - bx.x0, bx.y1 - bx.y0) >= near_px_;
      const double wcm = zm > 0 ? (bx.x1 - bx.x0) * zm / s.fx * 100 : -1.0;
      const double hcm = zm > 0 ? (bx.y1 - bx.y0) * zm / s.fy * 100 : -1.0;
      auto [ok, why, post] = judge_puck(judge_, wcm, hcm, color, clipped, "", near);
      posture = post;
      // 자세: 윗면 높이 먼저 (카메라 각도와 상관없음) · 모르면 옆에서 볼 때만 가로·세로 비율
      if (ok && zm > 0 && !clipped) {
        const auto h_post = posture_from_height(depth_m, bx, zm, rot, trans, s.fx, s.fy, s.cx, s.cy, floor_z,
                                                {0.0, location_box_size_[2]}).first;
        posture = !h_post.empty() ? h_post : (side_view ? posture : "");
      }
      const bool fine = std::max(bx.x1 - bx.x0, bx.y1 - bx.y0) >= fine_min_px_;  // 멀어 작으면 모양·자세를 픽셀로 못 가림
      const std::string m_posture = fine ? model_posture(name) : "";
      if (trust_model_posture_) posture = m_posture;
      if (!ok || zm <= 0) {
        s.drops.emplace_back(bx, (why.empty() ? std::string("no depth") : why) + " " + fmt("%.1f", wcm) + "x" + fmt("%.1f", hcm) +
                                   "cm" + (clipped ? " clip" : ""));
        continue;
      }
      if (posture == "lying") cls = "puck_lying";
      const auto tc = tag_in_box(tags, bx, puck_tag_ids_, puck_boxes);  // 번호만 붙임 — 색은 영상으로 정한 그대로
      if (tc) tag = tc->first;
      std::optional<double> tag_y;
      std::string face;
      if (tc) std::tie(tag_y, face) = tag_yaw(tc->second, puck_tag_size_, s.fx, s.fy, s.cx, s.cy, rot);
      shape = tag_shape(face, posture);
      if (shape.empty() && fine) shape = puck_shape(name);  // 몸통 면 태그가 보이면 YOLO 모양보다 우선
      if (tc && posture != "lying" && tag_center_)
        tag_c = tag_center(tc->second, depth_m, puck_tag_size_, puck_center_offset_, s.fx, s.fy, s.cx, s.cy, rot, trans, arm_mask);
      std::tie(yaw, yaw_period) = puck_yaw(s, depth_m, bx, color, shape, posture, tag_y, face, rot, trans);
    } else {
      std::vector<double> valid;
      for (int y = bx.y0; y < bx.y1; ++y)
        for (int x = bx.x0; x < bx.x1; ++x) {
          const float z = depth_m(y, x);
          if (z > 0.05f && z < 6.0f) valid.push_back(z);
        }
      if (valid.size() < 8) continue;
      zm = np_median(valid);
      // 신호등: 이름에 상태가 있으면 그대로 · 없으면 램프 HSV로
      color = !model_color.empty() ? model_color : classify_color(cls, s.bgr(cv::Range(bx.y0, bx.y1), cv::Range(bx.x0, bx.x1)));
    }
    const double u = (bx.x0 + bx.x1) / 2.0, v = (bx.y0 + bx.y1) / 2.0;
    cv::Vec3d z_map = rot * cv::Vec3d((u - s.cx) * zm / s.fx, (v - s.cy) * zm / s.fy, zm) + trans;
    if (is_puck_class(cls)) {  // depth는 보이는 표면 — 중심은 시선 방향으로 더 안쪽
      z_map = puck_center(z_map, trans, puck_center_offset_, puck_center_full_deg_);
      if (tag_c && cv::norm(*tag_c - z_map) < 0.03) z_map = *tag_c;  // 태그 가운데 중심이 더 정확 (3 cm 넘게 다르면 안 믿음)
    }
    const bool arm_in_box = !is_puck_class(cls) && box_has_arm(bx, arm_px, self_arm_hits_);  // 퍽은 그리퍼 밑에서 집으므로 뺌
    if (body && (arm_in_box || is_self(cls, z_map, *body, body_radius_, body_height_, self_radius_, body_box_))) {
      std::lock_guard<std::mutex> lk(mutex_);
      ++self_dropped_;
      s.drops.emplace_back(bx, "self");
      continue;
    }
    const cv::Matx33d r_cam = meas_cov_cam(zm, s.fx, s.fy, pixel_sigma_, depth_sigma0_, depth_sigma_k_);
    const cv::Vec3d fb = size_fallback(cls);
    Det d;
    d.class_name = cls;
    d.z = z_map;
    d.R = rot * r_cam * rot.t();
    d.size = {std::max((bx.x1 - bx.x0) * zm / s.fx, 0.01), std::max((bx.y1 - bx.y0) * zm / s.fy, 0.01), fb[2]};
    d.conf = b.conf;
    d.px = cv::Point(static_cast<int>(u), static_cast<int>(v));
    d.color = color;
    d.posture = posture;
    d.tag = tag;
    d.shape = shape;
    d.yaw = yaw;
    d.yaw_period = yaw_period;
    d.box = bx;
    dets.push_back(d);
  }
  // 상자 윗면 (depth) — 로봇 차체 둘레는 뺌
  std::optional<std::pair<cv::Vec2d, double>> avoid;
  if (base_now) avoid = std::make_pair(cv::Vec2d(base_now->second[0], base_now->second[1]), body_radius_);
  const double t_box = mono_s();
  std::vector<BoxTop> boxes;
  if (box_detect_) {
    const auto rois = box_rois(s.bgr);
    boxes = find_box_tops(depth_m, rot, trans, s.fx, s.fy, s.cx, s.cy, floor_z, box_h_, box_min_side_, box_max_side_, box_cell_, 4,
                          3.0, avoid, rois, arm_all);
    prof_add(cam, "상자", t_box);
  }
  for (auto & bt : boxes)  // 화면 가장자리에 잘린 상자 → 방향 모름
    if (box_cut(bt, rot, trans, s.fx, s.fy, s.cx, s.cy, depth_m.cols, depth_m.rows, box_edge_px_)) bt.yaw.reset();
  std::map<size_t, std::pair<int, std::optional<double>>> box_tag;  // 상자 번호 → (태그 번호, 태그 yaw)
  for (const auto & t : tags) {  // 장소 태그 (L1~L3) — 붙은 상자를 location_box로
    if (!location_tags_.count(t.id)) continue;
    auto d = marker_det(s, t.box, t.c, rot, trans, "location_box");
    if (!d) continue;
    auto [l_yaw, face] = tag_yaw(t.corners, location_tag_size_, s.fx, s.fy, s.cx, s.cy, rot);
    if (face != "side") l_yaw.reset();  // 옆면 태그의 법선만 방향으로 씀
    std::optional<size_t> on;
    for (size_t i = 0; i < boxes.size(); ++i)
      if (tag_on_box(cv::Vec2d(d->z[0], d->z[1]), boxes[i], box_tag_margin_)) { on = i; break; }
    if (on) {
      box_tag[*on] = {t.id, l_yaw};
      continue;
    }
    // 윗면을 못 찾음 → 태그에서 상자 안쪽으로 반 변 · 방향 모르면 태그 위치 그대로
    d->tag = t.id;
    d->size = location_box_size_;
    if (l_yaw) {
      d->z -= location_box_size_[0] / 2.0 * cv::Vec3d(std::cos(*l_yaw), std::sin(*l_yaw), 0.0);
      d->yaw = l_yaw;
      d->yaw_period = 2 * PI;
    }
    dets.push_back(*d);
  }
  for (size_t i = 0; i < boxes.size(); ++i) {
    const auto & bt = boxes[i];
    const cv::Vec3d p_img = rot.t() * (cv::Vec3d(bt.c[0], bt.c[1], bt.top_z) - trans);  // 윗면 가운데의 카메라 좌표
    const cv::Matx33d r_cam = meas_cov_cam(p_img[2], s.fx, s.fy, pixel_sigma_, depth_sigma0_, depth_sigma_k_);
    Det d;
    d.class_name = "start_box";
    d.z = {bt.c[0], bt.c[1], bt.top_z};
    d.R = rot * r_cam * rot.t();
    d.size = {bt.a, bt.b, std::max(bt.top_z - floor_z, 0.01)};
    d.conf = 1.0;
    d.px = cv::Point(static_cast<int>(s.fx * p_img[0] / p_img[2] + s.cx), static_cast<int>(s.fy * p_img[1] / p_img[2] + s.cy));
    d.yaw = bt.yaw;
    d.yaw_period = PI / 2;  // 윗면 사각형 변 방향 — 정사각형이라 90°마다 같음
    if (box_tag.count(i)) {  // 장소 태그가 붙은 상자 → location_box (방향은 태그 법선 우선)
      d.class_name = "location_box";
      d.tag = box_tag[i].first;
      if (box_tag[i].second) { d.yaw = box_tag[i].second; d.yaw_period = 2 * PI; }
    }
    dets.push_back(d);
  }
  const double now_s = stamp_s(now_stamp);
  dets = merge_overlap_dets(dets, det_merge_iou_);
  std::vector<int> tids;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    tids = fuse(cam, now_stamp, dets);
    const auto gone = merge_tracks(tracks_, track_merge_m_);
    for (auto & t : tids)
      if (gone.count(t)) t = gone.at(t);
    prune(now_s);
  }
  TrackedObjectArray out;
  out.header.stamp = now_stamp;
  out.header.frame_id = "map";
  for (size_t i = 0; i < dets.size(); ++i) {
    const auto & d = dets[i];
    Track tr;  // 검출 하나를 트랙 모양으로 (발행 형식을 같게)
    tr.class_name = d.class_name;
    tr.p = d.z; tr.P = d.R; tr.size = d.size; tr.conf = d.conf; tr.px = d.px;
    tr.color = d.color; tr.posture = d.posture; tr.shape = d.shape; tr.content = d.content;
    tr.tag = d.tag; tr.yaw = d.yaw; tr.yaw_period = d.yaw_period;
    tr.last_seen_sec = now_stamp.sec; tr.last_seen_nsec = now_stamp.nanosec;
    tr.camera = cam;
    out.objects.push_back(to_msg(tids[i], to_map(tr)));
  }
  det_pubs_[cam]->publish(out);
  if (debug_pubs_[cam]->get_subscription_count() > 0) publish_debug(cam, s, res, tags, dets, tids);  // 보는 쪽이 있을 때만
}

void PerceptionNode::publish_debug(const std::string & cam, const Frame & s, const std::vector<YoloBox> & res,
                                   const std::vector<Tag> & tags, const std::vector<Det> & dets, const std::vector<int> & tids) {
  // 회색 얇은 박스 = YOLO 전부 · 빨간 글씨 = 버린 이유 · 색 굵은 박스 = 받아들인 검출 · 흰 박스 = AprilTag · 하늘색 점 = 상자·장소 태그
  cv::Mat img = s.bgr.clone();
  const auto font = cv::FONT_HERSHEY_SIMPLEX;
  for (const auto & b : res) {
    const cv::Point p0(static_cast<int>(b.x0), static_cast<int>(b.y0)), p1(static_cast<int>(b.x1), static_cast<int>(b.y1));
    cv::rectangle(img, p0, p1, {128, 128, 128}, 1);
    const auto it = names_.find(b.cls);
    cv::putText(img, (it == names_.end() ? std::string("?") : it->second) + " " + fmt("%.2f", b.conf),
                {p0.x, std::max(p1.y + 12, 12)}, font, 0.4, {128, 128, 128}, 1);
  }
  for (const auto & [bx, why] : s.drops)
    cv::putText(img, "drop: " + why, {bx.x0, std::min(bx.y1 + 26, img.rows - 4)}, font, 0.45, {0, 0, 255}, 1);
  for (const auto & t : tags) {
    cv::rectangle(img, {t.box.x0, t.box.y0}, {t.box.x1, t.box.y1}, {255, 255, 255}, 1);
    cv::putText(img, "tag" + std::to_string(t.id), {t.box.x0, std::max(t.box.y0 - 3, 10)}, font, 0.4, {255, 255, 255}, 1);
  }
  static const std::map<std::string, cv::Scalar> col_map = {
    {"red", {0, 0, 255}}, {"green", {0, 200, 0}}, {"yellow", {0, 220, 255}}, {"off", {160, 160, 160}}, {"all", {255, 255, 255}}};
  for (size_t i = 0; i < dets.size(); ++i) {
    const auto & d = dets[i];
    cv::Scalar col{255, 0, 0};
    if (is_puck_class(d.class_name) || d.class_name == "traffic_light") {
      const auto it = col_map.find(d.color);
      if (it != col_map.end()) col = it->second;
    } else if (d.class_name == "obstacle") {
      col = {255, 255, 255};
    } else if (d.class_name == "barcode") {
      col = {200, 0, 200};
    }
    std::string text = "#" + std::to_string(tids[i]) + " " + d.class_name;
    if (!d.color.empty()) text += " " + d.color;
    if (!d.shape.empty()) text += " " + d.shape;
    if (!d.posture.empty()) text += " " + d.posture;
    if (d.tag) text += " tag" + std::to_string(*d.tag);
    if (d.box) {
      const auto & bx = *d.box;
      cv::rectangle(img, {bx.x0, bx.y0}, {bx.x1, bx.y1}, col, 2);
      int base = 0;
      const auto ts = cv::getTextSize(text, font, 0.5, 1, &base);
      const int ty = std::max(bx.y0 - 4, ts.height + 4);
      cv::rectangle(img, {bx.x0, ty - ts.height - 4}, {bx.x0 + ts.width + 4, ty + 3}, {0, 0, 0}, -1);
      cv::putText(img, text, {bx.x0 + 2, ty}, font, 0.5, col, 1);
    } else {
      cv::circle(img, d.px, 6, {255, 200, 0}, -1);
      cv::putText(img, text, {d.px.x + 8, d.px.y}, font, 0.5, {255, 200, 0}, 2);
    }
  }
  std_msgs::msg::Header hd;
  hd.stamp = s.stamp;
  hd.frame_id = s.frame_id;
  debug_pubs_[cam]->publish(mat_to_image(img, hd));
}

std::optional<std::vector<std::array<double, 4>>> PerceptionNode::box_rois(const cv::Mat & bgr) {
  // 상자 전용 YOLO → 상자 네모들 (box_roi_margin만큼 넓힘) · 모델이 없거나 실패면 없음 (화면 전체)
  if (!box_model_) return std::nullopt;
  std::vector<YoloBox> r;
  try {
    r = box_model_->predict(bgr, static_cast<float>(box_conf_), false);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "상자 추론 실패: %s", e.what());
    return std::nullopt;
  }
  std::vector<std::array<double, 4>> out;
  const auto & nm = box_model_->names();
  for (const auto & b : r) {
    const auto it = nm.find(b.cls);
    if (it == nm.end() || it->second != "box") continue;
    const double mx = (b.x1 - b.x0) * box_roi_margin_, my = (b.y1 - b.y0) * box_roi_margin_;
    out.push_back({std::max(0.0, b.x0 - mx), std::max(0.0, b.y0 - my), std::min(bgr.cols - 1.0, b.x1 + mx),
                   std::min(bgr.rows - 1.0, b.y1 + my)});
  }
  return out;
}

void PerceptionNode::on_capture_dice(const std_msgs::msg::Int32 & msg) {
  // 주사위 완성 사진 요청 (장소 태그 12~14 · 0 = 가장 가까운 장소 상자) → 상자 윗면을 반듯한 정사각형으로 펴서 /mission/dice_image
  //   카메라: 상자 윗면·퍽 윗면(+6 cm)이 다 화면에 들어오는 카메라 중 가장 수직으로 내려다보는 것 · 원본은 _raw로
  std::vector<std::pair<int, Track>> locs, any_box;
  std::map<std::string, Frame> frames;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto & [tid, tr] : tracks_) {
      if (tr.class_name == "location_box") locs.emplace_back(tid, tr);
      if (tr.class_name == "location_box" || tr.class_name == "start_box") any_box.emplace_back(tid, tr);
    }
    for (const auto & [c, s] : frames_)
      if (!s.bgr.empty() && s.fx > 0) frames[c] = s;
  }
  const int want = msg.data;
  if (want) {
    std::vector<std::pair<int, Track>> f;
    for (const auto & x : locs)
      if (x.second.tag && *x.second.tag == want) f.push_back(x);
    locs = f;
  }
  const std::string want_s = want ? std::to_string(want) : "아무거나";
  if (locs.empty() && !any_box.empty()) {
    RCLCPP_WARN(get_logger(), "주사위 사진: 장소 상자(tag %s) 트랙 없음 — 지금 보이는 가장 가까운 상자로 찍음", want_s.c_str());
    locs = any_box;
  }
  if (locs.empty()) {
    RCLCPP_WARN(get_logger(), "주사위 사진: 장소 상자(tag %s)를 아직 못 찾음 — 사진 없이 무시", want_s.c_str());
    return;
  }
  const auto base = lookup_tf(base_frame_, builtin_interfaces::msg::Time(), true);
  if (locs.size() > 1) {  // 로봇에서 가장 가까운 것
    const cv::Vec2d bxy = base ? cv::Vec2d(base->second[0], base->second[1]) : cv::Vec2d(0, 0);
    std::stable_sort(locs.begin(), locs.end(), [&](const auto & a, const auto & b) {
      return std::hypot(a.second.p[0] - bxy[0], a.second.p[1] - bxy[1]) < std::hypot(b.second.p[0] - bxy[0], b.second.p[1] - bxy[1]);
    });
  }
  const auto & tr = locs[0].second;
  const double floor_z = base ? base->second[2] : 0.0;
  cv::Vec3d c = tr.p;
  c[2] = floor_z + location_box_size_[2];  // 상자 윗면 높이
  const double side = std::max(tr.size[0], tr.size[1]) * (1.0 + 2 * dice_margin_);
  const double yaw = tr.yaw.value_or(0.0);
  const cv::Vec3d fwd = cv::Vec3d(std::cos(yaw), std::sin(yaw), 0.0) * (side / 2);   // 태그 쪽(상자 밖)
  const cv::Vec3d lft = cv::Vec3d(-std::sin(yaw), std::cos(yaw), 0.0) * (side / 2);
  const std::vector<cv::Vec3d> corners = {c - fwd + lft, c - fwd - lft, c + fwd - lft, c + fwd + lft};
  const cv::Vec3d puck_up(0, 0, size_fallback("puck")[2]);
  std::optional<std::tuple<double, std::string, Frame, std::vector<cv::Point2f>>> best;
  for (const auto & [cam, s] : frames) {
    const auto tf = lookup_tf(s.frame_id, s.stamp, true);
    if (!tf) continue;
    const auto & [rot, trans] = *tf;
    std::vector<cv::Point2f> uv;
    bool fits = true;
    for (int k = 0; k < 8 && fits; ++k) {
      const cv::Vec3d p = k < 4 ? corners[k] : corners[k - 4] + puck_up;
      const cv::Vec3d q = rot.t() * (p - trans);
      if (q[2] < 0.05) { fits = false; break; }
      const double u = s.fx * q[0] / q[2] + s.cx, v = s.fy * q[1] / q[2] + s.cy;
      fits = fits && -0.02 * s.bgr.cols <= u && u <= 1.02 * s.bgr.cols && -0.02 * s.bgr.rows <= v && v <= 1.02 * s.bgr.rows;
      uv.emplace_back(static_cast<float>(u), static_cast<float>(v));
    }
    if (!fits) continue;
    const double down = -rot(2, 2);  // 1이면 수직으로 내려다봄
    if (!best || down > std::get<0>(*best))
      best = std::make_tuple(down, cam, s, std::vector<cv::Point2f>(uv.begin(), uv.begin() + 4));
  }
  if (best) {
    const auto & [down, cam, s, uv] = *best;
    const float n = static_cast<float>(dice_px_);
    const cv::Mat hmat = cv::getPerspectiveTransform(uv, std::vector<cv::Point2f>{{0, 0}, {n, 0}, {n, n}, {0, n}});
    cv::Mat top;
    cv::warpPerspective(s.bgr, top, hmat, cv::Size(dice_px_, dice_px_), cv::INTER_LINEAR, cv::BORDER_CONSTANT, {114, 114, 114});
    const double tilt = std::acos(std::clamp(down, -1.0, 1.0)) * 180.0 / PI;
    publish_dice(top, s, "tag" + (tr.tag ? std::to_string(*tr.tag) : std::string("None")) + " 상자 윗면 (" + cam + " · 수직에서 " +
                         fmt("%.0f", tilt) + "° 기울어짐)", false);
    publish_dice(s.bgr, s, "원본", true);
    return;
  }
  const Frame * s = frames.count("gripper_cam") ? &frames.at("gripper_cam") : (frames.empty() ? nullptr : &frames.begin()->second);
  if (s) {
    RCLCPP_WARN(get_logger(), "주사위 사진: 상자 윗면이 카메라에 다 안 보임 — 원본 사진만 발행");
    publish_dice(s->bgr, *s, "원본(윗면 안 보임)", true);
  }
}

void PerceptionNode::publish_dice(const cv::Mat & img, const Frame & s, const std::string & what, bool raw) {
  std_msgs::msg::Header hd;
  hd.stamp = s.stamp;
  hd.frame_id = s.frame_id;
  (raw ? dice_raw_pub_ : dice_pub_)->publish(mat_to_image(img, hd));
  RCLCPP_INFO(get_logger(), "주사위 사진 발행: %s", what.c_str());
}

std::optional<Det> PerceptionNode::marker_det(const Frame & s, const Box & box, cv::Point2d center, const cv::Matx33d & rot,
                                              const cv::Vec3d & trans, const std::string & cls) {
  // 표식(장소 태그 · 바코드) 박스 → 검출 (위치 = 박스 안 깊이 중앙값으로 역투영) · 깊이 화소 4개 미만이면 없음
  const cv::Mat1f dm = depth_meters(s.depth_mm);
  const int h = dm.rows, w = dm.cols;
  const int x0 = std::max(box.x0, 0), y0 = std::max(box.y0, 0), x1 = std::min(box.x1, w - 1), y1 = std::min(box.y1, h - 1);
  if (x1 <= x0 || y1 <= y0) return std::nullopt;
  std::vector<double> valid;
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) {
      const float z = dm(y, x);
      if (z > 0.05f && z < 6.0f) valid.push_back(z);
    }
  if (valid.size() < 4) return std::nullopt;
  const double zm = np_median(valid);
  Det d;
  d.class_name = cls;
  d.z = rot * cv::Vec3d((center.x - s.cx) * zm / s.fx, (center.y - s.cy) * zm / s.fy, zm) + trans;
  d.R = rot * meas_cov_cam(zm, s.fx, s.fy, pixel_sigma_, depth_sigma0_, depth_sigma_k_) * rot.t();
  d.size = {std::max((x1 - x0) * zm / s.fx, 0.01), std::max((y1 - y0) * zm / s.fy, 0.01), size_fallback(cls)[2]};
  d.conf = 1.0;
  d.px = cv::Point(static_cast<int>(center.x), static_cast<int>(center.y));
  return d;
}

std::pair<std::optional<double>, double> PerceptionNode::puck_yaw(const Frame & s, const cv::Mat1f & depth_m, const Box & box,
                                                                  const std::string & color, const std::string & shape,
                                                                  const std::string & posture, std::optional<double> tag_y,
                                                                  const std::string & face, const cv::Matx33d & rot,
                                                                  const cv::Vec3d & trans) {
  // 누운 퍽: 퍽 색 화소의 바닥 위 점들의 긴 축 (주기 180°) · 서있는 원기둥: 방향 없음 · 그 밖: 태그 자세 (주기 90°)
  if (posture == "lying") {
    std::vector<cv::Vec2d> pts;  // puck_points_xy — 퍽 색이고 깊이 있는 화소 (최대 400점)
    if (is_hue_name(color)) {
      cv::Mat hsv;
      cv::cvtColor(s.bgr(cv::Range(box.y0, box.y1), cv::Range(box.x0, box.x1)), hsv, cv::COLOR_BGR2HSV);
      std::vector<cv::Point> idx;
      for (int y = 0; y < hsv.rows; ++y)
        for (int x = 0; x < hsv.cols; ++x) {
          const auto & p = hsv.at<cv::Vec3b>(y, x);
          const float z = depth_m(box.y0 + y, box.x0 + x);
          if (hue_dist(p[0], color) <= hue_tol_ && p[1] > 80 && p[2] > 50 && z > 0.05f && z < 3.0f) idx.emplace_back(x, y);
        }
      if (idx.size() > 400) {  // np.linspace(0, n−1, 400).astype(int)
        std::vector<cv::Point> sub;
        for (int k = 0; k < 400; ++k) sub.push_back(idx[static_cast<size_t>(k * (idx.size() - 1.0) / 399.0)]);
        idx = sub;
      }
      for (const auto & q : idx) {
        const float z = depth_m(box.y0 + q.y, box.x0 + q.x);
        const cv::Vec3d p = rot * cv::Vec3d((q.x + box.x0 - s.cx) * z / s.fx, (q.y + box.y0 - s.cy) * z / s.fy, z) + trans;
        pts.emplace_back(p[0], p[1]);
      }
    }
    auto yaw = axis_yaw(pts, yaw_min_points_, yaw_min_elong_);
    if (!yaw && shape == "cyl" && face == "side") yaw = tag_y;
    if (yaw) return {wrap_yaw(*yaw, PI), PI};
    return {std::nullopt, 0.0};
  }
  if (shape == "cyl" && posture == "standing") return {std::nullopt, 0.0};
  if (!tag_y || (shape == "cyl" && face == "top")) return {std::nullopt, 0.0};  // 원기둥 윗면 태그는 방향이 없음
  return {wrap_yaw(*tag_y, PI / 2), PI / 2};
}

std::vector<int> PerceptionNode::fuse(const std::string & cam, const builtin_interfaces::msg::Time & stamp,
                                      const std::vector<Det> & dets) {
  // 한 프레임 검출들을 트랙에 1:1로 붙여 칼만 갱신 (mutex 안에서) — 같은 물체 후보 · match_dist 안 → 마할라노비스 거리 작은 쌍부터
  const double t = stamp_s(stamp);
  std::map<int, cv::Matx33d> pred;
  for (const auto & [tid, tr] : tracks_) {
    const double q = DYNAMIC_CLASSES.count(tr.class_name) ? q_dyn_ : q_;
    pred[tid] = tr.P + cv::Matx33d::eye() * (q * std::max(0.0, t - tr.t));
  }
  std::vector<std::tuple<double, size_t, int>> pairs;
  for (size_t i = 0; i < dets.size(); ++i)
    for (const auto & [tid, tr] : tracks_) {
      if (!same_object(tr, dets[i])) continue;
      const cv::Vec3d e = dets[i].z - tr.p;
      if (cv::norm(e) >= match_dist_) continue;
      const cv::Vec3d x = (pred[tid] + dets[i].R).solve(e, cv::DECOMP_LU);
      pairs.emplace_back(e.dot(x), i, tid);
    }
  std::sort(pairs.begin(), pairs.end());
  std::map<size_t, int> used_d;
  std::set<int> used_t;
  for (const auto & [dist, i, tid] : pairs) {
    if (used_d.count(i) || used_t.count(tid)) continue;
    used_d[i] = tid;
    used_t.insert(tid);
    const Det & d = dets[i];
    Track & tr = tracks_.at(tid);
    cv::Matx33d K;
    std::tie(tr.p, tr.P, K) = kf_update(tr.p, pred[tid], d.z, d.R);
    const double a = (K(0, 0) + K(1, 1) + K(2, 2)) / 3.0;  // 이번 관측을 믿은 정도 — 크기도 같은 비율로
    tr.size = tr.size + a * (d.size - tr.size);
    tr.t = std::max(tr.t, t);
    if (t >= tr.last_seen_sec + tr.last_seen_nsec * 1e-9) { tr.last_seen_sec = stamp.sec; tr.last_seen_nsec = stamp.nanosec; }
    tr.seen[cam] = t;
    tr.conf = d.conf; tr.camera = cam; tr.px = d.px;
    if (tr.class_name != "puck" || !no_color(d.color)) tr.color = d.color;  // 퍽은 색 모르는 관측이 아는 색을 덮지 않게
    if (!d.posture.empty()) tr.posture = d.posture;
    if (d.tag) tr.tag = d.tag;  // 태그 번호는 한 번 읽으면 남김
    if (!d.content.empty()) tr.content = d.content;
    if (!d.shape.empty()) tr.shape = d.shape;
    if (d.yaw && tr.yaw && d.yaw_period < tr.yaw_period) {
      // 장소 상자: 태그로 구한 방향(360°)을 윗면 사각형 방향(90°)으로 덮지 않음
    } else if (d.yaw) {
      tr.yaw = d.yaw; tr.yaw_period = d.yaw_period;  // 방향은 구한 관측의 최신값
    } else if (!d.posture.empty() && d.posture != tr.yaw_posture) {
      tr.yaw.reset();  // 자세가 바뀜 → 옛 방향은 뜻이 없음
    }
    if (!d.posture.empty()) tr.yaw_posture = d.posture;
  }
  for (size_t i = 0; i < dets.size(); ++i) {
    if (used_d.count(i)) continue;
    const Det & d = dets[i];
    Track tr;
    tr.class_name = kind(d.class_name); tr.posture = d.posture;
    tr.p = d.z; tr.P = d.R; tr.t = t; tr.size = d.size; tr.conf = d.conf;
    tr.last_seen_sec = stamp.sec; tr.last_seen_nsec = stamp.nanosec;
    tr.seen[cam] = t;
    tr.camera = cam; tr.px = d.px; tr.color = d.color; tr.tag = d.tag; tr.content = d.content; tr.shape = d.shape;
    tr.yaw = d.yaw; tr.yaw_period = d.yaw_period; tr.yaw_posture = d.posture;
    tracks_[next_id_] = tr;
    used_d[i] = next_id_++;
  }
  std::vector<int> out;
  for (size_t i = 0; i < dets.size(); ++i) out.push_back(used_d[i]);
  return out;
}

void PerceptionNode::prune(double now_s) {
  // 장소 상자 자리의 start_box 삭제 · 2초 규칙 (mutex 안에서)
  std::vector<cv::Vec2d> loc;
  for (const auto & [tid, tr] : tracks_)
    if (tr.class_name == "location_box") loc.emplace_back(tr.p[0], tr.p[1]);
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    const auto & tr = it->second;
    bool del = false;
    if (tr.class_name == "start_box")
      for (const auto & q : loc)
        if (std::hypot(tr.p[0] - q[0], tr.p[1] - q[1]) < box_min_side_) { del = true; break; }
    if (now_s - tr.t > TRACK_TIMEOUT_S) del = true;
    it = del ? tracks_.erase(it) : std::next(it);
  }
}

std::optional<Body> PerceptionNode::robot_body(const builtin_interfaces::msg::Time & stamp) {
  const auto base = lookup_tf(base_frame_, stamp, true);
  if (!base) return std::nullopt;
  Body b;
  b.base = base->second;
  b.rot = base->first;
  for (const auto & link : self_links_) {
    const auto t = lookup_tf(link, stamp, true);
    if (t) b.chain.push_back(t->second);
  }
  return b;
}

Track PerceptionNode::to_map(const Track & tr) {
  // 추적 좌표(odom) → map 기준 사본 (지금의 map ← odom) · TF가 잠깐 없으면 마지막 값 · 한 번도 없었으면 그대로
  try {
    const auto t = tf_->lookupTransform("map", track_frame_, tf2::TimePointZero);
    const auto & tt = t.transform.translation;
    out_tf_ = std::make_pair(quat_to_rot(t.transform.rotation), cv::Vec3d(tt.x, tt.y, tt.z));
  } catch (const tf2::TransformException & e) {
    if (!out_tf_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "map ← %s TF 없음 — 그대로 발행: %s", track_frame_.c_str(), e.what());
      return tr;
    }
  }
  const auto & [rot, trans] = *out_tf_;
  Track out = tr;
  out.p = rot * tr.p + trans;
  out.P = rot * tr.P * rot.t();
  if (tr.yaw) out.yaw = wrap_yaw(*tr.yaw + std::atan2(rot(1, 0), rot(0, 0)), tr.yaw_period);
  return out;
}

TrackedObject PerceptionNode::to_msg(int tid, const Track & tr) const {
  TrackedObject o;
  o.id = tid;
  o.class_name = tr.class_name;
  if (o.class_name == "puck" && tr.posture == "lying") o.class_name = "puck_lying";  // 발행은 설계 이름
  o.confidence = static_cast<float>(tr.conf);
  o.pose.pose.position.x = tr.p[0]; o.pose.pose.position.y = tr.p[1]; o.pose.pose.position.z = tr.p[2];
  o.pose.pose.orientation.w = 1.0;
  if (tr.yaw) { o.pose.pose.orientation.z = std::sin(*tr.yaw / 2); o.pose.pose.orientation.w = std::cos(*tr.yaw / 2); }
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) o.pose.covariance[r * 6 + c] = tr.P(r, c);
  o.pose.covariance[3 * 6 + 3] = o.pose.covariance[4 * 6 + 4] = ROT_VAR_UNKNOWN;
  o.pose.covariance[5 * 6 + 5] = tr.yaw ? yaw_var_known() : ROT_VAR_UNKNOWN;
  o.size.x = tr.size[0]; o.size.y = tr.size[1]; o.size.z = tr.size[2];
  o.last_seen.sec = tr.last_seen_sec; o.last_seen.nanosec = tr.last_seen_nsec;
  // 본 카메라를 마지막 본 시각 순으로 — 끝은 px를 잰 카메라
  std::vector<std::pair<double, std::string>> seen;
  for (const auto & [c, t] : tr.seen)
    if (c != tr.camera) seen.emplace_back(t, c);
  std::stable_sort(seen.begin(), seen.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  for (const auto & [t, c] : seen) o.cameras.push_back(c);
  if (!tr.camera.empty()) o.cameras.push_back(tr.camera);
  o.px = {tr.px.x, tr.px.y};
  o.color = tr.color;
  auto kv = [](const std::string & k, const std::string & v) { KeyValue x; x.key = k; x.value = v; return x; };
  if ((o.class_name == "puck" || o.class_name == "puck_lying") && !tr.posture.empty()) o.details.push_back(kv("posture", tr.posture));
  else if (o.class_name == "traffic_light" && !o.color.empty()) o.details.push_back(kv("state", o.color));
  if (!tr.shape.empty()) o.details.push_back(kv("shape", tr.shape));
  if (tr.yaw) o.details.push_back(kv("yaw_period", std::to_string(static_cast<int>(std::lround(tr.yaw_period * 180.0 / PI)))));
  if (tr.distance) o.details.push_back(kv("distance", fmt("%.3f", *tr.distance)));
  if (tr.tag) o.details.push_back(kv("tag", std::to_string(*tr.tag)));
  if (!tr.content.empty()) o.details.push_back(kv("content", tr.content));
  return o;
}

std::optional<std::pair<cv::Matx33d, cv::Vec3d>> PerceptionNode::lookup_tf(const std::string & frame_id,
                                                                          const builtin_interfaces::msg::Time & stamp, bool quiet) {
  // track_frame ← frame_id, 영상 시각으로 (그 시각이 없으면 최신) · 기다리지 않음
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_->lookupTransform(track_frame_, frame_id, tf2_ros::fromMsg(stamp));
  } catch (const tf2::TransformException &) {
    try {
      t = tf_->lookupTransform(track_frame_, frame_id, tf2::TimePointZero);
      if (!quiet) {
        std::lock_guard<std::mutex> lk(mutex_);
        ++tf_fallback_;
      }
    } catch (const tf2::TransformException & e) {
      if (!quiet) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF 실패: %s", e.what());
      return std::nullopt;
    }
  }
  const auto & tr = t.transform.translation;
  return std::make_pair(quat_to_rot(t.transform.rotation), cv::Vec3d(tr.x, tr.y, tr.z));
}

// ---------- 발행 (rate_hz)
void PerceptionNode::on_publish() {
  TrackedObjectArray out;
  out.header.stamp = now();
  out.header.frame_id = "map";
  std::vector<std::pair<int, Track>> items;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto & [tid, tr] : tracks_) items.emplace_back(tid, tr);
  }
  const auto base = lookup_tf(base_frame_, builtin_interfaces::msg::Time(), true);  // 로봇 ~ 물체 바닥 위 거리
  for (auto & [tid, tr] : items) {
    if (base) tr.distance = std::hypot(tr.p[0] - base->second[0], tr.p[1] - base->second[1]);
    out.objects.push_back(to_msg(tid, to_map(tr)));
  }
  pub_->publish(out);
  std::lock_guard<std::mutex> lk(mutex_);
  ++published_;
}

// ---------- 바코드 (2Hz) — 작업 지시 문자열 (P2-01 …) → 물체 barcode + /mission/qr
void PerceptionNode::read_barcodes() {
  if (!barcode_) return;
  if (barcode_once_ && !barcode_text_.empty()) {
    // 이미 읽음 → 바코드 찾기는 더 안 하고 읽은 내용만 다시 냄 (mission_manager는 바코드 단계 시작 뒤에 받은 것만 씀)
    std_msgs::msg::String m;
    m.data = barcode_text_;
    qr_pub_->publish(m);
    return;
  }
  std::vector<std::pair<std::string, Frame>> frames;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto & [c, s] : frames_)
      if (!s.bgr.empty() && !s.depth_mm.empty() && s.fx > 0) frames.emplace_back(c, s);
  }
  for (const auto & [cam, s] : frames) {
    const auto hits = barcode_->read(s.bgr);
    if (hits.empty()) continue;
    const auto tf = lookup_tf(s.frame_id, s.stamp, true);
    std::vector<Det> dets;
    for (const auto & hit : hits) {
      double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9, u = 0, v = 0;
      for (const auto & p : hit.pts) {
        x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
        u += p.x / 4; v += p.y / 4;
      }
      if (tf) {
        auto d = marker_det(s, {static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(x1) + 1, static_cast<int>(y1) + 1},
                            {u, v}, tf->first, tf->second, "barcode");
        if (d) { d->content = hit.text; dets.push_back(*d); }
      }
      const double now_w = wall_s();
      if (hit.text != qr_text_ || now_w - qr_seen_ > QR_GAP_S) {
        qr_text_ = hit.text;
        std_msgs::msg::String m;
        m.data = hit.text;
        qr_pub_->publish(m);
        RCLCPP_INFO(get_logger(), "바코드 (%s): %s%s", cam.c_str(), hit.text.c_str(),
                    barcode_once_ ? " — 이제 바코드는 안 읽음 (barcode_once)" : "");
      }
      qr_seen_ = now_w;
      barcode_text_ = hit.text;
    }
    if (!dets.empty()) {
      std::lock_guard<std::mutex> lk(mutex_);
      fuse(cam, s.stamp, dets);
    }
  }
}

void PerceptionNode::on_diag() {
  DiagnosticStatus s;
  s.name = "perception: 상태";
  s.hardware_id = "beebot";
  if (!model_) { s.level = DiagnosticStatus::WARN; s.message = "모델 없음(기본 대기)"; }
  else { s.level = DiagnosticStatus::OK; s.message = "인지 중"; }
  size_t n, fused = 0, pub, fallback, dropped;
  std::map<std::string, size_t> synced;
  std::map<std::pair<std::string, std::string>, std::array<double, 3>> prof;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    n = tracks_.size();
    for (const auto & [tid, tr] : tracks_) {  // 최근 2초 안에 두 카메라가 모두 본 트랙
      if (tr.seen.size() < 2) continue;
      double lo = 1e18, hi = -1e18;
      for (const auto & [c, t] : tr.seen) { lo = std::min(lo, t); hi = std::max(hi, t); }
      fused += hi - lo < TRACK_TIMEOUT_S;
    }
    pub = published_; synced = synced_; fallback = tf_fallback_; dropped = self_dropped_;
    prof.swap(prof_);  // 지난 1초 처리 시간 (가져가고 비움)
  }
  auto kv = [](const std::string & k, const std::string & v) { KeyValue x; x.key = k; x.value = v; return x; };
  std::string sy;
  for (const auto & [c, k] : synced) sy += (sy.empty() ? "" : " · ") + c + " " + std::to_string(k);
  s.values = {kv("추적 물체", std::to_string(n)), kv("두 카메라 융합 물체", std::to_string(fused)),
              kv("동기화 프레임", sy.empty() ? "0" : sy), kv("TF 최신 대체", std::to_string(fallback)),
              kv("자기 몸이라 버린 검출", std::to_string(dropped)), kv("발행한 /tracked_objects", std::to_string(pub)),
              kv("YOLO", model_ ? model_path_ + " · " + model_->provider() : "없음"),
              kv("AprilTag (퍽 번호·장소)", std::string("OpenCV ") + CV_VERSION + (tag_det_.ok() ? " aruco 있음" : " aruco 없음")),
              kv("바코드 (장소 작업 지시)", barcode_ ? "zxing-cpp (1D)" + (barcode_text_.empty() ? std::string() : " · 읽음: " + barcode_text_)
                                                  : "없음 — zxing-cpp 없이 빌드됨 (libzxing-dev)")};
  std::set<std::string> cams;
  for (const auto & [k, v] : prof) cams.insert(k.first);
  for (const auto & cam : cams) {  // 카메라마다: 단계 평균/최대 [ms] (지난 1초)
    std::string parts;
    for (const char * k : {"YOLO", "AprilTag", "상자", "전체", "간격"}) {
      const auto it = prof.find({cam, k});
      if (it == prof.end()) continue;
      parts += (parts.empty() ? "" : " · ") + std::string(k) + " " + fmt("%.0f", it->second[0] / it->second[1]) + "/" +
               fmt("%.0f", it->second[2]);
    }
    s.values.push_back(kv("처리 시간 ms 평균/최대 " + cam, parts));
  }
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = now();
  msg.status = {s};
  diag_pub_->publish(msg);
}

}  // namespace perception
