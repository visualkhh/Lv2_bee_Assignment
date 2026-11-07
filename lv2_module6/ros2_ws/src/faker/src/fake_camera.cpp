// fake_camera 구현 — 설명·선언은 include/faker/fake_camera.hpp, main은 fake_camera_main.cpp
#include "faker/fake_camera.hpp"

#include <fstream>
#include <sstream>

#include "rapidjson/document.h"

namespace faker {

FakeCamera::FakeCamera()
: rclcpp::Node("fake_camera") {
  width_ = declare_parameter<int>("width", 640);
  height_ = declare_parameter<int>("height", 480);
  cloud_step_ = declare_parameter<int>("cloud_step", 4);
  dummy_timeout_s_ = declare_parameter<double>("dummy_timeout_s", 2.0);
  dummy_depth_mm_ = static_cast<uint16_t>(declare_parameter<int>("dummy_depth_mm", 0));
  dummy_hfov_deg_ = declare_parameter<double>("dummy_hfov_deg", 87.0);
  dummy_period_s_ = 1.0 / std::max(1.0, declare_parameter<double>("dummy_rate_hz", 15.0));
  map_depth_ = declare_parameter<bool>("map_depth", map_depth_);
  wall_height_m_ = declare_parameter<double>("wall_height_m", wall_height_m_);
  depth_min_m_ = declare_parameter<double>("depth_min_m", depth_min_m_);
  depth_max_m_ = declare_parameter<double>("depth_max_m", depth_max_m_);
  render_step_ = std::max(1, static_cast<int>(declare_parameter<int>("render_step", render_step_)));
  const double poll_s = declare_parameter<double>("poll_s", 0.02);  // 메타 확인 주기 (발행은 새 프레임마다)
  last_frame_ = now();  // 시작 시각 — 실제 프레임을 한 번도 못 봤을 때 더미 대기 기준
  dir_ = declare_parameter<std::string>("live_dir", "/debug/input/hardware/camera");
  meta_path_ = dir_ + "/" + get_name() + ".json";
  frame_ = std::string(get_name()) + "_color_optical_frame";
  const auto qos = rclcpp::SensorDataQoS();
  color_pub_ = create_publisher<Image>("~/color/image_raw", qos);
  info_pub_ = create_publisher<CameraInfo>("~/color/camera_info", qos);
  depth_info_pub_ = create_publisher<CameraInfo>("~/depth/camera_info", qos);
  depth_pub_ = create_publisher<Image>("~/aligned_depth_to_color/image_raw", qos);
  points_pub_ = create_publisher<PointCloud2>("~/depth/color/points", qos);
  timer_ = create_wall_timer(std::chrono::duration<double>(poll_s), [this]() { poll(); });
  if (map_depth_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        if (m->data.size() != size_t(m->info.width) * m->info.height || m->info.resolution <= 0) return;
        occ_.resize(m->data.size());
        for (size_t i = 0; i < m->data.size(); ++i) occ_[i] = m->data[i] >= 65 ? 1 : 0;  // 알 수 없음(-1)은 빈 칸
        map_info_ = m->info;
        map_frame_ = m->header.frame_id.empty() ? std::string("map") : m->header.frame_id;
        have_map_ = true;
      });
  }
}

bool FakeCamera::read_meta(Frame & f) {
  std::error_code ec;
  const auto mtime = fs::last_write_time(meta_path_, ec);
  if (ec || mtime == meta_mtime_) return false;
  std::ifstream in(meta_path_);
  std::stringstream ss;
  ss << in.rdbuf();
  rapidjson::Document doc;
  doc.Parse(ss.str().c_str());
  if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("seq") || !doc.HasMember("stamp") ||
      !doc.HasMember("hfov_deg") || !doc.HasMember("color") || !doc.HasMember("depth")) {
    return false;  // 교체 순간 — 다음 확인에서 다시
  }
  meta_mtime_ = mtime;
  f.seq = doc["seq"].GetInt64();
  f.stamp = doc["stamp"].GetDouble();
  f.hfov_deg = doc["hfov_deg"].GetDouble();
  f.color = dir_ + "/" + doc["color"].GetString();
  f.depth = dir_ + "/" + doc["depth"].GetString();
  return f.seq != last_seq_;
}

void FakeCamera::poll() {
  Frame f;
  if (!read_meta(f)) {
    // 실제 프레임을 한 번도 못 받았을 때만 더미 (시뮬레이터 없이 headless 테스트용).
    // 한 번이라도 받았으면 메타가 곧 '시뮬레이터 이미지를 써라' 신호 — 끊겨도 더미로 가리지 않는다 (카메라 끊김 진단).
    const bool never_saw_sim = last_seq_ < 0;
    if (dummy_timeout_s_ > 0.0 && never_saw_sim && (now() - last_frame_).seconds() > dummy_timeout_s_) {
      if (!dummy_on_) {
        RCLCPP_INFO(get_logger(), "시뮬레이터 프레임 없음 — 더미 발행 (회색 영상 · %s)",
                    map_depth_ ? "코스 지도 depth" : "고정 depth");
        dummy_on_ = true;
      }
      publish_dummy();
      return;
    }
    dummy_on_ = false;
    if (last_seq_ >= 0 && (now() - last_frame_).seconds() > 2.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "새 프레임 없음 — 시뮬레이터가 멈췄거나 카메라 송출 끔");
    } else if (last_seq_ < 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "장면 기다리는 중 (%s) — 시뮬레이터 실행?", meta_path_.c_str());
    }
    return;
  }
  cv::Mat bgr = cv::imread(f.color, cv::IMREAD_COLOR);
  cv::Mat depth = cv::imread(f.depth, cv::IMREAD_ANYDEPTH);
  if (bgr.empty() || depth.type() != CV_16UC1 || bgr.size() != depth.size()) {
    RCLCPP_WARN(get_logger(), "프레임 %ld 읽기 실패 (%s) — 건너뜀", f.seq, f.color.c_str());
    return;
  }
  if (bgr.cols != width_ || bgr.rows != height_) {
    cv::resize(bgr, bgr, cv::Size(width_, height_), 0, 0, cv::INTER_LINEAR);
    cv::resize(depth, depth, cv::Size(width_, height_), 0, 0, cv::INTER_NEAREST);  // 깊이 값은 섞지 않음
  }
  last_seq_ = f.seq;
  last_frame_ = now();
  dummy_on_ = false;  // 진짜 프레임이 오면 더미 모드 해제
  publish(f, bgr, depth);
}

void FakeCamera::publish_dummy() {
  // 회색 영상 + 코스 지도 depth. 헤더 시각은 지금 (TF 외삽 방지).
  // last_frame_은 건드리지 않는다 — 여길 갱신하면 다음 poll에서
  // (now - last_frame_) > dummy_timeout_s가 다시 2초를 기다려 더미가 0.5Hz가 된다 (실측).
  // 주기는 dummy_rate_hz (collision_monitor source_timeout 1.0s 여유) — 렌더 비용 때문에 poll마다 내지 않는다.
  const rclcpp::Time t = now();
  if (last_dummy_.nanoseconds() != 0 && (t - last_dummy_).seconds() < dummy_period_s_) return;
  last_dummy_ = t;
  Frame f;
  f.seq = last_seq_;
  f.stamp = t.seconds();
  f.hfov_deg = dummy_hfov_deg_;
  cv::Mat bgr(height_, width_, CV_8UC3, cv::Scalar(128, 128, 128));
  cv::Mat depth(height_, width_, CV_16UC1, cv::Scalar(dummy_depth_mm_));
  if (map_depth_) {
    const double fx = width_ / 2.0 / std::tan(f.hfov_deg * M_PI / 180.0 / 2.0);
    // 촬영 시각 = 그린 자세의 TF 시각 (진짜 RealSense가 노출 시각을 찍듯). 지금 시각을 찍으면 회전 중에
    //   costmap이 다른 자세로 되돌려 벽이 시야 전체에 번져 유령 장애물이 됐음 (1.8 rad/s × 0.16 s ≈ 17°, 2026-11-07 실측)
    if (!render_map_depth(fx, width_ / 2.0, height_ / 2.0, depth, f.stamp)) {
      depth.setTo(0);  // 지도·TF가 아직 없음 — 지어내지 않고 '측정 없음'
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "코스 지도 depth 대기 (%s) — 깊이 없음으로 발행",
                           have_map_ ? "TF map→카메라 없음" : "/map 없음");
    }
  }
  publish(f, bgr, depth);
}

bool FakeCamera::render_map_depth(double fx, double cx, double cy, cv::Mat & depth, double & stamp_s) {
  if (!have_map_) return false;
  geometry_msgs::msg::TransformStamped tr;
  try {
    tr = tf_buffer_->lookupTransform(map_frame_, frame_, tf2::TimePointZero);
  } catch (const std::exception &) {
    return false;
  }
  stamp_s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9;
  const auto & q = tr.transform.rotation;
  const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  const double R[3][3] = {
    {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
    {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
    {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}};
  const double ox = tr.transform.translation.x, oy = tr.transform.translation.y, oz = tr.transform.translation.z;
  const double res = map_info_.resolution;
  const double gx0 = map_info_.origin.position.x, gy0 = map_info_.origin.position.y;  // 코스 지도는 회전 없음
  const int W = int(map_info_.width), H = int(map_info_.height);
  const double wall = wall_height_m_;
  auto occupied = [&](int ix, int iy) {
    return ix >= 0 && iy >= 0 && ix < W && iy < H && occ_[size_t(iy) * W + ix];
  };
  const int step = render_step_;
  for (int v = 0; v < height_; v += step) {
    for (int u = 0; u < width_; u += step) {
      // 광학 프레임 광선 (z=1) → map. 광선 매개변수 t가 곧 광학축 깊이 z.
      const double ex = (u + 0.5 * step - cx) / fx, ey = (v + 0.5 * step - cy) / fx;
      const double dx = R[0][0] * ex + R[0][1] * ey + R[0][2];
      const double dy = R[1][0] * ex + R[1][1] * ey + R[1][2];
      const double dz = R[2][0] * ex + R[2][1] * ey + R[2][2];
      double t_end = depth_max_m_;
      if (dz < -1e-9) t_end = std::min(t_end, -oz / dz);  // 바닥 z=0
      double hit = (dz < -1e-9 && -oz / dz <= depth_max_m_) ? -oz / dz : -1.0;
      // 2D DDA (Amanatides-Woo) — 광선이 지나는 칸을 차례로, 장애물 기둥 [0, wall]에 닿는지
      const double fx0 = (ox - gx0) / res, fy0 = (oy - gy0) / res;
      int ix = int(std::floor(fx0)), iy = int(std::floor(fy0));
      const double vx = dx / res, vy = dy / res;  // t당 칸 수
      const int sx = vx > 0 ? 1 : -1, sy = vy > 0 ? 1 : -1;
      double tmx = std::abs(vx) > 1e-12 ? ((vx > 0 ? ix + 1 - fx0 : fx0 - ix) / std::abs(vx)) : 1e18;
      double tmy = std::abs(vy) > 1e-12 ? ((vy > 0 ? iy + 1 - fy0 : fy0 - iy) / std::abs(vy)) : 1e18;
      const double tdx = std::abs(vx) > 1e-12 ? 1.0 / std::abs(vx) : 1e18;
      const double tdy = std::abs(vy) > 1e-12 ? 1.0 / std::abs(vy) : 1e18;
      double t_in = 0.0;
      while (t_in <= t_end) {
        const double t_out = std::min(tmx, tmy);
        if (occupied(ix, iy)) {
          const double z_in = oz + dz * t_in;
          if (z_in >= 0.0 && z_in <= wall) { hit = t_in; break; }  // 기둥 옆면
          if (z_in > wall && dz < 0) {                               // 위에서 내려와 기둥 윗면
            const double t_top = (wall - oz) / dz;
            if (t_top <= t_out) { hit = std::max(t_in, t_top); break; }
          }
        }
        if (tmx < tmy) { tmx += tdx; ix += sx; } else { tmy += tdy; iy += sy; }
        t_in = t_out;
      }
      const uint16_t mm = (hit >= depth_min_m_ && hit <= depth_max_m_) ? uint16_t(std::lround(hit * 1000.0)) : 0;
      const int h = std::min(step, height_ - v), w = std::min(step, width_ - u);
      depth(cv::Rect(u, v, w, h)).setTo(mm);
    }
  }
  return true;
}

void FakeCamera::publish(const Frame & f, const cv::Mat & bgr, const cv::Mat & depth) {
  std_msgs::msg::Header h;
  // 촬영 시각 — 호스트·컨테이너 시계가 조금 어긋나 미래가 되면 지금으로 (TF 외삽 오류 방지)
  const rclcpp::Time shot(static_cast<int64_t>(f.stamp * 1e9), RCL_SYSTEM_TIME);
  const rclcpp::Time t = now();
  h.stamp = shot.nanoseconds() < t.nanoseconds() ? shot : t;
  h.frame_id = frame_;

  Image color;
  color.header = h;
  color.width = width_;
  color.height = height_;
  color.encoding = "rgb8";
  color.step = width_ * 3;
  color.data.resize(color.step * height_);
  cv::Mat rgb(height_, width_, CV_8UC3, color.data.data());  // 메시지 버퍼에 바로 씀
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  color_pub_->publish(color);

  Image dimg;
  dimg.header = h;
  dimg.width = width_;
  dimg.height = height_;
  dimg.encoding = "16UC1";  // mm
  dimg.step = width_ * 2;
  dimg.data.resize(dimg.step * height_);
  cv::Mat d(height_, width_, CV_16UC1, dimg.data.data());
  depth.copyTo(d);
  depth_pub_->publish(dimg);

  const double fx = width_ / 2.0 / std::tan(f.hfov_deg * M_PI / 180.0 / 2.0);
  const double cx = width_ / 2.0, cy = height_ / 2.0;
  CameraInfo info;
  info.header = h;
  info.width = width_;
  info.height = height_;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {fx, 0, cx, 0, fx, cy, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {fx, 0, cx, 0, 0, fx, cy, 0, 0, 0, 1, 0};
  info_pub_->publish(info);
  depth_info_pub_->publish(info);  // 정렬 depth라 컬러와 같은 해상도·프레임·K

  // depth → 점구름 (optical 프레임: z 앞 · x 오른쪽 · y 아래). 0(측정 안 됨)은 뺀다.
  PointCloud2 cloud;
  cloud.header = h;
  PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize((width_ / cloud_step_) * (height_ / cloud_step_));
  size_t n = 0;
  {
    PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (int v = 0; v + cloud_step_ <= height_; v += cloud_step_) {
      for (int u = 0; u + cloud_step_ <= width_; u += cloud_step_) {
        const uint16_t mm = d.at<uint16_t>(v, u);
        if (mm == 0) continue;
        const float zm = mm / 1000.0f;
        *x = static_cast<float>((u - cx) * zm / fx);
        *y = static_cast<float>((v - cy) * zm / fx);
        *z = zm;
        ++x, ++y, ++z, ++n;
      }
    }
  }
  mod.resize(n);
  points_pub_->publish(cloud);
}

}  // namespace faker
