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
        RCLCPP_INFO(get_logger(), "시뮬레이터 프레임 없음 — 더미 발행 (회색 영상 · 깊이 없음)");
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
  // 회색 영상 + 깊이 없음(0 → 점구름 빔) — 장애물 없음 가정. 헤더 시각은 지금 (TF 외삽 방지).
  cv::Mat bgr(height_, width_, CV_8UC3, cv::Scalar(128, 128, 128));
  cv::Mat depth(height_, width_, CV_16UC1, cv::Scalar(dummy_depth_mm_));
  Frame f;
  f.seq = last_seq_;
  f.stamp = now().seconds();
  f.hfov_deg = dummy_hfov_deg_;
  // last_frame_은 건드리지 않는다 — 여길 갱신하면 다음 poll에서
  // (now - last_frame_) > dummy_timeout_s가 다시 2초를 기다려 더미가 0.5Hz가 된다 (실측).
  // 더미는 poll마다 나간다 (진짜 카메라 30Hz급 — collision_monitor source_timeout 1.0s 여유).
  publish(f, bgr, depth);
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
