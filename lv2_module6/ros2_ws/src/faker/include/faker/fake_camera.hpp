#pragma once

// RealSense 대역: realsense2_camera(camera_namespace:='', camera_name:=<이름>) 대신 같은 토픽을 발행.
//   /<이름>/color/image_raw · color/camera_info · depth/camera_info · aligned_depth_to_color/image_raw (best_effort)
//   /<이름>/depth/color/points (PointCloud2, depth에서 cloud_step 픽셀 간격으로 만든 xyz — optical 프레임)
// 데이터는 시뮬레이터(lv2_module6/simulator)가 로봇 카메라 시점으로 렌더한 것만 낸다 — 지어내지 않는다.
//   <live_dir>/<이름>.json   메타 {seq, stamp(촬영 시각, 초), hfov_deg, color, depth} — 마지막에 쓰여 아래 두 파일을 가리킴
//   <live_dir>/<color>       컬러 PNG
//   <live_dir>/<depth>        16비트 depth PNG [mm], 0 = 측정 안 됨 (RealSense처럼 — 너무 가깝거나 멀거나 배경)
// 메타의 seq가 바뀔 때만 그 컬러·깊이 쌍을 한 번 발행한다 (같은 순간의 쌍 · 같은 프레임 반복 없음 · 시각 = 촬영 시각).
// 메타를 한 번이라도 받으면 시뮬레이터가 켜진 것 — 그 뒤로는 더미를 내지 않는다 (새 메타가 끊기면 침묵 = 카메라 끊김).
// 메타가 한 번도 없으면(시뮬레이터 없이 headless 실행) dummy_timeout_s 뒤부터 더미(회색 영상 · 깊이 없음)를 낸다.
// 내부 파라미터는 메타의 hfov_deg로 정한다 (시뮬레이터 렌더와 한 곳에서). 출력 크기는 width×height (깊이는 보간 없이 확대).
// launch에서 노드 이름을 pole_front_cam / gripper_cam 으로 띄운다 (~/ 토픽이 /pole_front_cam/... 이 됨).
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace faker {

using namespace sensor_msgs::msg;
using sensor_msgs::PointCloud2Iterator;
using sensor_msgs::PointCloud2Modifier;
namespace fs = std::filesystem;

class FakeCamera : public rclcpp::Node {
public:
  FakeCamera();

private:
  struct Frame {
    long seq = -1;
    double stamp = 0, hfov_deg = 0;
    std::string color, depth;
  };

  // 메타가 바뀌었으면 읽어 frame에 채움 → true (새 프레임). 없음·같은 seq·깨짐이면 false
  bool read_meta(Frame & frame);
  void poll();
  void publish(const Frame & f, const cv::Mat & bgr, const cv::Mat & depth);
  void publish_dummy();  // 시뮬레이터 없이 돌릴 때: 회색 영상 + 깊이 없음(0) 프레임 (장애물 없음 가정)

  int width_, height_, cloud_step_;
  double dummy_timeout_s_ = 2.0;   // 새 메타가 이 시간[초] 안 오면 더미 발행 (0 이하면 끔)
  uint16_t dummy_depth_mm_ = 0;    // 더미 깊이 [mm] — 0 = 측정 없음 (점구름 빔)
  double dummy_hfov_deg_ = 87.0;   // 더미 camera_info 화각
  bool dummy_on_ = false;          // 지금 더미 내보내는 중 (진짜 프레임 오면 false)
  std::string dir_, frame_, meta_path_;
  fs::file_time_type meta_mtime_{};
  long last_seq_ = -1;
  rclcpp::Time last_frame_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<Image>::SharedPtr color_pub_, depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr info_pub_, depth_info_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr points_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace faker
