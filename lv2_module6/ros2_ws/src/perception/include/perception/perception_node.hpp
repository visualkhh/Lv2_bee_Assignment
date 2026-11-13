#pragma once

// perception_node (🛠 직접 작성, 담당: 전승혜) — YOLO 인지 노드 C++판 (파이썬 scripts/perception_node.py를 옮김, 설계: documents/project/ros/02-sense.md)
//   왜 C++: 젯슨에서 파이썬 노드가 CPU 92%를 써서 지연이 컸음 (11/11 py-spy: AprilTag 37% · 바코드 17% · 영상 받기 14%)
//   → 영상 받기·계산을 C++로, YOLO는 ONNX Runtime(젯슨 GPU면 CUDA)로. 동작·토픽·파라미터 이름은 파이썬과 같음
//
// [입력 / 출력] (파이썬과 같음)
//   입력: pole_front_cam · gripper_cam 컬러 · 정렬 depth(시각 맞춘 쌍) · camera_info · TF · /mission/capture_dice
//   출력: /tracked_objects (map, reliable) · /perception/<cam>/detections · /perception/<cam>/debug_image(보는 쪽 있을 때만)
//         /mission/qr (바코드 원문 — 토픽 이름은 옛 QR 미션 그대로) · /mission/dice_image(_raw) · /diagnostics
// [처리] 카메라마다 최대 4Hz: YOLO → 퍽 판정(색·크기·자세·태그·방향) · 표식(장소 태그) · 상자 윗면(depth) → 두 카메라 융합(칼만)
//         계산 함수는 perception_core.hpp, YOLO 실행은 yolo_onnx.hpp
// [파이썬과 다른 점]
//   - 모델은 .onnx만 (ultralytics .pt는 못 읽음 — 경고 후 빈 목록). OBB 모델의 회전 박스(누운 퍽 축)는 안 씀
//   - 바코드는 zxing-cpp C++ 라이브러리가 빌드에 있을 때만 (libzxing-dev). 없으면 진단에 "없음"
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "perception/perception_core.hpp"
#include "perception/yolo_onnx.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace perception {

using perception::msg::TrackedObject;
using perception::msg::TrackedObjectArray;
using sensor_msgs::msg::Image;

class PerceptionNode : public rclcpp::Node {
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 카메라 한 대의 최신 프레임 (시각 맞춘 컬러·depth 쌍 + 내부 파라미터)
  struct Frame {
    cv::Mat bgr, depth_mm;  // depth: 컬러에 정렬된 영상 [mm] (16UC1) — float이면 [m]로 봄
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
    double fx{0}, fy{0}, cx{0}, cy{0};
    double infer_t{0};       // 마지막 추론 시작 (벽시계 [s])
    std::vector<std::pair<Box, std::string>> drops;  // 버린 퍽 박스와 이유 (디버그 영상)
  };
  // 시험용: 동기화된 쌍 · camera_info를 직접 넣음
  void on_frame(const std::string & cam, const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth);
  void on_info(const std::string & cam, const sensor_msgs::msg::CameraInfo & msg);

private:
  // ---- 파라미터 (이름은 파이썬과 같음)
  double rate_hz_, conf_th_, sync_slop_, match_dist_, puck_center_offset_, puck_center_full_deg_, det_merge_iou_,
    track_merge_m_, pixel_sigma_, depth_sigma0_, depth_sigma_k_, q_, q_dyn_, self_radius_, body_radius_, body_height_,
    yaw_min_aspect_, fine_min_px_, depth_min_valid_, puck_tag_size_, yaw_min_elong_, location_tag_size_, box_min_side_,
    box_max_side_, box_cell_, box_tag_margin_, box_edge_px_, box_conf_, box_roi_margin_, dice_margin_, tag_gamma_,
    tag_min_px_, tag_half_min_px_, tag_roi_margin_, robot_front_x_;
  int imgsz_, cpu_threads_, self_arm_hits_, hue_tol_, edge_px_, near_px_, yaw_min_points_, dice_px_;
  bool tag_center_, self_filter_, agnostic_nms_, trust_model_posture_, box_detect_, tag_half_, barcode_once_, tag_roi_, box_roi_yolo_;
  std::string device_, track_frame_, base_frame_;
  std::vector<std::string> self_links_;
  std::vector<double> body_box_;
  std::pair<double, double> box_h_;
  cv::Vec3d location_box_size_;
  std::set<int> puck_tag_ids_, location_tags_;
  JudgeParams judge_;

  // ---- 모델 · 검출기
  std::unique_ptr<YoloOnnx> model_, box_model_;
  std::map<int, std::string> names_;
  std::string model_path_;
  TagDetector tag_det_;
  struct Barcode;
  std::shared_ptr<Barcode> barcode_;  // zxing-cpp (빌드에 없으면 nullptr)

  // ---- 상태 (mutex_로 보호)
  std::mutex mutex_;
  std::map<std::string, Frame> frames_;
  std::map<int, Track> tracks_;
  int next_id_{1};
  size_t published_{0}, tf_fallback_{0}, self_dropped_{0};
  std::map<std::string, size_t> synced_;
  std::map<std::pair<std::string, std::string>, std::array<double, 3>> prof_;  // (카메라, 단계) → 합 ms · 횟수 · 최대 ms
  std::map<std::string, double> prof_last_;
  std::optional<std::pair<cv::Matx33d, cv::Vec3d>> out_tf_;  // 마지막 map ← track_frame
  std::string qr_text_, barcode_text_;
  double qr_seen_{0.0};

  // ---- ROS
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  struct CamSync {
    std::shared_ptr<message_filters::Subscriber<Image>> color, depth;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
  };
  std::vector<CamSync> syncs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dice_sub_;
  rclcpp::Publisher<TrackedObjectArray>::SharedPtr pub_;
  std::map<std::string, rclcpp::Publisher<TrackedObjectArray>::SharedPtr> det_pubs_;
  std::map<std::string, rclcpp::Publisher<Image>::SharedPtr> debug_pubs_;
  rclcpp::Publisher<Image>::SharedPtr dice_pub_, dice_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  // ---- 처리
  void on_infer();
  void update_tracks(const std::string & cam, Frame & s, const std::vector<YoloBox> & res);
  std::vector<int> fuse(const std::string & cam, const builtin_interfaces::msg::Time & stamp, const std::vector<Det> & dets);
  void prune(double now_s);
  std::optional<std::pair<cv::Matx33d, cv::Vec3d>> lookup_tf(const std::string & frame_id,
                                                            const builtin_interfaces::msg::Time & stamp, bool quiet = false);
  std::optional<Body> robot_body(const builtin_interfaces::msg::Time & stamp);
  Track to_map(const Track & tr);
  TrackedObject to_msg(int tid, const Track & tr) const;
  std::optional<Det> marker_det(const Frame & s, const Box & box, cv::Point2d center, const cv::Matx33d & rot,
                                const cv::Vec3d & trans, const std::string & cls);
  std::pair<std::optional<double>, double> puck_yaw(const Frame & s, const cv::Mat1f & depth_m, const Box & box,
                                                    const std::string & color, const std::string & shape,
                                                    const std::string & posture, std::optional<double> tag_y,
                                                    const std::string & face, const cv::Matx33d & rot, const cv::Vec3d & trans);
  // 상자 깊이 찾기 범위 — 상자 전용 모델(box_model_path) 네모 · 없으면 본 모델 'box' 네모(box_roi_yolo) · 둘 다 없으면 화면 전체(없음)
  std::optional<std::vector<std::array<double, 4>>> box_rois(const cv::Mat & bgr, const std::vector<YoloBox> & res);
  bool model_has_box_{false};  // 본 YOLO 모델에 'box' 종류가 있음 (퍽+상자 한 모델)
  void publish_debug(const std::string & cam, const Frame & s, const std::vector<YoloBox> & res, const std::vector<Tag> & tags,
                     const std::vector<Det> & dets, const std::vector<int> & tids);
  void on_publish();
  void read_barcodes();
  void on_capture_dice(const std_msgs::msg::Int32 & msg);
  void publish_dice(const cv::Mat & img, const Frame & s, const std::string & what, bool raw);
  void on_diag();
  void prof_add(const std::string & cam, const std::string & key, double t0, std::optional<double> t1 = std::nullopt);
};

}  // namespace perception
