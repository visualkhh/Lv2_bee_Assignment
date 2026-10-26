#pragma once

// 모니터 매니저 (module5 fake_camera_bringup/monitor_manager.cpp 그대로): 떠 있는 토픽을 파일로 떨궈
// 호스트(통제실·시뮬레이션)에서 보게 한다. 반대 방향(호스트 → 토픽)은 input_manager.
// 0.5초마다 그래프를 훑어 새 토픽을 자동 구독 (토픽·타입을 코드에 적지 않음 → 추가돼도 재빌드 불필요).
//   <debug_dir>/topic/<토픽명>/message    마지막 메시지 JSON {"type": "...", "data": {...}} (덮어씀)
//   <debug_dir>/topic/<토픽명>/image.jpg  Image / CompressedImage 토픽이면 마지막 영상 (덮어씀)
//   <debug_dir>/topic/<토픽명>/depth.png  16UC1 depth 영상이면 무손실 16비트 PNG [mm] (시뮬레이터 깊이·점구름 보기)
//   <debug_dir>/topic/<토픽명>/data       message JSON에서 빠지는 큰 바이트 데이터 — PointCloud2면 cloud_frame(기본 base_footprint)으로 옮긴 점 — 첫 줄 JSON {frame, stamp, count}
//                                         + '\n' + float32 x,y,z × count (little endian). NaN 점은 뺌, cloud_max_points 넘으면 솎음.
//                                         시뮬레이터 지도에 점으로 (실기에 붙여도 monitor_manager만 띄우면 같은 파일)
//                                         OccupancyGrid면 첫 줄 JSON {frame, stamp, resolution, width, height, origin:[x,y,yaw]}
//                                         + '\n' + int8 칸 값 width×height (행 = y, 0 빈 곳 · 1~100 비용 · −1 모름) — /map · 코스트맵
//   <debug_dir>/topic/list.json           {"topics": [{"name": "/task/status", "type": "plan/msg/TaskStatus"}]}
//   예) /task/status → /debug/output/topic/task/status/message (호스트 lv2_module6/debug/output/topic/task/status/message)
//   시작할 때 output/topic/ 을 비운다 — 지난 실행의 마지막 메시지가 지금 값처럼 보이지 않게 (바뀔 때만 오는 상태는 transient_local이라 다시 옴)
// 그래프 기록 (예전 test-logger를 대신 — ros2 CLI 프로세스를 토픽·노드마다 띄우지 않고 이 노드 하나가 ROS API로 조회):
//   <debug_dir>/topic/list · topic/<토픽>/type · topic/<토픽>/info   (타입 · 발행/구독 노드와 QoS — 2초마다, 바뀔 때만 다시 씀)
//   <debug_dir>/node/list · node/<노드>/info · node/<노드>/params     (pub/sub/서비스 · 파라미터 — params는 노드마다 1회)
//   <debug_dir>/pkg/list · pkg/<패키지>/prefix · pkg/<패키지>/executables (시작 때 1회 — 사람이 보는 기록. 시뮬레이터 메시는 debug/install에서 찾음)
// 갱신 시각은 파일 mtime으로 본다.
// 쓰기는 토픽별로 period_s에 한 번만 — message는 마지막 값을 모아 두었다 한 번에, image.jpg·data는 period_s 지난 것만
//   (100Hz 토픽·30fps 영상을 다 쓰면 Docker Desktop 공유 폴더가 못 따라가 이 노드가 쓰기만 기다리다 멈춤).
//
// 파라미터: debug_dir(기본 /debug — 그 아래 topic/에 저장), period_s(기본 0.2 — 파일 쓰기 간격),
//   raw_images (기본 true): false면 sensor_msgs/Image(무압축 영상)는 구독 자체를 안 함 (네트워크 너머에서 띄울 때)
//   exclude_topics (기본 []): 이름으로 뺄 토픽 목록
//   cloud_frame (기본 base_footprint): 점구름을 옮길 프레임 (TF 없으면 그 점구름은 안 씀) · cloud_max_points (기본 20000)
//
// 메시지 → JSON: rosx_introspection(PlotJuggler가 쓰는 라이브러리)이 .msg 정의로 런타임 파싱.
//   타입 없이 바이트로 받는 GenericSubscription + Parser::deserializeIntoJson.
//   100개 넘는 배열(영상 픽셀 등)은 JSON에서 빠진다 (DISCARD_LARGE_ARRAYS 기본값).
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/imgcodecs.hpp"
#include "ament_index_cpp/get_packages_with_prefixes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace faker {

namespace fs = std::filesystem;

class MonitorManager : public rclcpp::Node {
public:
  MonitorManager();

private:
  void report();

  void scan();

  void write_topic_list();

  // 그래프 기록 (topic/ · node/) — 2초마다
  void dump_graph();

  // 패키지 기록 (pkg/) — 시작 때 1회
  void dump_packages();

  // 내용이 지난번과 같으면 안 씀 (2초마다 같은 파일을 다시 쓰지 않게)
  void write_if_changed(const fs::path & out, const std::string & text);

  // 이미지: best-effort 구독은 reliable·best-effort 발행 모두와 연결된다
  static rclcpp::QoS qos();

  // 메시지: 발행자 QoS에 맞춰 구독 (ros2 topic echo와 같은 방식).
  // 발행자가 모두 transient_local이면 transient_local로 → 늦게 붙어도 저장된 마지막 값을 바로 받음
  //   (예: /task/status·/task/sub_goals는 바뀔 때만 발행 → volatile이면 다음 발행까지 아무것도 못 받음)
  // 발행자가 모두 reliable일 때만 reliable (best-effort 발행자가 있으면 reliable 구독은 연결 안 됨)
  rclcpp::QoS qos_for(const std::string & topic);

  void subscribe_message(const std::string & topic, const std::string & type);
  void flush_messages();  // 모아 둔 마지막 message를 파일로 (period_s마다)

  void subscribe_image(const std::string & topic, const std::string & type);

  // 점구름 → cloud_frame으로 옮겨 topic/<토픽>/data (period_s마다)
  void subscribe_cloud(const std::string & topic, const std::string & type);
  void save_cloud(const std::string & topic, const sensor_msgs::msg::PointCloud2 & cloud);
  // 격자 지도(/map · 코스트맵) → topic/<토픽>/data (period_s마다)
  void subscribe_grid(const std::string & topic, const std::string & type);

  bool due(const std::string & key);

  fs::path dir_of(const std::string & topic) const;

  // 임시 파일에 쓰고 rename → 호스트가 반쯤 쓰인 파일을 읽지 않게.
  // 폴더는 매번 만든다.
  static fs::path prepare_tmp(const fs::path & out);

  void write_file(const fs::path & out, const std::string & text);

  void save_image(const std::string & topic, const cv::Mat & image);

  void save_depth(const std::string & topic, const cv::Mat & depth);

  std::string debug_dir_;
  bool raw_images_;
  std::set<std::string> excluded_;
  double period_;
  std::string cloud_frame_;
  int cloud_max_points_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::set<std::string> seen_;
  std::map<std::string, std::string> types_;  // 토픽 -> 타입 (list.json용)
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::map<std::string, rclcpp::Time> last_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  std::map<std::string, std::string> pending_;  // 토픽 → 아직 안 쓴 마지막 message JSON
  std::map<std::string, std::string> written_text_;  // 경로 → 마지막으로 쓴 내용
  struct ParamQuery {  // 노드 파라미터 조회 (1회): NEW → LISTING → LISTED → DONE
    enum { NEW, LISTING, LISTED, DONE } state = NEW;
    rclcpp::AsyncParametersClient::SharedPtr client;
    std::vector<std::string> names;
  };
  std::map<std::string, ParamQuery> params_;  // 노드 -> 조회 상태 (바뀔 때만 렌더)
  size_t written_ = 0;         // 최근 5초 저장 건수 (report마다 초기화)
  size_t write_failed_ = 0;
  size_t excluded_count_ = 0;
};

}  // namespace faker
