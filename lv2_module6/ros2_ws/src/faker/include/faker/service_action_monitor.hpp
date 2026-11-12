#pragma once

// 서비스·액션 목록 모니터: monitor_manager(토픽 → debug/topic/)의 짝.
// 그래프의 서비스·액션을 주기적으로 훑어 호스트가 ROS 없이 볼 수 있게 파일로 떨군다.
//   <debug_dir>/service/list.json              {"services": [{"name": "/score_grasps", "type": "ml/srv/ScoreGrasps"}]}
//   <debug_dir>/service/<이름>/type            타입 문자열 한 줄 (예: ml/srv/ScoreGrasps)
//   <debug_dir>/service/<이름>/info            .srv 정의 (GetMessageDefinition, 실패시 사유)
//   <debug_dir>/action/list.json               {"actions": [{"name": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose"}]}
//   <debug_dir>/action/<이름>/type
//   <debug_dir>/action/<이름>/info              .action 정의 (실패시 사유)
//   <debug_dir>/interface/list                 `ros2 interface list` 그대로 (정적 카탈로그)
//   <debug_dir>/interface/<종류>/<패키지>/<이름>/info   종류별 폴더뎁스 (예: interface/srv/ml/ScoreGrasps/info)
//                                             프로젝트 타입은 기동시 생성,
//                                             그 외는 .../request 파일을 쓰면 생성
//                                             (예: interface/srv/ml/ScoreGrasps/request → info)
// 파라미터: debug_dir(기본 /debug), period_s(기본 2.0)
// 제외: 파라미터·type_description 계열, 액션 내부(/_action/) 서비스는 service 목록에서 빼고 action으로만 노출.
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

namespace faker {

namespace fs = std::filesystem;

class ServiceActionMonitor : public rclcpp::Node {
public:
  ServiceActionMonitor();

private:
  void scan();

  void dump_interfaces();

  // interface/<타입>/request → info 생성 (프로젝트 타입은 기동시 1회, 나머지는 요청시)
  // 실제 경로는 종류별: interface/<종류>/<패키지>/<이름>/ (예: srv/ml/ScoreGrasps)
  void poll_interface_requests();

  void ensure_interface_info(const std::string & type);
  static fs::path iface_dir(const std::string & debug_dir, const std::string & type);
  static bool is_param_service(const std::string & name);
  static bool is_action_internal(const std::string & name);
  static std::string action_name_of(const std::string & send_goal_service);
  static std::string action_type_of(const std::string & send_goal_type);
  static fs::path dir_of(const std::string & root, const std::string & name);
  static void write_text(const fs::path & out, const std::string & text);
  static std::string definition_of(const std::string & type);
  static void ensure_proto(const fs::path & dir, const std::string & type);

  std::string debug_dir_;
  std::map<std::string, std::string> services_;  // name -> type (직전 스캔)
  std::map<std::string, std::string> actions_;   // name -> type
  std::string interfaces_;  // 직전 덤프 (바뀌면만 기록)
  std::map<std::string, fs::file_time_type> iface_req_;  // type -> request mtime (on-demand info)
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr interface_timer_;
};

}  // namespace faker
