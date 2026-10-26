#pragma once

// state_manager (🛠 직접 작성, 담당: 김현하) — 로봇 상태의 주인. 모든 상태는 여기서 판단한다:
//   다른 토픽(요청 · 하위 노드 응답)을 보고 상태를 정하고, 상태 토픽 /state/<이름> 은 이 노드만 발행한다 (다른 노드는 구독만).
//   외부 명령은 전부 /cmd/... (보내고 끝 — cmd_vel 관례). 상태 명령 /cmd/operation_mode · /cmd/map_mode · /cmd/e_stop 은 이 노드만 구독 → 결과 /state/<이름> — /state/* 에는 이 노드 말고 아무도 쓰지 않음
//   (호스트 파일 → 토픽인 faker input_manager도 /state/* 는 거부).
//   상태를 바꾸는 일(RTAB-Map 모드 전환 등)도 여기서 직접 해서, 발행 값 = 진짜 상태. 건강·오류는 상태가 아니라 diagnostics 몫.
//   새 상태를 늘릴 때도 /state/<이름> (std_msgs, transient_local) 하나씩.
//   조작 모드
//     입력: ✉️ /cmd/operation_mode (std_msgs/String "MANUAL" | "AUTO" | "EXPLORE") — 시뮬레이터 [수동 조작] · [자동 탐사] · bee.py manual
//           EXPLORE(자동 탐사 — plan/explorer가 프론티어로 Nav2 목표를 보냄)는 지도 모드 MAPPING · 비상정지 아님일 때만 허락.
//           ✉️ /explore/result (std_msgs/String "DONE …" | "FAILED …", explorer) · MAPPING이 끝남 · 지도 바꿈 · 비상정지 → AUTO로 되돌림
//   출력: ✉️ /state/operation_mode (std_msgs/String "AUTO" | "MANUAL" | "EXPLORE", transient_local) → manual_control(수동 명령 통과) · coordinator(자동 실행 멈춤)
//         ✉️ /state/emergency (std_msgs/Bool, transient_local) — /cmd/e_stop 그대로. coordinator는 이것만 본다 (원본 직접 구독 금지)
//   지도 모드
//     입력: ✉️ /cmd/map_mode (std_msgs/String "MAPPING" | "LOCALIZATION") — 시뮬레이터 [스캔 시작] · [위치추정으로]
//           → 📞 /rtabmap/rtabmap/set_mode_mapping · /rtabmap/rtabmap/set_mode_localization (std_srvs/Empty, RTAB-Map 재시작 없이)
//              (rtabmap_launch: 이름공간 rtabmap + 노드 rtabmap의 ~/ 서비스 → /rtabmap/rtabmap/… — /rtabmap/set_mode_* 에는 서버 없음, 실측)
//     출력: ✉️ /state/map_mode (std_msgs/String "MAPPING" | "LOCALIZATION" | "COURSE", transient_local) — RTAB-Map 응답이 온 뒤에만 바꿈
//           COURSE = RTAB-Map 없이 정지 지도(테스트베드 코스 지도) — 바꿀 수 없음
//     파라미터 map_dir: 지도 폴더들이 있는 곳 (테스트베드 /ws/map — faker가 넘김, 비면 /cmd/map_name 거부)
//     시작 값: 파라미터 map_mode (launch가 RTAB-Map을 띄운 모드 그대로 — 실기 LOCALIZATION · faker map:=slam → MAPPING)
//     출력: ✉️ /state/map_list (std_msgs/String, transient_local) — 가진 지도 폴더 이름들 (줄바꿈 구분, <map_dir>/<이름>/rtabmap.db 있는 것)
//           2초마다 확인 · 바뀔 때만 발행 → 시뮬레이터 지도 목록 (실기에 붙여도 로봇에 있는 지도를 보여 줌)
//     입력: ✉️ /cmd/map_name (std_msgs/String 지도 폴더 이름) — 시뮬레이터 지도 목록에서 고름
//           → <map_dir>/<이름>/rtabmap.db 있으면 📞 /rtabmap/rtabmap/load_database (clear=false) → set_mode_localization
//           (고른 지도는 고정해서 쓰는 것 — 위치추정. 두 응답 뒤에만 /state/map_name · /state/map_mode 바꿈)
//     출력: ✉️ /state/map_name (std_msgs/String, transient_local) — 지금 쓰는 지도 폴더 이름 (파라미터 map_name = launch가 정한 RTAB-Map DB 폴더,
//           빈 값 = 폴더 없음) → 시뮬레이터 [지도 저장]이 같은 폴더에 map.pgm·yaml
//   재시작 (실기 bringup · 테스트베드 같은 길 — 예전엔 테스트베드 test-faker 0 셸만 받아서 실기엔 받는 쪽이 없었음)
//     입력: ✉️ /cmd/restart (std_msgs/String)
//           "node <이름>"         그 노드 프로세스만 — 명령줄·환경·작업 폴더를 /proc에서 그대로 읽어 끄고 같은 것으로 다시 (파라미터 유지)
//                                 이름 = __node:= 리매핑, 없으면 실행 파일 이름(<이름> · <이름>_node). 이 노드를 띄운 launch 아래에서만 찾음
//                                 (Nav2 수명주기 노드는 다시 떠도 unconfigured — lifecycle_manager가 다시 활성화해야 함)
//           "all [이름:=값 …]"    이 노드를 띄운 ros2 launch 전체 — 그 launch 명령줄 그대로 (인자를 주면 그 인자만 바꿈, map:=을 바꾸면 map_name:= 버림)
//           끄고 띄우는 건 분리된(setsid) 도우미 — 이 노드가 꺼져도 끝까지 (restart.hpp). 같은 요청이 두 번 안 오게 하는 건 보내는 쪽
//           (테스트베드 input_manager는 읽은 요청 파일을 지움). 도는 중(restart_marker 있음)엔 새 요청 거절
//     출력: ✉️ /state/restartable (std_msgs/String, transient_local) — node로 재시작할 수 있는 노드 "이름 pid" 줄들 (2초마다 · 바뀔 때만)
//           → 시뮬레이터 진단 칸이 누를 수 있는 노드로 표시 (컨트롤러 · costmap 하위 · 내부 노드는 프로세스가 아니라 없음)
//     출력: ✉️ /state/restart (std_msgs/String, transient_local) — 마지막 요청 결과 ("all: 재시작 중 …" · "node perception: 재시작 (pid …)" · "거절: …")
//   연결 장치 (센서·구동기 — 케이블을 나중에 꽂아도 state_manager는 계속 상태를 낸다)
//     출력: ✉️ /state/devices (std_msgs/String, transient_local) — "장치1,장치2,..." 쉼표 구분 이름 목록
//     출력: ✉️ /state/devices/<장치> (std_msgs/String, transient_local) — "종류|active|disconnected|세부 토픽·관절"
//           · pole_front_cam·gripper_cam = camera_info가 최근 1초 안에 실제로 흐르는지
//           · opencr = /joint_states가 최근 1초 안에 실제로 흐르는지
//           · wheels / arm / gripper = /joint_states의 관절 이름 + 최근성. 그 관절 모터에 Hardware Error(과부하·과열 등)가 있으면
//             상태 error · 세부 끝에 "joint2:과부하"처럼 — ✉️ /motor_status (control/MotorStatus, BeebotSystemHardware — 펌웨어 E 줄)
//             (진단 /diagnostics는 사람이 보는 것 — 노드가 진단 문구에 기대지 않게 전용 토픽을 읽음)
//             재부팅: /cmd/manual/reboot_motors (manual_control)
//           · imu = /imu가 최근 1초 안에 실제로 흐르는지
//     출력: ✉️ /state/topics (std_msgs/String, transient_local) — 현재 ROS 그래프의 "토픽|타입|publisher 수" 전체 목록
//           → 장치 목록에 없는 새 센서도 여기서 찾아 세부 토픽을 확인할 수 있다.
//     파라미터 restart_prepare: all 때 다시 띄우기 전에 실행할 셸 명령 (테스트베드 faker = build-ws — 고친 코드 반영, 실기 = 없음)
//             restart_log: all 때 다시 띄운 launch 출력 파일 · restart_marker: 도는 동안 있는 파일 (테스트베드 test-faker 0이 보고 기다림)
//             restart_wait_s: 정상 종료를 기다리는 시간 (RTAB-Map이 rtabmap.db를 닫을 시간)
//             restart_registry: node로 따로 다시 띄운 노드 기록 (launch 자식이 아니라 all이 놓치던 것 — all 때 같이 끄고 비움)
//   ✉️ /diagnostics
#include <chrono>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "control/msg/motor_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rtabmap_msgs/srv/load_database.hpp"
#include "std_srvs/srv/empty.hpp"

namespace manager {

class StateManager : public rclcpp::Node {
public:
  explicit StateManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void request_operation(const std::string & mode);
  void set_operation_mode(const std::string & mode);  // 판단 끝난 조작 모드를 바꾸고 발행
  void set_estop(bool engaged);
  void request_map_mode(const std::string & mode);
  void request_map_name(const std::string & name);
  void publish_map_list();
  void publish(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, const std::string & value);
  void request_restart(const std::string & text);
  void publish_restartable();
  void publish_devices();
  void publish_topics();
  bool joints_fresh() const;
  bool recent(const std::string & key) const;

  std::string operation_ = "AUTO", map_mode_, map_name_, map_dir_, map_list_, map_pending_, map_error_, explore_note_;
  std::string restart_prepare_, restart_log_, restart_marker_, restart_registry_, restart_note_, restartable_;
  bool restartable_published_ = false;
  // 연결된 장치·토픽 상태 (/state/devices · /state/topics)
  std::string devices_;
  std::map<std::string, std::string> device_status_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> device_pubs_;
  std::string topics_;
  bool devices_published_ = false, topics_published_ = false;
  std::set<std::string> joint_names_;
  std::chrono::steady_clock::time_point last_joint_time_{};
  std::map<std::string, std::chrono::steady_clock::time_point> last_seen_;
  std::map<std::string, std::string> motor_error_;  // 관절 → 에러 이름 (정상이면 없음) — /motor_status
  rclcpp::Subscription<control::msg::MotorStatus>::SharedPtr motor_sub_;
  rclcpp::TimerBase::SharedPtr restartable_timer_;
  double restart_wait_s_ = 25.0;
  bool map_list_published_ = false;
  bool estop_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_pub_, map_pub_, map_name_pub_, map_list_pub_, restart_pub_, restartable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr devices_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topics_pub_;
  rclcpp::TimerBase::SharedPtr map_list_timer_, device_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr op_req_sub_, map_req_sub_, map_name_sub_, explore_sub_, restart_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr front_cam_info_sub_, gripper_cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mapping_cli_, localization_cli_;
  rclcpp::Client<rtabmap_msgs::srv::LoadDatabase>::SharedPtr load_db_cli_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace manager
