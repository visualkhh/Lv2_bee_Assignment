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
//   ✉️ /diagnostics
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
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

  std::string operation_ = "AUTO", map_mode_, map_name_, map_dir_, map_list_, map_pending_, map_error_, explore_note_;
  bool map_list_published_ = false;
  bool estop_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_pub_, map_pub_, map_name_pub_, map_list_pub_;
  rclcpp::TimerBase::SharedPtr map_list_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr op_req_sub_, map_req_sub_, map_name_sub_, explore_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mapping_cli_, localization_cli_;
  rclcpp::Client<rtabmap_msgs::srv::LoadDatabase>::SharedPtr load_db_cli_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace manager
