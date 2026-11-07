#pragma once

// 서비스·액션 입력 브릿지: monitor(토픽→파일)·service_action_monitor(목록→파일)의 반대 방향.
// 호스트(시뮬레이터, ROS 없음)가 파일로 서비스 호출·액션 goal을 보낸다.
//
//   debug/input/service/<이름>/message   Host→ ({"request_id": "1015-001", "data": {...}}) — mtime 변경마다 1회 호출
//   debug/output/service/<이름>/message  ← {"request_id": "...", "ok": true, "data": {...}} | {"ok": false, "error": "..."}
//   debug/input/action/<이름>/goal       Host→ ({"goal_id": "...", "data": {...}})
//   debug/input/action/<이름>/cancel     Host→ ({"goal_id": "..."})
//   debug/output/action/<이름>/accept    ← {"goal_id": "...", "accepted": true} — goal 수락 ack
//   debug/output/action/<이름>/feedback  ← {"goal_id": "...", "data": {...}} — 최신 1건 (스트리밍)
//   debug/output/action/<이름>/result    ← {"goal_id": "...", "status": "succeeded|aborted|canceled", "data": {...}}
//   debug/input/<service|action>/<이름>/history — 요청·응답을 차례로 쌓음 (feedback 제외, 지우지 않음):
//     >>request:2026-10-18 17:02:11>>      (action: >>goal · >>cancel)
//     {...요청 JSON}
//     <<response:2026-10-18 17:02:12<<    (action: <<accept · <<result)
//     {...응답 JSON}
//   (토픽과 같은 규칙: input/ = 호스트 → ROS, output/ = ROS → 호스트)
//   ROS 그래프에 떠 있는 서비스·액션마다 input/<service|action>/<이름>/ 폴더를 2초마다 만든다 — 아래 목록에 없는 것(타입 미등록)에
//   요청이 오면 '지원 안 함' 오류로 답한다 (조용히 버리지 않음)
//
// 타입 지정 방식 (제네릭 아님 — 프로젝트에 쓰는 것만, 확정 동작):
//   service: /score_grasps · /clear_octomap · /apply_planning_scene · /diff_drive_controller/set_odometry · /reset(EKF)
//            /fake_opencr/set_parameters (시뮬레이터 환경 버튼 — 미끄럼·마찰·통신 끊김·팔 붙잡기)
//            /map_saver/save_map (지도 저장 — ros2_ws/map/map.pgm·yaml)
//   action:  /navigate_to_pose · /execute_task_solution
//            /move_action (MoveIt MoveGroup — 시뮬레이터 수동 조작 [end_effector 이동]: 손끝 위치 x·y·z → 계획+실행)
//              goal data: {"x","y","z"[, "frame_id"=base_footprint, "link"=end_effector_link, "group"=arm,
//                          "orientation"=[x,y,z,w] (기본 위→아래 [1,0,0,0]), "velocity_scaling"=0.3, "plan_only"=false]}
//              방향 허용: x·y ±0.15rad · 수직축 자유 (coordinator pick과 같게), 위치 허용 1cm 상자
//              "yaw"(rad) = 손끝 x축의 수평 방위 고정 (손가락은 손끝 y축 = yaw−90°로 벌어짐) · 허용 "yaw_tolerance"(기본 0.05)
//              또는 관절 목표 {"joints": {"joint1": rad, …}[, "velocity_scaling", "plan_only"]} — 시뮬레이터 관절 슬라이더 (MoveIt이 충돌 없는 경로로)
// 시작 전 잔재 파일은 무시, 서비스·액션마다 1건씩 순차 처리(처리 중 새 요청은 busy 에러).
// 파라미터: debug_dir(기본 /debug), period_s(폴링 간격, 기본 0.2), timeout_s(기본 10.0)
#include <chrono>
#include <functional>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <memory>
#include <string>

#include "control_msgs/srv/set_odometry.hpp"
#include "ml/srv/score_grasps.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_task_constructor_msgs/action/execute_task_solution.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

namespace faker {

namespace fs = std::filesystem;

class ServiceActionInput : public rclcpp::Node {
public:
  ServiceActionInput();

private:
  // --- 공용 파일 helpers
  static fs::path svc_dir(const std::string & root, const std::string & name);
  static void write_json(const fs::path & out, const std::string & text);
  void reply(const std::string & kind, const std::string & name, const std::string & what, const std::string & text);
  void make_dirs();
  void history(const std::string & kind, const std::string & name, const std::string & tag, const std::string & text);
  static bool changed(const fs::path & file, fs::file_time_type & last);
  static std::string read_all(const fs::path & file);
  static std::string get_str(const rapidjson::Value & v, const char * key, const std::string & dflt = "");

  void poll();

  // --- 서비스 1건 처리 (성공하면 true: request 소진)
  void poll_service(const std::string & name);
  void check_service(const std::string & name);

  // --- 액션 1건 처리
  void poll_goal(const std::string & name);
  void poll_cancel(const std::string & name);
  void check_action(const std::string & name);

  // --- 타입별 변환 (data JSON ↔ ROS)
  bool make_score_request(const rapidjson::Value & data, ml::srv::ScoreGrasps::Request & req, std::string & err);
  std::string score_response_json(const std::string & id, ml::srv::ScoreGrasps::Response::SharedPtr res);
  bool make_nav_goal(const rapidjson::Value & data, nav2_msgs::action::NavigateToPose::Goal & goal, std::string & err);
  std::string nav_feedback_json(const std::string & id,
      nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr fb);
  std::string nav_result_json(const std::string & id,
      const typename rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::WrappedResult & wr);
  bool make_mtc_goal(const rapidjson::Value & data,
      moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal & goal, std::string & err);
  std::string mtc_result_json(const std::string & id,
      const typename rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::WrappedResult & wr);
  bool make_move_goal(const rapidjson::Value & data, moveit_msgs::action::MoveGroup::Goal & goal, std::string & err);
  std::string move_result_json(const std::string & id,
      const typename rclcpp_action::Client<moveit_msgs::action::MoveGroup>::WrappedResult & wr);

  std::string debug_dir_;
  std::string map_mode_;  // /state/map_mode — MAPPING이면 /move_action(팔) 거절: 지도 만들 땐 팔이 스캔 자세에 고정
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_mode_sub_;
  double timeout_s_;
  rclcpp::TimerBase::SharedPtr timer_, dirs_timer_;
  std::map<std::string, fs::file_time_type> unsupported_;  // "service:<이름>" · "action:<이름>" → 요청 파일 mtime (타입 미등록)

  // 서비스 상태 (이름별 1건)
  struct SvcState {
    fs::file_time_type mtime{};
    std::string request_id;
    std::chrono::steady_clock::time_point deadline{};
    bool busy = false;
    bool timed_out = false;
    std::function<bool(std::string &)> done;  // 응답 왔으면 body(data JSON) 채우고 true — call()로 보낸 서비스
  };
  // 타입 있는 클라이언트로 보내고 응답을 data JSON으로 바꿀 함수를 st.done에 건다 (서버 없으면 false)
  template <class Srv>
  bool call(SvcState & st, typename rclcpp::Client<Srv>::SharedPtr cli, typename Srv::Request::SharedPtr req,
            std::function<std::string(const typename Srv::Response &)> to_json) {
    if (!cli->wait_for_service(std::chrono::seconds(0))) return false;
    auto fut = cli->async_send_request(req).future.share();
    st.done = [fut, to_json](std::string & body) {
      if (fut.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return false;
      body = to_json(*fut.get());
      return true;
    };
    return true;
  }
  std::map<std::string, SvcState> svc_;

  rclcpp::Client<ml::srv::ScoreGrasps>::SharedPtr score_cli_;
  std::shared_future<ml::srv::ScoreGrasps::Response::SharedPtr> score_fut_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_cli_;
  std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> clear_fut_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_cli_;
  std::shared_future<std_srvs::srv::Empty::Response::SharedPtr> reset_fut_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr scene_cli_;
  std::shared_future<moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr> scene_fut_;
  rclcpp::Client<control_msgs::srv::SetOdometry>::SharedPtr odom_cli_;
  std::shared_future<control_msgs::srv::SetOdometry::Response::SharedPtr> odom_fut_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr opencr_params_cli_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_cli_;

  // 액션 상태 (이름별 1 goal)
  struct ActState {
    fs::file_time_type goal_mtime{};
    fs::file_time_type cancel_mtime{};
    std::string goal_id;
    bool waiting_goal = false;  // send_goal 응답 대기
    bool active = false;        // result 대기
    std::chrono::steady_clock::time_point deadline{};
    std::chrono::steady_clock::time_point last_fb{};
  };
  std::map<std::string, ActState> act_;

  using NavClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
  using MtcClient = rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>;
  NavClient::SharedPtr nav_cli_;
  NavClient::GoalHandle::SharedPtr nav_handle_;
  std::shared_future<NavClient::GoalHandle::SharedPtr> nav_goal_fut_;
  std::shared_future<NavClient::WrappedResult> nav_res_fut_;
  MtcClient::SharedPtr mtc_cli_;
  MtcClient::GoalHandle::SharedPtr mtc_handle_;
  std::shared_future<MtcClient::GoalHandle::SharedPtr> mtc_goal_fut_;
  std::shared_future<MtcClient::WrappedResult> mtc_res_fut_;
  using MoveClient = rclcpp_action::Client<moveit_msgs::action::MoveGroup>;
  MoveClient::SharedPtr move_cli_;
  MoveClient::GoalHandle::SharedPtr move_handle_;
  std::shared_future<MoveClient::GoalHandle::SharedPtr> move_goal_fut_;
  std::shared_future<MoveClient::WrappedResult> move_res_fut_;
};

}  // namespace faker
