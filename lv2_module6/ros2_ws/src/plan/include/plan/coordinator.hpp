#pragma once

// coordinator (🛠 직접 작성, 담당: 권형중) — 설계: documents/ros2-design/03-plan.md "coordinator가 지키는 실행 규칙"
//   입력: /sub_goals (VLM 계획), /tracked_objects (대상 id 검증), /arm_safety_event (팔 접촉)
//   출력: 🎬 /navigate_to_pose (Nav2), 🎬 /plan_kinematic_path + /execute_trajectory (move_group),
//         🎬 /gripper_controller/gripper_cmd, 📞 /score_grasps (grasp_scorer),
//         📞 /clear_octomap · /apply_planning_scene (move_group), ✉️ /task_status → VLM·UI, ✉️ /robot_say → user_input
// 규칙 (골격에 들어간 것):
//   1. VLM 출력 검증 — 허용 동작(navigate·pick·place·say), 숫자 대상 id가 /tracked_objects에 있는지
//   4. 파지 재시도는 coordinator 몫 — grasp_scorer 후보를 점수순으로 최대 max_grasp_tries(2)개, 다 실패하면 FAILED
//   5. 같은 단계 2회 연속 실패면 /robot_say로 알리고 정지
//   적재함 (MISSION ③ MOVE_TO_B — 퍽을 실어 운반): target "tray"
//     place tray = 다음 빈 칸에 싣기 · pick tray = 마지막에 실은 것부터 꺼내기 (칸 = tray_slots, base_footprint 기준 — MoveIt 실측으로
//     손끝 아래 방향이 닿는 곳). 몇 개 실렸는지 tray_count로 센다 (노드가 다시 뜨면 0). 꽉 차면 tray_full · 비면 tray_empty로 실패
//   팔이 닿게 다가가기: pick · place <물체 id> 차례에 대상이 로봇(base_footprint, TF)에서 reach_m 넘게 떨어져 있으면
//     먼저 그 물체를 바라보며 approach_m 떨어진 자리로 navigate한 뒤 그 단계를 실행 (단계마다 한 번 — VLM이 navigate를 빼먹어도).
//     navigate <물체 id>도 같은 접근 자세 — 로봇이 있는 쪽에서 물체를 바라봄 (예전: 늘 물체의 −x 쪽 · yaw 0)
//   스캔 자세: 지도를 만들 때(/state/map_mode MAPPING · 자동 탐사 EXPLORE) 작업이 없으면 팔을 scan_pose로 (2초마다 확인, MoveIt 관절 목표)
//     — gripper_cam이 왼쪽 뒤를 보는 두 번째 고정 카메라가 됨 (ros_system/config/scan_pose.yaml · scan_cam_gate). 수동 모드여도
//     (스캔은 보통 수동 주행) — MAPPING 동안 팔 수동 조작은 manual_control(/cmd/manual/arm)·service_action_input(/move_action)이 거절
//   같은 물체(구역) 위에 여러 번 place하면 겹치지 않게 place_spacing 간격으로 비켜 놓는다 (B 지역 판에 퍽 여러 개)
//   AUTO가 아닌 모드(/state/operation_mode = MANUAL 수동 · EXPLORE 자동 탐사): 진행 중 이동·팔 goal을 취소하고 FAILED(manual_override · explore_override),
//     그동안 새 계획은 받지 않는다 (VLM은 manual_* · explore_* 실패에는 재계획하지 않음). AUTO로 돌아오면 새 명령을 기다린다.
//   상태: /diagnostics (모드 · 계획 · 단계 · 실패 횟수)
// TODO(권형중): 2. 이동 전 팔 stow·퍽을 든 채 footprint 확장, 3. 정지 후 0.5초 대기, map→odom TF 대기,
//               접근 자세 계산, MTC 태스크 구성(접근·열기·파지·attach·들어올리기), 장면에 퍽·장애물 등록
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "control/msg/arm_safety_event.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ml/srv/score_grasps.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

// 적재함 칸 — tray_slots 파라미터 [x0, y0, x1, y1, …] (base_footprint) → 칸 목록 (홀수 개면 마지막 값 버림)
std::vector<std::pair<double, double>> tray_slots(const std::vector<double> & flat);
// 물체를 바라보며 standoff 떨어진 로봇 자세 (map) — 로봇이 있는 쪽에서. 로봇이 물체와 거의 같은 자리면 물체의 −x 쪽
geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff);
// 같은 곳에 n번째(0부터)로 내려놓을 때 중심에서 비킬 (dx, dy) — 가운데 · 좌 · 우 · 앞 · 뒤 · 대각선 순, spacing 간격
std::pair<double, double> spread_offset(int n, double spacing);

using control::msg::ArmSafetyEvent;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using ml::srv::ScoreGrasps;
using moveit_msgs::srv::ApplyPlanningScene;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

class Coordinator : public rclcpp::Node {
public:
  Coordinator();

private:
  // ---------- 계획 받기 · 검증 (규칙 1)
  // ---------- 수동 모드
  void on_mode(const std::string & mode);

  std::string step_text() const;

  void on_plan(const SubGoalArray & plan);

  bool validate(const SubGoalArray & plan, std::string & why) const;

  static bool is_id(const std::string & s);

  const perception::msg::TrackedObject * find(uint32_t id) const;

  // ---------- 단계 실행
  void run_step();

  // Nav2·MoveIt이 아직 안 떴으면 (lifecycle 활성화 중 등) 1초 뒤 같은 단계를 다시. server_wait_s 넘으면 실패.
  bool ready_or_retry(bool ready, const std::string & reason);

  void next();

  // 규칙 5: 같은 단계 max_step_failures회 연속 실패면 알리고 정지. 그 전엔 FAILED만 보내고 VLM 재계획을 기다린다.
  void fail(const std::string & reason);

  static std::string key(const SubGoal & g);

  // ---------- navigate: 🎬 NavigateToPose
  void navigate(const SubGoal & g);
  void send_nav(const geometry_msgs::msg::Pose & pose, std::function<void(bool)> done);
  // 로봇 위치 (map → base_footprint TF). 없으면 false
  bool robot_xy(double & x, double & y) const;
  // pick·place <물체 id> 전에: 대상이 reach_m 밖이면 다가가는 navigate를 시작하고 true (끝나면 같은 단계를 다시) · 아니면 false
  bool approach_if_far(const SubGoal & g);

  // ---------- pick: octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보 순서대로 move_group (규칙 4)
  void pick(const SubGoal & g);

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i);

  // ---------- 🎬 move_group: 접근 → 벌리기 → 파지 → 닫기 → 들어올리기
  void execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done, double approach = 0.10);

  // ---------- 🎬 move_group: 위로 이동 → 벌리기
  void execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, double approach = 0.10);

  // ---------- 적재함: 칸 위치 (base_footprint) 손끝 목표
  geometry_msgs::msg::PoseStamped tray_pose(size_t slot, double z) const;

  // ---------- 🎬 /plan_kinematic_path + /execute_trajectory
  //     계획이 실패하면 같은 목표로 arm_plan_tries번까지 (tries_left −1 = 처음)
  void move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, int tries_left = -1);

  // ---------- 스캔 자세 — 지도 만드는 중이고 작업이 없으면 팔을 scan_pose로
  void check_scan_pose();
  bool at_scan_pose() const;
  void move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done);

  // ---------- 🎬 /gripper_controller/gripper_cmd (m)
  void set_gripper(double width_m, std::function<void(bool)> done);

  // ---------- 새 계획·직접 목표가 들어오면 진행 중 goal 전부 취소 (옛 결과가 새 단계를 건너뛰지 않게)
  void cancel_all();

  // ---------- "x,y" (map 좌표) 목표 파싱 — navigate 직접 지정용
  static bool parse_point(const std::string & target, double & x, double & y);

  // ---------- 출력
  void status(uint32_t step, uint8_t state, const std::string & reason);

  void say(const std::string & text);

  int max_grasp_tries_, max_step_failures_, arm_plan_tries_;
  double reach_m_, approach_m_;
  std::vector<std::string> scan_joints_;
  std::vector<double> scan_pose_;
  double scan_tol_ = 0.05;
  bool scan_moving_ = false;
  std::string map_mode_;
  std::map<std::string, double> joints_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_mode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  size_t approached_ = static_cast<size_t>(-1);  // 다가가기를 이미 한 단계 (idx_) — 단계마다 한 번만
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::pair<double, double>> tray_slots_;
  double tray_release_z_, tray_grasp_z_, tray_approach_m_, place_spacing_;
  int tray_count_ = 0;                 // 적재함에 실린 퍽 수 (칸 0부터 차례로)
  std::map<std::string, int> placed_;  // 계획 안에서 물체(구역) id별로 내려놓은 수 — 겹치지 않게 비킴
  std::string mode_ = "AUTO";
  bool estop_ = false;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  double server_wait_s_, waited_s_ = 0;
  rclcpp::TimerBase::SharedPtr retry_;
  SubGoalArray plan_;
  size_t idx_ = 0;
  bool running_ = false, arm_contact_ = false;
  uint64_t gen_ = 0;  // 새 계획·취소 때마다 증가 — 옛 goal 결과가 새 단계를 건드리지 않게
  std::map<std::string, int> failures_;
  TrackedObjectArray::ConstSharedPtr objects_;
  std::shared_ptr<SubGoalArray> waiting_plan_;
  rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub_;
  rclcpp::Subscription<SubGoalArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<ArmSafetyEvent>::SharedPtr safety_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr nav_goal_;
  rclcpp::Client<GetMotionPlan>::SharedPtr motion_plan_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_;
  rclcpp_action::Client<ParallelGripperCommand>::SharedPtr gripper_;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Client<ApplyPlanningScene>::SharedPtr apply_scene_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
