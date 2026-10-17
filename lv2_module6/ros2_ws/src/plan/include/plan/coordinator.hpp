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
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

namespace plan {

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

  // ---------- pick: octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보 순서대로 move_group (규칙 4)
  void pick(const SubGoal & g);

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i);

  // ---------- 🎬 move_group: 접근 → 벌리기 → 파지 → 닫기 → 들어올리기
  void execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done);

  // ---------- 🎬 move_group: 위로 이동 → 벌리기
  void execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done);

  // ---------- 🎬 /plan_kinematic_path + /execute_trajectory
  void move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done);

  // ---------- 🎬 /gripper_controller/gripper_cmd (m)
  void set_gripper(double width_m, std::function<void(bool)> done);

  // ---------- 새 계획·직접 목표가 들어오면 진행 중 goal 전부 취소 (옛 결과가 새 단계를 건너뛰지 않게)
  void cancel_all();

  // ---------- "x,y" (map 좌표) 목표 파싱 — navigate 직접 지정용
  static bool parse_point(const std::string & target, double & x, double & y);

  // ---------- 출력
  void status(uint32_t step, uint8_t state, const std::string & reason);

  void say(const std::string & text);

  int max_grasp_tries_, max_step_failures_;
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
