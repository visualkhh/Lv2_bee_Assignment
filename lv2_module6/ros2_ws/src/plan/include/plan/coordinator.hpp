#pragma once

// coordinator (🛠 직접 작성, 담당: 권형중) — 설계: documents/project/ros/03-plan.md "coordinator가 지키는 실행 규칙"
//   입력: /task/sub_goals (VLM 계획), /tracked_objects (대상 id 검증), /arm_safety_event (팔 접촉)
//   출력: 🎬 /navigate_to_pose (Nav2), 🎬 /plan_kinematic_path + /execute_trajectory (move_group),
//         🎬 /gripper_controller/gripper_cmd, 📞 /score_grasps (grasp_scorer),
//         📞 /clear_octomap · /apply_planning_scene (move_group), ✉️ /task/status → VLM·UI, ✉️ /robot/say → user_input
// 규칙 (골격에 들어간 것):
//   1. VLM 출력 검증 — 허용 동작(navigate·pick·place·inspect·say), 숫자 대상 id가 /tracked_objects에 있는지
//   inspect <퍽 id> (신호등 암호 미션 — 퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘): 멀면 다가가고 손끝을 퍽 윗면 위
//     inspect_height_m로 (손가락 아래) → inspect_wait_s 기다림 → 다음 단계. 읽었는지(details tag)는 planner가 /tracked_objects로 본다
//   4. 파지 재시도는 coordinator 몫 — grasp_scorer 후보를 점수순으로 최대 max_grasp_tries(2)개, 다 실패하면 FAILED
//   5. 같은 단계 2회 연속 실패면 /robot/say로 알리고 정지
//   적재함 (MISSION ③ MOVE_TO_B — 퍽을 실어 운반): target "tray"
//     place tray = 첫 빈 칸에 싣기 · pick tray = 마지막 찬 칸에서 꺼내기 (칸 = tray_slots, base_footprint 기준 — MoveIt 실측으로
//     손끝 아래 방향이 닿는 곳). "tray:<칸>"이면 그 칸 (mission_manager가 칸 → tag를 기억). 칸마다 찼는지 기억 (노드가 다시 뜨면 빔).
//     꽉 참·그 칸이 참이면 tray_full · 빔이면 tray_empty · 칸 번호가 틀리면 tray_slot_bad로 실패
//   팔이 닿게 다가가기: pick · place <물체 id> 차례에 대상이 로봇(base_footprint, TF)에서 reach_m 넘게 떨어져 있으면
//     먼저 그 물체를 바라보며 approach_m 떨어진 자리로 navigate한 뒤 그 단계를 실행 (단계마다 한 번 — VLM이 navigate를 빼먹어도).
//     navigate <물체 id>도 같은 접근 자세 — 로봇이 있는 쪽에서 물체를 바라봄 (예전: 늘 물체의 −x 쪽 · yaw 0)
//   시작 자세: bringup 직후 한 번 팔을 initial_pose(plan.yaml)로 — MoveIt 없이 /arm_controller/joint_trajectory 토픽
//   스캔 자세: 지도를 만들 때(/state/map_mode MAPPING · 자동 탐사 EXPLORE) 작업이 없으면 팔을 scan_pose로 (2초마다 확인, MoveIt 관절 목표)
//     — gripper_cam이 왼쪽 뒤를 보는 두 번째 고정 카메라가 됨 (ros_system/config/scan_pose.yaml · scan_cam_gate). 수동 모드여도
//     (스캔은 보통 수동 주행) — MAPPING 동안 팔 수동 조작은 manual_control(/cmd/manual/arm)·service_action_input(/move_action)이 거절
//   같은 물체(구역) 위에 여러 번 place하면 겹치지 않게 place_spacing 간격으로 비켜 놓는다 (B 지역 판에 퍽 여러 개)
//   좌표 대상 (mission_manager — 주사위 칸처럼 물체가 아닌 자리, map 기준 "x,y" · "x,y,z" · "x,y,yaw"):
//     navigate "x,y[,yaw]" — 그 자세로 (yaw 없으면 0) · place "x,y,z" — 퍽 바닥이 z가 되게 내려놓음
//       (위 place_approach_m → 퍽 바닥 + place_release_dz까지 내려 벌림 → 다시 위로. 다가가기는 안 함 — 부르는 쪽이 먼저 navigate)
//     inspect "x,y,z" — 그 점 위 inspect_height_m에서 gripper_cam으로 내려다봄 (시작 박스 위 퍽 ID·모양 · 재검증)
//   approach "x,y,yaw" (map) — 박스 앞 마지막 몇 cm: Nav2는 로봇을 원(0.18)으로 봐서 못 붙는 자리까지 저속으로 붙음.
//     목표 뒤 approach_pre_m 자리로 가서(돌고 → 가고) 목표 방향으로 돈 뒤 직선 전진 — 박스 바로 앞에선 제자리 회전 안 함 (앞 모서리가 닿음).
//     이미 목표 방향(±0.15 rad)이고 옆 어긋남 3 cm 안이면 바로 직선 (전진·후진 모두 — 박스에서 똑바로 물러날 때도 이것)
//     /cmd_vel_nav로 냄 (Nav2와 같은 입구 → twist_mux 비상정지·collision_monitor 그대로). 진행이 멈추면 approach_blocked
//   pose "scan" | "initial" — 팔 관절 자세 (scan = 팔을 왼쪽 뒤로 접어 pole_front_cam 앞을 비움 — 신호등·바코드 관측)
//   AUTO가 아닌 모드(/state/operation_mode = MANUAL 수동 · EXPLORE 자동 탐사): 진행 중 이동·팔 goal을 취소하고 FAILED(manual_override · explore_override),
//     그동안 새 계획은 받지 않는다 (VLM은 manual_* · explore_* 실패에는 재계획하지 않음). AUTO로 돌아오면 새 명령을 기다린다.
//   상태: /diagnostics (모드 · 계획 · 단계 · 실패 횟수)
// TODO(권형중): 2. 이동 전 팔 stow·퍽을 든 채 footprint 확장, 3. 정지 후 0.5초 대기, map→odom TF 대기,
//               접근 자세 계산, MTC 태스크 구성(접근·열기·파지·attach·들어올리기), 장면에 퍽·장애물 등록
#include <chrono>
#include <limits>
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
#include "geometry_msgs/msg/twist_stamped.hpp"
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
#include "trajectory_msgs/msg/joint_trajectory.hpp"
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
// "x,y[,z|yaw]" (map 좌표) 목표 파싱 — 쉼표로 나눈 숫자 2개 이상 (숫자가 아닌 게 섞이거나 1개면 빈 목록)
std::vector<double> parse_numbers(const std::string & target);

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
  bool robot_pose(double & x, double & y, double & yaw) const;
  // ---------- approach: 정밀 직선 접근 (20 Hz 루프)
  void approach(const SubGoal & g);
  void approach_tick();
  void stop_base();
  // pick·place <물체 id> 전에: 대상이 reach_m 밖이면 다가가는 navigate를 시작하고 true (끝나면 같은 단계를 다시) · 아니면 false
  bool approach_if_far(const SubGoal & g);

  // ---------- pick: octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보 순서대로 move_group (규칙 4)
  void pick(const SubGoal & g);

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i);

  // ---------- 🎬 move_group: 접근 → 벌리기 → 파지 → 닫기 → 들어올리기
  void execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done, double approach = 0.10);

  // ---------- 🎬 move_group: 위로 이동 → 벌리기
  void execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, double approach = 0.10);
  // ---------- 🎬 move_group: 위 → 내려가 벌리기 → 다시 위 (좌표 place — 퍽을 떨어뜨리지 않고 정확한 자리에)
  void place_at(const geometry_msgs::msg::PoseStamped & release, std::function<void(bool)> done);

  // ---------- inspect: 퍽 위에서 gripper_cam으로 내려다보기 (윗면 태그 읽기)
  void inspect(const SubGoal & g);

  // ---------- 적재함: 칸 위치 (base_footprint) 손끝 목표
  geometry_msgs::msg::PoseStamped tray_pose(size_t slot, double z) const;

  // ---------- 🎬 /plan_kinematic_path + /execute_trajectory
  //     계획이 실패하면 같은 목표로 arm_plan_tries번까지 (tries_left −1 = 처음)
  //     yaw(map)를 주면 손 방향도 고정 (±0.15 rad) — inspect: gripper_cam이 손끝 앞에 비스듬히 달려 손 방향이 자유면 엉뚱한 곳을 봄
  void move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, int tries_left = -1,
                double yaw = std::numeric_limits<double>::quiet_NaN());

  // ---------- 시작 자세 — bringup 직후 한 번 팔을 initial_pose로 (MoveIt 없이 JTC 토픽)
  //   못 가면 initial_pose_retries번만 다시 보내고 지금 자세로 멈춤 (걸린 채 계속 밀면 과부하)
  //   initial_pose_enabled는 실행 중에도 바꿀 수 있음 (false → 바로 멈춤, true → 다시 시도)
  void check_initial_pose();
  bool at_initial_pose() const;
  std::string initial_pose_error() const;  // 목표와 0.05rad 넘게 다른 관절 "joint2 −0.31" …
  void hold_arm();                          // 지금 각도를 목표로 보내 JTC가 더 밀지 않게
  void publish_arm_targets(const std::vector<std::string> & names, const std::vector<double> & pos, double seconds);

  // ---------- 스캔 자세 — 지도 만드는 중이고 작업이 없으면 팔을 scan_pose로
  void check_scan_pose();
  bool at_scan_pose() const;
  void move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done, bool background = true);

  // ---------- 🎬 /gripper_controller/gripper_cmd (m)
  void set_gripper(double width_m, std::function<void(bool)> done);

  // ---------- 새 계획·직접 목표가 들어오면 진행 중 goal 전부 취소 (옛 결과가 새 단계를 건너뛰지 않게)
  void cancel_all();


  // ---------- 출력
  void status(uint32_t step, uint8_t state, const std::string & reason);

  void say(const std::string & text);

  int max_grasp_tries_, max_step_failures_, arm_plan_tries_;
  double reach_m_, approach_m_;
  double inspect_height_m_, inspect_wait_s_;
  rclcpp::TimerBase::SharedPtr inspect_timer_;
  rclcpp::TimerBase::SharedPtr approach_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  double approach_speed_, approach_pre_m_, approach_pos_tol_, approach_yaw_tol_, approach_timeout_s_;
  double ap_x_ = 0, ap_y_ = 0, ap_yaw_ = 0, ap_best_ = 1e9;
  int ap_stage_ = 0;  // 0 = 앞 자리로 · 1 = 목표 방향으로 돌기 · 2 = 직선 전진
  rclcpp::Time ap_started_{0, 0, RCL_ROS_TIME}, ap_progress_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> initial_joints_;
  std::vector<double> initial_pose_;
  double initial_tol_ = 0.05;
  double initial_move_s_ = 3.0;
  bool initial_enabled_ = true;
  bool initial_waiting_ = false;
  int initial_retries_ = 1;
  double initial_gripper_ = 0.0;      // [m] 음수 = 안 건드림
  bool initial_gripper_sent_ = false;
  int initial_sent_count_ = 0;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  std::chrono::steady_clock::time_point initial_sent_{};
  rclcpp::TimerBase::SharedPtr initial_timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_traj_pub_;
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
  double place_approach_m_, place_release_dz_;  // 좌표 place — 위에서 내려오는 높이 · 퍽 바닥 위 벌리는 손끝 높이
  std::vector<bool> tray_full_;        // 적재함 칸마다 실렸는지 (노드가 다시 뜨면 모두 빔)
  // "tray" | "tray:<칸>" → 칸 번호. 없으면 -1 (싣기: 꽉 참 · 그 칸이 참 / 꺼내기: 빔 · 그 칸이 빔), 형식이 틀리면 -2
  int tray_slot(const std::string & target, bool for_place) const;
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
