#pragma once

// 팔 skill 공통 손발 — MoveIt(계획 + 실행) · 그리퍼 · 관절 궤적 · 닿는 구간(시뮬 기준 동작 + 학습 보정) · 잡기 판정 · 처짐 리셋
//   팔 skill 노드(pose · inspect · pick · place · load · unload · stand)가 하나씩 가짐. 옮겨 온 곳: 예전 coordinator (11-04 skill 노드로 나눔)
//   닿는 구간: plan()이 정한 above(지금 자세) → down → 쥐기/놓기 → above, 시뮬 place_env·pick_env와 같은 단계·시간
//     (정지 0.3 s · 팀 한계 0.3배 부드러운 보간 · 안정 0.3 s · 쥐기 0.5 s / 놓기 0.4 s · 대기). 단계마다 /skill_context에
//     skill 이름 + 그 skill 노드의 정책 파일(SkillModel) → rl_arm_correction이 그 정책으로 Δ를 계산 (100 Hz 루프 안). 끝나면 skill "" (보정 끔)
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "control/msg/arm_safety_event.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ml/msg/skill_context.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "perception/msg/hand_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "skills/model.hpp"
#include "skills/stand_plan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace skills {

// 잡기 판정 — 위치(손가락이 퍽에 걸려 덜 닫힘) ≥ min_pos · 전류 |effort| ≥ min_effort (effort가 NaN이면 위치만)
bool grasp_held(double pos, double effort, double min_pos, double min_effort);
// 위치형 쥐기 판정 (손가락 위치·전류, 50 ms마다): 1 쥠 (0.2 s 안 움직임 · 전류 ≥ min_current) · 0 빈손 (0.2 s 안 움직임 · 위치 < empty_m)
//   −1 timeout_s 넘음 · 2 아직. 그리퍼 액션 결과는 덜 닫혀 멈춰도 성공이라 안 믿음 (실기 11/03)
// 위→아래 손 자세 Rz(ψ)·Rx(π) = (cos ψ/2, sin ψ/2, 0, 0)에서 ψ를 꺼냄. 그 꼴이 아니면 false
bool hand_yaw(const geometry_msgs::msg::Quaternion & q, double & psi);
int grip_state(double pos, double current, double still_s, double t, double min_current, double empty_m, double timeout_s);

constexpr double GRIPPER_OPEN_M = 0.0265;   // URDF 한계
constexpr double GRIPPER_CLOSED_M = 0.0;
constexpr double TRAJ_DT = 0.02;            // 궤적 샘플 간격 (학습 때 정책 주기)
constexpr double CONTACT_ABOVE_M = 0.05;    // 시뮬 above_m (손끝이 내려갈 자리 위)

class ArmIO {
public:
  using Cb = std::function<void(bool)>;
  using Step = std::function<void(Cb)>;
  using Q5 = stand::Q5;

  explicit ArmIO(rclcpp::Node & node);
  bool moveit_ready() const;
  bool gripper_ready() const;
  bool sim_contact() const { return contact_mode_ == "sim"; }
  double grip_current_a() const;  // 닫을 때마다 읽음 — ros2 param set /skill_<이름> grip_current_a 0.2
  // 닫는 목표 gripper_left_joint — 끝(0)까지가 아니라 퍽 폭만큼 (grip_close_gap_m · 실행 중 ros2 param set으로 바뀜).
  //   끝까지 닫으면 퍽에 막힌 채 grip_current_a로 계속 누름 (11/08 실기: 30 mm 퍽에서 3.25 mm까지 · 0.36 A)
  double grip_close_m() const;
  double velocity_scale() const;  // arm_velocity_scale 파라미터 (MoveIt 속도·가속도 비율)
  // 진행 중 MoveIt·그리퍼 goal 취소 · 늦은 결과 버림 · 보정 끔 (skill 취소 · 새 goal 전)
  void stop();
  void clear_flags() { contact_ = false; grasp_empty_ = false; grip_fail_.clear(); }
  bool contact() const { return contact_; }          // 이번 실행 중 /arm_safety_event (과전류 접촉)
  bool grasp_empty() const { return grasp_empty_; }  // 마지막 잡기 판정이 빈손
  const std::string & grip_fail() const { return grip_fail_; }  // grip_settle_step 실패 이유

  // ---------- MoveIt: 손끝(end_effector_link) 목표 — 손은 늘 아래를 봄, yaw를 주면 손 방향도 고정 (±0.15 rad)
  //   계획이 실패하면 같은 목표로 arm_plan_tries번까지 (IK·OMPL이 무작위라 닿는 곳도 가끔 못 찾음)
  void move_arm(const geometry_msgs::msg::PoseStamped & target, Cb done,
                double yaw = std::numeric_limits<double>::quiet_NaN(), int tries_left = -1);
  void move_joints(const std::vector<double> & q5, Cb done);  // joint1~5 관절 목표
  // 그리퍼 (m, 손가락 하나) · effort_a [A] = 쥐는 힘 (goal effort → Goal Current, 음수 = controllers.yaml max_effort)
  void set_gripper(double width_m, Cb done, double effort_a = -1.0);

  // ---------- 단계 (차례로 run_steps — 하나라도 실패하면 done(false))
  void run_steps(std::shared_ptr<std::vector<Step>> steps, size_t i, Cb done);
  Step joints_step(const std::vector<double> & q5);
  Step gripper_step(double width_m, double effort_a = -1.0);
  Step held_step();                                       // 잡기 판정 (grasp_check 끄면 통과)
  Step traj_step(std::function<std::vector<Q5>(const Q5 &)> make);  // 지금 관절값 + 샘플(TRAJ_DT 간격) → /execute_trajectory
  Step grip_timed_step(double joint_m, double effort_a, double secs);  // 보내고 secs 뒤 다음 (가볍게 쥐면 '도착'을 안 할 수 있음)
  Step wait_step(double secs);
  // grip_state가 정해질 때까지 기다림 — 빈손 stand_no_grip · 시간 초과 stand_grip_timeout으로 실패 (grasp_check 끄면 통과)
  Step grip_stop_step(double timeout_s);  // 손가락이 멈출 때까지 기다림 (시간이 넘어도 다음으로 — 판정은 held_step)
  Step grip_settle_step(double min_current, double empty_m, double timeout_s);
  // open_m: close=false일 때 놓으며 여는 gripper_left_joint [m] (load는 간격 38 mm)
  // lift_on_empty: 빈손 판정이어도 위로 올린 뒤 실패 (unload — 칸 안에 남지 않게). false면 내려간 채 실패 (pick — 잘못 판정해도 퍽을 박스 위에서 놓음)
  Step contact_step(const SkillModel & policy, std::function<bool(Q5 &, Q5 &)> plan, bool close,
                    double open_m = GRIPPER_OPEN_M, bool lift_on_empty = false);

  // ---------- 보정 · 입력
  void skill_ctx(const SkillModel * policy, uint8_t phase, bool holding, bool live, const Q5 & above, const Q5 & down);
  void ctx_off();
  // hand_relative에 무엇을 잴지 (puck: 퍽 태그 · cell: 칸 가운데 map · slot: 적재함 칸 · none)
  void hand_target(const std::string & kind, int puck_tag, int slot, double x = 0.0, double y = 0.0, double z = 0.0);
  // /rl_arm_correction/reset_sag — 팔 skill 시작 때 한 번 (없으면 WARN 한 번).
  //   보정이 0이 되면 팔이 처졌다 다시 올라옴 → 다음 move_arm·move_joints·traj_step은 팔이 멈출 때까지 기다린 뒤 계획 (settle)
  void reset_sag();
  // 다음 move_arm·move_joints·traj_step은 팔이 멈출 때까지 기다린 뒤 (settle) — 앞 skill이 끝나며 보정 Δ가 0으로 빠져
  //   팔이 ~0.1 s 움직이는 동안 궤적을 보내면 start point deviates. 처짐 적분은 안 건드림 (닿는 구간 시작에서 rl_arm_correction이 0으로)
  void settle_next() { settle_ = true; }
  bool arm_now(Q5 & q) const;  // /joint_states joint1~5
  // map(등) 손끝 목표 → 손 아래 해석 IK (near의 손목 방향 그대로) — base_footprint 기준
  bool ik_ee(const geometry_msgs::msg::PoseStamped & ee, const Q5 & near, Q5 & out);
  // 손 방향 ψ까지 정한 해 — 손가락 닫는 축 = ψ − 90° (ee 좌표계). MoveIt pick_ik는 위치만 맞춰 손 방향을 보장 못 함
  bool ik_ee_yaw(const geometry_msgs::msg::PoseStamped & ee, double psi, const Q5 & near, Q5 & out);
  bool joint(const std::string & name, double & pos) const;
  tf2_ros::Buffer & tf() { return *tf_buffer_; }

private:
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GetMotionPlan = moveit_msgs::srv::GetMotionPlan;
  using Gripper = control_msgs::action::ParallelGripperCommand;
  rclcpp::Node & node_;
  int arm_plan_tries_ = 3;
  double grip_current_a_ = 0.1, grasp_min_pos_m_ = 0.003, grasp_min_effort_ = 0.25;
  bool grasp_check_ = true, contact_ = false, grasp_empty_ = false, sag_warned_ = false;
  std::string contact_mode_ = "sim", grip_fail_;
  uint64_t gen_ = 0;
  uint32_t run_ = 0;
  std::map<std::string, double> pos_, effort_;
  rclcpp::Client<GetMotionPlan>::SharedPtr plan_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_sag_;
  rclcpp::Publisher<ml::msg::SkillContext>::SharedPtr ctx_pub_;
  rclcpp::Publisher<perception::msg::HandTarget>::SharedPtr hand_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<control::msg::ArmSafetyEvent>::SharedPtr safety_sub_;
  rclcpp::TimerBase::SharedPtr timer_, settle_timer_;
  bool settle_ = false;  // reset_sag 뒤 아직 안 기다림
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void execute(const moveit_msgs::msg::RobotTrajectory & traj, Cb done);
  void settle(std::function<void()> go);
};

}  // namespace skills
