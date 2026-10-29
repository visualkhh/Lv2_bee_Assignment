#pragma once

// manual_control (🛠 직접 작성, 담당: 김현하) — 수동 조작 (원격 조종 · 팔 직접 움직이기)
//   입력: ✉️ /state/operation_mode (std_msgs/String "AUTO" | "MANUAL" | "EXPLORE" | "TEACH", transient_local) — 모드는 manager/state_manager가 정함 (/cmd/operation_mode → 거기)
//            TEACH = 손으로 자세 가르치기: 들어가면 팔(joint1~5)·그리퍼 토크 끔 → 📞 /beebotsystem/set_arm_torque (std_srvs/SetBool, 펌웨어 T)
//              나가면 JTC·그리퍼 목표를 지금 값으로 먼저 보내고 torque_on_delay_s 뒤 켬 (안 그러면 켜는 순간 예전 목표로 튐).
//              MANUAL은 토크를 늘 켠다. TEACH 동안 수동 바퀴·팔·그리퍼 명령은 버린다 (바퀴 토크는 그대로)
//         ✉️ /cmd/manual/cmd_vel (geometry_msgs/TwistStamped) — 바퀴 (teleop_twist_keyboard · 시뮬레이터 조작판)
//         ✉️ /cmd/manual/arm (sensor_msgs/JointState) — 관절 목표 (joint1~5 rad, gripper_left_joint m). 이름이 있는 것만 움직임
//         ✉️ /cmd/manual/reboot_motors (std_msgs/Empty) — Hardware Error(과부하·과열 등) 난 DYNAMIXEL 재부팅
//            → 📞 /beebotsystem/reboot_motors (std_srvs/Trigger, BeebotSystemHardware — 에러 난 모터만 펌웨어 R)
//         ✉️ /joint_states — 지금 관절각 (목표에 없는 팔 관절은 지금 각으로 채워 5축 전체 궤적을 보낸다 — JTC는 일부 관절 궤적을 거부)
//   출력: ✉️ /cmd_vel_manual (TwistStamped) → twist_mux (자동 /cmd_vel_corrected보다 우선) → velocity_smoother → collision_monitor
//         📞 /collision_monitor/toggle (nav2_msgs/srv/Toggle) — MANUAL이면 끔(통과) · 나가면 켬. 조작자가 보고 모니까 자기 몸 점·근접 정지에
//            막히지 않게. 비상정지 잠금(twist_mux)·가속 제한(velocity_smoother)은 수동도 그대로 거친다.
//            collision_monitor가 늦게 뜨거나 재시작되면(기본 켜짐) 서비스가 다시 보일 때 원하는 상태를 또 보낸다
//         ✉️ /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory) → JTC가 바로 실행 (JTC 표준 토픽 입력)
//         🎬 /gripper_controller/gripper_cmd (control_msgs/action/ParallelGripperCommand)
//         ✉️ /diagnostics
//   수동이 아닐 때 들어온 수동 명령은 버린다. 수동 속도 명령이 cmd_timeout_s 넘게 끊기면 0을 한 번 보낸다
//   (twist_mux도 자체 timeout으로 0으로 떨어지지만, 끊긴 순간을 확실히 멈추려고).
// 수동 바퀴 명령도 velocity_smoother는 지난다. collision_monitor는 MANUAL 동안 꺼 둔다(위 toggle) — 장애물은 조작자가 본다.
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/srv/toggle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace control {

class ManualControl : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  ManualControl();

  bool manual() const;

private:
  void set_mode(const std::string & mode);

  void on_cmd_vel(const TwistStamped & in);

  void watchdog();

  void stop_base();

  void on_arm(const sensor_msgs::msg::JointState & js);

  void send_gripper(double position);

  void on_reboot();

  void torque_on_here();  // TEACH를 나갈 때 — 지금 각을 JTC·그리퍼 목표로 → 지연 뒤 토크 켬

  void call_torque(bool on);

  void sync_collision_monitor();  // 원하는 켬/끔(MANUAL이면 끔)을 collision_monitor에 — 서비스가 새로 보이거나 바뀌었을 때만

  std::vector<std::string> arm_joints_;
  std::string gripper_joint_;
  double arm_move_s_, cmd_timeout_s_, torque_on_delay_s_;
  bool manual_ = false, moving_ = false, teach_ = false;
  std::string mode_ = "-";
  std::string map_mode_;           // /state/map_mode — MAPPING이면 팔 관절 명령을 막음 (지도 만들 땐 팔이 스캔 자세에 고정)
  size_t vel_count_ = 0, arm_count_ = 0, arm_blocked_ = 0, reboot_count_ = 0;
  std::string reboot_result_ = "-";  // 마지막 재부팅 서비스 응답
  std::string torque_result_ = "-";  // 마지막 팔 토크 서비스 응답
  rclcpp::Time last_vel_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_, map_mode_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_sub_, js_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reboot_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reboot_cli_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr torque_cli_;
  rclcpp::TimerBase::SharedPtr torque_timer_;  // 켜기 지연 (한 번)
  rclcpp::Client<nav2_msgs::srv::Toggle>::SharedPtr cm_toggle_;
  int cm_applied_ = -1;     // collision_monitor에 마지막으로 반영된 상태 (1 켬 · 0 끔 · −1 모름 — 서비스가 안 보이면 −1로)
  bool cm_pending_ = false; // 응답 기다리는 중 (3초 넘으면 다시 보냄 — 응답이 버려지는 DDS 설정 대비)
  std::chrono::steady_clock::time_point cm_sent_{};
  std::string cm_result_ = "-";
  std::map<std::string, double> current_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace control
