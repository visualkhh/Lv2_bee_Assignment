#pragma once

// manual_control (🛠 직접 작성, 담당: 김현하) — 수동 조작 (원격 조종 · 팔 직접 움직이기)
//   입력: ✉️ /state/operation_mode (std_msgs/String "AUTO" | "MANUAL", transient_local) — 모드는 manager/state_manager가 정함 (/cmd/operation_mode → 거기)
//         ✉️ /cmd/manual/cmd_vel (geometry_msgs/TwistStamped) — 바퀴 (teleop_twist_keyboard · 시뮬레이터 조작판)
//         ✉️ /cmd/manual/arm (sensor_msgs/JointState) — 관절 목표 (joint1~5 rad, gripper_left_joint m). 이름이 있는 것만 움직임
//         ✉️ /joint_states — 지금 관절각 (목표에 없는 팔 관절은 지금 각으로 채워 5축 전체 궤적을 보낸다 — JTC는 일부 관절 궤적을 거부)
//   출력: ✉️ /cmd_vel_manual (TwistStamped) → twist_mux (자동 /cmd_vel_corrected보다 우선) → velocity_smoother → collision_monitor
//         ✉️ /arm_controller/joint_trajectory (trajectory_msgs/JointTrajectory) → JTC가 바로 실행 (JTC 표준 토픽 입력)
//         🎬 /gripper_controller/gripper_cmd (control_msgs/action/ParallelGripperCommand)
//         ✉️ /diagnostics
//   수동이 아닐 때 들어온 수동 명령은 버린다. 수동 속도 명령이 cmd_timeout_s 넘게 끊기면 0을 한 번 보낸다
//   (twist_mux도 자체 timeout으로 0으로 떨어지지만, 끊긴 순간을 확실히 멈추려고).
// 수동 바퀴 명령도 안전 체인(velocity_smoother · collision_monitor)을 지난다 — 장애물 앞에서는 수동이어도 멈춘다.
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace control {

class ManualControl : public rclcpp::Node {
public:
  using GripperCommand = control_msgs::action::ParallelGripperCommand;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  ManualControl();

  bool manual() const;

private:
  void set_manual(bool manual);

  void on_cmd_vel(const TwistStamped & in);

  void watchdog();

  void stop_base();

  void on_arm(const sensor_msgs::msg::JointState & js);

  void send_gripper(double position);

  std::vector<std::string> arm_joints_;
  std::string gripper_joint_;
  double arm_move_s_, cmd_timeout_s_;
  bool manual_ = false, moving_ = false;
  std::string map_mode_;           // /state/map_mode — MAPPING이면 팔 관절 명령을 막음 (지도 만들 땐 팔이 스캔 자세에 고정)
  size_t vel_count_ = 0, arm_count_ = 0, arm_blocked_ = 0;
  rclcpp::Time last_vel_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_, map_mode_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_sub_, js_sub_;
  std::map<std::string, double> current_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace control
