#pragma once

// arm_safety_monitor (🛠 직접 작성, 담당: 김현하) — 설계: documents/project/ros/05-arm.md 4-2
//   입력: /joint_states effort (= 다이나믹셀 Present Current, joint_state_broadcaster)
//   출력: 📞 /arm_controller/follow_joint_trajectory/_action/cancel_goal (이상 시에만)
//         ✉️ /arm_safety_event (control/ArmSafetyEvent) → coordinator
// 판단: 관절 전류가 기준을 넘고 **거의 멈춰 있는**(|velocity| < stall_velocity) 상태가 hold_s 넘게 이어지면 접촉.
//   움직이는 중의 전류는 중력·가속 부하라 정상 — 정지 상태에서 전류만 높으면 막힌 것(접촉).
// TODO(김현하): 퍽을 든 상태까지 정상 동작 전류를 기록해 관절별 기준값(effort_threshold) 정하기
#include <map>
#include <string>

#include "action_msgs/srv/cancel_goal.hpp"
#include "control/msg/arm_safety_event.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace control {

using action_msgs::srv::CancelGoal;
using control::msg::ArmSafetyEvent;
using sensor_msgs::msg::JointState;

class ArmSafetyMonitor : public rclcpp::Node {
public:
  ArmSafetyMonitor();

private:
  void on_joint_states(const JointState & js);

  void trip(const std::string & joint, double current);

  double threshold_, hold_s_, stall_velocity_;
  size_t joint_states_ = 0, trips_ = 0;
  rclcpp::Time last_trip_{0, 0, RCL_ROS_TIME};
  std::string last_joint_;
  diagnostic_updater::Updater diag_{this};
  std::map<std::string, rclcpp::Time> over_since_;
  rclcpp::Subscription<JointState>::SharedPtr sub_;
  rclcpp::Publisher<ArmSafetyEvent>::SharedPtr event_pub_;
  rclcpp::Client<CancelGoal>::SharedPtr cancel_;
};

}  // namespace control
