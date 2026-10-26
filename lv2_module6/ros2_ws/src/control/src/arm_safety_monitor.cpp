// arm_safety_monitor 구현 — 노드 설명·입출력·멤버 선언은 include/control/arm_safety_monitor.hpp, main은 arm_safety_monitor_main.cpp
#include "control/arm_safety_monitor.hpp"

namespace control {

ArmSafetyMonitor::ArmSafetyMonitor()
: rclcpp::Node("arm_safety_monitor") {
  threshold_ = declare_parameter<double>("effort_threshold", 3.0);  // A — 관절별로 나눌 것 (실측 전 중력 부하 여유)
  hold_s_ = declare_parameter<double>("hold_s", 0.3);
  stall_velocity_ = declare_parameter<double>("stall_velocity", 0.10);  // [rad/s] 이보다 느리면 "멈춘 것"으로 봄
  sub_ = create_subscription<JointState>(
    "/joint_states", 10, [this](JointState::ConstSharedPtr msg) { on_joint_states(*msg); });
  event_pub_ = create_publisher<ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable());
  cancel_ = create_client<CancelGoal>("/arm_controller/follow_joint_trajectory/_action/cancel_goal");
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const double since_trip = trips_ ? (now() - last_trip_).seconds() : -1;
    if (since_trip >= 0 && since_trip < 5.0) s.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "접촉 감지: " + last_joint_);
    else if (!joint_states_) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/joint_states 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "감시 중");
    s.add("기준 [A]", threshold_);
    s.add("정지 판정 [rad/s]", stall_velocity_);
    s.add("접촉 감지 횟수", trips_);
    s.add("마지막 접촉 관절", last_joint_);
  });
}

void ArmSafetyMonitor::on_joint_states(const JointState & js) {
  ++joint_states_;
  const rclcpp::Time t = js.header.stamp;
  for (size_t i = 0; i < js.name.size() && i < js.effort.size(); ++i) {
    const auto & name = js.name[i];
    if (name.rfind("joint", 0) != 0) continue;  // 팔 관절(joint1~5)만
    // 전류가 커도 움직이는 중이면 중력·가속 부하 — 접촉은 "멈춘 채 전류만 큰" 상태
    const bool moving = i < js.velocity.size() && std::abs(js.velocity[i]) >= stall_velocity_;
    if (moving || std::abs(js.effort[i]) <= threshold_) { over_since_.erase(name); continue; }
    auto [it, first] = over_since_.try_emplace(name, t);
    if (!first && (t - it->second).seconds() >= hold_s_) {
      trip(name, js.effort[i]);
      over_since_.erase(name);
    }
  }
}

void ArmSafetyMonitor::trip(const std::string & joint, double current) {
  RCLCPP_WARN(get_logger(), "접촉 감지: %s %.2fA > %.2fA → 궤적 취소", joint.c_str(), current, threshold_);
  // goal_id·stamp가 0이면 진행 중인 goal 전체 취소 (action_msgs/srv/CancelGoal 규약)
  if (cancel_->service_is_ready()) cancel_->async_send_request(std::make_shared<CancelGoal::Request>());
  ArmSafetyEvent ev;
  ev.header.stamp = now();
  ev.joint_name = joint;
  ev.current_a = static_cast<float>(current);
  ev.threshold_a = static_cast<float>(threshold_);
  ev.reason = "overcurrent";
  ++trips_;
  last_trip_ = now();
  last_joint_ = joint;
  event_pub_->publish(ev);
}

}  // namespace control
