// manual_control 구현 — 노드 설명·입출력·멤버 선언은 include/control/manual_control.hpp, main은 manual_control_main.cpp
#include "control/manual_control.hpp"

namespace control {

ManualControl::ManualControl()
: rclcpp::Node("manual_control") {
  arm_joints_ = declare_parameter<std::vector<std::string>>("arm_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper_left_joint");
  arm_move_s_ = declare_parameter<double>("arm_move_s", 1.0);
  cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);
  vel_pub_ = create_publisher<TwistStamped>("/cmd_vel_manual", rclcpp::QoS(1).reliable());
  traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
  gripper_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_controller/gripper_cmd");
  mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { set_manual(m->data == "MANUAL"); });
  vel_sub_ = create_subscription<TwistStamped>("/cmd/manual/cmd_vel", 10,
    [this](TwistStamped::ConstSharedPtr m) { on_cmd_vel(*m); });
  arm_sub_ = create_subscription<sensor_msgs::msg::JointState>("/cmd/manual/arm", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { on_arm(*m); });
  js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) current_[m->name[i]] = m->position[i];
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { watchdog(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(manual_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK,
      manual_ ? "수동 모드" : "자동 모드");
    s.add("모드", manual_ ? "MANUAL" : "AUTO");
    s.add("수동 바퀴 명령 수", vel_count_);
    s.add("수동 팔 명령 수", arm_count_);
    s.add("마지막 바퀴 명령 [s 전]", vel_count_ ? (now() - last_vel_).seconds() : -1.0);
  });
}

bool ManualControl::manual() const { return manual_; }

void ManualControl::set_manual(bool manual) {
  if (manual == manual_) return;
  manual_ = manual;
  if (!manual_) stop_base();
  RCLCPP_WARN(get_logger(), "운영 모드 → %s", manual_ ? "MANUAL" : "AUTO");
}

void ManualControl::on_cmd_vel(const TwistStamped & in) {
  if (!manual_) return;
  TwistStamped out = in;
  out.header.stamp = now();
  if (out.header.frame_id.empty()) out.header.frame_id = "base_footprint";
  vel_pub_->publish(out);
  last_vel_ = now();
  moving_ = true;
  ++vel_count_;
}

void ManualControl::watchdog() {
  if (moving_ && (now() - last_vel_).seconds() > cmd_timeout_s_) stop_base();
}

void ManualControl::stop_base() {
  TwistStamped zero;
  zero.header.stamp = now();
  zero.header.frame_id = "base_footprint";
  vel_pub_->publish(zero);
  moving_ = false;
}

void ManualControl::on_arm(const sensor_msgs::msg::JointState & js) {
  if (!manual_) return;
  std::map<std::string, double> target;
  for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
    if (js.name[i] == gripper_joint_) send_gripper(js.position[i]);
    else if (std::find(arm_joints_.begin(), arm_joints_.end(), js.name[i]) != arm_joints_.end()) target[js.name[i]] = js.position[i];
  }
  if (target.empty()) return;
  trajectory_msgs::msg::JointTrajectory traj;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (const auto & j : arm_joints_) {  // 목표에 없는 관절은 지금 각 그대로
    const auto t = target.find(j);
    const auto c = current_.find(j);
    if (t == target.end() && c == current_.end()) {
      RCLCPP_WARN(get_logger(), "%s 지금 각을 모름 (/joint_states 대기) — 팔 명령 보류", j.c_str());
      return;
    }
    traj.joint_names.push_back(j);
    point.positions.push_back(t != target.end() ? t->second : c->second);
  }
  point.time_from_start = rclcpp::Duration::from_seconds(arm_move_s_);
  traj.points.push_back(point);
  traj_pub_->publish(traj);
  ++arm_count_;
}

void ManualControl::send_gripper(double position) {
  if (!gripper_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/gripper_controller/gripper_cmd 서버 없음");
    return;
  }
  GripperCommand::Goal goal;
  goal.command.name = {gripper_joint_};
  goal.command.position = {position};
  gripper_->async_send_goal(goal);
  ++arm_count_;
}

}  // namespace control
