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
  map_mode_sub_ = create_subscription<std_msgs::msg::String>("/state/map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { map_mode_ = m->data; });
  vel_sub_ = create_subscription<TwistStamped>("/cmd/manual/cmd_vel", 10,
    [this](TwistStamped::ConstSharedPtr m) { on_cmd_vel(*m); });
  arm_sub_ = create_subscription<sensor_msgs::msg::JointState>("/cmd/manual/arm", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { on_arm(*m); });
  reboot_cli_ = create_client<std_srvs::srv::Trigger>(declare_parameter<std::string>("reboot_service", "/beebotsystem/reboot_motors"));
  reboot_sub_ = create_subscription<std_msgs::msg::Empty>("/cmd/manual/reboot_motors", 10,
    [this](std_msgs::msg::Empty::ConstSharedPtr) { on_reboot(); });
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
    s.add("지도 만드는 중 막은 팔 명령", arm_blocked_);
    s.add("모터 재부팅 요청 수", reboot_count_);
    s.add("마지막 모터 재부팅", reboot_result_);
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

void ManualControl::on_reboot() {
  if (!manual_) {  // 다른 수동 명령과 같음 — 자동 작업 중엔 받지 않는다
    RCLCPP_WARN(get_logger(), "수동 모드가 아님 — 모터 재부팅 명령 버림");
    return;
  }
  ++reboot_count_;
  if (!reboot_cli_->service_is_ready()) {
    reboot_result_ = "서비스 없음: " + std::string(reboot_cli_->get_service_name());
    RCLCPP_WARN(get_logger(), "%s", reboot_result_.c_str());
    return;
  }
  reboot_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      const auto res = f.get();
      reboot_result_ = (res->success ? "" : "거절: ") + res->message;
      RCLCPP_WARN(get_logger(), "모터 재부팅: %s", reboot_result_.c_str());
    });
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
  if (map_mode_ == "MAPPING") {  // 지도 만드는 중엔 팔이 스캔 자세에 고정 (gripper_cam = 두 번째 지도 카메라) — 그리퍼만 받음
    ++arm_blocked_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "지도 만드는 중(MAPPING) — 팔 수동 명령 거절 (스캔 자세 고정)");
    return;
  }
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
