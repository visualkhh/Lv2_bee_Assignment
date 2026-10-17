// user_input 구현 — 노드 설명·입출력·멤버 선언은 include/plan/user_input.hpp, main은 user_input_main.cpp
#include "plan/user_input.hpp"

namespace plan {

UserInput::UserInput()
: rclcpp::Node("user_input") {
  const auto command = declare_parameter<std::string>("command", "");
  if (!command.empty()) pending_.push_back(command);
  cmd_pub_ = create_publisher<String>("/cmd/user_command", rclcpp::QoS(10).reliable());
  say_sub_ = create_subscription<String>("/robot_say", rclcpp::QoS(10).reliable(),
    [this](String::ConstSharedPtr m) { RCLCPP_INFO(get_logger(), "🔊 \"%s\"", m->data.c_str()); });
  status_sub_ = create_subscription<TaskStatus>("/task_status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr m) {
      RCLCPP_INFO(get_logger(), "task plan=%u step=%u state=%u %s", m->plan_id, m->step, m->state, m->reason.c_str());
    });
  // launch로 띄우면 표준입력이 터미널이 아니다 → 터미널일 때만 읽는다
  if (isatty(STDIN_FILENO)) {
    std::thread([this]() {
      std::string line;
      while (rclcpp::ok() && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(line);
      }
    }).detach();
    RCLCPP_INFO(get_logger(), "명령을 입력하세요 (한 줄 = 명령 하나)");
  }
  timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() { send_pending(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_.empty()) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "vlm_task_planner 대기 — 못 보낸 명령 있음");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "대기");
    s.add("못 보낸 명령", pending_.size());
    s.add("보낸 명령 수", sent_);
    s.add("마지막 명령", last_);
  });
}

void UserInput::send_pending() {
  std::lock_guard<std::mutex> lock(mutex_);
  while (!pending_.empty() && cmd_pub_->get_subscription_count() > 0) {
    String m;
    m.data = pending_.front();
    pending_.pop_front();
    cmd_pub_->publish(m);
    ++sent_;
    last_ = m.data;
    RCLCPP_INFO(get_logger(), "/cmd/user_command \"%s\"", m.data.c_str());
  }
}

}  // namespace plan
