// fake_rl_nav_correction 구현 — 설명·선언은 include/faker/fake_rl_nav_correction.hpp, main은 fake_rl_nav_correction_main.cpp
#include "faker/fake_rl_nav_correction.hpp"

namespace faker {

using diagnostic_msgs::msg::DiagnosticStatus;

FakeRlNavCorrection::FakeRlNavCorrection()
: rclcpp::Node("rl_nav_correction") {
  max_v_ = declare_parameter<double>("max_vel_x", 0.26);
  limit_v_ = max_v_;
  cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", 10,
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { on_cmd(*m); });
  limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable(),
    [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
      const double v = m->percentage ? max_v_ * m->speed_limit / 100.0 : m->speed_limit;
      limit_v_ = (m->speed_limit <= 0.0) ? max_v_ : std::min(max_v_, v);
    });
  pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable());
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — Δ=0 통과");
    s.add("통과한 명령 수", passed_);
    s.add("속도 상한 [m/s]", limit_v_);
  });
}

void FakeRlNavCorrection::on_cmd(const geometry_msgs::msg::TwistStamped & in) {
  auto out = in;
  out.twist.linear.x = std::clamp(in.twist.linear.x, -limit_v_, limit_v_);
  pub_->publish(out);
  ++passed_;
}

}  // namespace faker
