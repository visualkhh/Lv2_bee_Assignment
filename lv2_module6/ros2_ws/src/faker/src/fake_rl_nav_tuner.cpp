// fake_rl_nav_tuner 구현 — 설명·선언은 include/faker/fake_rl_nav_tuner.hpp, main은 fake_rl_nav_tuner_main.cpp
#include "faker/fake_rl_nav_tuner.hpp"

namespace faker {

using diagnostic_msgs::msg::DiagnosticStatus;

FakeRlNavTuner::FakeRlNavTuner()
: rclcpp::Node("rl_nav_tuner") {
  const double pct = declare_parameter<double>("speed_percent", 100.0);
  pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable());
  timer_ = create_wall_timer(std::chrono::seconds(2), [this, pct]() {
    nav2_msgs::msg::SpeedLimit m;
    m.header.stamp = now();
    m.percentage = true;
    m.speed_limit = pct;
    pub_->publish(m);
  });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [pct](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — 프리셋 고정");
    s.add("속도 상한 [%]", pct);
  });
}

}  // namespace faker
