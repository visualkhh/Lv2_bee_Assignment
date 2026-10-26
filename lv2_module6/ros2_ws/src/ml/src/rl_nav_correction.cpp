// rl_nav_correction 구현 — 노드 설명·입출력·멤버 선언은 include/ml/rl_nav_correction.hpp, main은 rl_nav_correction_main.cpp
#include "ml/rl_nav_correction.hpp"

namespace ml {

RlNavCorrection::RlNavCorrection()
: rclcpp::Node("rl_nav_correction") {
  declare_parameter<bool>("enabled", true);
  max_v_ = declare_parameter<double>("max_vel_x", 0.26);        // Waffle Pi 최대 선속도
  max_w_ = declare_parameter<double>("max_vel_theta", 1.82);
  timeout_s_ = declare_parameter<double>("timeout_s", 0.2);
  limit_v_ = max_v_;
  cmd_sub_ = create_subscription<TwistStamped>(
    "/cmd_vel_nav", 10, [this](TwistStamped::ConstSharedPtr m) { on_cmd(*m); });
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 1, [this](nav_msgs::msg::Path::ConstSharedPtr m) { plan_ = m; });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; last_sensor_ = now(); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS().keep_last(1), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_ = m; });
  limit_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
    "/speed_limit", rclcpp::QoS(1).reliable(), [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
      const double v = m->percentage ? max_v_ * m->speed_limit / 100.0 : m->speed_limit;
      limit_v_ = (m->speed_limit <= 0.0) ? max_v_ : std::min(max_v_, v);  // 0 = 제한 없음
    });
  pub_ = create_publisher<TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable());
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const bool fresh = (now() - last_sensor_).seconds() <= timeout_s_;
    if (!fresh) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/odom 끊김 — Δ=0 (통과만)");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, get_parameter("enabled").as_bool() ? "보정 중" : "Δ=0 (enabled=false)");
    s.add("enabled", get_parameter("enabled").as_bool());
    s.add("속도 상한 [m/s]", limit_v_);
    s.add("보정한 명령 수", corrected_);
  });
}

void RlNavCorrection::on_cmd(const TwistStamped & in) {
  double dv = 0.0, dw = 0.0;
  const bool fresh = (now() - last_sensor_).seconds() <= timeout_s_;
  if (get_parameter("enabled").as_bool() && fresh) {
    // TODO: 정책 추론. 지금은 Δ=0
    dv = std::clamp(dv, -0.05, 0.05);
    dw = std::clamp(dw, -0.10, 0.10);
  }
  TwistStamped out = in;
  out.twist.linear.x = std::clamp(in.twist.linear.x + dv, -limit_v_, limit_v_);
  out.twist.angular.z = std::clamp(in.twist.angular.z + dw, -max_w_, max_w_);
  pub_->publish(out);
  ++corrected_;
}

}  // namespace ml
