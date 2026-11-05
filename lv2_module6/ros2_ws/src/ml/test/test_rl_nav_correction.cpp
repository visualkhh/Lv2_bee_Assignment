// ml 기본 테스트: rl_nav_correction은 보정 뒤 /speed_limit 상한으로 다시 자른다 (40% × 0.26 = 0.104m/s)
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "ml/rl_nav_correction.hpp"

using namespace ml;  // 노드 클래스·메시지 별칭 (include/ml/*.hpp)

// 조건이 맞을 때까지 step()으로 입력을 넣으며 spin. discovery 전에 보낸 메시지는 사라질 수 있어서 반복해 넣는다
template <class Step, class Done>
bool spin_until(rclcpp::Executor & exec, Step step, Done done, double timeout_s = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    step();
    exec.spin_some(std::chrono::milliseconds(50));
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosTest, ClipsToSpeedLimit) {
  auto node = std::make_shared<RlNavCorrection>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto limit_pub = probe->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", rclcpp::QoS(1).reliable());
  auto odom_pub = probe->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto cmd_pub = probe->create_publisher<TwistStamped>("/cmd_vel_nav", 10);
  TwistStamped::SharedPtr got;
  auto sub = probe->create_subscription<TwistStamped>("/cmd_vel_corrected", rclcpp::QoS(1).reliable(),
    [&](TwistStamped::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ASSERT_TRUE(spin_until(exec,
    [&]() {
      nav2_msgs::msg::SpeedLimit limit;
      limit.percentage = true;
      limit.speed_limit = 40.0;
      limit_pub->publish(limit);
      odom_pub->publish(nav_msgs::msg::Odometry());
      TwistStamped cmd;
      cmd.twist.linear.x = 0.20;
      cmd.twist.angular.z = 0.30;
      cmd_pub->publish(cmd);
    },
    [&]() { return got && std::abs(got->twist.linear.x - 0.104) < 1e-9; }));
  EXPECT_DOUBLE_EQ(got->twist.angular.z, 0.30);  // 각속도는 max_vel_theta(1.82) 안이라 그대로
}
