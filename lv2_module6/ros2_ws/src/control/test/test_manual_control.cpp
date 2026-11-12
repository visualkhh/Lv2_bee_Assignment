// control 테스트: manual_control은 /operation_mode가 MANUAL일 때만 /cmd/manual/cmd_vel을 /cmd_vel_manual로 넘긴다 (모드는 state_manager가 발행)
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "control/manual_control.hpp"

using namespace control;  // 노드 클래스·메시지 별칭 (include/control/*.hpp)

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

TEST_F(RosTest, ForwardsOnlyInManualMode) {
  auto node = std::make_shared<ManualControl>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto mode_pub = probe->create_publisher<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local());
  auto cmd_pub = probe->create_publisher<ManualControl::TwistStamped>("/cmd/manual/cmd_vel", 10);
  ManualControl::TwistStamped::SharedPtr out;
  auto out_sub = probe->create_subscription<ManualControl::TwistStamped>("/cmd_vel_manual", rclcpp::QoS(1).reliable(),
    [&](ManualControl::TwistStamped::SharedPtr m) { if (m->twist.linear.x > 0) out = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ManualControl::TwistStamped cmd;
  cmd.twist.linear.x = 0.1;
  std_msgs::msg::String mode;
  // 자동 모드: 수동 명령은 버린다
  mode.data = "AUTO";
  spin_until(exec, [&]() { mode_pub->publish(mode); cmd_pub->publish(cmd); }, [] { return false; }, 0.5);
  EXPECT_EQ(out, nullptr);
  EXPECT_FALSE(node->manual());
  // 수동 모드: 넘긴다
  mode.data = "MANUAL";
  ASSERT_TRUE(spin_until(exec, [&]() { mode_pub->publish(mode); cmd_pub->publish(cmd); },
    [&]() { return node->manual() && out != nullptr; }));
  EXPECT_DOUBLE_EQ(out->twist.linear.x, 0.1);
}

TEST_F(RosTest, RebootOnlyInManualMode) {
  // /cmd/manual/reboot_motors → 수동일 때만 /beebotsystem/reboot_motors 서비스 호출
  auto node = std::make_shared<ManualControl>();
  auto probe = std::make_shared<rclcpp::Node>("probe_reboot");
  auto mode_pub = probe->create_publisher<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local());
  auto cmd_pub = probe->create_publisher<std_msgs::msg::Empty>("/cmd/manual/reboot_motors", 10);
  int calls = 0;
  auto srv = probe->create_service<std_srvs::srv::Trigger>("/beebotsystem/reboot_motors",
    [&](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
      ++calls;
      res->success = true;
      res->message = "재부팅 요청: joint2";
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std_msgs::msg::String mode;
  mode.data = "AUTO";
  spin_until(exec, [&]() { mode_pub->publish(mode); cmd_pub->publish(std_msgs::msg::Empty()); }, [] { return false; }, 0.5);
  EXPECT_EQ(calls, 0);
  mode.data = "MANUAL";
  ASSERT_TRUE(spin_until(exec, [&]() { mode_pub->publish(mode); if (node->manual()) cmd_pub->publish(std_msgs::msg::Empty()); },
    [&]() { return calls > 0; }));
}
