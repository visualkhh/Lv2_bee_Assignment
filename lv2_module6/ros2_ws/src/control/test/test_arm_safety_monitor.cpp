// control 기본 테스트: 팔 관절이 거의 멈춘 채 전류가 기준(3.0A)을 넘은 상태가 0.3초 넘게 이어지면 /arm_safety_event
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "control/arm_safety_monitor.hpp"

using namespace control;  // 노드 클래스·메시지 별칭 (include/control/*.hpp)

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

TEST_F(RosTest, OvercurrentRaisesSafetyEvent) {
  auto node = std::make_shared<ArmSafetyMonitor>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto js_pub = probe->create_publisher<JointState>("/joint_states", 10);
  ArmSafetyEvent::SharedPtr got;
  auto sub = probe->create_subscription<ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable(),
    [&](ArmSafetyEvent::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  rclcpp::Time t(1, 0, RCL_ROS_TIME);
  ASSERT_TRUE(spin_until(exec,
    [&]() {
      JointState js;
      js.header.stamp = t;
      js.name = {"wheel_left_joint", "joint2"};
      js.position = {0, 0};
      js.velocity = {0, 0};     // 정지 상태여야 접촉으로 봄 (움직이는 중 전류는 정상 부하)
      js.effort = {5.0, 5.0};   // 바퀴는 감시 대상 아님 · joint2 5A > 3A
      js_pub->publish(js);
      t += rclcpp::Duration::from_seconds(0.05);
    },
    [&]() { return got != nullptr; }));
  EXPECT_EQ(got->joint_name, "joint2");
  EXPECT_EQ(got->reason, "overcurrent");
}

// 움직이는 중(속도 큼)의 높은 전류는 중력·가속 부하 — 접촉으로 보지 않는다
TEST_F(RosTest, MovingHighCurrentDoesNotTrip) {
  auto node = std::make_shared<ArmSafetyMonitor>();
  auto probe = std::make_shared<rclcpp::Node>("probe2");
  auto js_pub = probe->create_publisher<JointState>("/joint_states", 10);
  ArmSafetyEvent::SharedPtr got;
  auto sub = probe->create_subscription<ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable(),
    [&](ArmSafetyEvent::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  rclcpp::Time t(1, 0, RCL_ROS_TIME);
  spin_until(exec,
    [&]() {
      JointState js;
      js.header.stamp = t;
      js.name = {"joint2"};
      js.position = {0};
      js.velocity = {1.0};   // 움직이는 중
      js.effort = {5.0};     // 전류는 큼
      js_pub->publish(js);
      t += rclcpp::Duration::from_seconds(0.05);
    },
    [&]() { return got != nullptr; }, 2.0);
  EXPECT_EQ(got, nullptr);
}
