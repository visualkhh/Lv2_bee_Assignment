// plan 기본 테스트: coordinator는 허용 동작(navigate·pick·place·say)이 아닌 계획을 거부하고 FAILED를 보낸다 (규칙 1)
#include <cmath>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "plan/coordinator.hpp"
#include "plan/vlm_task_planner.hpp"

using namespace plan;  // 노드 클래스·메시지 별칭 (include/plan/*.hpp)

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

TEST_F(RosTest, RejectsUnknownSkill) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  TaskStatus::SharedPtr got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got = m; });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  SubGoalArray plan;
  plan.plan_id = 7;
  SubGoal g;
  g.step = 1;
  g.skill = "fly";
  g.target = "3";
  plan.goals.push_back(g);
  goals_pub->publish(plan);
  ASSERT_TRUE(spin_until(exec,
    [&]() {
      TrackedObjectArray objects;
      perception::msg::TrackedObject puck;
      puck.id = 3;
      puck.class_name = "puck";
      objects.objects.push_back(puck);
      objects_pub->publish(objects);
    },
    [&]() { return got != nullptr; }));
  EXPECT_EQ(got->plan_id, 7u);
  EXPECT_EQ(got->state, TaskStatus::FAILED);
  EXPECT_EQ(got->reason.rfind("invalid_plan", 0), 0u) << got->reason;
}

// 수동 모드로 바뀌면 진행 중 작업을 멈추고 FAILED(manual_override)를 보낸다
TEST_F(RosTest, ManualModeStopsRunningPlan) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe2");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  auto mode_pub = probe->create_publisher<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local());
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got.push_back(*m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  SubGoalArray plan;
  plan.plan_id = 8;
  SubGoal g;
  g.step = 1;
  g.skill = "navigate";
  g.target = "3";
  plan.goals.push_back(g);
  goals_pub->publish(plan);
  TrackedObjectArray objects;
  perception::msg::TrackedObject puck;
  puck.id = 3;
  puck.class_name = "puck";
  objects.objects.push_back(puck);
  // 계획 수락 (Nav2 서버가 없어 이동 단계에서 기다리는 중 = 실행 중)
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(objects); },
    [&]() { for (auto & s : got) if (s.plan_id == 8 && s.state == TaskStatus::RUNNING) return true; return false; }));
  std_msgs::msg::String manual;
  manual.data = "MANUAL";
  mode_pub->publish(manual);
  ASSERT_TRUE(spin_until(exec, [] {},
    [&]() { for (auto & s : got) if (s.plan_id == 8 && s.reason == "manual_override") return true; return false; }));
}

// 적재함 (MISSION ③): 칸 파라미터 해석 · 같은 곳 여러 개 비켜 놓기 · MOVE_TO_B 템플릿 = 싣고(place tray) → 이동 → 꺼내서(pick tray) 내려놓기
TEST(Tray, SlotsAndSpread) {
  const auto slots = tray_slots({-0.14, 0.05, -0.095, -0.05, 9.0});  // 홀수 — 마지막 버림
  ASSERT_EQ(slots.size(), 2u);
  EXPECT_DOUBLE_EQ(slots[1].first, -0.095);
  EXPECT_EQ(spread_offset(0, 0.08), std::make_pair(0.0, 0.0));
  EXPECT_EQ(spread_offset(1, 0.08), std::make_pair(0.0, 0.08));
  EXPECT_EQ(spread_offset(2, 0.08), std::make_pair(0.0, -0.08));
  EXPECT_EQ(spread_offset(9, 0.08), std::make_pair(0.0, 0.0));     // 9개째부터 한 칸 바깥 고리 — 0은 가운데 그대로
  EXPECT_EQ(spread_offset(10, 0.08), std::make_pair(0.0, 0.16));
}


TEST(Coordinator, ParseNumbers) {  // 좌표 대상 — navigate "x,y[,yaw]" · place/inspect "x,y,z"
  EXPECT_EQ(parse_numbers("1.5,-2"), (std::vector<double>{1.5, -2.0}));
  EXPECT_EQ(parse_numbers("1,2,3.25"), (std::vector<double>{1.0, 2.0, 3.25}));
  EXPECT_TRUE(parse_numbers("12").empty());     // 물체 id
  EXPECT_TRUE(parse_numbers("tray").empty());
  EXPECT_TRUE(parse_numbers("1,2x").empty());
  EXPECT_TRUE(parse_numbers("1,,2").empty());
}

TEST(Coordinator, ApproachPose) {
  // 로봇 (0,0) · 물체 (0.6, 0) → 물체 앞 0.24 = (0.36, 0), 물체를 바라봄 (yaw 0)
  auto p = plan::approach_pose(0.6, 0.0, 0.0, 0.0, 0.24);
  EXPECT_NEAR(p.position.x, 0.36, 1e-9);
  EXPECT_NEAR(p.position.y, 0.0, 1e-9);
  EXPECT_NEAR(2 * std::atan2(p.orientation.z, p.orientation.w), 0.0, 1e-9);
  // 로봇이 물체의 +y 쪽 → 그쪽에서 다가가 −y를 바라봄 (예전엔 늘 물체 −x 쪽 · yaw 0)
  p = plan::approach_pose(1.0, 0.0, 1.0, 1.0, 0.24);
  EXPECT_NEAR(p.position.x, 1.0, 1e-9);
  EXPECT_NEAR(p.position.y, 0.24, 1e-9);
  EXPECT_NEAR(2 * std::atan2(p.orientation.z, p.orientation.w), -M_PI / 2, 1e-9);
  // 로봇이 물체 바로 위면 −x 쪽에서
  p = plan::approach_pose(1.0, 2.0, 1.0, 2.0, 0.3);
  EXPECT_NEAR(p.position.x, 0.7, 1e-9);
  EXPECT_NEAR(p.position.y, 2.0, 1e-9);
}
