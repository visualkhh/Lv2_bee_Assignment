// plan 기본 테스트: coordinator는 허용 동작(navigate·pick·place·inspect·pose·load·unload)이 아닌 계획을 거부하고 FAILED를 보낸다 (규칙 1)
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "plan/coordinator.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

// pose는 아는 자세 이름만 · load·unload의 target(칸 번호)은 물체 id로 검증하지 않음 (05-arm.md)
TEST_F(RosTest, ValidatesPoseAndSlotTargets) {
  auto node = std::make_shared<Coordinator>();
  auto probe = std::make_shared<rclcpp::Node>("probe_skills");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched,
    [&](TaskStatus::SharedPtr m) { got.push_back(*m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  auto send = [&](uint32_t id, const std::string & skill, const std::string & target) {
    SubGoalArray plan;
    plan.plan_id = id;
    SubGoal g;
    g.step = 1;
    g.skill = skill;
    g.target = target;
    plan.goals.push_back(g);
    goals_pub->publish(plan);
  };
  auto has = [&](uint32_t id, auto pred) { for (auto & s : got) if (s.plan_id == id && pred(s)) return true; return false; };
  send(21, "pose", "fly");
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(TrackedObjectArray()); },
    [&]() { return has(21, [](const TaskStatus & s) { return s.state == TaskStatus::FAILED; }); }));
  EXPECT_TRUE(has(21, [](const TaskStatus & s) { return s.reason == "invalid_plan:pose fly"; }));
  // 칸 5 — /tracked_objects에 id 5가 없어도 계획은 받음 (실행은 MoveIt 대기)
  send(22, "load", "5");
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(TrackedObjectArray()); },
    [&]() { return has(22, [](const TaskStatus & s) { return s.state == TaskStatus::RUNNING; }); }));
  EXPECT_FALSE(has(22, [](const TaskStatus & s) { return s.reason.rfind("invalid_plan", 0) == 0; }));
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

// 단계를 skill 노드에 맡김: load next → /skills/load에 빈 칸 0 · 모양을 보내고, 성공이면 장부에 적고 계획 SUCCEEDED
TEST_F(RosTest, HandsStepToSkillNodeAndKeepsLedger) {
  auto node = std::make_shared<Coordinator>(rclcpp::NodeOptions().parameter_overrides({{"basket_file", ""}}));
  auto probe = std::make_shared<rclcpp::Node>("probe_skill_server");
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  auto objects_pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto goals_pub = probe->create_publisher<SubGoalArray>("/task/sub_goals", latched);
  std::vector<TaskStatus> got;
  auto sub = probe->create_subscription<TaskStatus>("/task/status", latched, [&](TaskStatus::SharedPtr m) { got.push_back(*m); });
  plan::msg::BasketState basket;
  auto bsub = probe->create_subscription<plan::msg::BasketState>("/coordinator/basket", rclcpp::QoS(1).reliable().transient_local(),
    [&](plan::msg::BasketState::SharedPtr m) { basket = *m; });
  int slot = -1;
  std::string shape;
  using GH = rclcpp_action::ServerGoalHandle<Skill>;
  auto server = rclcpp_action::create_server<Skill>(probe, "/skills/load",
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const Skill::Goal> g) {
      slot = g->slot;
      shape = g->shape;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GH>) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](const std::shared_ptr<GH> h) {
      auto r = std::make_shared<Skill::Result>();
      r->success = true;
      h->succeed(r);
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  SubGoalArray plan;
  plan.plan_id = 31;
  SubGoal g;
  g.step = 1;
  g.skill = "load";
  g.target = "next";
  plan.goals.push_back(g);
  goals_pub->publish(plan);
  ASSERT_TRUE(spin_until(exec, [&]() { objects_pub->publish(TrackedObjectArray()); },
    [&]() { for (auto & s : got) if (s.plan_id == 31 && s.state == TaskStatus::SUCCEEDED) return true; return false; }, 15.0));
  EXPECT_EQ(slot, 0);
  EXPECT_EQ(shape, "box");  // 손 모양 모름 → 사각 표
  ASSERT_TRUE(spin_until(exec, [] {}, [&]() { return basket.slots.size() == 12 && basket.slots[0].filled; }));
  EXPECT_EQ(basket.order.size(), 1u);
}

// 적재함 (MISSION ③): 칸 파라미터 해석 · 같은 곳 여러 개 비켜 놓기 · MOVE_TO_B 템플릿 = 싣고(place tray) → 이동 → 꺼내서(pick tray) 내려놓기
TEST(Tray, BasketSlot) {
  // 적재함 12칸 고르기 (load next · unload last · 칸 번호, 옛 "tray" = 같은 뜻)
  std::vector<std::string> filled(12, "");
  std::vector<int> order;
  std::string why;
  EXPECT_EQ(basket_slot("next", filled, order, true, why), 0);
  EXPECT_EQ(basket_slot("tray", filled, order, true, why), 0);
  EXPECT_EQ(basket_slot("last", filled, order, false, why), -1);
  EXPECT_EQ(why, "slot_empty");
  filled[0] = "box";
  filled[3] = "cyl";
  order = {3, 0};
  EXPECT_EQ(basket_slot("next", filled, order, true, why), 1);   // 빈 칸 중 가장 낮은 번호
  EXPECT_EQ(basket_slot("last", filled, order, false, why), 0);  // 마지막에 실은 칸
  EXPECT_EQ(basket_slot("tray", filled, order, false, why), 0);
  EXPECT_EQ(basket_slot("3", filled, order, false, why), 3);
  EXPECT_EQ(basket_slot("3", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_full");
  EXPECT_EQ(basket_slot("5", filled, order, false, why), -1);
  EXPECT_EQ(why, "slot_empty");
  EXPECT_EQ(basket_slot("12", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_invalid");
  EXPECT_EQ(basket_slot("-1", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_invalid");
  // load next는 쓰는 칸(usable)만 — 1번 칸을 빼면 빈 칸 중 다음 2번
  std::vector<bool> usable(12, true);
  usable[1] = false;
  EXPECT_EQ(basket_slot("next", filled, order, true, why, usable), 2);
  EXPECT_EQ(basket_slot("1", filled, order, true, why, usable), 1);  // 칸 번호를 직접 주면 그대로
  std::fill(filled.begin(), filled.end(), "box");
  EXPECT_EQ(basket_slot("next", filled, order, true, why), -1);
  EXPECT_EQ(why, "slot_full");
}

TEST(Tray, BoxSpread) {
  // 상자 0.205 각 · 가장자리 0.03 안쪽까지 → 비킬 수 있는 최대 0.0725. 상자 yaw 90°면 축이 돌아감
  auto [x0, y0] = box_spread(0, 0.08, 0.1025, 0.1025, 0.03, 0.0);
  EXPECT_NEAR(x0, 0.0, 1e-9);
  EXPECT_NEAR(y0, 0.0, 1e-9);
  auto [x1, y1] = box_spread(1, 0.08, 0.1025, 0.1025, 0.03, 0.0);  // spread_offset (0, 0.08) → 0.0725로 자름
  EXPECT_NEAR(x1, 0.0, 1e-9);
  EXPECT_NEAR(y1, 0.0725, 1e-9);
  auto [x2, y2] = box_spread(1, 0.08, 0.1025, 0.1025, 0.03, M_PI / 2);
  EXPECT_NEAR(x2, -0.0725, 1e-9);
  EXPECT_NEAR(y2, 0.0, 1e-9);
}

TEST(Tray, Spread) {
  EXPECT_EQ(spread_offset(0, 0.08), std::make_pair(0.0, 0.0));
  EXPECT_EQ(spread_offset(1, 0.08), std::make_pair(0.0, 0.08));
  EXPECT_EQ(spread_offset(2, 0.08), std::make_pair(0.0, -0.08));
  EXPECT_EQ(spread_offset(9, 0.08), std::make_pair(0.0, 0.0));     // 9개째부터 한 칸 바깥 고리 — 0은 가운데 그대로
  EXPECT_EQ(spread_offset(10, 0.08), std::make_pair(0.0, 0.16));
}

TEST(Coordinator, ParseNumbers) {
  // 좌표 대상 "x,y" · "x,y,z|yaw" (navigate · approach · place · inspect) — 숫자가 아닌 게 섞이거나 1개면 빈 목록
  EXPECT_EQ(parse_numbers("1.5,-2"), (std::vector<double>{1.5, -2.0}));
  EXPECT_EQ(parse_numbers("0.1,0.2,0.07"), (std::vector<double>{0.1, 0.2, 0.07}));
  EXPECT_TRUE(parse_numbers("3").empty());
  EXPECT_TRUE(parse_numbers("tray").empty());
  EXPECT_TRUE(parse_numbers("1,a").empty());
  EXPECT_TRUE(parse_numbers("").empty());
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
