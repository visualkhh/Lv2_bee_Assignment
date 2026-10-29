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

TEST(Tray, CarryGoals) {
  const auto g = carry_goals({"11", "12"}, "7");  // 퍽 2개를 적재함에 싣고 장소 7로
  ASSERT_EQ(g.size(), 2u * 3 + 1 + 2u * 2);
  EXPECT_EQ(g[0].skill, "navigate");
  EXPECT_EQ(g[1].skill, "pick");
  EXPECT_EQ(g[1].target, "11");
  EXPECT_EQ(g[2].skill, "place");
  EXPECT_EQ(g[2].target, "tray");
  EXPECT_EQ(g[6].skill, "navigate");
  EXPECT_EQ(g[6].target, "7");
  EXPECT_EQ(g[7].target, "tray");   // 꺼내기
  EXPECT_EQ(g[8].target, "7");      // 장소에 내려놓기
  for (size_t i = 0; i < g.size(); ++i) EXPECT_EQ(g[i].step, i + 1);
}

namespace {
perception::msg::TrackedObject obj(uint32_t id, const std::string & cls, const std::string & color, int tag, double x) {
  perception::msg::TrackedObject o;
  o.id = id;
  o.class_name = cls;
  o.color = color;
  o.pose.pose.position.x = x;
  if (tag >= 0) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "tag";
    kv.value = std::to_string(tag);
    o.details.push_back(kv);
  }
  return o;
}
}  // namespace

TEST(Signal, PickCombo) {
  // MISSION §3: 빨간 퍽(0~3) 중 합 5인 2개 = 2+3 · 답이 여러 개(합 3 = 0+3 · 1+2)면 가까운(cost 작은) 쪽
  EXPECT_EQ(pick_combo({0, 1, 2, 3}, {1, 1, 1, 1}, 2, 5), (std::vector<size_t>{2, 3}));
  EXPECT_EQ(pick_combo({0, 1, 2, 3}, {5, 1, 1, 5}, 2, 3), (std::vector<size_t>{1, 2}));
  EXPECT_TRUE(pick_combo({0, 1, 2, 3}, {1, 1, 1, 1}, 2, 6).empty());  // MISSION §4: 빨강 2개로 6은 불가능
  EXPECT_EQ(pick_combo({4, 5, 6, 7}, {1, 1, 1, 1}, 4, 22).size(), 4u);
}

TEST(Signal, MissionPlan) {
  perception::msg::MissionSignal sig;
  sig.color = "red";
  sig.count = 2;
  sig.sum = 5;
  sig.dest_tag = 13;
  TrackedObjectArray objs;
  objs.objects = {obj(1, "puck", "red", 2, 0.5), obj(2, "puck", "red", -1, 0.6), obj(3, "puck", "green", -1, 0.4),
                  obj(4, "puck", "", -1, 0.9), obj(9, "location", "", 13, 2.0)};
  std::set<uint32_t> inspected;
  // 번호 아는 빨간 퍽이 1개뿐 → 번호 모르는 빨간·색 모름 퍽을 가까운 순으로 inspect (초록은 안 봄)
  auto r = signal_plan(objs, sig, 0.0, 0.0, inspected, 4);
  ASSERT_EQ(r.kind, "inspect");
  EXPECT_EQ(r.inspect_ids, (std::vector<uint32_t>{2, 4}));
  EXPECT_EQ(r.goals[0].skill, "inspect");
  // 봤더니 2번 퍽 = 태그 3 → 2+3 = 5 → 옮기기 (목적지 = 장소 태그 13인 물체 9)
  objs.objects[1] = obj(2, "puck", "red", 3, 0.6);
  inspected = {2, 4};
  r = signal_plan(objs, sig, 0.0, 0.0, inspected, 4);
  ASSERT_EQ(r.kind, "carry") << r.why;
  EXPECT_EQ(r.carry_tags, (std::vector<int>{2, 3}));
  EXPECT_EQ(r.goals[1].target, "1");
  EXPECT_EQ(r.goals[4].target, "2");
  EXPECT_EQ(r.goals[6].target, "9");
  // 목적지 태그를 아직 못 봤으면 기다림
  objs.objects.pop_back();
  EXPECT_EQ(signal_plan(objs, sig, 0.0, 0.0, inspected, 4).kind, "wait");
  // 다 봤는데 합을 못 만들면 실패 (say)
  sig.sum = 6;
  EXPECT_EQ(signal_plan(objs, sig, 0.0, 0.0, inspected, 4).kind, "fail");
}

TEST(Signal, VerifyPlan) {
  // 목적지 = 장소 태그 13인 물체 9 (x 2.0) · 옮긴 번호 2·3 · 반경 0.2
  TrackedObjectArray objs;
  objs.objects = {obj(9, "location", "", 13, 2.0), obj(1, "puck", "red", 2, 2.05), obj(2, "puck", "red", -1, 1.9),
                  obj(5, "puck", "red", -1, 0.5)};
  // 보러 가기 — 목적지로 navigate · 근처의 번호 모르는 퍽(2)만 inspect (멀리 있는 5는 안 봄)
  auto r = verify_plan(objs, 13, {2, 3}, 0.2, false);
  ASSERT_EQ(r.kind, "look");
  ASSERT_EQ(r.goals.size(), 2u);
  EXPECT_EQ(r.goals[0].target, "9");
  EXPECT_EQ(r.goals[1].skill, "inspect");
  EXPECT_EQ(r.goals[1].target, "2");
  // 봤더니 2번 = 태그 3 → 다 있음
  objs.objects[2] = obj(2, "puck", "red", 3, 1.9);
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, true).kind, "ok");
  // 태그 3 퍽이 다른 곳(0.5)에 있으면 다시 옮김
  objs.objects[2] = obj(2, "puck", "red", 3, 0.5);
  r = verify_plan(objs, 13, {2, 3}, 0.2, true);
  ASSERT_EQ(r.kind, "recarry") << r.why;
  EXPECT_EQ(r.missing, (std::vector<int>{3}));
  EXPECT_EQ(r.goals[1].target, "2");
  EXPECT_EQ(r.goals.back().target, "9");
  // 어디에도 안 보이면 missing · 목적지 태그가 안 보이면 wait
  objs.objects.erase(objs.objects.begin() + 2);
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, true).kind, "missing");
  objs.objects.erase(objs.objects.begin());
  EXPECT_EQ(verify_plan(objs, 13, {2, 3}, 0.2, false).kind, "wait");
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
