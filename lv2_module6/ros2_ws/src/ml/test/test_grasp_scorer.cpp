// grasp_scorer 테스트: 규칙(rule_grasps)은 메시지만으로 · /score_grasps 서비스는 노드를 같은 프로세스에 띄워 확인
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "ml/grasp_scorer.hpp"

using namespace ml;  // 노드 클래스·규칙 함수·메시지 별칭 (include/ml/grasp_scorer.hpp)

namespace {

constexpr double DEG = M_PI / 180.0;

TrackedObject puck(uint32_t id, double x, double y, const std::string & shape = "", double yaw_deg = NAN,
                   int period_deg = 0, bool lying = false) {
  TrackedObject o;
  o.id = id;
  o.class_name = lying ? "puck_lying" : "puck";
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.orientation.w = 1.0;
  auto kv = [&](const std::string & k, const std::string & v) {
    diagnostic_msgs::msg::KeyValue d;
    d.key = k;
    d.value = v;
    o.details.push_back(d);
  };
  kv("posture", lying ? "lying" : "standing");
  if (!shape.empty()) kv("shape", shape);
  if (!std::isnan(yaw_deg)) {  // perception to_msg와 같음: 방향을 알면 z축 회전 + yaw_period
    o.pose.pose.orientation.z = std::sin(yaw_deg * DEG / 2);
    o.pose.pose.orientation.w = std::cos(yaw_deg * DEG / 2);
    kv("yaw_period", std::to_string(period_deg));
  }
  return o;
}

// 닫힘축(ψ − 90°)과 방향 a가 이루는 각 [0, 90°] — 선은 앞뒤가 같아 mod 180°
double axis_angle_deg(double psi, double a) {
  const double d = std::abs(std::remainder(psi - M_PI / 2 - a, M_PI));
  return d / DEG;
}

// 사각기둥 면 법선(yaw + k·90°)과 닫힘축이 이루는 가장 작은 각 [0, 45°]
double face_err_deg(double psi, double yaw) {
  return std::abs(std::remainder(psi - M_PI / 2 - yaw, M_PI / 2)) / DEG;
}

std::vector<GraspCandidate> grasps(const TrackedObject & t, const std::vector<TrackedObject> & all) {
  std::string note;
  return rule_grasps(t, all, note);
}

}  // namespace

TEST(RuleGrasps, CylinderAloneTwoMapDirections) {
  const auto t = puck(1, 0.3, 0.0, "cyl");
  const auto c = grasps(t, {t});
  ASSERT_EQ(c.size(), 2u);
  EXPECT_NEAR(c[0].yaw, 0.0, 1e-9);
  EXPECT_NEAR(c[1].yaw, M_PI / 2, 1e-9);
  EXPECT_GT(c[0].score, c[1].score);
}

TEST(RuleGrasps, CylinderNeighbourClosingAxisPerpendicular) {
  // 시작 박스: 두 퍽이 y로 80 mm — 닫힘축은 x(옆 퍽 선에 직각)여야 손가락이 옆 퍽을 안 침
  const auto t = puck(1, 0.3, 0.0, "cyl"), nb = puck(2, 0.3, 0.08, "cyl");
  const auto c = grasps(t, {t, nb});
  ASSERT_EQ(c.size(), 1u);
  EXPECT_NEAR(axis_angle_deg(c[0].yaw, M_PI / 2), 90.0, 1e-6);
  EXPECT_FLOAT_EQ(c[0].score, 0.9f);
}

TEST(RuleGrasps, NeighbourTooFarOrNotPuckIgnored) {
  const auto t = puck(1, 0.3, 0.0, "cyl"), far = puck(2, 0.3, 0.2, "cyl");
  auto box = puck(3, 0.3, 0.05);
  box.class_name = "start_box";
  EXPECT_EQ(grasps(t, {t, far, box}).size(), 2u);  // 옆 퍽 없음과 같음
}

TEST(RuleGrasps, BoxAloneAlignsToFaces) {
  const auto t = puck(1, 0.3, 0.0, "box", 30.0, 90);
  const auto c = grasps(t, {t});
  ASSERT_EQ(c.size(), 2u);
  for (const auto & g : c) EXPECT_NEAR(face_err_deg(g.yaw, 30 * DEG), 0.0, 1e-6);
  EXPECT_NEAR(std::abs(std::remainder(c[1].yaw - c[0].yaw, M_PI)) / DEG, 90.0, 1e-6);  // 서로 직교
}

TEST(RuleGrasps, BoxNeighbourFaceWithin45OfPerpendicular) {
  const auto t = puck(1, 0.3, 0.0, "box", 30.0, 90), nb = puck(2, 0.3, 0.08, "cyl");
  const auto c = grasps(t, {t, nb});
  ASSERT_EQ(c.size(), 1u);
  EXPECT_NEAR(face_err_deg(c[0].yaw, 30 * DEG), 0.0, 1e-6);   // 면에 맞음
  EXPECT_GE(axis_angle_deg(c[0].yaw, M_PI / 2), 45.0 - 1e-6);  // 옆 퍽 선에서 45° 넘게 벗어남 (직각 쪽 면)
}

TEST(RuleGrasps, BoxYawWrapsOver90) {
  // perception은 사각기둥 yaw를 [−45°, 45°)로 감지만, 다른 값이 와도 같은 면
  const auto a = puck(1, 0.3, 0.0, "box", 10.0, 90), b = puck(1, 0.3, 0.0, "box", 100.0, 90);
  EXPECT_NEAR(grasps(a, {a})[0].yaw, grasps(b, {b})[0].yaw, 1e-9);
}

TEST(RuleGrasps, BoxYawUnknownFallsBackLowerScore) {
  const auto t = puck(1, 0.3, 0.0, "box"), nb = puck(2, 0.3, 0.08, "cyl");
  const auto c = grasps(t, {t, nb});
  ASSERT_EQ(c.size(), 1u);
  EXPECT_NEAR(axis_angle_deg(c[0].yaw, M_PI / 2), 90.0, 1e-6);
  EXPECT_FLOAT_EQ(c[0].score, 0.7f);  // 방향을 몰라 모서리를 잡을 수 있음
}

TEST(RuleGrasps, LyingPuckIsForStand) {
  // 누운 퍽은 방향을 알아도 위에서 집지 않음 — coordinator stand로 세운 뒤 pick
  for (const auto & t : {puck(1, 0.3, 0.0, "box", 20.0, 180, true), puck(1, 0.3, 0.0, "cyl", NAN, 0, true)}) {
    std::string note;
    EXPECT_TRUE(rule_grasps(t, {t}, note).empty());
    EXPECT_EQ(note, "lying puck: use stand");
  }
}

TEST(TopDown, ZeroYawMatchesCoordinator) {
  const auto q = top_down(0.0);  // coordinator move_arm의 (1, 0, 0, 0)
  EXPECT_DOUBLE_EQ(q.x, 1.0);
  EXPECT_NEAR(q.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(q.z, 0.0);
  EXPECT_DOUBLE_EQ(q.w, 0.0);
}

// ---- /score_grasps 서비스 (노드 + probe를 같은 executor에)

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// 장면을 보내며 서비스를 부르고, 응답이 올 때까지 spin
ScoreGrasps::Response::SharedPtr call(rclcpp::Executor & exec, rclcpp::Publisher<TrackedObjectArray>::SharedPtr pub,
                                      rclcpp::Client<ScoreGrasps>::SharedPtr cli, const TrackedObjectArray & scene,
                                      uint32_t id, bool need_success) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < end) {
    pub->publish(scene);  // discovery 전 메시지는 사라질 수 있어 매번 보냄
    exec.spin_some(std::chrono::milliseconds(50));
    if (!cli->service_is_ready()) continue;
    auto req = std::make_shared<ScoreGrasps::Request>();
    req->object_id = id;
    req->max_candidates = 3;
    auto fut = cli->async_send_request(req);
    if (exec.spin_until_future_complete(fut, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) continue;
    auto res = fut.get();
    if (!need_success || res->success) return res;  // 장면이 아직 안 닿았으면 다시
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return nullptr;
}

TEST_F(RosTest, ServiceReturnsFaceAlignedTopDownGrasp) {
  auto node = std::make_shared<GraspScorer>();
  auto probe = std::make_shared<rclcpp::Node>("probe_grasp");
  auto pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto cli = probe->create_client<ScoreGrasps>("/score_grasps");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  TrackedObjectArray scene;
  scene.header.frame_id = "map";
  scene.objects = {puck(7, 0.3, 0.0, "box", 30.0, 90)};

  const auto res = call(exec, pub, cli, scene, 7, true);
  ASSERT_TRUE(res);
  ASSERT_EQ(res->grasps.size(), 2u);
  EXPECT_EQ(res->grasps[0].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(res->grasps[0].pose.position.x, 0.3);
  const auto & q = res->grasps[0].pose.orientation;  // Rz(ψ)·Rx(π) → ψ = 2·atan2(y, x)
  EXPECT_NEAR(face_err_deg(2 * std::atan2(q.y, q.x), 30 * DEG), 0.0, 1e-6);
  EXPECT_EQ(res->message, "rule: box face");
}

TEST_F(RosTest, ServiceUnknownIdAndLying) {
  auto node = std::make_shared<GraspScorer>();
  auto probe = std::make_shared<rclcpp::Node>("probe_grasp2");
  auto pub = probe->create_publisher<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable());
  auto cli = probe->create_client<ScoreGrasps>("/score_grasps");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  TrackedObjectArray scene;
  scene.objects = {puck(3, 0.3, 0.0, "box", 20.0, 180, true)};

  // 장면이 닿은 뒤(누운 퍽 메시지)에 판정 — 처음엔 '대상 없음'일 수 있음
  ScoreGrasps::Response::SharedPtr res;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < end) {
    res = call(exec, pub, cli, scene, 3, false);
    if (res && res->message == "lying puck: use stand") break;
  }
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_EQ(res->message, "lying puck: use stand");

  const auto missing = call(exec, pub, cli, scene, 99, false);
  ASSERT_TRUE(missing);
  EXPECT_FALSE(missing->success);
  EXPECT_EQ(missing->message, "object 99 not in /tracked_objects");
}

TEST_F(RosTest, ServiceUsesRememberedObjects) {
  // coordinator가 기억한 퍽 (지금 /tracked_objects에 없음) — 요청에 실린 대상·옆 퍽으로 후보, 프레임은 map
  auto node = std::make_shared<GraspScorer>();
  auto probe = std::make_shared<rclcpp::Node>("probe_grasp3");
  auto cli = probe->create_client<ScoreGrasps>("/score_grasps");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  auto req = std::make_shared<ScoreGrasps::Request>();
  req->object_id = 9;
  req->max_candidates = 3;
  req->objects = {puck(9, 0.25, 0.0, "cyl"), puck(4, 0.25, 0.08, "cyl")};  // 옆 퍽이 +y로 8 cm
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  ScoreGrasps::Response::SharedPtr res;
  while (std::chrono::steady_clock::now() < end && !res) {
    exec.spin_some(std::chrono::milliseconds(50));
    if (!cli->service_is_ready()) continue;
    auto fut = cli->async_send_request(req);
    if (exec.spin_until_future_complete(fut, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS) res = fut.get();
  }
  ASSERT_TRUE(res);
  ASSERT_TRUE(res->success) << res->message;
  ASSERT_EQ(res->grasps.size(), 1u);  // 옆 퍽이 있으면 닫힘축이 옆 퍽 선에 직각인 후보 하나
  EXPECT_EQ(res->grasps[0].header.frame_id, "map");
  EXPECT_DOUBLE_EQ(res->grasps[0].pose.position.x, 0.25);
  const auto & q = res->grasps[0].pose.orientation;
  EXPECT_NEAR(axis_angle_deg(2 * std::atan2(q.y, q.x), M_PI / 2), 90.0, 1e-6);  // 닫힘축 ⟂ 옆 퍽 선(+y)
}
