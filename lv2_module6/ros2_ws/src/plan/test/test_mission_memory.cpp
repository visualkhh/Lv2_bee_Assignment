// puck_for — 집기 직전에 퍽 트랙 다시 찾기 (인지 long_memory: false로 안 보이던 퍽이 다른 id로 다시 나타나도 tag·자리로 찾음)
#include <gtest/gtest.h>

#include "plan/mission_manager.hpp"

using perception::msg::TrackedObject;
using perception::msg::TrackedObjectArray;

namespace {
TrackedObject puck(uint32_t id, double x, double y, const std::string & tag, int sec = 1) {
  TrackedObject o;
  o.id = id;
  o.class_name = "puck_standing";
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.last_seen.sec = sec;
  if (!tag.empty()) { o.details.push_back(diagnostic_msgs::msg::KeyValue().set__key("tag").set__value(tag)); }
  return o;
}
}  // namespace

TEST(PuckFor, FindsByTagEvenWithNewTrackId) {
  TrackedObjectArray a;
  a.objects = {puck(40, 1.0, 1.0, "3"), puck(41, 1.5, 1.0, "7", 2), puck(42, 1.52, 1.0, "7", 5)};
  const auto * o = plan::puck_for(&a, 7, nullptr, 0.04);
  ASSERT_NE(o, nullptr);
  EXPECT_EQ(o->id, 42u);  // 같은 tag면 가장 최근에 본 트랙
}

TEST(PuckFor, FallsBackToSpotWhenTagUnread) {
  TrackedObjectArray a;
  a.objects = {puck(50, 2.00, 1.00, "5"), puck(51, 2.02, 1.01, "")};
  const double spot[2] = {2.0, 1.0};
  const auto * o = plan::puck_for(&a, 9, spot, 0.04);
  ASSERT_NE(o, nullptr);
  EXPECT_EQ(o->id, 51u);  // 다른 tag(5)를 읽은 퍽은 고르지 않음 — tag 못 읽은 근처 퍽
  EXPECT_EQ(plan::puck_for(&a, 9, nullptr, 0.04), nullptr);  // 자리를 모르면 tag로만
  const double far[2] = {3.0, 1.0};
  EXPECT_EQ(plan::puck_for(&a, 9, far, 0.04), nullptr);
  EXPECT_EQ(plan::puck_for(nullptr, 9, spot, 0.04), nullptr);
}
