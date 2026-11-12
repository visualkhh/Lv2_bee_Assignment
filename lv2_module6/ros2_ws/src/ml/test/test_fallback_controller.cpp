// RlNavController inner(바이패스) controller 이름 해석 시험 — ROS 없이
#include <gtest/gtest.h>

#include "ml/fallback_controller.hpp"

using ml::Fallback;
using ml::inner_namespace;
using ml::resolve_fallback;

TEST(FallbackController, aliases) {
  struct Case { const char * in; const char * key; const char * cls; int kind; };
  const Case cases[] = {
    {"dwb", "dwb", "dwb_core::DWBLocalPlanner", 1},
    {"DWB", "dwb", "dwb_core::DWBLocalPlanner", 1},
    {"mppi", "mppi", "nav2_mppi_controller::MPPIController", 2},
    {"nav2_mppi_controller::MPPIController", "mppi", "nav2_mppi_controller::MPPIController", 2},
    {"rpp", "rpp", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController", 3},
    {"regulated_pure_pursuit", "rpp", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController", 3},
    {"rotation_shim", "rotation_shim", "nav2_rotation_shim_controller::RotationShimController", 4},
  };
  for (const auto & c : cases) {
    Fallback f;
    ASSERT_TRUE(resolve_fallback(c.in, f)) << c.in;
    EXPECT_EQ(f.key, c.key) << c.in;
    EXPECT_EQ(f.cls, c.cls) << c.in;
    EXPECT_EQ(f.kind, c.kind) << c.in;
  }
}

TEST(FallbackController, emptyIsRejectedAndUnknownIsCustomClass) {
  Fallback f;
  EXPECT_FALSE(resolve_fallback("", f));
  ASSERT_TRUE(resolve_fallback("my_pkg::MyController", f));
  EXPECT_EQ(f.key, "custom");
  EXPECT_EQ(f.cls, "my_pkg::MyController");  // 대소문자 그대로 (pluginlib 이름)
  EXPECT_EQ(f.kind, 0);
}

TEST(FallbackController, parameterNamespaceIsSeparatedPerController) {
  Fallback dwb, mppi, custom;
  resolve_fallback("dwb", dwb);
  resolve_fallback("mppi", mppi);
  resolve_fallback("my_pkg::MyController", custom);
  EXPECT_EQ(inner_namespace("FollowPath", dwb), "FollowPath.inner_dwb");
  EXPECT_EQ(inner_namespace("FollowPath", mppi), "FollowPath.inner_mppi");
  EXPECT_EQ(inner_namespace("FollowPath", custom), "FollowPath.inner");  // 직접 준 클래스는 예전 공간
  EXPECT_NE(inner_namespace("FollowPath", dwb), inner_namespace("FollowPath", mppi));
}
