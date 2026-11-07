// inspect 카메라 자세 (look_plan) — 카메라 FK가 URDF와 같은지 · 퍽 둘을 한 장에 담는 자세를 찾는지 · 같은 입력이면 같은 답
#include <gtest/gtest.h>

#include <cmath>

#include "skills/look_plan.hpp"

using namespace skills::look;

namespace {
double dist(const V3 & a, const V3 & b) { return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2)); }
}  // namespace

TEST(LookPlan, CameraMatchesUrdf) {
  // urdf/default/urdf.xacro (xacro → base_footprint → gripper_cam_color_optical_frame), isaac/scripts/tools/inspect_cover.py Robot.fk
  struct Case { Q5 q; V3 pos, axis; };
  const Case cases[] = {
    {{0.0, -1.682497, 0.324631, 1.926843, 0.0}, {0.085126, 0.032001, 0.312824}, {0.637467, -0.001047, -0.770477}},
    {{-0.2, -1.2, 0.3, 1.8, 0.0}, {0.139103, 0.014186, 0.304365}, {0.345213, -0.071046, -0.935831}},
    {{0.3, -0.7, 0.1, 1.5, 0.4}, {0.202071, 0.061538, 0.327617}, {0.319566, 0.222431, -0.921087}},
  };
  for (const auto & c : cases) {
    V3 o;
    std::array<V3, 3> ax;
    camera(c.q, o, ax);
    EXPECT_LT(dist(o, c.pos), 1e-5);
    EXPECT_LT(dist(ax[2], c.axis), 1e-5);
  }
}

TEST(LookPlan, ProjectMatchesIsaacImage) {
  // 11/07 Isaac 전체 스택: 사용자 자세에서 퍽 윗면 태그 8 (0.296, −0.04, 0.1305) 이 화면 (479, 136)에 찍힘
  double u, v, d;
  ASSERT_TRUE(project({0.0, -1.682497, 0.324631, 1.926843, 0.0}, {0.296, -0.04, 0.1305}, Params{}, u, v, d));
  EXPECT_NEAR(u, 479, 3);
  EXPECT_NEAR(v, 136, 3);
}

TEST(LookPlan, SearchCoversTwoPucks) {
  const Params prm;
  const std::vector<V3> tops = {{0.296, -0.04, 0.13}, {0.248, 0.062, 0.13}};
  Q5 q, q2;
  Score s, s2;
  ASSERT_TRUE(search(tops, prm, q, s, 0));
  EXPECT_DOUBLE_EQ(s.cover, 1.0);
  EXPECT_GE(s.min_px, 35);
  EXPECT_LT(s.aim_off, 30);
  EXPECT_EQ(q[4], 0.0);
  ASSERT_TRUE(search(tops, prm, q2, s2, 0));  // seed가 같으면 같은 답
  for (int i = 0; i < 5; ++i) EXPECT_DOUBLE_EQ(q[i], q2[i]);
  for (const auto & p : tops) {  // 두 퍽 모두 화면 안 · 손가락 밖
    double u, v, d;
    ASSERT_TRUE(project(q, p, prm, u, v, d));
    EXPECT_GT(u, prm.edge_px);
    EXPECT_LT(u, prm.width - prm.edge_px);
    EXPECT_GT(v, prm.edge_px);
    EXPECT_FALSE(u > prm.finger_u0 && u < prm.finger_u1 && v > prm.finger_v0);
  }
}

TEST(LookPlan, NothingToSee) {
  Q5 q;
  Score s;
  EXPECT_FALSE(search({}, Params{}, q, s));
}
