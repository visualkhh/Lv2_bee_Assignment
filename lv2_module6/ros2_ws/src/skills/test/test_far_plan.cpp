// 먼 퍽 경유점 (far_plan) — 시뮬 pick_env._build_far_plan(pinocchio 수치 IK)과 같은 관절값이 나오는지
//   기준값: pick_env.py spot()을 그대로 풀어 출력 (Isaac Lab venv, 2026-11-12). 시뮬 좌표는 base_link(바닥 위 1 cm)라 퍽 바닥 z 0.06
//   = base_footprint 0.07 (박스 윗면)
#include <gtest/gtest.h>

#include <cmath>

#include "skills/far_plan.hpp"

using skills::far::Q5;
using skills::far::V3;

namespace {
constexpr double TOL = 0.01;  // [rad] 시뮬 수치 IK 허용 오차 (위치 2 mm · 각 2°)보다 작게

void expect_q(const Q5 & got, const Q5 & want, const char * what) {
  for (int i = 0; i < 5; ++i) EXPECT_NEAR(got[i], want[i], TOL) << what << " joint" << i + 1;
}

struct Ref {
  double x, y, roll_deg;  // 퍽 바닥 가운데 [m] · 손목 돌림 (퍽 면) [deg]
  Q5 retreat, in, k60, k75, k90, down, lift;
};
// 시뮬 출력 (far_ref.py) — 손 방향 = joint1 방향 + roll, 손가락 닫는 축 = 손 방향 + 90°
const Ref REFS[] = {
  {0.27, 0.0, 0,
   {0.001, -0.9136, 1.0296, 0.6695, 0.0}, {0.001, -0.0139, 0.9787, -0.179, 0.0}, {0.001, 0.0411, 0.575, 0.4295, 0.0},
   {0.001, 0.2772, 0.0768, 0.9533, 0.0}, {0.001, 0.6635, -0.6132, 1.5188, 0.0}, {0.001, 0.6635, -0.6132, 1.5188, 0.0},
   {0.001, 0.6285, -0.9595, 1.9018, 0.0}},
  {0.26, 0.06, 20,
   {0.2768, -0.9322, 1.0349, 0.6827, 0.0}, {0.2768, -0.024, 0.9919, -0.182, 0.0}, {0.2768, 0.0301, 0.589, 0.4264, 0.0},
   {0.2768, 0.2655, 0.0942, 0.9476, 0.0}, {0.2768, 0.6461, -0.5823, 1.5053, 0.0}, {0.2768, 0.6506, -0.5906, 1.5108, -0.3477},
   {0.2768, 0.6962, -1.0845, 1.959, -0.3491}},
  {0.25, -0.08, -30,
   {-0.3761, -0.9696, 1.045, 0.71, 0.0}, {-0.3761, -0.0444, 1.0179, -0.1877, 0.0}, {-0.3761, 0.0082, 0.6167, 0.4206, 0.0},
   {-0.3761, 0.2425, 0.1283, 0.9365, 0.0}, {-0.3761, 0.613, -0.5239, 1.48, 0.0}, {-0.3761, 0.6172, -0.5314, 1.4851, 0.5226},
   {-0.3761, 0.6311, -0.9642, 1.9039, 0.5236}},
};
}  // namespace

// ik_tilt는 손 기울기 a에서 stand::fk와 같은 손끝 위치 · 손 피치 (joint2+3+4 = a)
TEST(FarPlan, IkTiltMatchesFk) {
  for (double a_deg : {45.0, 60.0, 75.0, 90.0}) {
    const double a = a_deg * M_PI / 180;
    const V3 p = {0.25, 0.03, 0.12};
    const double hd = std::atan2(p[1] + 0.00023, p[0] - 0.047985);
    Q5 q;
    ASSERT_TRUE(skills::far::ik_tilt(p, a, hd, 0.2, Q5{hd, -0.2, 0.7, 1.0, 0.0}, q)) << a_deg;
    const V3 f = skills::stand::fk(q);
    for (int i = 0; i < 3; ++i) EXPECT_NEAR(f[i], p[i], 1e-6) << a_deg;
    EXPECT_NEAR(q[1] + q[2] + q[3], a, 1e-9);
    EXPECT_NEAR(q[4], 0.2, 1e-9);
  }
}

// 계획 전체가 시뮬 표와 같음 (뒤로 뺀 자세 · 들어간 자세 · 세우기 경유점 · 쥐는 자세 · 들어올린 자세)
TEST(FarPlan, MatchesSimPlan) {
  for (const auto & r : REFS) {
    const V3 pb = {r.x, r.y, 0.07};
    const double hd = r.retreat[0];
    const double close_yaw = hd + r.roll_deg * M_PI / 180 + M_PI / 2;
    skills::far::Plan p;
    std::string why;
    ASSERT_TRUE(skills::far::plan(pb, close_yaw, skills::far::Params{}, p, why)) << r.x << " " << r.y << " " << why;
    ASSERT_EQ(p.path.size(), 6u);  // 뒤로 뺀 · 들어간 · 60° · 75° · 90° · 쥐는
    expect_q(p.path[0], r.retreat, "retreat");
    expect_q(p.path[1], r.in, "in");
    expect_q(p.path[2], r.k60, "k60");
    expect_q(p.path[3], r.k75, "k75");
    expect_q(p.path[4], r.k90, "k90");
    expect_q(p.down(), r.down, "down");
    expect_q(p.lift, r.lift, "lift");
  }
}

// 박스보다 훨씬 먼 퍽은 못 닿음 → pick_far_unreachable
TEST(FarPlan, TooFar) {
  skills::far::Plan p;
  std::string why;
  EXPECT_FALSE(skills::far::plan({0.40, 0.0, 0.07}, M_PI / 2, skills::far::Params{}, p, why));
  EXPECT_EQ(why, "pick_far_unreachable");
}

// 경로 샘플: 끝은 정확히 쥐는 자세, 이웃 샘플 차는 작음 (경유점에서 튀지 않음), 첫 샘플은 출발 자세 바로 옆
TEST(FarPlan, SamplePath) {
  skills::far::Plan p;
  std::string why;
  ASSERT_TRUE(skills::far::plan({0.27, 0.0, 0.07}, M_PI / 2, skills::far::Params{}, p, why));
  const auto s = skills::far::sample_path(p.path, 0.3, 0.02);
  ASSERT_GT(s.size(), 10u);
  for (int j = 0; j < 5; ++j) EXPECT_NEAR(s.back()[j], p.down()[j], 1e-9);
  Q5 prev = p.path.front();
  for (const auto & q : s) {
    for (int j = 0; j < 5; ++j) EXPECT_LT(std::abs(q[j] - prev[j]), 0.05);  // 0.02 s에 0.05 rad 아래 (팀 한계 × 0.3 = 0.9 rad/s)
    prev = q;
  }
}
