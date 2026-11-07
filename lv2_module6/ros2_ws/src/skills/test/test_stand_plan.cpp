// 세우기 경유점 (stand_plan) — 기구학 · 해석 IK · 팀원 right_env 계획과 같은지 · 표 모드 · 시간 매기기
#include <gtest/gtest.h>

#include <cmath>

#include "skills/stand_plan.hpp"

using namespace skills::stand;

namespace {
double dist(const V3 & a, const V3 & b) { return std::sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2)); }
}  // namespace

TEST(StandPlan, ForwardKinematicsMatchesUrdf) {
  // pinocchio(urdf/beebot_mesh/beebot.urdf, base_footprint 기준)로 구한 값
  V3 axis;
  const V3 p = fk({0.3, -0.5, 0.9, 1.1, -0.7}, &axis);
  EXPECT_NEAR(dist(p, {0.130211472, 0.025205628, 0.126699656}), 0.0, 1e-8);
  EXPECT_NEAR(dist(axis, {-0.839929288, 0.540779209, -0.045570156}), 0.0, 1e-8);
}

TEST(StandPlan, InverseKinematicsHandDown) {
  for (double x : {0.14, 0.19, 0.24})
    for (double y : {-0.06, 0.0, 0.05})
      for (double z : {0.09, 0.13, 0.17})  // 세우기 경유점 높이 범위 (멀고 높은 곳은 손을 수직으로 못 둠)
        for (double cy : {-1.2, 0.0, 0.7, 2.5}) {
          Q5 q;
          ASSERT_TRUE(ik_down({x, y, z}, cy, {0, -0.2, 0.7, 1.0, 0}, q)) << x << " " << y << " " << z;
          V3 axis;
          const V3 p = fk(q, &axis);
          EXPECT_NEAR(dist(p, {x, y, z}), 0.0, 1e-9);
          EXPECT_NEAR(axis[2], 0.0, 1e-9);  // 닫는 축 수평 = 손이 바로 아래
          EXPECT_NEAR(std::remainder(std::atan2(axis[1], axis[0]) - cy, 2 * M_PI), 0.0, 1e-9);
        }
  Q5 q;
  EXPECT_FALSE(ik_down({0.8, 0.0, 0.1}, 0.0, {0, -0.2, 0.7, 1.0, 0}, q));  // 닿지 않음
}

TEST(StandPlan, MatchesTeammateRightEnvPlan) {
  // feature/isaac-lab right_env.plan_waypoints (tilt, regrasp) — 퍽 가운데 (0.19, 0.03) · 축 35° · 사각.
  // 팀원 쪽은 수치 IK(수렴 2e-4 m · 2e-3 rad)라 관절값이 그만큼 다를 수 있음
  std::vector<Q5> qs;
  std::string why;
  ASSERT_TRUE(plan_ik({0.19, 0.03, 0.07}, 35 * M_PI / 180, true, Params{}, qs, why)) << why;
  ASSERT_EQ(qs.size(), 27u);  // 위 · 호 17 · 멈춤 · 되돌림 4 · 놓기 · 위로 · 다시 잡기 2
  const std::vector<std::pair<int, Q5>> ref = {
    {0, {0.277328, 0.214378, -0.051033, 1.407307, -0.333538}},
    {1, {0.277413, 0.309478, 0.146124, 1.115196, -0.333453}},
    {24, {-0.045030, -0.392858, 0.129647, 1.834005, -0.655895}},
    {25, {0.041870, -0.249483, 0.663779, 1.156501, -0.568995}},
    {26, {0.042000, -0.330033, 0.322743, 1.578084, -0.568865}}};
  for (const auto & [i, q] : ref)
    for (int j = 0; j < 5; ++j) EXPECT_NEAR(qs[i][j], q[j], 3e-3) << "waypoint " << i << " joint " << j + 1;
}

TEST(StandPlan, WaypointGeometry) {
  double cy;
  const V3 c = {0.19, 0.0, 0.07};
  const auto pts = waypoints(c, 0.0, true, Params{}, cy);
  ASSERT_EQ(pts.size(), 27u);
  EXPECT_NEAR(cy, M_PI / 2, 1e-12);  // 닫는 축 ⟂ 퍽 축
  EXPECT_NEAR(pts[0][2] - pts[1][2], 0.04, 1e-12);                 // 위에서 내려감
  EXPECT_NEAR(pts[1][0], c[0] + PUCK_L / 2, 1e-12);                // 먼 끝을 쥠
  EXPECT_NEAR(pts[24][2] - pts[23][2], 0.08, 1e-12);               // 벌린 뒤 위로
  EXPECT_NEAR(pts[25][0], c[0] - PUCK_L / 2 - PUCK_W / 2, 1e-12);  // 다시 잡기 = 선 퍽 중심선
  EXPECT_NEAR(pts[25][2], c[2] + GRASP_Z, 1e-12);
  EXPECT_NEAR(away_yaw(M_PI), 0.0, 1e-12);  // 180° 대칭 → 로봇에서 먼 쪽
  EXPECT_NEAR(away_yaw(-2.0), -2.0 + M_PI, 1e-12);
}

TEST(StandPlan, TableNearestBinAndLinearCorrection) {
  // 표 = 격자점마다 해석 IK + 손끝 위치 Jacobian(수치 미분) — 팀원 표와 같은 배열 순서
  Table t;
  t.yaw_deg = {-10, -5, 0, 5, 10};
  t.ycs = {-0.04, 0.0, 0.04};
  t.box_cx = 0.1825;
  t.box_h = 0.07;
  t.w = 27;
  std::string why;
  EXPECT_FALSE(t.ready(why));
  for (int k = 0; k < 2; ++k)
    for (double yc : t.ycs)
      for (double yd : t.yaw_deg) {
        std::vector<Q5> q0, qx, qy;
        const double yw = yd * M_PI / 180, h = 1e-4;
        ASSERT_TRUE(plan_ik({t.box_cx, yc, 0.07}, yw, k == 1, Params{}, q0, why));
        ASSERT_TRUE(plan_ik({t.box_cx + h, yc, 0.07}, yw, k == 1, Params{}, qx, why));
        ASSERT_TRUE(plan_ik({t.box_cx, yc + h, 0.07}, yw, k == 1, Params{}, qy, why));
        for (int w = 0; w < t.w; ++w)
          for (int j = 0; j < 5; ++j) {
            t.q[k].push_back(q0[w][j]);
            t.P[k].insert(t.P[k].end(), {(qx[w][j] - q0[w][j]) / h, (qy[w][j] - q0[w][j]) / h, 0.0});
          }
      }
  ASSERT_TRUE(t.ready(why));
  std::vector<Q5> qt, qi;
  ASSERT_TRUE(plan_table(t, {0.1825, 0.04, 0.07}, 5 * M_PI / 180, true, qt, why));  // 격자점 = 표 그대로
  ASSERT_TRUE(plan_ik({0.1825, 0.04, 0.07}, 5 * M_PI / 180, true, Params{}, qi, why));
  for (int w = 0; w < 27; ++w) for (int j = 0; j < 5; ++j) EXPECT_NEAR(qt[w][j], qi[w][j], 1e-9);
  ASSERT_TRUE(plan_table(t, {0.19, 0.012, 0.07}, 0.0, false, qt, why));  // 격자 사이 → 위치 선형 보정
  ASSERT_TRUE(plan_ik({0.19, 0.012, 0.07}, 0.0, false, Params{}, qi, why));
  for (int w = 0; w < 27; ++w) EXPECT_LT(dist(fk(qt[w]), fk(qi[w])), 3e-3) << w;  // 선형 보정 오차 (Δ 7.5mm·12mm에서 호 가운데가 가장 큼)
}

TEST(StandPlan, Timing) {
  EXPECT_NEAR(trapezoid_duration(0.0, 0.3), 0.1, 1e-12);
  EXPECT_NEAR(trapezoid_duration(3.0, 1.0), 3.0 / 3.0 + 3.0 / 5.0, 1e-12);
  EXPECT_NEAR(move_s(0.5, 1.0), 0.5, 1e-12);
  const Q5 a = {0, 0, 0, 0, 0}, b = {0.5, -0.2, 0.1, 0, 0};
  const auto mv = sample_move(a, b, 0.3, 0.02);
  for (int j = 0; j < 5; ++j) EXPECT_NEAR(mv.back()[j], b[j], 1e-12);
  EXPECT_LT(std::abs(mv.front()[0]), 0.01);  // 시각 dt — 지금 자리에서 조금만
  std::vector<Q5> qs;
  std::string why;
  ASSERT_TRUE(plan_ik({0.19, 0.0, 0.07}, 0.3, true, Params{}, qs, why));
  const std::vector<Q5> wp(qs.begin(), qs.end() - 2);  // 다시 잡기 2개 빼고 (right_env)
  const auto groups = arc_groups(Params{}, static_cast<int>(wp.size()) - 3);
  ASSERT_EQ(groups.size(), 3u);
  EXPECT_EQ(groups[0], std::make_pair(0, 16));  // 0 → 120° (7.5°마다)
  EXPECT_EQ(groups[1], std::make_pair(16, 17));  // 멈춤
  const auto arc = sample_arc(wp, Params{}, 0.5, 0.5, 0.02);
  for (int j = 0; j < 5; ++j) EXPECT_NEAR(arc.back()[j], wp[wp.size() - 2][j], 1e-9);  // 내려놓기 자리에서 끝
  for (size_t i = 1; i < arc.size(); ++i)
    for (int j = 0; j < 5; ++j) EXPECT_LT(std::abs(arc[i][j] - arc[i - 1][j]), 0.05) << i;  // smooth step 최고 속도 ≤ 1.5 × (0.5 × 3 rad/s)
}

TEST(StandPlan, DragWaypoints) {  // 실기 대각선 끌기: 먼 끝을 쥐고 로봇 쪽으로 끌며 올린 뒤 눌러 놓음
  Params p;
  p.drag_m = 0.04;
  p.drag_lift_m = 0.035;
  p.set_dz_m = -0.02;
  double cy;
  const V3 c = {0.19, 0.0, 0.07};
  const auto pts = waypoints(c, 0.0, true, p, cy);
  ASSERT_EQ(pts.size(), 11u);  // 위 · 쥐기 · 끌기 4 · 멈춤 · 놓기 · 위로 · 다시 잡기 2
  EXPECT_NEAR(pts[1][0], c[0] + PUCK_L / 2, 1e-12);
  EXPECT_NEAR(pts[5][0] - pts[1][0], -0.04, 1e-12);
  EXPECT_NEAR(pts[5][2] - pts[1][2], 0.035, 1e-12);
  EXPECT_NEAR(pts[7][2] - pts[6][2], -0.02, 1e-12);
  std::vector<Q5> qs;
  std::string why;
  ASSERT_TRUE(plan_ik(c, 0.0, true, p, qs, why)) << why;
  const std::vector<Q5> wp(qs.begin(), qs.end() - 2);
  EXPECT_EQ(arc_groups(p, static_cast<int>(wp.size()) - 3)[0], std::make_pair(0, 4));
  const auto path = sample_arc(wp, p, 0.5, 0.5, 0.02);
  for (int j = 0; j < 5; ++j) EXPECT_NEAR(path.back()[j], wp[wp.size() - 2][j], 1e-9);  // 눌러 놓는 자리에서 끝
}

TEST(StandPlan, LowersAboveWhenUnreachable) {  // 대기 높이를 못 가면 낮춰서 품
  Params p;
  p.above_m = 0.5;
  std::vector<Q5> qs;
  std::string why;
  ASSERT_TRUE(plan_ik({0.19, 0.0, 0.07}, 0.0, true, p, qs, why)) << why;
  EXPECT_NEAR(fk(qs[0])[2] - fk(qs[1])[2], 0.03, 1e-6);
}

TEST(StandPlan, SymmetricCloseAxisPicksNearestWrist) {  // 손가락 대칭 — 같은 파지 중 손목이 덜 도는 쪽
  const V3 p = {0.2, 0.0, 0.15};
  const Q5 near = {0.0, 0.0, 0.5, 1.0, 0.0};
  Q5 q;
  ASSERT_TRUE(ik_down_sym(p, M_PI / 2 + M_PI, near, q));  // 닫는 축 270° = 90°와 같음 → joint5 ≈ 0
  EXPECT_NEAR(q[4], 0.0, 0.01);  // joint1이 팔 받침 옆 오프셋만큼 살짝 돎
  V3 axis;
  const V3 tip = fk(q, &axis);
  EXPECT_LT(dist(tip, p), 1e-6);
  EXPECT_NEAR(std::abs(axis[1]), 1.0, 1e-6);  // 닫는 축이 y 방향
  EXPECT_NEAR(axis[2], 0.0, 1e-6);
  EXPECT_FALSE(ik_down_sym({0.9, 0.0, 0.15}, 0.0, near, q));  // 안 닿음
}

TEST(StandPlan, StandingGraspHeightIgnoresPerceivedFace) {  // 인지 z가 윗면 위·앞면이어도 상자 윗면 + GRASP_Z
  EXPECT_NEAR(standing_grasp_z(0.152), 0.07 + GRASP_Z, 1e-12);
  EXPECT_NEAR(standing_grasp_z(0.104), 0.07 + GRASP_Z, 1e-12);
  EXPECT_NEAR(standing_grasp_z(0.05), GRASP_Z, 1e-12);  // 바닥
}
