// mission_solver 테스트 — 발제 공개 예제 Q1 · 동률 · 바코드 · 할당(풀 수 있음/없음) · 주사위 칸 · 좌표 (documents/mission/README.md)
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <tuple>

#include "plan/mission_solver.hpp"

using namespace plan;

namespace {
std::vector<Puck> make(int red_cyl, int red_cub, int green_cyl, int green_cub) {
  std::vector<Puck> out;
  for (auto [color, shape, n] : {std::tuple{"red", "cylinder", red_cyl}, {"red", "cuboid", red_cub},
                                 {"green", "cylinder", green_cyl}, {"green", "cuboid", green_cub}})
    for (int i = 0; i < n; ++i) out.push_back({static_cast<int>(out.size()), color, shape});
  return out;
}
int count(const std::map<int, char> & a, char g) {
  return static_cast<int>(std::count_if(a.begin(), a.end(), [&](const auto & kv) { return kv.second == g; }));
}
}  // namespace

TEST(MissionSolver, Q1) {
  const Counts c{4, 2, 6};  // R G R R G R G G Y G Y G
  std::string why;
  EXPECT_TRUE(valid_counts(c, why));
  bool asc = false;
  ASSERT_TRUE(parse_barcode("P2-01", asc));
  EXPECT_TRUE(asc);
  EXPECT_EQ(assign_locations(c, asc), "YRG");  // L1 = Y 2 · L2 = R 4 · L3 = G 6
  EXPECT_EQ(assign_locations(c, false), "GRY");
  std::map<int, char> a;
  ASSERT_TRUE(allocate(make(3, 3, 3, 3), c, a, why)) << why;  // 빨강 6 · 초록 6 → 초록 전부 G · 빨강 같은 모양 2개 Y
  EXPECT_EQ(count(a, 'R'), 4);
  EXPECT_EQ(count(a, 'Y'), 2);
  EXPECT_EQ(count(a, 'G'), 6);
}

TEST(MissionSolver, TieKeepsRYG) {
  EXPECT_EQ(assign_locations({4, 4, 4}, true), "RYG");
  EXPECT_EQ(assign_locations({4, 4, 4}, false), "RYG");
  EXPECT_EQ(assign_locations({5, 2, 5}, true), "YRG");
  EXPECT_EQ(assign_locations({5, 2, 5}, false), "RGY");
}

TEST(MissionSolver, InvalidSignalAndBarcode) {
  std::string why;
  EXPECT_FALSE(valid_counts({7, 3, 2}, why));
  EXPECT_EQ(why, "count_out_of_range");
  EXPECT_FALSE(valid_counts({4, 2, 5}, why));
  EXPECT_EQ(why, "sum_not_12");
  bool asc;
  EXPECT_TRUE(parse_barcode("P2-04", asc));
  EXPECT_FALSE(asc);
  EXPECT_FALSE(parse_barcode("P2-05", asc));
}

TEST(MissionSolver, AllocationNeedsWholePlan) {
  // 발제 p5 예: 빨강 8 · 초록 4, 같은 모양 빨강 3개 → R5 · Y3 · G4
  std::map<int, char> a;
  std::string why;
  ASSERT_TRUE(allocate(make(3, 5, 2, 2), {5, 3, 4}, a, why)) << why;
  EXPECT_EQ(count(a, 'Y'), 3);
  // Y에 빨강 1 + 초록 1이 같은 모양이어야 — 원기둥은 빨강뿐이라 안 되고 직육면체로 풂 (색 섞임 허용)
  ASSERT_TRUE(allocate(make(4, 3, 0, 5), {6, 2, 4}, a, why)) << why;
  for (const auto & [tag, g] : a) if (g == 'Y') EXPECT_GE(tag, 4);  // 0~3 = 빨강 원기둥 → Y 아님
  // 풀 수 없음: Y = 빨강 2 + 초록 4를 한 모양으로 못 만듦 (빨강 원기둥 1개뿐 · 초록 직육면체 3개뿐)
  EXPECT_FALSE(allocate(make(1, 3, 5, 3), {2, 6, 4}, a, why));
  EXPECT_EQ(why, "no_allocation");
  // 모양 모름 → 추정하지 않음
  auto p = make(3, 3, 3, 3);
  p[0].shape = "";
  EXPECT_FALSE(allocate(p, {4, 2, 6}, a, why));
  EXPECT_EQ(why, "unknown_puck");
  p = make(3, 3, 3, 3);
  p[1].tag = 0;
  EXPECT_FALSE(allocate(p, {4, 2, 6}, a, why));
  EXPECT_EQ(why, "duplicate_tag");
}

TEST(MissionSolver, DiceAndGeometry) {
  EXPECT_EQ(dice_cells(2), (std::vector<int>{0, 8}));
  EXPECT_EQ(dice_cells(6).size(), 6u);
  EXPECT_TRUE(dice_cells(7).empty());
  auto [dx, dy] = cell_offset(0);
  EXPECT_DOUBLE_EQ(dx, -0.06);  // 북서
  EXPECT_DOUBLE_EQ(dy, 0.06);
  std::tie(dx, dy) = cell_offset(8);
  EXPECT_DOUBLE_EQ(dx, 0.06);
  EXPECT_DOUBLE_EQ(dy, -0.06);
  // 출발 (0.734, 0.950)에서 −X를 보고 켬 → L1 (3.670, 0.950)은 로봇 뒤 2.936 m
  const Pose2 start{0.734, 0.950, M_PI};
  const auto l1 = arena_to_map({3.670, 0.950, 0.0}, start);
  EXPECT_NEAR(l1.x, -2.936, 1e-9);
  EXPECT_NEAR(l1.y, 0.0, 1e-9);
  EXPECT_NEAR(std::abs(l1.yaw), M_PI, 1e-9);
  // L1 박스(205) 서쪽 칸 → 서쪽 변에서 동쪽을 보고 섬
  const auto a = approach_poses(3.670, 0.950, 0.1025, 3.610, 0.950, 0.20);
  EXPECT_NEAR(a[0].x, 3.670 - 0.1025 - 0.20, 1e-9);
  EXPECT_NEAR(a[0].y, 0.950, 1e-9);
  EXPECT_NEAR(a[0].yaw, 0.0, 1e-9);
}

TEST(MissionFlow, NextPhase) {  // 설계 D1·D9 — 관측 실패만 정지, 그 뒤는 끝까지 (복귀까지)
  EXPECT_EQ(next_phase(Phase::IDLE, true), Phase::GO_OBSERVE);
  EXPECT_EQ(next_phase(Phase::READ, true), Phase::SURVEY);
  EXPECT_EQ(next_phase(Phase::BARCODE, false), Phase::ERROR);
  EXPECT_EQ(next_phase(Phase::READ, false), Phase::ERROR);
  EXPECT_EQ(next_phase(Phase::SURVEY, false), Phase::RETURN);
  EXPECT_EQ(next_phase(Phase::BUILD, false), Phase::VERIFY);
  EXPECT_EQ(next_phase(Phase::VERIFY, false), Phase::RETURN);
  EXPECT_EQ(next_phase(Phase::RETURN, true), Phase::DONE);
  EXPECT_EQ(next_phase(Phase::RETURN, false), Phase::ERROR);
  EXPECT_EQ(next_phase(Phase::DONE, true), Phase::DONE);
  EXPECT_STREQ(phase_name(Phase::VERIFY), "VERIFY");
}

TEST(MissionFlow, TagCorrection) {  // 설계 D4
  auto [dx, dy] = tag_correction(1.0, 2.0, 1.012, 1.995, 0.05);
  EXPECT_NEAR(dx, 0.012, 1e-9);
  EXPECT_NEAR(dy, -0.005, 1e-9);
  std::tie(dx, dy) = tag_correction(1.0, 2.0, 1.2, 2.0, 0.05);  // 20 cm — 잘못 본 것
  EXPECT_EQ(dx, 0.0);
  EXPECT_EQ(dy, 0.0);
}

TEST(MissionFlow, JudgeLocation) {  // 설계 D11 — 칸별 tag·오차 · 빈 칸 퍽
  const std::vector<std::pair<int, int>> want{{3, 0}, {7, 8}};  // 주사위 2: 북서 · 남동
  const auto [x0, y0] = cell_offset(0);
  const auto [x8, y8] = cell_offset(8);
  auto r = judge_location(want, 1.0, 1.0, 0.060, {{3, 1.0 + x0 + 0.004, 1.0 + y0}, {7, 1.0 + x8, 1.0 + y8 - 0.006}}, 0.010);
  EXPECT_TRUE(r.ok);
  ASSERT_EQ(r.cells.size(), 2u);
  EXPECT_NEAR(r.cells[0].err, 0.004, 1e-9);
  r = judge_location(want, 1.0, 1.0, 0.060, {{3, 1.0 + x0 + 0.02, 1.0 + y0}, {7, 1.0 + x8, 1.0 + y8}}, 0.010);
  EXPECT_FALSE(r.ok);  // 2 cm 벗어남
  r = judge_location(want, 1.0, 1.0, 0.060, {{3, 1.0 + x0, 1.0 + y0}, {7, 1.0 + x8, 1.0 + y8}, {-1, 1.0, 1.0}}, 0.010);
  EXPECT_FALSE(r.ok);  // 가운데 칸에 번호 못 읽은 퍽
  ASSERT_EQ(r.extra.size(), 1u);
  r = judge_location(want, 1.0, 1.0, 0.060, {{3, 1.0 + x0, 1.0 + y0}}, 0.010);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.cells[1].found);
}

TEST(MissionFlow, GlobalCheck) {  // 설계 D11 — 12개 중복·누락
  std::vector<int> dup, missing;
  global_check({{0, 1, 2}, {2, 3}, {4, -1}}, {0, 1, 2, 3, 4, 5}, dup, missing);
  EXPECT_EQ(dup, std::vector<int>{2});
  EXPECT_EQ(missing, std::vector<int>{5});
}

TEST(MissionFlow, FootprintInZone) {  // 설계 D12 — 출발 구역 600×600 · 로봇 외곽 (앞 0.069 · 뒤 −0.233 · 옆 ±0.153)
  const std::vector<double> zone{0.434, 0.650, 1.034, 1.250};
  EXPECT_TRUE(footprint_in_zone({0.734, 0.950, M_PI}, 0.069, -0.233, 0.153, zone));
  EXPECT_FALSE(footprint_in_zone({0.480, 0.950, M_PI}, 0.069, -0.233, 0.153, zone));  // −X를 봄 → 앞 모서리 x 0.411 < 0.434
  EXPECT_FALSE(footprint_in_zone({0.734, 0.950, 0.0}, 0.069, -0.233, 0.153, {0.434, 0.650, 0.780, 1.250}));  // +X를 봄 → 앞 0.803 > 0.780
}
