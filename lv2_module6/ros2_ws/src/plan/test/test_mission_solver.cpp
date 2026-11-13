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
  for (auto [color, shape, n] : {std::tuple{"red", "cyl", red_cyl}, {"red", "box", red_cub},
                                 {"green", "cyl", green_cyl}, {"green", "box", green_cub}})
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
  EXPECT_EQ(dice_cells(3).front(), 4);  // 가운데 먼저
  EXPECT_EQ(dice_cells(5).front(), 4);
  EXPECT_EQ(dice_cells(6).size(), 6u);
  EXPECT_TRUE(dice_cells(7).empty());
  // 놓는 순서: 로봇 기준 뒤 → 앞, 같은 줄은 왼 → 오 (남쪽에서 북쪽을 보고 서면 북쪽 줄이 뒤, 서쪽 칸이 왼쪽)
  EXPECT_EQ(dice_cells_ordered(6, M_PI / 2), (std::vector<int>{0, 2, 3, 5, 6, 8}));
  EXPECT_EQ(dice_cells_ordered(6, -M_PI / 2), (std::vector<int>{8, 6, 5, 3, 2, 0}));  // 북쪽에서 남쪽을 봄
  EXPECT_EQ(dice_cells_ordered(6, 0.0), (std::vector<int>{2, 5, 8, 0, 3, 6}));  // 서쪽에서 동쪽을 봄: 동쪽 열이 뒤
  EXPECT_EQ(dice_cells_ordered(5, M_PI / 2), (std::vector<int>{0, 2, 4, 6, 8}));  // 가운데도 순서대로
  EXPECT_EQ(dice_cells_ordered(5, M_PI / 2).size(), dice_cells(5).size());
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

TEST(MissionFlow, NextPhase) {  // — 관측 실패만 정지, 그 뒤는 끝까지 (복귀까지)
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

TEST(MissionFlow, DiagNextPhase) {  // 현장 진단 — 관측 실패도 다음 단계로 (SURVEY까지 자료 모음), 복귀 실패만 ERROR
  EXPECT_EQ(next_phase(Phase::GO_OBSERVE, false, true), Phase::BARCODE);
  EXPECT_EQ(next_phase(Phase::BARCODE, false, true), Phase::READ);
  EXPECT_EQ(next_phase(Phase::READ, false, true), Phase::SURVEY);
  EXPECT_EQ(next_phase(Phase::SURVEY, false, true), Phase::RETURN);
  EXPECT_EQ(next_phase(Phase::BUILD, false, true), Phase::VERIFY);
  EXPECT_EQ(next_phase(Phase::RETURN, false, true), Phase::ERROR);
  EXPECT_EQ(next_phase(Phase::RETURN, true, true), Phase::DONE);
}

TEST(MissionFlow, DiagFatal) {  // 치명(정지) vs 계속
  int stuck = 0;
  for (const char * why : {"estop", "estop_blocked", "manual_override", "manual_teach_override", "explore_override",
                           "cancelled", "preempted", "aborted", "arm_contact", "retreat:arm_contact", "collision_monitor_stop",
                           "overcurrent", "coordinator_unavailable",
                           "motor_error: 모터 에러: joint2 과부하", "arm_sag:joint2", "barcode:arm_sag:joint2"})
    EXPECT_TRUE(diag_fatal(false, why, stuck, 3)) << why;
  for (const char * why : {"barcode_missing", "unknown_barcode:X", "signal_timeout", "signal_count", "approach_blocked",
                           "approach_failed", "grasp_empty", "grasp_failed_all", "place_failed", "pick_timeout", "puck_lost",
                           "no_order", "stand_still_lying", ""})
    EXPECT_FALSE(diag_fatal(false, why, stuck, 3)) << why;
  EXPECT_EQ(stuck, 0);
  // 주행 갇힘 — 이어서 3번째에 치명 · 사이에 성공이 있으면 다시 0부터
  stuck = 0;
  EXPECT_FALSE(diag_fatal(false, "nav_no_progress", stuck, 3));
  EXPECT_FALSE(diag_fatal(false, "nav_failed", stuck, 3));
  EXPECT_FALSE(diag_fatal(true, "", stuck, 3));
  EXPECT_EQ(stuck, 0);
  EXPECT_FALSE(diag_fatal(false, "nav_no_progress", stuck, 3));
  EXPECT_FALSE(diag_fatal(false, "approach_blocked", stuck, 3));  // 갇힘 아닌 실패는 셈을 안 바꿈
  EXPECT_FALSE(diag_fatal(false, "nav_failed", stuck, 3));
  EXPECT_TRUE(diag_fatal(false, "nav_no_progress", stuck, 3));
}

TEST(MissionFlow, TagCorrection) {  // 
  auto [dx, dy] = tag_correction(1.0, 2.0, 1.012, 1.995, 0.05);
  EXPECT_NEAR(dx, 0.012, 1e-9);
  EXPECT_NEAR(dy, -0.005, 1e-9);
  std::tie(dx, dy) = tag_correction(1.0, 2.0, 1.2, 2.0, 0.05);  // 20 cm — 잘못 본 것
  EXPECT_EQ(dx, 0.0);
  EXPECT_EQ(dy, 0.0);
}

TEST(MissionFlow, JudgeLocation) {  // — 칸별 tag·오차 · 빈 칸 퍽
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

TEST(MissionFlow, GlobalCheck) {  // — 12개 중복·누락
  std::vector<int> dup, missing;
  global_check({{0, 1, 2}, {2, 3}, {4, -1}}, {0, 1, 2, 3, 4, 5}, dup, missing);
  EXPECT_EQ(dup, std::vector<int>{2});
  EXPECT_EQ(missing, std::vector<int>{5});
}

TEST(MissionFlow, FootprintInZone) {  // — 출발 구역 600×600 · 로봇 외곽 (앞 0.069 · 뒤 −0.233 · 옆 ±0.153)
  const std::vector<double> zone{0.434, 0.650, 1.034, 1.250};
  EXPECT_TRUE(footprint_in_zone({0.734, 0.950, M_PI}, 0.069, -0.233, 0.153, zone));
  EXPECT_FALSE(footprint_in_zone({0.480, 0.950, M_PI}, 0.069, -0.233, 0.153, zone));  // −X를 봄 → 앞 모서리 x 0.411 < 0.434
  EXPECT_FALSE(footprint_in_zone({0.734, 0.950, 0.0}, 0.069, -0.233, 0.153, {0.434, 0.650, 0.780, 1.250}));  // +X를 봄 → 앞 0.803 > 0.780
}

TEST(MissionSolver, SignalAction) {
  // ok → 확정 · unconfirmed → 기다림 · 오류(count·unknown·range)는 limit번 이어져야 정지 (한 사이클 잡음은 다음을 봄)
  int streak = 0;
  EXPECT_EQ(signal_action(false, "unconfirmed", streak, 2), SignalAction::WAIT);
  EXPECT_EQ(signal_action(false, "unknown", streak, 2), SignalAction::WAIT);
  EXPECT_EQ(streak, 1);
  EXPECT_EQ(signal_action(false, "range", streak, 2), SignalAction::STOP);
  streak = 1;
  EXPECT_EQ(signal_action(false, "unconfirmed", streak, 2), SignalAction::WAIT);  // 정상 사이클이 끼면 처음부터
  EXPECT_EQ(streak, 0);
  EXPECT_EQ(signal_action(true, "", streak, 2), SignalAction::ACCEPT);
  streak = 0;
  EXPECT_EQ(signal_action(false, "count", streak, 1), SignalAction::STOP);  // limit 1이면 바로
}


TEST(MissionSolver, VisitOrder) {  // plan.yaml survey_order · build_location_order
  EXPECT_EQ(plan::visit_order({1, 2, 6, 4, 5, 3}, 6), (std::vector<size_t>{0, 1, 5, 3, 4, 2}));
  EXPECT_EQ(plan::visit_order({}, 3), (std::vector<size_t>{0, 1, 2}));          // 비면 번호 순서
  EXPECT_EQ(plan::visit_order({1, 1, 2}, 3), (std::vector<size_t>{0, 1, 2}));   // 중복 → 번호 순서
  EXPECT_EQ(plan::visit_order({0, 1, 2}, 3), (std::vector<size_t>{0, 1, 2}));   // 범위 밖
  EXPECT_EQ(plan::visit_order({3, 2, 1}, 3), (std::vector<size_t>{2, 1, 0}));
}

TEST(MissionSolver, WallDistanceDropsBlockedSide) {  // 시작 박스 1 서쪽 다가설 자리는 x 1.101 벽 위 → 거름
  const std::vector<double> walls = {1.101, 0.650, 1.101, 1.315};
  const auto poses = plan::approach_poses(1.468, 0.950, 0.1025, 1.428, 0.950, 0.25);
  ASSERT_EQ(poses.size(), 4u);
  EXPECT_NEAR(poses[0].x, 1.1155, 1e-3);                                   // 퍽에서 가까운 서쪽 변이 먼저
  EXPECT_LT(plan::wall_distance(poses[0].x, poses[0].y, walls), 0.22);     // 벽에서 1.5 cm → 뺄 대상
  for (size_t i = 1; i < 4; ++i) EXPECT_GT(plan::wall_distance(poses[i].x, poses[i].y, walls), 0.22);
  EXPECT_NEAR(plan::wall_distance(1.101, 0.200, walls), 0.450, 1e-9);      // 선분 끝점까지
  EXPECT_TRUE(std::isinf(plan::wall_distance(0, 0, {})));
}

TEST(MissionFlow, ParseCounts) {  // "R4Y2G6" — 순서·대소문자 상관없음, 1~6 · 합 12
  Counts c;
  std::string why;
  ASSERT_TRUE(parse_counts("R4Y2G6", c, why));
  EXPECT_EQ(c.r, 4); EXPECT_EQ(c.y, 2); EXPECT_EQ(c.g, 6);
  ASSERT_TRUE(parse_counts("g6r4y2", c, why));
  EXPECT_EQ(c.r, 4); EXPECT_EQ(c.y, 2); EXPECT_EQ(c.g, 6);
  EXPECT_FALSE(parse_counts("R4Y2G7", c, why)); EXPECT_EQ(why, "count_out_of_range");
  EXPECT_FALSE(parse_counts("R4Y3G4", c, why)); EXPECT_EQ(why, "sum_not_12");
  EXPECT_FALSE(parse_counts("R0Y6G6", c, why)); EXPECT_EQ(why, "count_out_of_range");
  for (const char * bad : {"", "R4Y2", "R4Y2G6R1", "R4 Y2 G6", "R4Y2G", "X4Y2G6", "R4Y2G006"}) {
    EXPECT_FALSE(parse_counts(bad, c, why)) << bad;
    EXPECT_EQ(why, "bad_counts") << bad;
  }
}

TEST(MissionFlow, ParseStart) {
  StartCmd s;
  std::string why;
  ASSERT_TRUE(parse_start("start", s, why));
  EXPECT_EQ(s.only, Phase::IDLE); EXPECT_FALSE(s.diag); EXPECT_FALSE(s.from_survey); EXPECT_FALSE(s.given);
  ASSERT_TRUE(parse_start("start:SURVEY", s, why));
  EXPECT_EQ(s.only, Phase::SURVEY); EXPECT_FALSE(s.diag);
  ASSERT_TRUE(parse_start("start:DIAG", s, why));
  EXPECT_TRUE(s.diag); EXPECT_FALSE(s.from_survey); EXPECT_FALSE(s.given); EXPECT_EQ(s.only, Phase::IDLE);
  ASSERT_TRUE(parse_start("start:DIAG:P2-01:R2Y4G6", s, why));
  EXPECT_TRUE(s.diag); EXPECT_FALSE(s.from_survey); EXPECT_TRUE(s.given);
  EXPECT_EQ(s.barcode, "P2-01"); EXPECT_EQ(s.counts.y, 4);
  ASSERT_TRUE(parse_start("start:FROM_SURVEY:P2-04:R4Y2G6", s, why));
  EXPECT_FALSE(s.diag); EXPECT_TRUE(s.from_survey); EXPECT_TRUE(s.given); EXPECT_EQ(s.barcode, "P2-04");
  ASSERT_TRUE(parse_start("start:DIAG_FROM_SURVEY:P2-04:R4Y2G6", s, why));
  EXPECT_TRUE(s.diag); EXPECT_TRUE(s.from_survey); EXPECT_EQ(s.counts.r, 4); EXPECT_EQ(s.counts.g, 6);
  const std::vector<std::pair<const char *, const char *>> bad = {
    {"abort", "not_start"}, {"starts", "not_start"}, {"start:BUILD", "unknown_command"}, {"start:VERIFY", "unknown_command"},
    {"start:", "unknown_command"}, {"start:SURVEY:P2-04:R4Y2G6", "unknown_command"},
    {"start:FROM_SURVEY", "need_barcode_signal"}, {"start:DIAG_FROM_SURVEY:P2-04", "need_barcode_signal"},
    {"start:DIAG:P2-04", "need_barcode_signal"}, {"start:FROM_SURVEY:P2-05:R4Y2G6", "unknown_barcode"},
    {"start:FROM_SURVEY:p2-04:R4Y2G6", "unknown_barcode"}, {"start:FROM_SURVEY:P2-04:R4Y2G5", "sum_not_12"},
    {"start:FROM_SURVEY:P2-04:R4Y2G6:x", "need_barcode_signal"}, {"start:DIAG:P2-04:RYG", "bad_counts"}};
  for (const auto & [cmd, w] : bad) {
    EXPECT_FALSE(parse_start(cmd, s, why)) << cmd;
    EXPECT_EQ(why, w) << cmd;
  }
}

TEST(MissionSolver, PartialAllocationField1114) {
  // 실기 11-14: 태그 퍽 7개 (빨강 직육 2 · 빨강 원기둥 2 · 초록 직육 2 · 초록 원기둥 1) · 신호 R4 Y2 G6 · P2-04 (내림 → L1 G · L2 R · L3 Y)
  const Counts c{4, 2, 6};
  bool asc = true;
  ASSERT_TRUE(parse_barcode("P2-04", asc));
  EXPECT_EQ(assign_locations(c, asc), "GRY");
  const auto pucks = make(2, 2, 1, 2);  // red cyl 2 · red box 2 · green cyl 1 · green box 2
  std::map<int, char> a;
  std::string why;
  EXPECT_FALSE(allocate(pucks, c, a, why));
  EXPECT_EQ(why, "puck_count");
  std::map<char, int> filled;
  ASSERT_TRUE(allocate_partial(pucks, c, a, filled));
  EXPECT_EQ(filled['R'], 4);  // L2 R4 다 채움
  EXPECT_EQ(filled['Y'], 2);  // L3 Y2 ← 초록 직육면체 2
  EXPECT_EQ(filled['G'], 1);  // L1 G6 중 1 (초록 원기둥)
  EXPECT_EQ(a.size(), 7u);
  for (const auto & p : pucks) {
    const char g = a.at(p.tag);
    if (p.color == "red") EXPECT_EQ(g, 'R');
    else if (p.shape == "box") EXPECT_EQ(g, 'Y');
    else EXPECT_EQ(g, 'G');
  }
  // 12개로 풀리는 경우는 allocate와 같은 수만큼 채움 · 퍽이 없으면 false
  ASSERT_TRUE(allocate_partial(make(3, 1, 2, 6), Counts{4, 2, 6}, a, filled));
  EXPECT_EQ(filled['R'] + filled['Y'] + filled['G'], 12);
  EXPECT_FALSE(allocate_partial({}, c, a, filled));
}

// ---------- 도면 보정 (arena → map) — 출발 자세가 (0.1, −0.05, 5°) 어긋났다고 보고 인지 상자 위치를 만들어 맞춤
namespace {
const Pose2 kStart{0.734, 0.950, M_PI};
const ArenaCorr kTrue{0.10, -0.05, 5.0 * M_PI / 180, 0};
CorrPair seen(double ax, double ay, double ex = 0, double ey = 0) {  // 도면 상자 (ax, ay) — 인지 = 참 보정 + 잡음 (ex, ey)
  const auto m = arena_to_map({ax, ay, 0}, kStart);
  const auto q = apply_corr(kTrue, m);
  return {m.x, m.y, q.x + ex, q.y + ey};
}
}  // namespace

TEST(ArenaCorr, ApplyUnapplyRoundTripAndToMap) {
  const auto m = arena_to_map({3.670, 3.152, 0.3}, kStart);
  const auto c = apply_corr(kTrue, m);  // mission_manager to_map = apply_corr(acorr_, arena_to_map(...))
  EXPECT_GT(std::hypot(c.x - m.x, c.y - m.y), 0.1);
  const auto back = unapply_corr(kTrue, c);
  EXPECT_NEAR(back.x, m.x, 1e-9);
  EXPECT_NEAR(back.y, m.y, 1e-9);
  EXPECT_NEAR(back.yaw, m.yaw, 1e-9);
  const auto z = apply_corr(ArenaCorr{}, m);  // 보정 0 = 예전 to_map 그대로
  EXPECT_DOUBLE_EQ(z.x, m.x);
  EXPECT_DOUBLE_EQ(z.y, m.y);
}

TEST(ArenaCorr, OnePairTranslationOnly) {
  const auto p = seen(1.468, 0.950);
  const auto c = fit_arena_corr({p}, 1.0, 0.8, 0.35, 0.15);
  EXPECT_EQ(c.n, 1);
  EXPECT_DOUBLE_EQ(c.dth, 0.0);
  EXPECT_NEAR(c.dx, p.qx - p.mx, 1e-9);
  EXPECT_NEAR(c.dy, p.qy - p.my, 1e-9);
  EXPECT_EQ(fit_arena_corr({}, 1.0, 0.8, 0.35, 0.15).n, 0);
}

TEST(ArenaCorr, TwoSeparatedPairsRecoverRotation) {
  const auto c = fit_arena_corr({seen(1.468, 0.950), seen(3.670, 3.152)}, 1.0, 0.8, 0.35, 0.15);
  EXPECT_EQ(c.n, 2);
  EXPECT_NEAR(c.dth, kTrue.dth, 1e-9);
  EXPECT_NEAR(c.dx, kTrue.dx, 1e-9);
  EXPECT_NEAR(c.dy, kTrue.dy, 1e-9);
  // 가까운 두 상자(0.5 m < min_sep)면 회전은 안 맞춤
  EXPECT_DOUBLE_EQ(fit_arena_corr({seen(1.468, 0.950), seen(1.468, 1.450)}, 1.0, 0.8, 0.35, 0.15).dth, 0.0);
}

TEST(ArenaCorr, DropsOutlierAndClamps) {
  // 4개 중 하나는 0.4 m 틀린 짝 — 빼고 나머지 3개로
  const auto c = fit_arena_corr({seen(1.468, 0.950, 0.01), seen(3.670, 3.152, -0.01), seen(0.734, 5.354), seen(3.670, 3.886, 0.4, 0)},
                                1.0, 0.8, 0.35, 0.15);
  EXPECT_EQ(c.n, 3);
  EXPECT_NEAR(c.dth, kTrue.dth, 0.01);
  EXPECT_NEAR(c.dx, kTrue.dx, 0.02);
  // 한계: 2 m 이동 → 0.8 m · 40° 회전 → 20°
  const auto far = fit_arena_corr({{0, 0, 2.0, 0}}, 1.0, 0.8, 0.35, 0.15);
  EXPECT_NEAR(far.dx, 0.8, 1e-9);
  const double r = 40.0 * M_PI / 180;
  const auto rot = fit_arena_corr({{1, 0, std::cos(r), std::sin(r)}, {-1, 0, -std::cos(r), -std::sin(r)}}, 1.0, 0.8, 0.35, 0.15);
  EXPECT_NEAR(rot.dth, 0.35, 1e-9);
}

TEST(ArenaCorr, MatchLandmarkGateAndUnique) {
  const std::vector<std::pair<double, double>> boxes{{0.0, 0.0}, {0.734, 0.0}, {3.0, 3.0}};
  EXPECT_EQ(match_landmark(0.10, 0.05, boxes, 0.6), 0);
  EXPECT_EQ(match_landmark(3.3, 3.2, boxes, 0.6), 2);
  EXPECT_EQ(match_landmark(0.37, 0.0, boxes, 0.6), -1);  // 두 상자 가운데 — 헷갈림
  EXPECT_EQ(match_landmark(1.6, 1.6, boxes, 0.6), -1);   // 어느 상자에서도 gate 밖
  EXPECT_EQ(match_landmark(0.0, 0.0, {}, 0.6), -1);
}
