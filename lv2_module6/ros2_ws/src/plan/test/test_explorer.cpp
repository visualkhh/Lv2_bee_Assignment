// plan 테스트: explorer 프론티어 찾기 — 빈 칸과 모르는 칸의 경계를 덩어리로 묶고, 작은 덩어리는 버림
//   + 코스트맵에서 로봇과 이어진 칸만 목표로 (벽 너머 X) · 로봇 칸이 막혀도 둘레에서 시작
#include <gtest/gtest.h>

#include "plan/explorer.hpp"

using plan::find_frontiers;
using plan::reach_cells;
using plan::reachable_goal;

// 10×10 지도: 왼쪽 5열 빈 칸(0) · 오른쪽 5열 모르는 칸(−1), 가운데 한 줄 벽(100)으로 위쪽 경계만 남김
nav_msgs::msg::OccupancyGrid make_map() {
  nav_msgs::msg::OccupancyGrid m;
  m.info.width = 10;
  m.info.height = 10;
  m.info.resolution = 0.1;
  m.info.origin.position.x = -1.0;
  m.info.origin.position.y = 2.0;
  m.data.assign(100, -1);
  for (int r = 0; r < 10; ++r)
    for (int c = 0; c < 5; ++c) m.data[r * 10 + c] = 0;
  for (int r = 0; r < 5; ++r) m.data[r * 10 + 4] = 100;  // 아래 절반은 벽이 막음 → 경계는 위쪽 5칸(r=5~9, c=4)
  return m;
}

TEST(ExplorerFrontier, ClustersBoundaryAndFiltersSmall) {
  const auto m = make_map();
  auto fs = find_frontiers(m, 3);
  ASSERT_EQ(fs.size(), 1u);
  EXPECT_EQ(fs[0].size, 5u);                 // c=4, r=5..9
  EXPECT_NEAR(fs[0].x, -1.0 + 4.5 * 0.1, 1e-6);  // resolution은 float32
  EXPECT_NEAR(fs[0].y, 2.0 + 7.5 * 0.1, 1e-6);  // 중심(r=7)
  EXPECT_TRUE(find_frontiers(m, 6).empty());  // 5칸 < 6 → 버림
}

TEST(ExplorerFrontier, NoUnknownNoFrontier) {
  auto m = make_map();
  for (auto & v : m.data) if (v < 0) v = 0;  // 다 알려짐
  EXPECT_TRUE(find_frontiers(m, 1).empty());
}

// 10×10 코스트맵 (칸 0.1m, 원점 0,0): 가운데 c=5 세로 벽(100)이 위아래 끝까지 → 오른쪽은 왼쪽에서 못 감
nav_msgs::msg::OccupancyGrid make_costmap() {
  nav_msgs::msg::OccupancyGrid m;
  m.info.width = 10;
  m.info.height = 10;
  m.info.resolution = 0.1;
  m.data.assign(100, 0);
  for (int r = 0; r < 10; ++r) m.data[r * 10 + 5] = 100;
  return m;
}

TEST(ExplorerReach, WallBlocksOtherSide) {
  const auto cm = make_costmap();
  const auto d = reach_cells(cm, 0.15, 0.15, 0.3);  // 로봇 (c1, r1)
  EXPECT_EQ(d[1 * 10 + 1], 0);
  EXPECT_EQ(d[1 * 10 + 4], 3);    // 같은 쪽 3칸
  EXPECT_EQ(d[1 * 10 + 6], -1);   // 벽 너머
  EXPECT_FALSE(reachable_goal(cm, d, 0.85, 0.55, 0.15));  // 오른쪽 프론티어 — 0.15m 안에 갈 칸 없음
  const auto g = reachable_goal(cm, d, 0.35, 0.55, 0.15);  // 왼쪽 프론티어
  ASSERT_TRUE(g);
  EXPECT_LT(g->x, 0.5);
  auto open = cm;
  open.data[9 * 10 + 5] = -1;  // 맨 위 벽 한 칸이 모름 → 지나갈 수 있음 (planner allow_unknown과 같게)
  EXPECT_GE(reach_cells(open, 0.15, 0.15, 0.3)[1 * 10 + 6], 0);
}

TEST(ExplorerReach, BlockedRobotCellSeedsAround) {
  auto cm = make_costmap();
  cm.data[1 * 10 + 1] = 99;  // 로봇 칸이 로봇 반경 안 장애물 (벽에 붙음)
  const auto d = reach_cells(cm, 0.15, 0.15, 0.15);
  EXPECT_EQ(d[1 * 10 + 1], -1);
  EXPECT_EQ(d[1 * 10 + 2], 0);    // 둘레에서 시작
  EXPECT_EQ(reach_cells(cm, 0.15, 0.15, 0.0)[1 * 10 + 2], -1);  // 반경 0이면 시작 칸 없음
}
