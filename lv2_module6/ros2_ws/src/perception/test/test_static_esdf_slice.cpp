// static_esdf_slice 테스트: 지도 + 점 → nvblox ESDF slice 형식 점구름, 그리고 RL controller 가상 스캔이 기대대로 나오는지
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "perception/static_esdf_slice.hpp"

using namespace perception;

// 4 m × 4 m, 0.05 m, 원점 (-2, -2) — 로봇은 (0, 0)
static nav_msgs::msg::OccupancyGrid grid() {
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = "map";
  m.info.resolution = 0.05;
  m.info.width = m.info.height = 80;
  m.info.origin.position.x = m.info.origin.position.y = -2.0;
  m.info.origin.orientation.w = 1.0;
  m.data.assign(80 * 80, 0);
  return m;
}

// ml::RlNavController::on_cloud 와 같은 계산 (ml/src/rl_nav_controller.cpp) — 로봇 = 원점·yaw 0이라 TF 생략
//   esdf 0~esdf_max 점만 · r = |p| − esdf − inflation · 각도 빈 최소
static std::vector<float> virtual_scan(const std::vector<float> & pts, int rays = 360, double lidar_max = 3.5,
                                       double esdf_max = 0.20, double infl = 0.02, double z_min = 0.03, double z_max = 0.30) {
  std::vector<float> best(rays, float(lidar_max));
  for (size_t i = 0; i + 3 < pts.size(); i += 4) {
    const double x = pts[i], y = pts[i + 1], z = pts[i + 2], e = pts[i + 3];
    if (e < 0.0 || e > esdf_max || z < z_min || z > z_max) continue;
    const double r = std::max(0.0, std::hypot(x, y) - e - infl);
    if (r > lidar_max) continue;
    int bin = int(std::floor((std::atan2(y, x) + M_PI) / (2 * M_PI) * rays)) % rays;
    if (r < best[bin]) best[bin] = float(r);
  }
  return best;
}

// 정면 ±5° 빈 최소 (칸 중심이 y=±0.025라 정확히 0° 빈은 비어 있을 수 있음)
static float front(const std::vector<float> & scan) {
  return *std::min_element(scan.begin() + 175, scan.begin() + 186);
}

static const float * find(const std::vector<float> & pts, double x, double y) {
  for (size_t i = 0; i + 3 < pts.size(); i += 4)
    if (std::abs(pts[i] - x) < 1e-4 && std::abs(pts[i + 1] - y) < 1e-4) return &pts[i];
  return nullptr;
}

TEST(StaticEsdfSlice, MapWallGivesNvbloxStyleDistances) {
  auto m = grid();
  for (int r = 0; r < 80; ++r) m.data[r * 80 + 60] = 100;  // 벽: x 1.00~1.05 (칸 중심 1.025)
  const EsdfSliceParams p;
  const auto pts = esdf_slice(m, {}, 0.0, 0.0, 0.0, p);
  ASSERT_FALSE(pts.empty());
  ASSERT_NE(find(pts, 1.025, 0.025), nullptr);
  EXPECT_FLOAT_EQ(find(pts, 1.025, 0.025)[3], 0.0f);           // 장애물 칸 = 0
  EXPECT_NEAR(find(pts, 0.825, 0.025)[3], 0.20, 1e-4);          // 4칸 앞 = 0.20 m
  EXPECT_FLOAT_EQ(find(pts, 1.025, 0.025)[2], float(p.z_m));    // z = 로봇 바닥 + z_m
  for (size_t i = 0; i < pts.size(); i += 4) {
    EXPECT_LE(pts[i + 3], p.max_distance_m + 1e-6);
    EXPECT_LE(std::hypot(pts[i], pts[i + 1]), p.radius_m + 0.05);
  }
  // 정면 가상 스캔 = 벽 1.025 − 2×0.15 − 0.02: 벽 앞 칸은 |p|−esdf라 esdf만큼 더 가깝게 읽힌다 (nvblox 입력도 같은 계산).
  //   0.20 칸은 float로 0.2000001 > esdf_max 0.20이라 빠지고 3칸(0.15)까지 들어감
  const auto scan = virtual_scan(pts);
  EXPECT_NEAR(front(scan), 1.025 - 0.30 - 0.02, 0.01);
  EXPECT_FLOAT_EQ(scan[0], 3.5f);  // 뒤는 비었음
}

TEST(StaticEsdfSlice, LivePointsAddObstacles) {
  const auto m = grid();
  const auto pts = esdf_slice(m, {0.51f, 0.01f}, 0.0, 0.0, 0.0, EsdfSliceParams{});
  ASSERT_NE(find(pts, 0.525, 0.025), nullptr);
  EXPECT_FLOAT_EQ(find(pts, 0.525, 0.025)[3], 0.0f);
  const float f = front(virtual_scan(pts));  // 점 장애물 0.525 m — 벽과 같은 식으로 최대 2×0.20 + 0.02 가깝게
  EXPECT_LT(f, 0.525 - 0.02);
  EXPECT_GT(f, 0.525 - 0.42);
}

TEST(StaticEsdfSlice, OpenSpaceStillPublishesOnePoint) {
  // 빈 점구름이면 controller가 stamp를 안 갱신 → 바이패스. 트인 곳에서도 점 하나(거리 > 0.20이라 스캔엔 안 들어감)
  const auto pts = esdf_slice(grid(), {}, 0.0, 0.0, 0.0, EsdfSliceParams{});
  ASSERT_EQ(pts.size(), 4u);
  EXPECT_GT(pts[3], 0.20f);
  EXPECT_FLOAT_EQ(front(virtual_scan(pts)), 3.5f);
}

TEST(StaticEsdfSlice, RotatedMapOrigin) {
  auto m = grid();
  m.info.origin.position.x = 1.0;
  m.info.origin.position.y = 2.0;
  m.info.origin.orientation.z = std::sin(M_PI / 4);  // yaw 90°
  m.info.origin.orientation.w = std::cos(M_PI / 4);
  m.data[0] = 100;  // 칸 (0, 0) — 격자 (0.025, 0.025) → map (1 − 0.025, 2 + 0.025)
  const auto pts = esdf_slice(m, {}, 1.0, 2.0, 0.0, EsdfSliceParams{});
  ASSERT_NE(find(pts, 0.975, 2.025), nullptr);
  EXPECT_FLOAT_EQ(find(pts, 0.975, 2.025)[3], 0.0f);
}
