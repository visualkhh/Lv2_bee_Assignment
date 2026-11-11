// 주행 정책 입력 (시뮬 drive_env와 같은지)
#include <gtest/gtest.h>

#include <cmath>

#include "ml/nav_obs.hpp"

TEST(NavObs, GoalInRobotFrame) {
  ml::NavObsIn in;
  in.x = 1.0; in.y = 2.0; in.yaw = M_PI / 2;   // +y를 봄
  in.gx = 1.0; in.gy = 2.5; in.gyaw = M_PI;   // 앞 0.5 m, 왼쪽으로 90° 돌아야
  in.elapsed_s = 5.0; in.limit_s = 25.0;
  double o[ml::NAV_OBS];
  ml::nav_obs(in, o);
  EXPECT_NEAR(o[0], 0.5, 1e-12);
  EXPECT_NEAR(o[1], 0.0, 1e-12);
  EXPECT_NEAR(o[2], 0.0, 1e-12);   // cos 90°
  EXPECT_NEAR(o[3], 1.0, 1e-12);   // sin 90°
  EXPECT_NEAR(o[11], 0.2, 1e-12);  // 시간 비율
}

TEST(NavObs, Tilt) {
  EXPECT_NEAR(ml::tilt_of(0, 0, 0, 1), 0.0, 1e-12);
  const double h = std::sin(0.1 / 2);
  EXPECT_NEAR(ml::tilt_of(h, 0, 0, std::cos(0.1 / 2)), 0.1, 1e-9);
}


TEST(NavObs, HeadingSpeedCap) {  // 11-10 시뮬 RUN1: 경로와 80° 어긋난 채 0.22 m/s
  const double r80 = 80.0 * M_PI / 180.0;
  EXPECT_NEAR(ml::heading_speed_cap(0.22, 0.26, std::cos(r80), std::sin(r80)), 0.26 * std::pow(std::cos(r80), 2), 1e-9);  // ≈ 0.008
  EXPECT_LT(ml::heading_speed_cap(0.22, 0.26, std::cos(r80), std::sin(r80)), 0.01);
  EXPECT_DOUBLE_EQ(ml::heading_speed_cap(0.20, 0.26, 0.3, 0.0), 0.20);    // 경로 정면 — 그대로
  EXPECT_NEAR(ml::heading_speed_cap(0.26, 0.26, 0.3, 0.3 * std::tan(M_PI / 6)), 0.26 * 0.75, 1e-9);  // 30° — 0.195
  EXPECT_DOUBLE_EQ(ml::heading_speed_cap(0.20, 0.26, -0.3, 0.1), 0.0);    // 뒤쪽 경로 — 제자리 회전만
  EXPECT_DOUBLE_EQ(ml::heading_speed_cap(-0.05, 0.26, -0.3, 0.1), -0.05); // 후진은 그대로
}
