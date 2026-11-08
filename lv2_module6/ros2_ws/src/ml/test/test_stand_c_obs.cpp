// 세우기 방식 C 정책 입력 (시뮬 stand_c_env와 같은지) · 경첩으로 구한 퍽 기울기
#include <gtest/gtest.h>

#include <cmath>

#include "ml/stand_c_obs.hpp"

TEST(StandCObs, LayoutAndScale) {
  ml::StandCObsIn in;
  in.u = {0.0, 1.0, 0.0};  // 퍽 축이 로봇 왼쪽(+y)으로 → lat = (−1, 0, 0)
  in.pinch = {0.20, 0.05, 0.13};
  in.pivot = {0.20, -0.01, 0.07};
  in.ee = {0.21, 0.04, 0.15};
  in.ee_prev = {0.21, 0.04, 0.152};
  in.axis = {0.0, 1.0, 0.0};
  in.w = {-0.5, 0.0, 0.0};
  in.grip = 0.01;
  in.phase = 3;
  in.prev_act = {0.1, -0.2, 0.3};
  in.t_in = 5.0;
  double o[ml::STAND_C_OBS];
  ml::stand_c_obs(in, o);
  EXPECT_NEAR(o[0], 1.0, 1e-9);    // 쥘 자리 − 손: u 방향 +1 cm
  EXPECT_NEAR(o[1], 1.0, 1e-9);    // 옆 (lat = −x): x −1 cm → +1
  EXPECT_NEAR(o[2], -2.0, 1e-9);   // 위 −2 cm
  EXPECT_NEAR(o[3], 0.5, 1e-9);    // 손 − 모서리 u 방향 5 cm = 0.5 dm
  EXPECT_NEAR(o[5], 0.8, 1e-9);
  EXPECT_NEAR(o[6], 1.0, 1e-9);    // 축 = u
  EXPECT_NEAR(o[10], 0.1, 1e-9);   // 0.2·각속도 옆 성분 (−0.5·−1)
  EXPECT_NEAR(o[12], 0.5, 1e-9);   // 50·그리퍼
  for (int i = 0; i < 8; ++i) EXPECT_EQ(o[13 + i], i == 3 ? 1.0 : 0.0);
  EXPECT_NEAR(o[22], -0.2, 1e-12);
  EXPECT_NEAR(o[24], 3.0, 1e-12);  // 단계 시간 3 s에서 자름
  EXPECT_NEAR(o[27], -0.5, 1e-9);  // 5·손 속도 위 (−0.1 m/s)
}

TEST(StandCObs, PinchPhiFollowsTheHinge) {
  const ml::V3c u = {1, 0, 0}, pivot = {0.17, 0.0, 0.07};
  const double a = 0.06, b = 0.03, ee_up = 0.0506;
  for (double phi : {0.0, 0.4, M_PI / 2, 2.0}) {  // stand_plan 호 위의 손끝
    const ml::V3c ee = {pivot[0] + a * std::cos(phi) - b * std::sin(phi), 0.0, pivot[2] + a * std::sin(phi) + b * std::cos(phi) + ee_up};
    EXPECT_NEAR(ml::pinch_phi(u, pivot, ee, ee_up, a, b), phi, 1e-9);
  }
  const auto ax = ml::axis_of(u, M_PI / 2);
  EXPECT_NEAR(ax[2], 1.0, 1e-12);
  EXPECT_NEAR(ml::w_of(u, 1.0)[1], -1.0, 1e-12);  // u = x → 축 u × z = −y
}
