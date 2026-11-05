// rl_arm_correction 처짐 보정 적분 (sag_step) 확인 — colcon test --packages-select ml
#include <gtest/gtest.h>

#include "ml/rl_arm_correction.hpp"

// 10 mrad 처짐을 100 Hz로 0.3 s (학습의 hold) 보정: 시뮬(85 ms마다 30 % 제거, 0.3 s에 약 72 %)과 같게 줄어야 한다
TEST(SagStep, RemovesSagLikeTraining) {
  const double sag = 0.010, dt = 0.01, gain = 4.2;
  double corr = 0.0;
  for (int k = 0; k < 30; ++k) {
    const double q = 0.5 + corr - sag;  // 정상상태 근사: 실제각 = 명령 − 처짐
    corr = ml::sag_step(corr, 0.5, 0.0, q, dt, gain, 0.2);
  }
  EXPECT_NEAR(corr / sag, 0.72, 0.02);
}

// 정책 Δ는 오차에 포함: q가 ref + Δ에 있으면 corr은 변하지 않는다 (Δ를 지우지 않음)
TEST(SagStep, KeepsResidual) {
  EXPECT_DOUBLE_EQ(ml::sag_step(0.0, 0.5, 0.03, 0.53, 0.01, 4.2, 0.2), 0.0);
}

TEST(SagStep, Clamps) {
  EXPECT_DOUBLE_EQ(ml::sag_step(0.19, 0.0, 0.0, -10.0, 0.01, 4.2, 0.2), 0.2);
}

// URDF 질량으로 낸 값 (팔을 앞으로 편 자세 · stow 근처) — 모델이 바뀌면 opencr_bridge.py도 함께 갱신
TEST(Gravity, TorquesMatchUrdf) {
  const auto zero = ml::gravity::torques(0.0, 0.0, 0.0);
  EXPECT_NEAR(zero[0], 1.2757, 1e-3);
  EXPECT_NEAR(zero[1], 1.1287, 1e-3);
  const auto bent = ml::gravity::torques(-0.28, 0.98, 0.87);
  EXPECT_NEAR(bent[0], 0.4788, 1e-3);
  EXPECT_NEAR(bent[1], 0.5750, 1e-3);
}

TEST(SagGuard, RestPoseReleases) {  // stow: 쌓인 보정을 0으로 풀고 더 쌓지 않음
  ml::SagJoint j;
  j.corr = 0.1;
  for (int k = 0; k < 200; ++k) ml::sag_joint_step(j, true, true, 0.5, 0.0, 0.4, 0.01, 4.2, 0.2, 2.0);
  EXPECT_NEAR(j.corr, 0.0, 1e-3);
}

TEST(SagGuard, BlockedReleases) {  // 닿아서 못 감 (q가 안 따라옴): 상한에 2 s 붙으면 풀고 쉼
  ml::SagJoint j;
  for (int k = 0; k < 300; ++k) ml::sag_joint_step(j, true, false, 0.5, 0.0, 0.0, 0.01, 4.2, 0.2, 2.0);
  EXPECT_TRUE(j.blocked);
  for (int k = 0; k < 200; ++k) ml::sag_joint_step(j, true, false, 0.5, 0.0, 0.0, 0.01, 4.2, 0.2, 2.0);
  EXPECT_NEAR(j.corr, 0.0, 1e-3);
}

TEST(SagGuard, NormalSagNotBlocked) {  // 보통 처짐 (상한 안에서 따라옴)은 그대로 고침
  ml::SagJoint j;
  double q = 0.45;
  for (int k = 0; k < 500; ++k) {
    ml::sag_joint_step(j, true, false, 0.5, 0.0, q, 0.01, 4.2, 0.2, 2.0);
    q = 0.45 + j.corr;  // 처짐 0.05 rad
  }
  EXPECT_FALSE(j.blocked);
  EXPECT_NEAR(j.corr, 0.05, 1e-3);
}
