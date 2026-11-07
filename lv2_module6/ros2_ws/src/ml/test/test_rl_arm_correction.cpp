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

// 중력 토크 (팔 평면 점질량) — isaac/scripts/calib/arm_model.py(URDF 전체 FK)와 같은 값. grasp 자세 [deg] q2 29.78 q3 −11.31 q4 71.52
TEST(Gravity, TorquesMatchUrdfModel) {
  const double d2r = M_PI / 180.0;
  const auto t = ml::gravity::torques(29.78 * d2r, -11.31 * d2r, 71.52 * d2r);
  EXPECT_NEAR(t[0], 1.252, 0.01);
  EXPECT_NEAR(t[1], 0.698, 0.01);
  EXPECT_NEAR(t[2], 0.063, 0.005);
  const auto n = ml::gravity::torques(-22.68 * d2r, 46.72 * d2r, 65.96 * d2r);  // 가까이 접은 자세
  EXPECT_NEAR(n[0], 0.478, 0.01);
  EXPECT_NEAR(n[1], 0.674, 0.01);
}

// 고정 높이 오차를 올리는 관절 변화: 손가락 끝면 중심이 dz만큼 올라가고 x·손 피치는 그대로
TEST(Gravity, LiftMovesFaceCentreUpOnly) {
  using namespace ml::gravity;
  const double q2 = 0.3, q3 = 0.1, q4 = 1.1, dz = 0.005;  // 손이 거의 아래를 향한 자세
  std::array<double, 3> dq{};
  ASSERT_TRUE(lift(q2, q3, q4, dz, dq));
  const auto mark = [](double a, double b, double c) {
    const V2 p3 = rot(a, J3_X, J3_Z), p4 = rot(a + b, J4_X, 0.0), m = rot(a + b + c, HAND_X + MARK_X, 0.0);
    return V2{p3.x + p4.x + m.x, p3.z + p4.z + m.z};
  };
  const V2 before = mark(q2, q3, q4), after = mark(q2 + dq[0], q3 + dq[1], q4 + dq[2]);
  EXPECT_NEAR(after.z - before.z, dz, 1e-4);
  EXPECT_NEAR(after.x - before.x, 0.0, 1e-4);
  EXPECT_NEAR(dq[0] + dq[1] + dq[2], 0.0, 1e-9);  // 손 피치 그대로
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
