// 팔 정책 입력 순서·크기 (시뮬 place_env · pick_env와 같은지) · 처짐 적분
#include <gtest/gtest.h>

#include <vector>

#include "ml/arm_obs.hpp"

TEST(ArmObs, PlaceLayoutOrder) {
  ml::ArmObsIn in;
  for (int i = 0; i < 5; ++i) {
    in.q[i] = 1.0 + i; in.q_ref[i] = 1.0; in.q_down[i] = 2.0; in.q_above[i] = 0.5; in.qd[i] = 10.0;
    in.prev_act[i] = -0.5; in.corr[i] = 0.01;
  }
  in.phase = 2; in.prog_s = 3.0; in.cam_x = 0.004; in.cam_y = -0.002;
  std::vector<double> o(41, -9);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::Place, in, o.data()), 39u);
  EXPECT_DOUBLE_EQ(o[0], 0.0);      // q − q_ref
  EXPECT_DOUBLE_EQ(o[4], 4.0);
  EXPECT_DOUBLE_EQ(o[5], 1.0);      // q_down − q_ref
  EXPECT_DOUBLE_EQ(o[10], 1.0);     // 0.1 qd
  EXPECT_DOUBLE_EQ(o[15], 0.5);     // q_ref − q_above
  EXPECT_DOUBLE_EQ(o[20], -0.5);    // 이전 행동
  EXPECT_DOUBLE_EQ(o[25 + 2], 1.0); // 단계 2 one-hot
  EXPECT_DOUBLE_EQ(o[25 + 1], 0.0);
  EXPECT_DOUBLE_EQ(o[31], 2.0);     // 진행은 2 s에서 자름
  EXPECT_NEAR(o[32], 0.4, 1e-12);   // 100·cam [cm]
  EXPECT_NEAR(o[33], -0.2, 1e-12);
  EXPECT_NEAR(o[34], 0.1, 1e-12);   // 10·corr
  EXPECT_DOUBLE_EQ(o[39], -9);      // 39개만 씀
}

TEST(ArmObs, PickLayoutAddsFace) {
  ml::ArmObsIn in;
  in.face_yaw = 0.2; in.is_box = true; in.corr.fill(0.02);
  std::vector<double> o(41, -9);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::Pick, in, o.data()), 41u);
  EXPECT_DOUBLE_EQ(o[34], 0.2);     // 면 방향 (사각 퍽)
  EXPECT_DOUBLE_EQ(o[35], 1.0);     // is_box
  EXPECT_NEAR(o[36], 0.2, 1e-12);   // 10·corr 시작
  in.is_box = false;
  ml::arm_obs(ml::ArmLayout::Pick, in, o.data());
  EXPECT_DOUBLE_EQ(o[34], 0.0);     // 원기둥은 면 방향 0
}
