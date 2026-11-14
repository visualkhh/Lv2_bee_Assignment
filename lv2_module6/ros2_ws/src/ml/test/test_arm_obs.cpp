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

// pick_box 67 = pick 41 + 시뮬 pick_env 설계 추가 26 (같은 순서·단위)
TEST(ArmObs, PickBoxLayout) {
  ml::ArmObsIn in;
  in.cam_x = 0.03; in.cam_y = 0.0; in.cam_z = -0.04;  // |rel| 0.05 m
  in.insp_x = 0.01; in.insp_y = -0.02; in.insp_z = -0.05;
  in.meas_valid = true; in.meas_age = 5.0;
  in.act1.fill(0.1); in.act2.fill(-0.2);
  in.grip_pos = 0.01; in.grip_load = 0.8; in.load = {0.1, 0.5, 4.0, 0.0, 0.2};
  std::vector<double> o(ml::ARM_OBS_MAX, -9);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::PickBox, in, o.data()), 67u);
  EXPECT_NEAR(o[32], 3.0, 1e-12);   // 100·cam x (pick 41 부분)
  EXPECT_DOUBLE_EQ(o[41], 1.0);     // 측정 유효
  EXPECT_DOUBLE_EQ(o[42], 4.0);     // 나이 비는 4에서 자름
  EXPECT_NEAR(o[43], 0.5, 1e-12);   // 10·|rel| [dm]
  EXPECT_NEAR(o[44], 1.0, 1e-12);   // 100·insp
  EXPECT_NEAR(o[46], -5.0, 1e-12);
  EXPECT_NEAR(o[47], 3.0, 1e-12);   // 100·rel x y z
  EXPECT_NEAR(o[49], -4.0, 1e-12);
  EXPECT_DOUBLE_EQ(o[50], 0.1);     // 2스텝 전 행동
  EXPECT_DOUBLE_EQ(o[55], -0.2);    // 3스텝 전 행동
  EXPECT_NEAR(o[60], 0.5, 1e-12);   // 그리퍼 / 0.02
  EXPECT_DOUBLE_EQ(o[61], 0.8);
  EXPECT_DOUBLE_EQ(o[64], 3.0);     // 관절 부하 비는 3에서 자름
}

// pick_far 68 = pick_box 67 + 먼 퍽 표시 1 (시뮬 pick_env far_prob > 0의 마지막 칸) — 앞 67개는 pick_box와 똑같음
TEST(ArmObs, PickFarLayout) {
  ml::ArmObsIn in;
  in.cam_x = 0.03; in.cam_z = -0.04; in.insp_y = -0.02;
  in.act1.fill(0.1); in.grip_pos = 0.01; in.load = {0.1, 0.5, 4.0, 0.0, 0.2};
  std::vector<double> box(ml::ARM_OBS_MAX, -9), far(ml::ARM_OBS_MAX, -9);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::PickBox, in, box.data()), 67u);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::PickFar, in, far.data()), 68u);
  EXPECT_EQ(ml::arm_obs_size(ml::ArmLayout::PickFar), 68u);
  for (size_t i = 0; i < 67; ++i) EXPECT_DOUBLE_EQ(far[i], box[i]) << i;
  EXPECT_DOUBLE_EQ(far[67], 1.0);  // 먼 퍽 표시
  ml::ArmLayout l;
  ASSERT_TRUE(ml::arm_layout_from("pick_far", l));  // meta.yaml layout: pick_far
  EXPECT_EQ(l, ml::ArmLayout::PickFar);
}

// unload_slot 58 = pick 41 + 행동 기록 10 + 50·그리퍼 + 그리퍼 부하 + 관절 부하 5 (시뮬 UnloadEnv obs_extra)
TEST(ArmObs, UnloadSlotLayout) {
  ml::ArmObsIn in;
  in.act1.fill(0.3); in.act2.fill(0.4); in.grip_pos = 0.01; in.grip_load = 0.2; in.load.fill(0.5);
  std::vector<double> o(ml::ARM_OBS_MAX, -9);
  ASSERT_EQ(ml::arm_obs(ml::ArmLayout::UnloadSlot, in, o.data()), 58u);
  EXPECT_DOUBLE_EQ(o[41], 0.3);
  EXPECT_DOUBLE_EQ(o[46], 0.4);
  EXPECT_NEAR(o[51], 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(o[52], 0.2);
  EXPECT_DOUBLE_EQ(o[57], 0.5);
  EXPECT_DOUBLE_EQ(o[58], -9);
}
