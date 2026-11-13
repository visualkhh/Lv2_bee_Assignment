#pragma once

// 팔 보정 정책 입력 — 시뮬 학습 환경과 같은 순서·단위 (isaac/lab, 브랜치 feature/skill-envs · test/test_arm_obs.cpp)
//   place 모양 (place_env, slot_env.LoadEnv) 39개:
//     q − q_ref (5) · q_down − q_ref (5) · 0.1·qd (5) · q_ref − q_above (5) · 이전 행동 (5) · 단계 one-hot (6) · 진행 [s] (1)
//     · 100·카메라 (칸 − 손 안 퍽, x y [cm]) (2) · 10·처짐 적분 (5)
//   pick 모양 (pick_env, slot_env.UnloadEnv) 41개: place 모양에서 카메라 2개 뒤에 면 방향·사각 퍽 2개를 끼움
//     … · 100·카메라 (퍽 − 손, x y) (2) · 면 방향 오차·is_box (1) · is_box (1) · 10·처짐 적분 (5)
//   pick_box 모양 (pick_env 설계 환경, train.py --task pick) 67개: pick 41개 뒤에
//     측정 유효 (1) · 측정 나이 / max_cam_age_s, 0~4 (1) · 10·|퍽 − 손| [dm] (1) · 100·(계획이 겨냥한 퍽 − 손) [cm] (3)
//     · 100·(퍽 − 손) [cm] (3) · 2·3스텝 전 행동 (5+5) · 그리퍼 위치 / 0.02 (1) · 그리퍼 부하 비 (1) · 관절 부하 비 0~3 (5)
//   pick_far 모양 (pick_env 먼 퍽 모드, train.py --task pick_far) 68개: pick_box 67개 뒤에 먼 퍽 표시 1.0 (1)
//     — 시뮬 PickFarEnvCfg는 모든 판이 먼 퍽이라 늘 1. 위 자세 = 뒤로 뺀 자세, 아래 자세 = 쥐는 자세 (skills/pick_far_skill.cpp)
//   unload_slot 모양 (slot_env.UnloadEnv scenario_v2, 카메라 대신 칸 TF) 58개: pick 41개 뒤에
//     2·3스텝 전 행동 (5+5) · 50·그리퍼 위치 (1) · 그리퍼 부하 비 (1) · 관절 부하 비 0~3 (5)
//   부하 비 = |effort| / 한계 (시뮬: 토크 / 토크 한계 · 실기: 전류 / 전류 한계, rl_arm_correction 파라미터)
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace ml {

enum class ArmLayout { Place, Pick, PickBox, UnloadSlot, PickFar };

inline bool arm_layout_from(const std::string & s, ArmLayout & out) {
  if (s == "place") { out = ArmLayout::Place; return true; }
  if (s == "pick") { out = ArmLayout::Pick; return true; }
  if (s == "pick_box") { out = ArmLayout::PickBox; return true; }
  if (s == "unload_slot") { out = ArmLayout::UnloadSlot; return true; }
  if (s == "pick_far") { out = ArmLayout::PickFar; return true; }
  return false;
}

using J5 = std::array<double, 5>;

struct ArmObsIn {
  J5 q{}, qd{}, q_ref{}, q_above{}, q_down{}, prev_act{}, corr{};
  int phase = 0;        // 0~5 (6 이상은 5로)
  double prog_s = 0.0;  // 지금 단계 들어온 뒤 시간 [s]
  double cam_x = 0.0, cam_y = 0.0;  // [m] hand_relative rel x y
  double face_yaw = 0.0;            // [rad]
  bool is_box = false;
  // pick_box · unload_slot
  J5 act1{}, act2{};                // 2·3스텝 전 행동 (prev_act가 1스텝 전)
  double grip_pos = 0.0;            // 그리퍼 관절 [m] (gripper_left_joint, 시뮬과 같은 관절)
  double grip_load = 0.0;           // |그리퍼 effort| / 한계
  J5 load{};                        // |관절 effort| / 한계
  // pick_box
  bool meas_valid = false;          // 손 카메라 값이 max_cam_age_s 안
  double meas_age = 0.0;            // 측정 나이 / max_cam_age_s
  double cam_z = 0.0;               // [m] hand_relative rel z
  double insp_x = 0.0, insp_y = 0.0, insp_z = 0.0;  // [m] hand_relative insp_rel
};

inline size_t arm_obs_size(ArmLayout l) {
  switch (l) {
    case ArmLayout::Place: return 39;
    case ArmLayout::Pick: return 41;
    case ArmLayout::PickBox: return 67;
    case ArmLayout::UnloadSlot: return 58;
    case ArmLayout::PickFar: return 68;
  }
  return 0;
}
constexpr size_t ARM_OBS_MAX = 68;  // 가장 큰 모양 (버퍼 크기) — pick_far

// out[arm_obs_size(l)]에 채움
inline size_t arm_obs(ArmLayout l, const ArmObsIn & in, double * out) {
  size_t k = 0;
  for (int i = 0; i < 5; ++i) out[k++] = in.q[i] - in.q_ref[i];
  for (int i = 0; i < 5; ++i) out[k++] = in.q_down[i] - in.q_ref[i];
  for (int i = 0; i < 5; ++i) out[k++] = 0.1 * in.qd[i];
  for (int i = 0; i < 5; ++i) out[k++] = in.q_ref[i] - in.q_above[i];
  for (int i = 0; i < 5; ++i) out[k++] = in.prev_act[i];
  const int ph = std::clamp(in.phase, 0, 5);
  for (int i = 0; i < 6; ++i) out[k++] = i == ph ? 1.0 : 0.0;
  out[k++] = std::clamp(in.prog_s, 0.0, 2.0);
  out[k++] = 100.0 * in.cam_x;
  out[k++] = 100.0 * in.cam_y;
  if (l != ArmLayout::Place) {
    out[k++] = in.is_box ? in.face_yaw : 0.0;
    out[k++] = in.is_box ? 1.0 : 0.0;
  }
  for (int i = 0; i < 5; ++i) out[k++] = 10.0 * in.corr[i];
  if (l == ArmLayout::PickBox || l == ArmLayout::PickFar) {
    out[k++] = in.meas_valid ? 1.0 : 0.0;
    out[k++] = std::clamp(in.meas_age, 0.0, 4.0);
    out[k++] = 10.0 * std::sqrt(in.cam_x * in.cam_x + in.cam_y * in.cam_y + in.cam_z * in.cam_z);
    out[k++] = 100.0 * in.insp_x;
    out[k++] = 100.0 * in.insp_y;
    out[k++] = 100.0 * in.insp_z;
    out[k++] = 100.0 * in.cam_x;
    out[k++] = 100.0 * in.cam_y;
    out[k++] = 100.0 * in.cam_z;
    for (int i = 0; i < 5; ++i) out[k++] = in.act1[i];
    for (int i = 0; i < 5; ++i) out[k++] = in.act2[i];
    out[k++] = in.grip_pos / 0.02;
    out[k++] = in.grip_load;
    for (int i = 0; i < 5; ++i) out[k++] = std::min(in.load[i], 3.0);
    if (l == ArmLayout::PickFar) out[k++] = 1.0;  // 먼 퍽 표시 (시뮬 obs 마지막 칸 self.far)
  } else if (l == ArmLayout::UnloadSlot) {
    for (int i = 0; i < 5; ++i) out[k++] = in.act1[i];
    for (int i = 0; i < 5; ++i) out[k++] = in.act2[i];
    out[k++] = 50.0 * in.grip_pos;
    out[k++] = in.grip_load;
    for (int i = 0; i < 5; ++i) out[k++] = std::min(in.load[i], 3.0);
  }
  return k;
}

}  // namespace ml
