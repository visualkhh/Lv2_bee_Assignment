#pragma once

// 팔 보정 정책 입력 — 시뮬 학습 환경과 같은 순서·단위 (isaac/lab, 브랜치 feature/skill-envs · test/test_arm_obs.cpp)
//   place 모양 (place_env, slot_env.LoadEnv) 39개:
//     q − q_ref (5) · q_down − q_ref (5) · 0.1·qd (5) · q_ref − q_above (5) · 이전 행동 (5) · 단계 one-hot (6) · 진행 [s] (1)
//     · 100·카메라 (칸 − 손 안 퍽, x y [cm]) (2) · 10·처짐 적분 (5)
//   pick 모양 (pick_env, slot_env.UnloadEnv) 41개: place 모양에서 카메라 2개 뒤에 면 방향·사각 퍽 2개를 끼움
//     … · 100·카메라 (퍽 − 손, x y) (2) · 면 방향 오차·is_box (1) · is_box (1) · 10·처짐 적분 (5)
#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

namespace ml {

enum class ArmLayout { Place, Pick };

inline bool arm_layout_from(const std::string & s, ArmLayout & out) {
  if (s == "place") { out = ArmLayout::Place; return true; }
  if (s == "pick") { out = ArmLayout::Pick; return true; }
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
};

inline size_t arm_obs_size(ArmLayout l) { return l == ArmLayout::Place ? 39 : 41; }
constexpr size_t ARM_OBS_MAX = 41;  // 가장 큰 모양 (버퍼 크기)

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
  if (l == ArmLayout::Pick) {
    out[k++] = in.is_box ? in.face_yaw : 0.0;
    out[k++] = in.is_box ? 1.0 : 0.0;
  }
  for (int i = 0; i < 5; ++i) out[k++] = 10.0 * in.corr[i];
  return k;
}

}  // namespace ml
