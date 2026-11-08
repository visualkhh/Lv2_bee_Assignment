#pragma once

// 누운 퍽 세우기 방식 C 정책 입력 28개 — 시뮬 stand_c_env와 같은 순서·단위 (isaac/rl/bee_isaac/tasks/stand_c_env.py
// _get_observations · test/test_stand_c_obs.cpp). 정책은 손 속도 3개(퍽 축 방향 · 옆 · 위)를 내고 skill_stand가 IK로 관절 목표를 만듦.
//   벡터는 모두 (퍽 축 u 방향, 옆 lat = z × u, 위 z) 성분 — 로봇 기준(base_footprint) 값을 이 틀로 돌림
//   100·(쥘 자리 − 손) [cm] (3) · 10·(손 − 가까운 끝 바닥 모서리) [dm] (3) · 퍽 축 (3) · 0.2·퍽 각속도 (3) · 50·그리퍼 (1)
//   · 단계 one-hot 8 · 직전 행동 3 · 단계 안 시간 [s] 0~3 (1) · 5·손 속도 [m/s] (3)
// 실기에는 퍽 기울기 센서가 없음 → 퍽 축은 손에서 구함 (pinch_axis): 쥔 뒤에는 퍽이 가까운 끝 바닥 모서리를 축으로 손가락 사이에서
// 돈다고 보고 (시뮬 호 기준 동작과 같은 경첩), 쥐기 전엔 누운 축 u 그대로.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace ml {

using V3c = std::array<double, 3>;

constexpr size_t STAND_C_OBS = 28;

struct StandCObsIn {
  V3c u{1, 0, 0};          // 누운 퍽 축 (가까운 끝 → 먼 끝, 수평 단위)
  V3c pinch{}, pivot{};    // 쥘 자리 손끝 · 가까운 끝 바닥 모서리 (로봇 기준)
  V3c ee{}, ee_prev{};     // 손끝 지금 · 한 정책 스텝 전
  V3c axis{1, 0, 0};       // 퍽 축 (로봇 기준 단위)
  V3c w{};                 // 퍽 각속도 [rad/s] (로봇 기준)
  double grip = 0.0;       // gripper_left_joint [m]
  int phase = 0;           // 0 위에서 멈춤 · 1 내려가기 · 2 쥐기 · 3 세우기 · 4 안정 · 5 벌리기 · 6 위로 · 7 대기 (8 끝)
  std::array<double, 3> prev_act{};
  double t_in = 0.0;       // 단계 안 시간 [s]
  double dt = 0.02;        // 정책 주기 [s]
};

inline V3c lat_of(const V3c & u) { return {-u[1], u[0], 0.0}; }

// 로봇 기준 벡터 → (u 방향, 옆, 위)
inline V3c frame(const V3c & u, const V3c & v) {
  const V3c l = lat_of(u);
  return {v[0] * u[0] + v[1] * u[1] + v[2] * u[2], v[0] * l[0] + v[1] * l[1] + v[2] * l[2], v[2]};
}

inline void stand_c_obs(const StandCObsIn & in, double * out) {
  size_t k = 0;
  const auto put = [&](const V3c & v, double s) { for (double x : frame(in.u, v)) out[k++] = s * x; };
  put({in.pinch[0] - in.ee[0], in.pinch[1] - in.ee[1], in.pinch[2] - in.ee[2]}, 100.0);
  put({in.ee[0] - in.pivot[0], in.ee[1] - in.pivot[1], in.ee[2] - in.pivot[2]}, 10.0);
  put(in.axis, 1.0);
  put(in.w, 0.2);
  out[k++] = 50.0 * in.grip;
  for (int i = 0; i < 8; ++i) out[k++] = std::min(in.phase, 7) == i ? 1.0 : 0.0;
  for (double a : in.prev_act) out[k++] = a;
  out[k++] = std::clamp(in.t_in, 0.0, 3.0);
  put({(in.ee[0] - in.ee_prev[0]) / in.dt, (in.ee[1] - in.ee_prev[1]) / in.dt, (in.ee[2] - in.ee_prev[2]) / in.dt}, 5.0);
}

// 쥔 퍽의 기울기 φ [rad] (0 누움 · π/2 섬): 손끝 − ee_up 이 경첩(pivot) 둘레 반지름 (a, b) 자리에 있다고 봄
//   a = 퍽 길이 (먼 끝 면까지), b = 손가락 중심 높이 (누운 퍽 바닥 기준) — stand_plan waypoints · right_env plan_waypoints와 같은 호
inline double pinch_phi(const V3c & u, const V3c & pivot, const V3c & ee, double ee_up, double a, double b) {
  const double r = (ee[0] - pivot[0]) * u[0] + (ee[1] - pivot[1]) * u[1];
  const double h = ee[2] - ee_up - pivot[2];
  return std::atan2(h, r) - std::atan2(b, a);
}

inline V3c axis_of(const V3c & u, double phi) {
  return {std::cos(phi) * u[0], std::cos(phi) * u[1], std::sin(phi)};
}

// u에서 z로 도는 회전 (축 u × z = −lat) — φ̇ [rad/s] → 각속도 벡터
inline V3c w_of(const V3c & u, double phi_dot) {
  const V3c l = lat_of(u);
  return {-phi_dot * l[0], -phi_dot * l[1], 0.0};
}

}  // namespace ml
