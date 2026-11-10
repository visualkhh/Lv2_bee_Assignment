#pragma once

// 주행 보정 정책 입력 13개 — 시뮬 drive_env와 같은 순서·단위 (isaac/rl/bee_isaac/tasks/drive_env.py _get_observations · test/test_nav_obs.cpp)
//   목표(로봇 기준) dx dy [m] · cos·sin 방향 차 · 차체 속도 vx vy [m/s] · 회전 wz [rad/s] · 직전 명령 v w · 직전 행동 2
//   · 시간 비율 (시작 뒤 시간 / 제한 시간) · 차체 기울기 [rad]
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ml {

struct NavObsIn {
  double x = 0, y = 0, yaw = 0;          // 로봇 (map)
  double gx = 0, gy = 0, gyaw = 0;       // 목표 (map)
  double vx = 0, vy = 0, wz = 0;         // 차체 속도 (odom twist, 로봇 기준)
  double cmd_v = 0, cmd_w = 0, act0 = 0, act1 = 0;
  double elapsed_s = 0, limit_s = 25.0, tilt = 0;
};

constexpr size_t NAV_OBS = 13;

inline void nav_obs(const NavObsIn & in, double * out) {
  const double c = std::cos(in.yaw), s = std::sin(in.yaw);
  const double dx = in.gx - in.x, dy = in.gy - in.y;
  const double dyaw = std::remainder(in.gyaw - in.yaw, 2 * M_PI);
  size_t k = 0;
  out[k++] = c * dx + s * dy;
  out[k++] = -s * dx + c * dy;
  out[k++] = std::cos(dyaw);
  out[k++] = std::sin(dyaw);
  out[k++] = in.vx;
  out[k++] = in.vy;
  out[k++] = in.wz;
  out[k++] = in.cmd_v;
  out[k++] = in.cmd_w;
  out[k++] = in.act0;
  out[k++] = in.act1;
  out[k++] = std::clamp(in.elapsed_s / in.limit_s, 0.0, 1.0);
  out[k++] = in.tilt;
}

// IMU 자세 쿼터니언 → 기울기 [rad] (중력 방향과 몸 z축 사이)
inline double tilt_of(double qx, double qy, double qz, double qw) {
  (void)qz; (void)qw;
  return std::acos(std::clamp(1.0 - 2.0 * (qx * qx + qy * qy), -1.0, 1.0));
}

// 경로 방향 속도 상한 — 전진 속도를 max_v · cos²(오차)까지만 (오차 = 로봇 앞과 경로 앞 웨이포인트(wx, wy, 로봇 좌표) 방향 차).
//   90° 넘게 어긋나면 0 (제자리 회전만). 후진(v < 0)은 그대로. 표준 Nav2 RPP의 rotate_to_heading과 같은 생각.
//   11-10 시뮬: RL 정책이 경로와 80° 어긋난 채 0.22 m/s로 벽 쪽 전진 → 이 상한이면 0.007 m/s
inline double heading_speed_cap(double v, double max_v, double wx, double wy) {
  if (v <= 0.0 || (wx == 0.0 && wy == 0.0)) return v;
  const double c = std::cos(std::atan2(wy, wx));
  return c <= 0.0 ? 0.0 : std::min(v, max_v * c * c);
}

}  // namespace ml
