#include "skills/look_plan.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace skills::look {

namespace {
// gripper_link → gripper_cam_color_optical_frame (URDF gripper_cam_joint · gripper_cam_optical_joint을 stand::fk_gripper 축으로 옮긴 것)
constexpr V3 CAM_T = {0.0776744, 0.0322314, 0.0417911};
constexpr double CAM_R[3][3] = {{-0.0010481, -0.3420198, 0.9396922},
                                {-0.9999994, 0.0002266, -0.0010329},
                                {0.0001404, -0.9396927, -0.3420198}};
// joint2~4 한계 (SRDF) 안쪽으로 0.1 rad
constexpr double LO[3] = {-1.919862 + 0.1, -1.570796 + 0.1, -1.7 + 0.1};
constexpr double HI[3] = {1.919862 - 0.1, 1.396263 - 0.1, 1.97 - 0.1};

double dot(const V3 & a, const V3 & b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
}  // namespace

bool Score::better(const Score & o) const {
  if (cover != o.cover) return cover > o.cover;
  if (min_px != o.min_px) return min_px > o.min_px;
  return aim_off < o.aim_off;
}

void camera(const Q5 & q, V3 & origin, std::array<V3, 3> & axes) {
  V3 g;
  std::array<V3, 3> ga;
  stand::fk_gripper(q, g, ga);
  for (int i = 0; i < 3; ++i) origin[i] = g[i] + ga[0][i] * CAM_T[0] + ga[1][i] * CAM_T[1] + ga[2][i] * CAM_T[2];
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i) axes[j][i] = ga[0][i] * CAM_R[0][j] + ga[1][i] * CAM_R[1][j] + ga[2][i] * CAM_R[2][j];
}

bool project(const Q5 & q, const V3 & p, const Params & prm, double & u, double & v, double & dist) {
  V3 c;
  std::array<V3, 3> ax;
  camera(q, c, ax);
  const V3 d = {p[0] - c[0], p[1] - c[1], p[2] - c[2]};
  const double z = dot(d, ax[2]);
  dist = std::sqrt(dot(d, d));
  if (z < 0.05) return false;
  u = prm.width / 2 + prm.fx * dot(d, ax[0]) / z;
  v = prm.height / 2 + prm.fx * dot(d, ax[1]) / z;
  return true;
}

Score score(const Q5 & q, const std::vector<V3> & tops, const Params & prm) {
  Score s;
  if (tops.empty()) return s;
  double top = -1e9;
  for (const auto & p : tops) top = std::max(top, p[2]);
  if (stand::fk(q)[2] < top + prm.hand_clear_m) return s;
  V3 c;
  std::array<V3, 3> ax;
  camera(q, c, ax);
  int n = 0;
  double min_px = 1e9, su = 0, sv = 0;
  for (const auto & p : tops) {
    bool seen = true;
    double tu = 0, tv = 0, px = 0;
    for (const double dz : {0.0, -prm.mid_dz}) {
      const V3 d = {p[0] - c[0], p[1] - c[1], p[2] + dz - c[2]};
      const double z = dot(d, ax[2]), dist = std::sqrt(dot(d, d));
      if (z < 0.05) return s;  // 점이 카메라 뒤 — 이 자세는 안 씀
      const double u = prm.width / 2 + prm.fx * dot(d, ax[0]) / z, v = prm.height / 2 + prm.fx * dot(d, ax[1]) / z;
      seen = seen && dist >= prm.depth_min_m && u > prm.edge_px && u < prm.width - prm.edge_px && v > prm.edge_px &&
             v < prm.height - prm.edge_px && !(u > prm.finger_u0 && u < prm.finger_u1 && v > prm.finger_v0);
      if (dz == 0.0) {  // 태그 크기: 거리 · 윗면을 비스듬히 보는 만큼 (두 변 중 한 변만 줄어듦 → √cos)
        tu = u, tv = v;
        px = prm.fx * prm.tag_m / dist * std::sqrt(std::abs(d[2]) / dist);
      }
    }
    if (!seen) continue;
    ++n;
    su += tu, sv += tv;
    min_px = std::min(min_px, px);
  }
  s.cover = static_cast<double>(n) / tops.size();
  if (n > 0) {
    s.min_px = std::round(std::min(min_px, prm.tag_cap_px));
    s.aim_off = std::hypot(su / n - prm.aim_u, sv / n - prm.aim_v);
  }
  return s;
}

bool search(const std::vector<V3> & tops, const Params & prm, Q5 & out, Score & best, unsigned seed) {
  best = Score{};
  if (tops.empty()) return false;
  double cx = 0, cy = 0;
  for (const auto & p : tops) cx += p[0] / tops.size(), cy += p[1] / tops.size();
  const double yaw = std::atan2(cy, cx);
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  const auto in_limits = [](const Q5 & q) {
    for (int i = 0; i < 3; ++i) if (q[i + 1] < LO[i] || q[i + 1] > HI[i]) return false;
    return true;
  };
  bool found = false;
  const auto consider = [&](const Q5 & q) {
    const Score s = score(q, tops, prm);
    if (s.cover > 0 && (!found || s.better(best))) { best = s, out = q, found = true; }
  };
  for (int k = 0; k < prm.samples; ++k) {
    Q5 q = {yaw + prm.joint1_span * (2 * u01(rng) - 1), 0, 0, 0, 0};
    for (int i = 0; i < 3; ++i) q[i + 1] = LO[i] + (HI[i] - LO[i]) * u01(rng);
    consider(q);
  }
  if (!found) return false;
  for (const double sigma : {0.05, 0.02, 0.005}) {  // 주변 다듬기
    std::normal_distribution<double> nd(0.0, sigma);
    const Q5 base = out;
    for (int k = 0; k < prm.refine; ++k) {
      Q5 q = base;
      for (int i = 0; i < 4; ++i) q[i] += nd(rng);
      if (in_limits(q)) consider(q);
    }
  }
  return true;
}

}  // namespace skills::look
