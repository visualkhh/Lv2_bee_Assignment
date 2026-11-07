#include "skills/stand_plan.hpp"

#include <algorithm>
#include <cmath>

namespace skills::stand {

namespace {
// urdf/beebot_mesh/beebot.urdf — base_footprint → base_link(z 0.01) → arm_base_link → joint1(z) … joint5(x) → end_effector_link
constexpr V3 ARM_BASE = {0.047985, -0.00023, 0.130175 + 0.010};
constexpr double D1 = 0.0409, D2 = 0.0416;    // joint1 · joint2 높이
constexpr double L1X = 0.024, L1Z = 0.128;    // joint2 → joint3
constexpr double L2 = 0.124, L3 = 0.0629;     // joint3 → joint4 → joint5
constexpr double EE = 0.109055;               // gripper_link → end_effector_link (손끝, gripper x)
constexpr Q5 LO = {-3.14159, -1.919862, -1.570796, -1.7, -3.141593};
constexpr Q5 HI = {3.14159, 1.919862, 1.396263, 1.97, 3.141593};
constexpr double MAX_JUMP = 0.8;              // [rad] 쥔 채 도는 경로의 이웃 점이 이보다 튀면 손목 뒤집힘 — 실패로 (실기 stand_run)

using M3 = std::array<std::array<double, 3>, 3>;
M3 mul(const M3 & a, const M3 & b) {
  M3 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r[i][j] += a[i][k] * b[k][j];
  return r;
}
V3 col(const M3 & m, int j) { return {m[0][j], m[1][j], m[2][j]}; }
M3 rz(double a) { const double c = std::cos(a), s = std::sin(a); return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }
M3 ry(double a) { const double c = std::cos(a), s = std::sin(a); return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
M3 rx(double a) { const double c = std::cos(a), s = std::sin(a); return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
V3 add(V3 a, const M3 & r, double x, double z) {
  for (int i = 0; i < 3; ++i) a[i] += r[i][0] * x + r[i][2] * z;
  return a;
}
V3 plus(V3 a, const V3 & b, double k = 1.0) { for (int i = 0; i < 3; ++i) a[i] += k * b[i]; return a; }
double wrap(double a) { return std::remainder(a, 2 * M_PI); }
double linf(const Q5 & a, const Q5 & b) {
  double m = 0;
  for (int i = 0; i < 5; ++i) m = std::max(m, std::abs(a[i] - b[i]));
  return m;
}
bool in_limits(const Q5 & q) {
  for (int i = 0; i < 5; ++i) if (q[i] < LO[i] - 1e-9 || q[i] > HI[i] + 1e-9) return false;
  return true;
}
}  // namespace

V3 fk(const Q5 & q, V3 * close_axis) {
  const V3 p2 = {ARM_BASE[0], ARM_BASE[1], ARM_BASE[2] + D1 + D2};
  const M3 r2 = mul(rz(q[0]), ry(q[1]));
  const V3 p3 = add(p2, r2, L1X, L1Z);
  const M3 r3 = mul(r2, ry(q[2]));
  const V3 p4 = add(p3, r3, L2, 0);
  const M3 r4 = mul(r3, ry(q[3]));
  const V3 p5 = add(p4, r4, L3, 0);
  const M3 rg = mul(r4, rx(q[4]));
  if (close_axis) *close_axis = col(rg, 1);
  return add(p5, rg, EE, 0);
}

void fk_gripper(const Q5 & q, V3 & origin, std::array<V3, 3> & axes) {
  const V3 p2 = {ARM_BASE[0], ARM_BASE[1], ARM_BASE[2] + D1 + D2};
  const M3 r2 = mul(rz(q[0]), ry(q[1]));
  const M3 r3 = mul(r2, ry(q[2]));
  const M3 r4 = mul(r3, ry(q[3]));
  const M3 rg = mul(r4, rx(q[4]));
  origin = add(add(add(p2, r2, L1X, L1Z), r3, L2, 0), r4, L3, 0);
  for (int j = 0; j < 3; ++j) axes[j] = col(rg, j);
}

bool ik_down(const V3 & p, double close_yaw, const Q5 & near, Q5 & out) {
  // 손이 아래를 보면 gripper x = −z → joint4 = 손끝 + (0, 0, EE + L3). joint1이 그 점을 바라보고, joint2~3이 팔 평면의 2링크,
  // joint4 = π/2 − joint2 − joint3 (손을 수직으로), joint5 = 닫는 축 방향 (gripper y 수평각 = joint1 + π/2 − joint5)
  const double wx = p[0] - ARM_BASE[0], wy = p[1] - ARM_BASE[1], wz = p[2] + EE + L3 - (ARM_BASE[2] + D1 + D2);
  const double r = std::hypot(wx, wy);
  const double q1 = std::atan2(wy, wx);
  const double a1 = std::hypot(L1X, L1Z), al = std::atan2(L1Z, L1X);
  // 팔 평면 복소수 (반지름 + i·높이): joint4 = e^{−i q2}(L1 + L2 e^{−i q3}), Ry(+)는 평면에서 −회전
  const double d2 = r * r + wz * wz;
  const double k = (d2 - a1 * a1 - L2 * L2) / (2 * a1 * L2);
  if (k < -1.0 || k > 1.0) return false;
  bool found = false;
  for (double sgn : {1.0, -1.0}) {
    const double g = sgn * std::acos(k);  // = al + q3
    const double q3 = g - al;
    const double bx = L1X + L2 * std::cos(-q3), bz = L1Z + L2 * std::sin(-q3);
    const double q2 = -(std::atan2(wz, r) - std::atan2(bz, bx));
    Q5 q = {q1, wrap(q2), wrap(q3), 0.0, 0.0};
    q[3] = wrap(M_PI / 2 - q[1] - q[2]);
    q[4] = wrap(q1 + M_PI / 2 - close_yaw);
    if (!in_limits(q)) continue;
    if (!found || linf(q, near) < linf(out, near)) out = q;
    found = true;
  }
  return found;
}

bool ik_down_sym(const V3 & p, double close_yaw, const Q5 & near, Q5 & out) {
  Q5 a, b;
  const bool ok_a = ik_down(p, close_yaw, near, a), ok_b = ik_down(p, close_yaw + M_PI, near, b);
  if (!ok_a && !ok_b) return false;
  out = !ok_b || (ok_a && linf(a, near) <= linf(b, near)) ? a : b;
  return true;
}

double away_yaw(double yaw) { return yaw - M_PI * std::floor((yaw + M_PI / 2) / M_PI); }

std::vector<V3> waypoints(const V3 & centre, double yaw, bool is_box, const Params & p, double & close_yaw) {
  const double a = PUCK_L - p.pinch_from_end_m - p.press_m, b = is_box ? p.contact_h_box : p.contact_h_cyl;
  const double ee_up = GRASP_Z - PUCK_H + GRIP_DEPTH / 2;  // 손끝이 손가락 접촉 중심 위 (손 아래 향함)
  const V3 u = {std::cos(yaw), std::sin(yaw), 0.0};
  const V3 pivot = plus(centre, u, -PUCK_L / 2);  // 가까운 끝 아래 모서리
  close_yaw = yaw + M_PI / 2;                      // 닫는 축 = 퍽 축과 직각 (perp)
  const auto on_arc = [&](double f, double h) {
    V3 q = plus(pivot, u, a * std::cos(f) - b * std::sin(f));
    q[2] += a * std::sin(f) + b * std::cos(f) + ee_up + h;
    return q;
  };
  std::vector<V3> pts;
  if (p.drag_m > 0.0) {  // 먼 끝을 잡은 채 로봇 쪽으로 대각선으로 끌고, 끝점에서 눌러 놓기 (기하는 실기 11-04 세우기 시험 292ec509)
    const V3 pinch = on_arc(0.0, 0.0);
    const int nd = up_segments(p);
    pts.push_back(pinch);
    for (int k = 1; k <= nd; ++k) {
      const double f = static_cast<double>(k) / nd;
      V3 q = plus(pinch, u, -p.drag_m * f);
      q[2] += p.drag_lift_m * f;
      pts.push_back(q);
    }
    pts.push_back(pts.back());  // 멈춤
    V3 set = pts.back();
    set[2] += p.set_dz_m;
    pts.push_back(set);
  } else {
    std::vector<double> fs;
    for (int i = 0;; ++i) {
      const double f = i * p.arc_step_deg;
      if (f > p.arc_end_deg + 1e-6) break;
      fs.push_back(f * M_PI / 180);
    }
    const double f_end = fs.back();
    for (double f : fs) pts.push_back(on_arc(f, p.tilt_over_m * f / f_end));  // 호 0 → 끝, 가며 tilt_over만큼 더 올림
    pts.push_back(pts.back());                                                // 멈춤
    for (auto it = fs.rbegin(); it != fs.rend(); ++it)                        // 같은 호로 90°까지 되돌림
      if (*it < f_end - 1e-9 && *it >= M_PI / 2 - 1e-9) pts.push_back(on_arc(*it, p.tilt_over_m));
    pts.push_back(on_arc(M_PI / 2, p.set_dz_m));  // 눌러 놓기
  }
  V3 above = pts.front();
  above[2] += p.above_m;
  V3 away = pts.back();
  away[2] += p.lift_away_m;
  pts.insert(pts.begin(), above);
  pts.push_back(away);
  V3 g = plus(pivot, u, -PUCK_W / 2);  // 선 퍽의 중심선 위 집기 높이
  g[2] += GRASP_Z;
  V3 lift = g;
  lift[2] += p.regrasp_lift_m;
  pts.push_back(g);
  pts.push_back(lift);
  return pts;
}

bool plan_ik(const V3 & centre, double yaw, bool is_box, const Params & p, std::vector<Q5> & qs, std::string & why) {
  double close_yaw;
  const auto pts = waypoints(centre, yaw, is_box, p, close_yaw);
  Q5 near = {std::atan2(pts[1][1], pts[1][0]), -0.2, 0.7, 1.0, 0.0};  // right_env 시드 (팔꿈치 위)
  qs.clear();
  const size_t away_i = pts.size() - 3;  // 뒤에서 [위로 · 다시 잡기 · 들어올림]
  for (size_t i = 0; i < pts.size(); ++i) {
    // 손을 아래로 향한 채 높이 못 가는 자리가 많음 — 대기·손 빼기 높이는 낮춰 가며 (실기 11/03)
    std::vector<V3> tries = {pts[i]};
    const auto lower = [&](const V3 & base, const std::array<double, 3> & hs) {
      for (double h : hs) tries.push_back({base[0], base[1], base[2] + h});
    };
    if (i == 0) lower(pts[1], {0.03, 0.02, 0.01});
    else if (i == away_i) lower(pts[i - 1], {0.06, 0.04, 0.03});
    Q5 q;
    bool ok = false;
    for (const auto & pt : tries)
      if ((ok = ik_down(pt, close_yaw, near, q))) break;
    if (!ok || (i >= 2 && i < away_i && linf(q, near) > MAX_JUMP)) { why = "stand_ik_unreachable"; qs.clear(); return false; }
    qs.push_back(q);
    near = q;
  }
  return true;
}

bool Table::ready(std::string & why) const {
  const size_t n = yaw_deg.size() * ycs.size() * static_cast<size_t>(std::max(w, 0)) * 5;
  for (int k = 0; k < 2; ++k)
    if (n == 0 || q[k].size() != n || P[k].size() != n * 3) { why = "stand_table_missing"; return false; }
  return true;
}

bool plan_table(const Table & t, const V3 & centre, double yaw, bool is_box, std::vector<Q5> & qs, std::string & why) {
  if (!t.ready(why)) return false;
  const auto nearest = [](const std::vector<double> & v, double x) {
    size_t best = 0;
    for (size_t i = 1; i < v.size(); ++i) if (std::abs(v[i] - x) < std::abs(v[best] - x)) best = i;
    return best;
  };
  const size_t b = nearest(t.yaw_deg, yaw * 180 / M_PI), yi = nearest(t.ycs, centre[1]);
  const double rel[3] = {centre[0] - t.box_cx, centre[1] - t.ycs[yi], 0.0};
  const auto & q = t.q[is_box ? 1 : 0];
  const auto & P = t.P[is_box ? 1 : 0];
  const size_t base = (yi * t.yaw_deg.size() + b) * static_cast<size_t>(t.w);
  qs.assign(t.w, Q5{});
  for (int w = 0; w < t.w; ++w)
    for (int j = 0; j < 5; ++j) {
      const size_t i = (base + w) * 5 + j;
      qs[w][j] = q[i] + P[i * 3] * rel[0] + P[i * 3 + 1] * rel[1] + P[i * 3 + 2] * rel[2];
    }
  for (const auto & qq : qs) if (!in_limits(qq)) { why = "stand_table_out_of_limits"; qs.clear(); return false; }
  return true;
}

double trapezoid_duration(double dist, double vel_scale) {
  const double vmax = VMAX * vel_scale, amax = AMAX * vel_scale, t_acc = vmax / amax;
  if (dist <= vmax * t_acc) return std::max(2 * std::sqrt(dist / amax), 0.1);
  return dist / vmax + t_acc;
}

double move_s(double t, double dur) {
  const double x = std::clamp(t / dur, 0.0, 1.0);
  return x * x * (3 - 2 * x);
}

int up_segments(const Params & p) {
  if (p.drag_m > 0.0) return std::max(1, static_cast<int>(std::ceil(p.drag_m / 0.01 - 1e-9)));
  return static_cast<int>(std::lround(p.arc_end_deg / p.arc_step_deg));
}

std::vector<std::pair<int, int>> arc_groups(const Params & p, int n_seg) {
  const int n = up_segments(p);
  return {{0, n}, {n, n + 1}, {n + 1, n_seg}};
}

std::vector<Q5> sample_move(const Q5 & from, const Q5 & to, double vel_scale, double dt) {
  const double t = trapezoid_duration(linf(from, to), vel_scale);
  const int n = std::max(1, static_cast<int>(std::ceil(t / dt - 1e-9)));
  std::vector<Q5> out;
  for (int k = 1; k <= n; ++k) {
    const double s = move_s(std::min(k * dt, t), t);
    Q5 q;
    for (int j = 0; j < 5; ++j) q[j] = from[j] + s * (to[j] - from[j]);
    out.push_back(q);
  }
  return out;
}

std::vector<Q5> sample_arc(const std::vector<Q5> & wp, const Params & p, double vel_scale, double hang_s, double dt) {
  const int n_seg = static_cast<int>(wp.size()) - 3;  // 경유점 1 … W−2 사이
  std::vector<double> d(n_seg);
  for (int k = 0; k < n_seg; ++k) d[k] = linf(wp[1 + k], wp[2 + k]);
  const auto groups = arc_groups(p, n_seg);
  const double vmax = VMAX * vel_scale, amax = AMAX * vel_scale;
  std::vector<double> dur;
  for (const auto & [i0, i1] : groups) {
    double L = 0;
    for (int k = i0; k < i1; ++k) L += d[k];
    dur.push_back(std::max(L <= vmax * vmax / amax ? 2 * std::sqrt(L / amax) : L / vmax + vmax / amax, hang_s));
  }
  double total = 0;
  for (double x : dur) total += x;
  std::vector<Q5> out;
  const int n = static_cast<int>(std::ceil(total / dt - 1e-9));
  for (int s = 1; s <= n; ++s) {
    const double tr = std::min(s * dt, total);
    size_t g = 0;
    double t0 = 0;
    while (g + 1 < groups.size() && tr >= t0 + dur[g]) t0 += dur[g++];
    const auto [i0, i1] = groups[g];
    double len = 0;
    for (int k = i0; k < i1; ++k) len += d[k];
    const double target = move_s(tr - t0, dur[g]) * len;
    int k = 0;
    double c = 0;  // 묶음 안 앞 구간까지 길이
    while (k < i1 - i0 - 1 && target > c + d[i0 + k]) c += d[i0 + k++];
    const double fr = std::clamp((target - c) / std::max(d[i0 + k], 1e-9), 0.0, 1.0);
    Q5 q;
    for (int j = 0; j < 5; ++j) q[j] = wp[1 + i0 + k][j] + fr * (wp[2 + i0 + k][j] - wp[1 + i0 + k][j]);
    out.push_back(q);
  }
  return out;
}

}  // namespace skills::stand
