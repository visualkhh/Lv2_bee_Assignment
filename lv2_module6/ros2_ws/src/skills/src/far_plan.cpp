// 먼 퍽 집기 경유점 — 설명은 include/skills/far_plan.hpp (시뮬 pick_env.py _build_far_plan · _along)
#include "skills/far_plan.hpp"

#include <algorithm>
#include <cmath>

namespace skills::far {

namespace {
// 팔 기구 — stand_plan.cpp와 같은 값 (urdf/beebot_mesh/beebot.urdf). URDF가 바뀌면 두 곳 다 고칠 것 (test_far_plan이 stand::fk로 검산)
constexpr V3 ARM_BASE = {0.047985, -0.00023, 0.130175 + 0.010};  // joint1 축 (시뮬 J1_X, J1_Y)
constexpr double D1 = 0.0409, D2 = 0.0416;    // joint1 · joint2 높이
constexpr double L1X = 0.024, L1Z = 0.128;    // joint2 → joint3
constexpr double L2 = 0.124, L3 = 0.0629;     // joint3 → joint4 → joint5
constexpr double EE = 0.109055;               // gripper_link → end_effector_link (손끝)
constexpr Q5 LO = {-3.14159, -1.919862, -1.570796, -1.7, -3.141593};
constexpr Q5 HI = {3.14159, 1.919862, 1.396263, 1.97, 3.141593};
constexpr double MAX_JUMP = 0.5;      // [rad] 이웃 자세의 관절 차 상한 — 넘으면 팔꿈치 해가 바뀐 것 (시뮬 step()과 같음)
constexpr double SLIDE_STEP = 0.005;  // [m] 밀기를 이 걸음으로 풂 (시뮬과 같음)
constexpr double PITCH_STEPS = 5;     // 세우기 경유점 사이를 이만큼 나눠 풂 (15° / 5 = 3°, 시뮬과 같음)

double wrap(double a) { return std::remainder(a, 2 * M_PI); }
bool in_limits(const Q5 & q) {
  for (int i = 0; i < 5; ++i)
    if (q[i] < LO[i] - 1e-9 || q[i] > HI[i] + 1e-9) return false;
  return true;
}
// 관절 0..n−1의 최대 차 (손목 돌리기 단계는 joint5가 돌아가는 게 정상이라 n = 4로 봄)
double linf(const Q5 & a, const Q5 & b, int n = 5) {
  double m = 0;
  for (int i = 0; i < n; ++i) m = std::max(m, std::abs(a[i] - b[i]));
  return m;
}
}  // namespace

bool ik_tilt(const V3 & p, double a, double heading, double roll, const Q5 & near, Q5 & out) {
  // 팔 평면(joint1 축을 지나고 heading 방향) 안: 반지름 r(축에서 바깥) · 높이 z.
  // 손 방향(gripper x) = (cos a, −sin a) → joint4 = 손끝 − (EE + L3)·손 방향. 그다음은 stand::ik_down과 같은 2링크 풀이
  const double ux = std::cos(heading), uy = std::sin(heading);
  const double r_ee = (p[0] - ARM_BASE[0]) * ux + (p[1] - ARM_BASE[1]) * uy;  // 손끝은 이 평면 위 (옆 오차는 heading이 이미 맞춤)
  const double wr = r_ee - (EE + L3) * std::cos(a);
  const double wz = p[2] + (EE + L3) * std::sin(a) - (ARM_BASE[2] + D1 + D2);
  const double a1 = std::hypot(L1X, L1Z), al = std::atan2(L1Z, L1X);
  const double k = (wr * wr + wz * wz - a1 * a1 - L2 * L2) / (2 * a1 * L2);
  if (k < -1.0 || k > 1.0) return false;  // 너무 멀거나 가까움
  bool found = false;
  for (double sgn : {1.0, -1.0}) {  // 팔꿈치 위·아래
    const double q3 = sgn * std::acos(k) - al;
    const double bx = L1X + L2 * std::cos(-q3), bz = L1Z + L2 * std::sin(-q3);
    const double q2 = -(std::atan2(wz, wr) - std::atan2(bz, bx));
    Q5 q = {wrap(heading), wrap(q2), wrap(q3), 0.0, wrap(roll)};
    q[3] = wrap(a - q[1] - q[2]);  // 손 피치 = joint2 + joint3 + joint4 = a
    if (!in_limits(q)) continue;
    if (!found || linf(q, near) < linf(out, near)) out = q;
    found = true;
  }
  return found;
}

bool plan(const V3 & pb, double close_yaw, const Params & p, Plan & out, std::string & why) {
  why.clear();
  const double hd = std::atan2(pb[1] - ARM_BASE[1], pb[0] - ARM_BASE[0]);  // joint1 축 → 퍽 (시뮬 hd)
  const double tilt = p.tilt_deg * M_PI / 180.0;
  const V3 rad = {std::cos(hd), std::sin(hd), 0.0};
  // 손을 a만큼 기울이고 패드 가운데를 퍽 축 위 pad_h 높이에 둔 손끝 — 손 방향으로 back만큼 뒤 (시뮬 pose(a, back))
  const auto pose = [&](double a, double back) {
    const V3 axis = {std::cos(a) * rad[0], std::cos(a) * rad[1], -std::sin(a)};
    V3 e;
    for (int i = 0; i < 3; ++i) e[i] = pb[i] - (p.pad_m + back) * axis[i];
    e[2] += p.pad_h_m;
    return e;
  };
  // 앞 자세를 near로 이어 풂 — 못 풀면 unreachable, 튀면 jump
  const auto step = [&](const V3 & e, double a, const Q5 & prev, Q5 & q) {
    if (!ik_tilt(e, a, hd, 0.0, prev, q)) { why = "pick_far_unreachable"; return false; }
    if (linf(q, prev) > MAX_JUMP) { why = "pick_far_jump"; return false; }
    return true;
  };

  Q5 q_in;
  if (!ik_tilt(pose(tilt, 0.0), tilt, hd, 0.0, Q5{hd, -0.2, 0.7, 1.0, 0.0}, q_in)) {  // 들어간 자세 (시뮬 첫 seed와 같은 쪽 해)
    why = "pick_far_unreachable";
    return false;
  }
  Q5 q = q_in;
  for (double back = SLIDE_STEP; back <= p.back_m + 1e-6; back += SLIDE_STEP) {  // 뒤로 빼며 풂 → 마지막이 뒤로 뺀 자세
    Q5 nq;
    if (!step(pose(tilt, back), tilt, q, nq)) return false;
    q = nq;
  }
  std::vector<Q5> path = {q, q_in};
  q = q_in;
  const int keys = std::max(2, p.pitch_keys);
  const int n_fine = (keys - 1) * static_cast<int>(PITCH_STEPS);
  for (int i = 1; i <= n_fine; ++i) {  // 45° → 90° 3°씩, 5걸음마다 경유점 (시뮬 fine[1:], i % 5 == 0)
    const double a = tilt + (M_PI / 2 - tilt) * i / n_fine;
    Q5 nq;
    if (!step(pose(a, 0.0), a, q, nq)) return false;
    q = nq;
    if (i % static_cast<int>(PITCH_STEPS) == 0) path.push_back(q);
  }
  // 수직 자세에서 손목만 돌려 퍽 면에 맞춤 — 손끝 = 퍽 바닥 + pad_h + pad (= stand::GRASP_Z) · 손가락 대칭이라 덜 도는 쪽
  const V3 e_dn = {pb[0], pb[1], pb[2] + p.pad_h_m + p.pad_m};
  Q5 q_dn;
  if (!stand::ik_down_sym(e_dn, close_yaw, path.back(), q_dn)) { why = "pick_far_unreachable"; return false; }
  if (linf(q_dn, path.back(), 4) > MAX_JUMP) { why = "pick_far_jump"; return false; }
  path.push_back(q_dn);
  // 들어올리기: above_m 위 — 안 닿으면 로봇 쪽으로 5 mm씩 당겨 처음 풀리는 곳 (시뮬 r_in)
  for (double r_in = 0.0; r_in <= p.lift_in_max_m + 1e-6; r_in += SLIDE_STEP) {
    const V3 e_up = {e_dn[0] - r_in * rad[0], e_dn[1] - r_in * rad[1], e_dn[2] + p.above_m};
    Q5 q_up;
    if (stand::ik_down_sym(e_up, close_yaw, q_dn, q_up) && linf(q_up, q_dn) <= 1.0) {  // 손목이 뒤집히지 않는 해만
      out.path = std::move(path);
      out.lift = q_up;
      out.heading = hd;
      out.lift_in_m = r_in;
      return true;
    }
  }
  why = "pick_far_unreachable";
  return false;
}

std::vector<Q5> sample_path(const std::vector<Q5> & pts, double vel_scale, double dt) {
  std::vector<double> cum = {0.0};  // 경유점까지 누적 길이 (구간마다 관절 최대 차)
  for (size_t k = 1; k < pts.size(); ++k) cum.push_back(cum.back() + linf(pts[k - 1], pts[k]));
  const double len = cum.back();
  const double t = stand::trapezoid_duration(len, vel_scale);
  const int n = std::max(1, static_cast<int>(std::ceil(t / dt - 1e-9)));
  std::vector<Q5> out;
  for (int i = 1; i <= n; ++i) {
    const double d = stand::move_s(std::min(i * dt, t), t) * len;  // 경로 위 거리
    size_t k = 1;
    while (k + 1 < pts.size() && d > cum[k]) ++k;  // d가 들어 있는 구간 k−1 → k
    const double seg = cum[k] - cum[k - 1];
    const double f = seg > 1e-9 ? std::clamp((d - cum[k - 1]) / seg, 0.0, 1.0) : 1.0;
    Q5 q;
    for (int j = 0; j < 5; ++j) q[j] = pts[k - 1][j] + f * (pts[k][j] - pts[k - 1][j]);
    out.push_back(q);
  }
  return out;
}

}  // namespace skills::far
