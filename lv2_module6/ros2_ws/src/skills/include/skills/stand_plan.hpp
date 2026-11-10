#pragma once

// 누운 퍽 세우기 (stand · stand_pick) 경유점 — 팀원 feature/isaac-lab isaac/lab/right_env.py "tilt" 방식을 그대로 옮김
//   (plan_waypoints · arc_groups · arc_durations · arc_q, bee_common.trapezoid_duration · move_s). ROS 없는 순수 함수.
//   좌표: 로봇 root(base_footprint) — x 앞 · y 왼쪽 · z 위, 바닥 z=0. 관절: joint1~5 [rad].
//   경유점 두 방식 (coordinator stand_waypoints):
//     ik    — 퍽마다 손끝 경유점을 계산하고, 손이 아래를 향하는 5축 해석 IK로 관절값 (기본)
//     table — 팀원이 isaac-lab에서 미리 푼 표(모양 × 퍽 축 방향 5° × 옆 위치)에서 가까운 칸 + 위치 선형 보정 q = q_nom + P·Δp
//   문서: documents/project/ros/05-arm.md 6-1절
#include <array>
#include <string>
#include <utility>
#include <vector>

namespace skills::stand {

using Q5 = std::array<double, 5>;
using V3 = std::array<double, 3>;

// 퍽·손 치수 (bee_common · right_env)
constexpr double PUCK_L = 0.060, PUCK_W = 0.030, PUCK_H = 0.060;
constexpr double GRIP_DEPTH = 0.020, FINGER_TIP = 0.0206;
constexpr double GRASP_Z = PUCK_H - GRIP_DEPTH + FINGER_TIP;  // 쥘 때 손끝(end_effector_link)이 퍽 바닥 위 0.0606
constexpr double GRIP_STOP = 0.0062;                          // 30mm 퍽에서 손가락이 멈추는 gripper_left_joint
// 서 있는 퍽을 쥘 손끝 높이 — 인지 z는 보이는 면 쪽이라 믿지 않고 받침만 가림 (상자 위 실측 0.10~0.15 · 바닥이면 0.09 아래)
inline double standing_grasp_z(double perceived_z) { return (perceived_z > 0.09 ? 0.07 : 0.0) + GRASP_Z; }
inline double gap_joint(double gap_m) { return GRIP_STOP + (gap_m - 0.030) / 2; }  // 손가락 사이 → gripper_left_joint

struct Params {               // right_env RightEnvCfg 값 (tilt)
  double above_m = 0.04;      // 쥐는 자리 위 — 여기서 내려감
  double lift_away_m = 0.08;  // 벌린 뒤 위로 (선 퍽에서 비킴)
  double contact_h_box = 0.030, contact_h_cyl = 0.015;  // 손가락 중심 높이 (누운 퍽 바닥 기준): 사각 윗면 · 원기둥 축
  double pinch_from_end_m = 0.0, press_m = 0.0;
  double tilt_over_m = 0.010;  // 호를 도는 동안 더 올림 (가볍게 쥔 퍽이 손을 늦게 따라옴)
  double arc_end_deg = 120.0;  // 호 끝 — 선 자세(90°)를 지나침
  double arc_step_deg = 7.5;   // 경유점 간격 (관절 직선 보간이 원에서 0.5mm 안)
  double set_dz_m = -0.010;    // 90° 자리에서 이만큼 눌러 놓음
  double regrasp_lift_m = 0.05;
  double drag_m = 0.0;         // > 0이면 호 대신 잡은 채 로봇 쪽으로 곧게 끌고 눌러 놓음 (실기 11/03~04 — 호는 너무 높이 듦)
  double drag_lift_m = 0.0;    // 끌면서 끝까지 올리는 양 (0 = 수평 — 수평은 퍽이 안 들림)
};

// 팔 기구 (urdf/beebot_mesh/beebot.urdf) — FK: 손끝(end_effector_link) 위치 + 손가락 닫는 축(gripper_link y) 방향
V3 fk(const Q5 & q, V3 * close_axis = nullptr);
// gripper_link 원점(joint5 축 위)과 축 방향(열: x 손가락 · y 닫는 축 · z) — base_footprint. inspect 카메라 자세(look_plan)가 씀
void fk_gripper(const Q5 & q, V3 & origin, std::array<V3, 3> & axes);
// 손이 바로 아래를 향하고 손가락 닫는 축이 수평 방향 close_yaw [rad]인 손끝 위치 p → 관절값 (해석해, 관절 한계 안).
// 해가 둘(팔꿈치 위·아래)이면 near에 가까운 것. 닿지 않거나 한계 밖이면 false
bool ik_down(const V3 & p, double close_yaw, const Q5 & near, Q5 & out);
// 손가락은 대칭이라 close_yaw와 close_yaw + π가 같은 파지 — 둘 중 near에 가까운 해
bool ik_down_sym(const V3 & p, double close_yaw, const Q5 & near, Q5 & out);

// 누운 퍽(가운데 xy·상자 윗면 z = centre, 축 방향 yaw: 가까운 끝 → 먼 끝)의 손끝 경유점 [위 · 시작 · 호 … · 내려놓기 · 위로 · 다시 잡기 · 들어올림]
//   + 손가락 닫는 축 방향 (퍽 축과 직각)
std::vector<V3> waypoints(const V3 & centre, double yaw, bool is_box, const Params & p, double & close_yaw);
// 퍽 축 yaw를 로봇에서 먼 쪽 방향으로 [−90°, 90°) — 인지 yaw는 180° 대칭(긴 축)이라
double away_yaw(double yaw);

// ik: 경유점마다 ik_down (앞 경유점을 near로 이어 감). 실패하면 why = "stand_ik_unreachable"
bool plan_ik(const V3 & centre, double yaw, bool is_box, const Params & p, std::vector<Q5> & qs, std::string & why);

// table: 팀원 표 (right_env 표와 같은 배열 — 모양마다 [ycs][yaw_bins][W][5] q, [ycs][yaw_bins][W][5][3] P, 평탄화)
struct Table {
  std::vector<double> yaw_deg;  // 퍽 축 방향 칸 (−90 … 90, 5° 간격)
  std::vector<double> ycs;      // 옆 위치 칸 [m] (−0.04 · 0 · 0.04)
  double box_cx = 0.0, box_h = 0.0;  // 표를 푼 퍽 가운데 x [m] (right_env BOX_CX) · 상자 윗면 z
  int w = 0;                    // 경유점 수 (다시 잡기 2개 포함)
  std::vector<double> q[2], P[2];  // [0] 원기둥 · [1] 사각
  bool ready(std::string & why) const;
};
// 가장 가까운 방향 칸 · 옆 칸 → q = q_nom + P·(x − box_cx, y − yc, 0) (right_env._reset_idx 그대로)
bool plan_table(const Table & t, const V3 & centre, double yaw, bool is_box, std::vector<Q5> & qs, std::string & why);

// ---------- 시간 매기기 (팀원 학습 때와 같은 궤적)
constexpr double VMAX = 3.0, AMAX = 5.0;  // 팀 관절 한계 [rad/s, rad/s²]
double trapezoid_duration(double dist, double vel_scale);
double move_s(double t, double dur);  // 0..1 smooth step
// 올라가는 구간 수 — 호: 경유점 간격 수 · 끌기: 1 cm마다
int up_segments(const Params & p);
// 호·끌기 구간을 [올라가기 · 멈춤 · 되돌려 놓기]로 묶음 (경유점 1 … W−2 사이 구간 번호)
std::vector<std::pair<int, int>> arc_groups(const Params & p, int n_seg);
// 샘플 궤적 — 시각 dt, 2dt, … 의 점 (from 자체는 빼고, 끝은 정확히 to). 실행할 때 지금 관절값을 시각 0으로 앞에 붙임
std::vector<Q5> sample_move(const Q5 & from, const Q5 & to, double vel_scale, double dt);
// from → to를 via를 지나는 관절 공간 2차 베지어로 (제어점 2·via − (from+to)/2 → 곡선 가운데가 via) — 경유 자세에서 멈추지 않음.
//   곡선이 관절 한계를 넘으면 via에서 꺾는 두 직선으로. 시간은 곡선 길이(관절 최대 차의 합)로 사다리꼴 · 부드러운 시작·끝 (move_s)
std::vector<Q5> sample_via(const Q5 & from, const Q5 & via, const Q5 & to, double vel_scale, double dt);
// 호: 경유점 wp[1] → … → wp[W−2]를 묶음마다 smooth step으로, 묶음 안은 관절 길이 비례 직선 보간
std::vector<Q5> sample_arc(const std::vector<Q5> & wp, const Params & p, double vel_scale, double hang_s, double dt);

}  // namespace skills::stand
