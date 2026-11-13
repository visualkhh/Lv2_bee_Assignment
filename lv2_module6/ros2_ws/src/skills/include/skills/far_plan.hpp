#pragma once

// 먼 퍽 집기 (pick_far) 경유점 — 시뮬 isaac/rl/bee_isaac/tasks/pick_env.py _build_far_plan · _apply_action 을 그대로 옮김 (ROS 없는 순수 함수)
//   손을 수직으로 내려서는 안 닿는 박스 먼 쪽 퍽(로봇 기준점에서 x 약 250~285 mm)을 집는 동작:
//     ① 뒤로 뺀 자세 (손을 45° 기울여, 쥘 자리에서 손 방향으로 80 mm 뒤) — MoveIt으로 여기까지 옴, 손가락은 66 mm 벌림
//     ② 밀고 들어가기 (45° 그대로 쥘 자리까지) → ③ 손 세우기 (퍽 축을 중심으로 45° → 90°) → ④ 손목 돌리기 (퍽 면에 맞춤)
//     ⑤ 닫기 → ⑥ 들어올리기 (5 cm 위, 안 닿으면 로봇 쪽으로 조금 당겨서)
//   좌표: 로봇 root(base_footprint) — x 앞 · y 왼쪽 · z 위. 관절: joint1~5 [rad]. 손끝 = end_effector_link (stand_plan과 같음)
#include <string>
#include <vector>

#include "skills/stand_plan.hpp"

namespace skills::far {

using stand::Q5;
using stand::V3;

// 시뮬 PickEnvCfg far_* 값 (pick_env.py) — 바꾸면 정책 학습 조건과 달라짐
struct Params {
  double tilt_deg = 45.0;       // 들어갈 때 손 기울기 (수평 아래로) — far_tilt_deg
  double back_m = 0.08;         // 뒤로 뺀 자세: 쥘 자리에서 손 방향으로 이만큼 뒤 — far_back_m
  int pitch_keys = 4;           // 45° → 90° 사이 경유점 수 (들어간 자세 포함) — far_pitch_keys
  double pad_m = 0.0106;        // 손끝 → 손가락 패드 가운데 (손 방향) — PAD
  double pad_h_m = 0.05;        // 들어가고 세우는 동안 패드 가운데 높이 (퍽 바닥 위) — PAD_H
  double above_m = 0.05;        // 들어올리기 높이 (쥔 손끝 위) — above_m
  double lift_in_max_m = 0.08;  // 들어올릴 때 안 닿으면 로봇 쪽으로 당기는 최대 — far_lift_in_max_m
};

// 손이 수평 아래로 a [rad] 기울고 (a = 90°면 바로 아래) 팔 평면 방향이 heading [rad] (joint1 축에서 본 방향), 손목 roll [rad]인
// 손끝 위치 p → 관절값 (해석해: joint1 = heading, joint2~3 = 팔 평면 2링크, joint4 = a − joint2 − joint3, joint5 = roll).
// 팔꿈치 해가 둘이면 near에 가까운 것. 닿지 않거나 관절 한계 밖이면 false. (stand::ik_down은 a = 90°인 경우)
bool ik_tilt(const V3 & p, double a, double heading, double roll, const Q5 & near, Q5 & out);

// 한 번 계획한 결과
struct Plan {
  std::vector<Q5> path;  // 내려가는 관절 경로: [뒤로 뺀 자세, 들어간 자세, 세우기 경유점 …(마지막 = 수직), 쥐는 자세(q_down)]
  Q5 lift{};             // 들어올린 자세 (올라가는 구간 끝)
  double heading = 0.0;  // joint1 축에서 퍽으로 가는 수평 방향 [rad]
  double lift_in_m = 0.0;  // 들어올릴 때 로봇 쪽으로 당긴 양 [m]
  const Q5 & retreat() const { return path.front(); }  // 시뮬 q_above (정책 입력의 '위 자세')
  const Q5 & down() const { return path.back(); }      // 시뮬 q_down
};

// puck_bottom: 퍽 바닥 가운데 [m, base_footprint] · close_yaw: 쥘 때 손가락 닫는 축의 수평 방향 [rad] (stand::ik_down과 같은 뜻 —
// 손가락이 대칭이라 close_yaw + π도 같은 파지, 손목이 덜 도는 쪽을 씀).
// 시뮬처럼 작은 걸음(밀기 5 mm · 세우기 3°)으로 풀며 이웃 자세가 0.5 rad 넘게 튀면(팔꿈치 해가 바뀜) 실패.
// 실패하면 why = "pick_far_unreachable" (팔이 못 닿음) · "pick_far_jump" (경로 중 자세가 튐)
bool plan(const V3 & puck_bottom, double close_yaw, const Params & p, Plan & out, std::string & why);

// 내려가는 경로 샘플 — 시뮬 _along: 구간마다 관절 최대 차로 길이를 재고, 전체를 smooth step(move_s) 한 번으로
// (경유점에서 멈추지 않음). 시간 = 전체 길이의 사다리꼴 시간 (팀 한계 × vel_scale). 시각 dt, 2dt, … 의 점 (첫 점 자체는 뺌)
std::vector<Q5> sample_path(const std::vector<Q5> & pts, double vel_scale, double dt);

}  // namespace skills::far
