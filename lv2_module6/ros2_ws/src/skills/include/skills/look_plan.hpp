#pragma once

// inspect가 퍽 여러 개를 gripper_cam 한 장에 담을 팔 자세 — ROS 없는 순수 함수 (skill_inspect 상자 보기 · test_look_plan).
//   카메라: URDF gripper_cam_color_optical_frame (gripper_link에 고정) · Isaac gripper_cam_rgb 핀홀 (fx 615.4, 640×480).
//   점마다 퍽 윗면 태그(z)와 몸통 가운데(z − mid_dz)가 둘 다 화면 가장자리 안 · 손가락이 찍히는 화면 아래 가운데 밖 ·
//   카메라에서 depth 하한 넘게 떨어짐 · 손끝이 점들보다 hand_clear 위 → 담은 비율이 가장 큰 자세, 그다음 가장 작은 태그가 큰 것
//   (tag_cap까지), 그다음 담은 점들 평균이 aim 화소에 가까운 것. 무작위 탐색 + 주변 다듬기 (seed 고정 → 같은 입력이면 같은 답).
//   Python 원형: isaac/scripts/tools/inspect_cover.py (11/07 Isaac 시험: 1차 자세 뒤 이 자세로 다시 보면 퍽 40개 중 37개 판독)
#include <array>
#include <vector>

#include "skills/stand_plan.hpp"

namespace skills::look {

using stand::Q5;
using stand::V3;

struct Params {
  double fx = 615.37, width = 640, height = 480;     // Isaac gripper_cam_rgb (focalLength 15.2449 / aperture 15.855 × 640)
  double edge_px = 30;                               // 화면 가장자리 이만큼 안
  double finger_u0 = 290, finger_u1 = 510, finger_v0 = 360;  // 손가락이 찍히는 화면 아래 가운데 (11/07 Isaac 영상)
  double depth_min_m = 0.17;                         // 실물 D435 640×480 depth 하한 근처
  double mid_dz = 0.03;                              // 몸통 가운데 = 윗면 − 이만큼 (서 있는 퍽 높이 6 cm의 반)
  double tag_m = 0.016;                              // 퍽 태그 한 변
  double tag_cap_px = 40;                            // 이만큼 크면 더 키워도 이득 없음 (판독 약 34 px + 여유)
  double aim_u = 320, aim_v = 220;                   // 담은 점 평균을 둘 화소 — 가운데보다 조금 위 (아래 가운데는 손가락)
  double hand_clear_m = 0.04;                        // 손끝(end_effector_link)이 가장 높은 점보다 이만큼 위
  double joint1_span = 0.6;                          // joint1 탐색 = 점들 방향 ± 이만큼 [rad]
  int samples = 20000, refine = 2000;
};

struct Score {
  double cover = -1;   // 담은 점 비율 (−1 = 손이 낮거나 점이 카메라 뒤)
  double min_px = 0;   // 담은 점 중 가장 작은 태그 [px] (tag_cap까지)
  double aim_off = 0;  // 담은 점 평균 ↔ aim 화소 거리 [px] (작을수록 좋음)
  bool better(const Score & o) const;
};

// 카메라 광학 좌표계 (원점 · 열: x 오른쪽 · y 아래 · z 앞) — base_footprint
void camera(const Q5 & q, V3 & origin, std::array<V3, 3> & axes);
// 점 p를 카메라 화면 (u, v)로 · 카메라 뒤면 false
bool project(const Q5 & q, const V3 & p, const Params & prm, double & u, double & v, double & dist);
Score score(const Q5 & q, const std::vector<V3> & tops, const Params & prm);
// tops = 퍽 윗면 가운데들 (base_footprint) → 자세 (joint5 = 0) · 하나도 못 담으면 false
bool search(const std::vector<V3> & tops, const Params & prm, Q5 & out, Score & best, unsigned seed = 0);

}  // namespace skills::look
