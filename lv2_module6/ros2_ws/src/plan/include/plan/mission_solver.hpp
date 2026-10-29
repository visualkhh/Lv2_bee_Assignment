#pragma once

// mission_solver — 주사위 기억 미션 (documents/mission/README.md) 판단만 하는 순수 함수 (ROS 없음 · test_mission_solver)
//   인지가 낸 사실(신호 횟수 · 바코드 원문 · 퍽 ID·색·모양)을 받아 "그래서 어떻게 할지"를 정한다
//   (경계: documents/proposal/mission-perception-interface.md)
#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace plan {

struct Counts { int r = 0, y = 0, g = 0; };

// 발제 p3: 각 1~6 · 합 12. 아니면 why = count_out_of_range | sum_not_12
bool valid_counts(const Counts & c, std::string & why);

// 바코드 원문 → 오름차순(true) · 내림차순(false). P2-01·P2-03 오름 · P2-02·P2-04 내림. 그 밖은 false 반환 (미등록 바코드)
bool parse_barcode(const std::string & code, bool & ascending);

// 그룹(R·Y·G)을 횟수로 정렬해 L1·L2·L3 순서로 — 동률이면 R → Y → G 순서 유지 (발제 p3). 예: "YRG"
std::string assign_locations(const Counts & c, bool ascending);

int group_count(const Counts & c, char group);

struct Puck {
  int tag = -1;
  std::string color;  // red | green (그 밖 = 모름)
  std::string shape;  // cylinder | cuboid (그 밖 = 모름)
};

// 퍽 12개를 그룹에 나눈다 — R = 빨강 r개 · G = 초록 g개 · Y = 남은 y개가 한 모양 (발제 p5, 도전 B).
//   먼저 집은 퍽 때문에 Y를 못 만드는 일이 없게 전체를 한 번에 정한다: 모양 s를 Y로 하면 Y에 들어갈 빨강 = 빨강 수 − r,
//   초록 = 초록 수 − g가 정해지므로, 그 수만큼 모양 s 퍽이 있는지만 보면 된다 (원기둥 먼저, 안 되면 직육면체).
//   → tag → 'R'|'Y'|'G'. 실패 why = unknown_puck(색·모양 모름 — 추정 금지) | duplicate_tag | puck_count | no_allocation
bool allocate(const std::vector<Puck> & pucks, const Counts & c, std::map<int, char> & out, std::string & why);

// 주사위 눈 n → 3×3 칸 번호 (0 = 북서 · 8 = 남동, 행 우선). 2·3은 ↘ 대각선, 6은 좌우 두 열 (발제 p4 그림)
std::vector<int> dice_cells(int n);
// 칸 → 장소 중심에서 (dx, dy) [m], 경기장 축 (+Y = 북)
std::pair<double, double> cell_offset(int cell, double step = 0.060);

struct Pose2 { double x = 0, y = 0, yaw = 0; };

// 경기장(arena) 자세 → map 자세. start = 로봇이 켜진 자리의 arena 자세 (map 원점 = 그 자리, map x = 그때 로봇 앞)
Pose2 arena_to_map(const Pose2 & p, const Pose2 & start);

// 정사각 박스(중심 c, 반 변 half) 위의 점 p를 팔로 다루려고 다가설 자세 후보 — 가까운 변부터 (변 4개).
//   변 바깥 standoff [m]에 서서 박스 안쪽을 바라봄, 변을 따라서는 p와 나란히 (박스 범위 안으로 자름). arena 좌표
std::vector<Pose2> approach_poses(double cx, double cy, double half, double px, double py, double standoff);

}  // namespace plan
