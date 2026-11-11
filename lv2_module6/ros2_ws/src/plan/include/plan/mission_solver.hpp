#pragma once

// mission_solver — 주사위 기억 미션 (documents/mission/README.md) 판단만 하는 순수 함수 (ROS 없음 · test_mission_solver)
//   인지가 낸 사실(신호 횟수 · 바코드 원문 · 퍽 ID·색·모양)을 받아 "그래서 어떻게 할지"를 정한다
//   단계·skill 흐름: documents/project/ros/05-arm.md · 인지 인터페이스: documents/API.md (MissionSignal · TrackedObject)
#include <array>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace plan {

struct Counts { int r = 0, y = 0, g = 0; };

// 발제 p3: 각 1~6 · 합 12. 아니면 why = count_out_of_range | sum_not_12
bool valid_counts(const Counts & c, std::string & why);

// 신호 사이클 결과(signal_decoder /mission/signal_status · /mission/signal) → 할 일
//   ok = 두 사이클 같은 답으로 확정 → ACCEPT · error unconfirmed(아직 한 사이클) → WAIT
//   그 밖(count 12개 아님 · unknown 모르는 색 · range 1~6 벗어남)은 error_streak를 늘려 limit번 이어지면 STOP (발제: 추정 금지 → 오류 기록·안전 정지),
//   그 전엔 WAIT (한 사이클 잡음이면 다음 사이클을 봄). 정상 사이클이 오면 error_streak = 0
enum class SignalAction { WAIT, ACCEPT, STOP };
SignalAction signal_action(bool ok, const std::string & error, int & error_streak, int limit);

// 바코드 원문 → 오름차순(true) · 내림차순(false). P2-01·P2-03 오름 · P2-02·P2-04 내림. 그 밖은 false 반환 (미등록 바코드)
bool parse_barcode(const std::string & code, bool & ascending);

// 그룹(R·Y·G)을 횟수로 정렬해 L1·L2·L3 순서로 — 동률이면 R → Y → G 순서 유지 (발제 p3). 예: "YRG"
std::string assign_locations(const Counts & c, bool ascending);

int group_count(const Counts & c, char group);

struct Puck {
  int tag = -1;
  std::string color;  // red | green (그 밖 = 모름)
  std::string shape;  // cyl | box (perception details shape — 그 밖 = 모름)
};

// 퍽 12개를 그룹에 나눈다 — R = 빨강 r개 · G = 초록 g개 · Y = 남은 y개가 한 모양 (발제 p5, 도전 B).
//   먼저 집은 퍽 때문에 Y를 못 만드는 일이 없게 전체를 한 번에 정한다: 모양 s를 Y로 하면 Y에 들어갈 빨강 = 빨강 수 − r,
//   초록 = 초록 수 − g가 정해지므로, 그 수만큼 모양 s 퍽이 있는지만 보면 된다 (원기둥 먼저, 안 되면 직육면체).
//   → tag → 'R'|'Y'|'G'. 실패 why = unknown_puck(색·모양 모름 — 추정 금지) | duplicate_tag | puck_count | no_allocation
bool allocate(const std::vector<Puck> & pucks, const Counts & c, std::map<int, char> & out, std::string & why);

// 주사위 눈 n → 3×3 칸 번호 (0 = 북서 · 8 = 남동, 행 우선). 2·3은 ↘ 대각선, 6은 좌우 두 열 (발제 p4 그림)
std::vector<int> dice_cells(int n);
// 같은 칸 집합을 로봇이 서 있는 방향 기준으로 놓는 순서로 정렬: 뒤(로봇에서 먼 칸) → 앞, 같은 줄은 왼 → 오른쪽.
//   facing_yaw = 로봇이 장소 상자를 바라보는 방향 [rad, 경기장 축 · 0 = 동(+X) · π/2 = 북(+Y)]. 먼 줄부터 놓아야 팔이 놓인 퍽 위를 안 지난다.
//   칸 집합은 dice_cells(n)과 같다 (눈 방향은 경기장 기준 고정이라 서는 변이 달라도 같은 칸을 놓는다)
std::vector<int> dice_cells_ordered(int n, double facing_yaw);
// 칸 → 장소 중심에서 (dx, dy) [m], 경기장 축 (+Y = 북)
std::pair<double, double> cell_offset(int cell, double step = 0.060);

struct Pose2 { double x = 0, y = 0, yaw = 0; };

// 경기장(arena) 자세 → map 자세. start = 로봇이 켜진 자리의 arena 자세 (map 원점 = 그 자리, map x = 그때 로봇 앞)
Pose2 arena_to_map(const Pose2 & p, const Pose2 & start);

// 정사각 박스(중심 c, 반 변 half) 위의 점 p를 팔로 다루려고 다가설 자세 후보 — 가까운 변부터 (변 4개).
//   변 바깥 standoff [m]에 서서 박스 안쪽을 바라봄, 변을 따라서는 p와 나란히 (박스 범위 안으로 자름). arena 좌표
std::vector<Pose2> approach_poses(double cx, double cy, double half, double px, double py, double standoff);

// 점 (x, y)에서 가장 가까운 벽 선분까지 거리 [m] — walls = [x0, y0, x1, y1, …] (arena). 벽이 없으면 무한대
double wall_distance(double x, double y, const std::vector<double> & walls);

// ---------- 미션 단계 (순서와 실패 시 갈 곳만 정한다. 각 단계에서 무엇을 하는지는 mission_manager)
enum class Phase { IDLE, GO_OBSERVE, BARCODE, READ, SURVEY, BUILD, VERIFY, RETURN, DONE, ERROR };
const char * phase_name(Phase p);
// 단계가 끝났을 때 다음 단계 (그때까지 점수는 유효하므로 끝까지 간다):
//   관측(GO_OBSERVE · BARCODE · READ) 실패 = ERROR (발제: 신호·바코드 오류면 정지 · 추정 금지)
//   SURVEY 실패(할당 불가) = 쌓을 게 없으니 RETURN · BUILD는 퍽 단위로 실패를 기록하고 VERIFY · VERIFY 실패도 RETURN
//   RETURN 실패 = ERROR. DONE · ERROR는 그대로
Phase next_phase(Phase p, bool ok);

// ---------- 놓기 기준 보정 — 장소 태그를 예상 자리(ex, ey)가 아닌 (ox, oy)에서 봤으면 그만큼 칸을 옮긴다 (arena).
//   max_m 넘게 어긋나면 잘못 본 것으로 보고 보정 안 함 (0, 0)
std::pair<double, double> tag_correction(double ex, double ey, double ox, double oy, double max_m);

// ---------- 재검증 — 이번에 새로 본 퍽(arena)만으로 판정
struct Seen { int tag = -1; double x = 0, y = 0; };  // tag -1 = 번호 못 읽음
struct CellResult { int tag = -1, cell = -1; bool found = false; double err = 0; };  // err [m] = 칸 중심 ~ 본 자리
struct LocationResult {
  std::vector<CellResult> cells;
  std::vector<Seen> extra;  // 기대하지 않은 퍽 (빈 칸 · 다른 장소 퍽 · 번호 못 읽은 퍽)
  bool ok = false;          // 모든 칸이 found · err ≤ tol 이고 extra 없음
};
// expected = (tag, 칸) 목록 · (cx, cy) 장소 중심 · step 칸 간격
LocationResult judge_location(const std::vector<std::pair<int, int>> & expected, double cx, double cy, double step,
                              const std::vector<Seen> & seen, double tol);
// 12개 전체 — 두 장소 이상에서 보인 tag(dup) · 어디서도 안 보인 tag(missing)
void global_check(const std::vector<std::vector<int>> & seen_per_location, const std::vector<int> & all_tags,
                  std::vector<int> & dup, std::vector<int> & missing);

// ---------- 복귀 — 로봇 외곽 직사각형(base_footprint 기준 앞 front · 뒤 rear(음수) · 옆 ±side)의
//   네 모서리가 구역 [x0, y0, x1, y1] 안인지. p = 로봇 자세 (arena)
bool footprint_in_zone(const Pose2 & p, double front, double rear, double side, const std::vector<double> & zone);

}  // namespace plan
