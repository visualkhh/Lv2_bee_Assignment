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
// 현장 진단 전용 — allocate가 puck_count · no_allocation으로 실패할 때(퍽이 12개가 아님) 가진 퍽으로 최대한 채움.
//   규칙은 같음: R = 빨강만 · G = 초록만 · Y = 한 모양(색 상관없음). 다 채운 그룹(장소) 수가 가장 많게, 같으면 채운 눈 합이 많게.
//   색·모양 모르는 퍽 · 같은 tag는 뺌. 남는 퍽은 out에 없음. 그룹마다 채운 수 → filled ('R'·'Y'·'G'). 하나도 못 채우면 false
bool allocate_partial(const std::vector<Puck> & pucks, const Counts & c, std::map<int, char> & out, std::map<char, int> & filled);

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

// 도면 보정 — 도면 → map 변환(arena_to_map)이 실제와 어긋난 만큼 (출발 자세 오차 · 도면 오차). map 안의 2D 강체:
//   보정 map 점 = R(dth)·(도면 map 점) + (dx, dy)  — 회전 중심 = map 원점(켠 자리)
struct ArenaCorr { double dx = 0, dy = 0, dth = 0; int n = 0; };
Pose2 apply_corr(const ArenaCorr & c, const Pose2 & m);
Pose2 unapply_corr(const ArenaCorr & c, const Pose2 & m);
// 짝(도면 map 점 m ↔ 인지 map 점 q)으로 보정을 맞춤. 1개거나 도면 점끼리 min_sep보다 가까우면 이동만, 아니면 이동+회전 (2D 최소제곱).
//   잔차가 outlier_m을 넘는 짝은 가장 큰 것부터 하나씩 빼고 다시 (2개 남을 때까지) · 회전은 ±max_rot, 이동은 max_shift로 자름
//   n = 쓴 짝 수 (0 = 짝 없음 → 보정 0)
struct CorrPair { double mx, my, qx, qy; };
ArenaCorr fit_arena_corr(std::vector<CorrPair> pairs, double min_sep, double max_shift, double max_rot, double outlier_m);
// 인지 점 q에 맞는 도면 상자 — cands(보정한 map 점) 중 gate 안 가장 가까운 것, 두 번째가 그보다 0.25 m 넘게 멀 때만 (헷갈리면 −1)
int match_landmark(double qx, double qy, const std::vector<std::pair<double, double>> & cands, double gate);

// 정사각 박스(중심 c, 반 변 half) 위의 점 p를 팔로 다루려고 다가설 자세 후보 — 가까운 변부터 (변 4개).
//   변 바깥 standoff [m]에 서서 박스 안쪽을 바라봄, 변을 따라서는 p와 나란히 (박스 범위 안으로 자름). arena 좌표
std::vector<Pose2> approach_poses(double cx, double cy, double half, double px, double py, double standoff);

// 방문 순서 파라미터(1부터 번호) → 0부터 인덱스. n개의 순열이 아니면(빈 값·중복·범위 밖) 0..n-1 그대로
std::vector<size_t> visit_order(const std::vector<int64_t> & order, size_t n);
// 점 (x, y)에서 가장 가까운 벽 선분까지 거리 [m] — walls = [x0, y0, x1, y1, …] (arena). 벽이 없으면 무한대
double wall_distance(double x, double y, const std::vector<double> & walls);

// ---------- 미션 단계 (순서와 실패 시 갈 곳만 정한다. 각 단계에서 무엇을 하는지는 mission_manager)
enum class Phase { IDLE, GO_OBSERVE, BARCODE, READ, SURVEY, BUILD, VERIFY, RETURN, DONE, ERROR };
const char * phase_name(Phase p);
// 단계가 끝났을 때 다음 단계 (그때까지 점수는 유효하므로 끝까지 간다):
//   관측(GO_OBSERVE · BARCODE · READ) 실패 = ERROR (발제: 신호·바코드 오류면 정지 · 추정 금지)
//   SURVEY 실패(할당 불가) = 쌓을 게 없으니 RETURN · BUILD는 퍽 단위로 실패를 기록하고 VERIFY · VERIFY 실패도 RETURN
//   RETURN 실패 = ERROR. DONE · ERROR는 그대로
//   diag (현장 진단 실행 — "start:DIAG…" · plan.yaml continue_on_fail): 관측 실패도 기록만 하고 다음 단계로 (SURVEY로 자료를 더 모음)
Phase next_phase(Phase p, bool ok, bool diag = false);

// ---------- 현장 진단 실행 — 실패해도 다음 퍽·장소·단계로 가되, 아래 "치명" 이유면 지금처럼 바로 정지 (ERROR · coordinator 정지)
//   사람·안전: estop(estop_blocked 포함) · manual(수동·티칭 전환) · explore · cancelled · preempted · aborted
//   부딪힘·과전류: arm_contact · collision · overcurrent
//   계속해도 소용없음: coordinator_unavailable
//   주행 갇힘: nav_no_progress · nav_failed가 stuck_limit번 이어짐 (사이에 성공한 계획이 있으면 다시 0부터)
//   ok = 계획 성공 (stuck 셈을 0으로) · 이유는 앞부분이 아니라 어디에 있어도 봄 ("retreat:arm_contact")
bool diag_fatal(bool ok, const std::string & why, int & stuck_streak, int stuck_limit);

// "R4Y2G6" → 신호 횟수 (R·Y·G 한 번씩, 순서 상관없음, 대소문자 상관없음) + valid_counts. why = bad_counts | count_out_of_range | sum_not_12
bool parse_counts(const std::string & text, Counts & c, std::string & why);

// /cmd/mission 시작 명령 (TUI 미션 메뉴 · ros2 topic pub)
//   start                                   처음부터 끝까지
//   start:<BARCODE|READ|SURVEY|RETURN>      그 단계만
//   start:DIAG[:<바코드>:<신호>]            처음부터 + 실패해도 계속 (diag). 바코드·신호를 주면 관측이 실패했을 때 대신 씀
//   start:FROM_SURVEY:<바코드>:<신호>       관측(GO_OBSERVE·BARCODE·READ) 없이 준 값으로 SURVEY부터 끝까지
//   start:DIAG_FROM_SURVEY:<바코드>:<신호>  위 둘을 같이
//   start:GOTO:<목록>                       S1~S6(시작 박스)·L1~L3(장소)을 적은 순서로 SURVEY처럼 다가감(Nav2 → 상자 도킹) 뒤 복귀 — 실기 이동 시험
//   예: start:DIAG_FROM_SURVEY:P2-04:R4Y2G6 — 바코드는 parse_barcode 표(P2-01~04), 신호는 parse_counts
//   why = not_start | unknown_command | need_barcode_signal | unknown_barcode | bad_counts | count_out_of_range | sum_not_12
struct StartCmd {
  Phase only = Phase::IDLE;  // 단계만 (IDLE = 전체)
  bool diag = false, from_survey = false, given = false;  // given = 바코드·신호를 받음
  std::string barcode;
  Counts counts;
  std::vector<std::string> go_to;  // start:GOTO:S1,S2,L1 — 그 상자들에 미션처럼 다가가기만 (S1~S6 시작 박스 · L1~L3 장소)
};
bool parse_start(const std::string & cmd, StartCmd & out, std::string & why);

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
