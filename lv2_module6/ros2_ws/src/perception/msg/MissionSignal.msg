# 신호등 12회 점멸 (발제 v5 §02) — signal_decoder가 한 사이클(12번 점멸)을 읽은 결과
#   한 사이클 = 12번 점멸 (한 번 = 1초 켜짐 + 1초 꺼짐 · 같은 색 연속도 꺼짐으로 구분) → 3초 꺼짐 → 다시 반복
#   새 답이 확정될 때만 발행 (두 사이클 같게 읽힘 → ok=true · reliable + transient_local). 같은 답은 다시 안 냄
#   사이클마다 결과(확정 전·오류 count · unknown · range 포함)는 /mission/signal_status (같은 타입) · signal_decoder 로그 · /diagnostics
#   정렬(바코드 오름·내림) · L1~L3 배정 · 퍽 할당은 통합(판단) 쪽 — 여기는 본 그대로만
std_msgs/Header header  # stamp = 사이클이 끝난(3초 꺼짐을 확인한) 시각
string[] sequence       # 본 순서대로 색 — red | yellow | green | unknown(두 색 동시 등 모르는 색) (채점: 시점별 색 일치 12점)
int32 red               # R 횟수 → 빨간 퍽 그룹 (모양 무관)
int32 yellow            # Y 횟수 → 한 모양으로 통일한 퍽 그룹 (색 무관)
int32 green             # G 횟수 → 초록 퍽 그룹 (모양 무관)
bool ok                 # true = 12개 · 각 1~6회 · 모르는 색 없음 · confirm번 연속 같은 답
string error            # ok=false 이유 — count(12개 아님) | unknown(모르는 색) | range(1~6 벗어남) | unconfirmed(아직 confirm번 못 봄) · ok면 빈 값
int32 cycles            # 같은 답이 연달아 나온 사이클 수 (정상 사이클이 아니면 0)
string raw              # 읽은 그대로 (예: "R G R R G R G G Y G Y G" · 모르는 색은 ?) — 로그·확인용
# ---- 옛 3단계 규칙 칸 — vlm_task_planner가 아직 읽어서 빌드용으로만 남김 (값 안 채움: "" · 0). planner가 위 칸으로 옮기면 삭제
string color
int32 count
int32 sum
int32 dest_tag
