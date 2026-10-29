# 주사위 기억 미션 (documents/mission/README.md §3) — signal_decoder가 신호등 12회 점멸을 관측한 결과
# 해석(정렬·배정)은 plan이 한다. 여기는 본 그대로의 사실만 (발제: 순서·횟수는 실제 센서 관측 기록으로 평가)
std_msgs/Header header
uint8 WAITING=0          # 신호 시작 전
uint8 OBSERVING=1        # 세는 중 — 점멸마다 갱신
uint8 DONE=2             # 12회 + 마지막 3초 연속 소등 확인
uint8 ERROR=3            # 관측 실패 — 추정하지 않고 멈춤
uint8 state
string[] sequence                  # 관측 순서 "R" | "Y" | "G" — DONE이면 정확히 12개
builtin_interfaces/Time[] onsets   # 각 점멸이 켜진 시각 (기록·재현용)
uint8 count_r
uint8 count_y
uint8 count_g
string error             # "" | too_many | unknown_color
