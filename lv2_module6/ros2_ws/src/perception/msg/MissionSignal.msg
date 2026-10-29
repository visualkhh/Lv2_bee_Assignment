# 신호등 암호 미션 (documents/mission/README.md) — signal_decoder가 깜빡임 3단계를 읽은 결과
string color      # 1단계: 옮길 퍽 색 (red | green | yellow — 깜빡인 색)
int32 count       # 1단계: 그 색 퍽 개수 (깜빡인 횟수)
int32 sum         # 2단계: 고를 퍽들의 태그 번호 합 (깜빡인 색 점수의 합)
int32 dest_tag    # 3단계: 목적지 장소 태그 번호 (노랑 1·2·3번 → 12·13·14)
string raw        # 읽은 깜빡임 그대로 (예: "R R | R Y G G R | Y Y") — 확인용
