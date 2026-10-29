# mission_manager 진행 상태 (설계 D10) — /mission/state (reliable + transient_local). 시뮬레이터 패널 · RViz · 상태 화면이 구독
std_msgs/Header header
string phase            # IDLE | GO_OBSERVE | BARCODE | READ | SURVEY | BUILD | VERIFY | RETURN | DONE | ERROR
string note             # 지금 하는 일 (사람이 읽는 설명)
string barcode          # 읽은 바코드 원문 ("" = 아직)
uint8 count_r           # 신호 횟수 (READ 뒤)
uint8 count_y
uint8 count_g
string order            # L1·L2·L3 그룹 — 예 "YRG" ("" = 아직)
string[] tray           # 적재함 칸 → 퍽 tag — "칸:tag" (예 "0:5")
uint16 placed           # 놓기를 마친 퍽 수 (놓은 뒤 확인 통과)
uint16 total            # 놓을 퍽 수
string[] results        # 퍽·장소별 결과 — 예 "puck 5: place_off 14mm" · "L1: ok" (실패해도 다음으로 계속 — 설계 D1)
string error            # ERROR 이유 (정지한 경우만)
