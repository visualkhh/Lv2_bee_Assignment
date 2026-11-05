# hand_relative → rl_arm_correction (학습 보정 정책 입력): 손 카메라로 잰 상대 위치 — 시뮬 정책 입력과 같은 정의
#   (isaac/lab pick_env · place_env · right_env · slot_env, 설명: scripts/hand_geom.py)
std_msgs/Header header    # frame_id = base_link · stamp = 그 값을 만든 손 카메라 영상 시각 (slot은 TF 시각)
string kind               # puck | cell | slot (HandTarget과 같음)
bool valid                # 목표를 찾아 잰 값이면 true · 못 찾으면 false (값은 0)
geometry_msgs/Vector3 rel # [m] puck: 퍽 가운데 − 손끝 · cell: 칸 가운데 − 손 안 퍽 바닥 · slot: 칸 가운데 − 손 안 퍽 바닥
float32 face_yaw          # [rad] 사각 퍽 면 − 손 방향 (±45°) · 원기둥·모름 0
bool is_box               # 퍽 모양 box(사각기둥)
geometry_msgs/Vector3 axis      # 퍽 축 단위벡터 (서 있으면 0,0,1 · 누우면 가까운 끝 → 먼 끝)
bool neighbour            # 같은 상자 옆 퍽이 있음
geometry_msgs/Vector3 neighbour_rel  # [m] 옆 퍽 − 목표 퍽
float32 age               # [s] 쓴 검출이 얼마나 오래됐는지 (손 카메라 검출은 ≤ 4Hz — 받는 쪽이 새 값 올 때까지 유지)
