# 적재함 칸 하나 (coordinator 장부 — /coordinator/basket)
int32 slot          # 칸 번호 0~11 (URDF basket_slot_N) · held(손)일 때 -1
bool filled         # 퍽이 있음
string shape        # box | cyl (perception details shape — 모르면 box)
string color        # red | green … (perception color — 모르면 "")
int32 tag           # 퍽 AprilTag 번호 0~11 (못 읽었으면 -1)
bool check          # 확인 필요 — 꺼내려고 잡았는데 빈손(grasp_empty). 비우지 않고 표시만
