# 적재함 장부 — coordinator 하나만 씀 (실제로 싣고 꺼내는 쪽). mission_manager · 화면은 읽기만
#   /coordinator/basket (reliable + transient_local) — 바뀔 때마다. 파일(basket_file)에도 저장 → 노드가 다시 떠도 이어 감
#   비우기: /coordinator/basket_clear (std_srvs/Trigger) — 사람이 적재함을 손으로 비웠을 때
std_msgs/Header header
BasketSlot[] slots   # 칸 0~11 전부 (빈 칸 포함)
int32[] order        # 실은 순서 (칸 번호) — unload last = 끝
BasketSlot held      # 지금 손에 든 퍽 (filled=false면 빈손)
