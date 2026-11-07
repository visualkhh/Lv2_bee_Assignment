uint32 step
string skill        # navigate | approach | pick | place | inspect | say | pose | load | unload | stand | stand_pick — coordinator가 skill 노드 /skills/<skill>에 맡김 (정의: documents/project/ros/05-arm.md)
string target       # 물체 id (/tracked_objects의 id) · "x,y"(navigate) · 자세 이름(pose) · 칸 0~11 · next(load) · last(unload) · "tray"(pick·place — unload last·load next와 같음) · 말할 내용(say)
string relation     # front | on | ... · approach: 주행 정책 dock_box(기본) | park_home | drive_arc | drive_straight · inspect: box (좌표 = 상자 가운데, look 자세 → 다시 담기)
float32 offset_m
