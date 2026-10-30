uint32 step
string skill        # 스킬 13개: read_barcode | observe_signal | inspect_box | verify_location | navigate | dock | undock |
                    #           arm_pose | pick_box | load_slot | unload_slot | place_pip | nudge
                    # 예전 이름 (VLM 일반 명령): pick | place | inspect | say  — 설명은 include/plan/coordinator.hpp
string target       # 물체 id (/tracked_objects의 id) · "x,y"(navigate) · "x,y,z"(place_pip·verify_location, map)
                    # · 적재함 칸 번호 또는 ""(load_slot·unload_slot, "tray"도 같음) · scan|initial|carry(arm_pose) · 말할 내용(say)
string relation     # front | on | ...
float32 offset_m
