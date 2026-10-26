uint32 step
string skill        # navigate | pick | place | inspect | say
string target       # 물체 id (/tracked_objects의 id) · "x,y"(navigate) · "tray"(pick·place 적재함) · 말할 내용(say)
string relation     # front | on | ...
float32 offset_m
