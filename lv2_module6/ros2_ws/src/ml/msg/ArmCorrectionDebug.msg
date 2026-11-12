# rl_arm_correction이 매 debug_hz(기본 20 Hz)로 내는 상태 — 실기 디버깅 (rosbag · PlotJuggler)
#   보낸 명령 = ref + corr + delta
std_msgs/Header header
string skill            # /skill_context skill ("" = 보정 꺼짐)
uint32 run_id
float64[5] ref          # 들어온 목표 관절각 (JTC) [rad]
float64[5] q            # 실제 관절각 [rad]
float64[5] corr         # 처짐 적분 [rad]
float64[5] delta        # 학습 정책 Δ [rad]
bool[5] sag_blocked     # 상한에 붙어 풀고 쉬는 관절 (닿아서 못 감)
bool still              # reference 멈춤 → 처짐 적분 도는 중
bool rest               # 쉬는 자세(stow) → 처짐 적분 0으로 풂
bool policy_on          # 이 skill 정책이 있음 (Δ 계산 중)
bool cam_ok             # 손 카메라 상대값이 쓸 만함 (kind · 나이)
float64 cam_age         # 손 카메라 상대값 나이 [s]
