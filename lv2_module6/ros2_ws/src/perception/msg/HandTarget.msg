# coordinator → hand_relative: 지금 skill이 손 카메라로 무엇을 재야 하는지 (닿는 구간 시작 전에 한 번, 끝나면 kind=none)
#   skill별: pick·unload·stand·regrasp = puck · place = cell · load = slot (documents/API.md)
string kind               # puck | cell | slot | none (멈춤)
int32 puck_tag            # puck: AprilTag 번호 0~11 · -1 = 손에 가장 가까운 퍽
int32 box_tag             # cell: 장소 상자 태그 12~14 (-1 = 상자 보정 안 함)
geometry_msgs/Point point       # cell: 계획 때 칸 가운데 (map, 상자 윗면 높이)
geometry_msgs/Point box_point   # cell: 계획 때 상자 가운데 (map) — 지금 본 상자와의 차이만큼 칸을 옮김
int32 slot                # slot: 적재함 칸 0~11
