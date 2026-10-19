uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준 — 두 카메라 융합 위치 + 위치 공분산 (필드 뜻: documents/API.md)
geometry_msgs/Vector3 size              # m — x·y = 카메라 영상 박스 가로·세로 폭 환산 (map 축 아님), z = 클래스 기본값
builtin_interfaces/Time last_seen       # 마지막 관측 영상 시각
string camera                           # 마지막으로 본 카메라 (pole_cam | gripper_cam) — VLM 이미지 마킹용
int32[2] px                             # 그 카메라 영상 속 박스 중심 픽셀 [u, v]
string color                            # 퍽: red | orange | yellow | green | blue | purple | white | black | unknown · 신호등 상태: red | yellow | green | off · 그 외 빈 값 (박스 가운데 HSV)
