uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준 — 두 카메라 융합 위치 + 위치 공분산 (필드 뜻: documents/API.md)
geometry_msgs/Vector3 size              # m — x·y = 카메라 영상 박스 가로·세로 폭 환산 (map 축 아님), z = 클래스 기본값
builtin_interfaces/Time last_seen       # 마지막 관측 영상 시각
string[] cameras                        # 본 카메라 전부 (pole_front_cam | gripper_cam) — 오래 전에 본 것부터, 마지막 = 가장 최근
int32[2] px                             # 마지막 카메라(cameras 끝) 영상 속 박스 중심 픽셀 [u, v] — VLM 이미지 마킹용
string color                            # 퍽: red | orange | yellow | green | blue | purple | white | black | unknown (윗면 태그를 읽었으면 태그 번호대 색) · 신호등 상태: red | yellow | green | off | all(두 색 이상 동시) · 그 외 빈 값 (박스 가운데 HSV)
diagnostic_msgs/KeyValue[] details      # 클래스별 속성 — puck: posture · shape(box|cyl) · yaw(누운 퍽 축 deg 0~180, OBB 모델만) · tag(윗면 AprilTag 0~11) / location: tag(12~14) / barcode: content(작업 지시) / traffic_light: state · 그 외 빈 배열
