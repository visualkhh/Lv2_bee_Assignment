uint32 id
string class_name
float32 confidence
geometry_msgs/PoseWithCovariance pose   # map 기준 — 두 카메라 융합 위치 + 위치 공분산 · 퍽은 방향(yaw)을 구하면 orientation + 공분산 yaw 대각, 모르면 단위 쿼터니언 (필드 뜻: documents/API.md)
geometry_msgs/Vector3 size              # m — x·y = 카메라 영상 박스 가로·세로 폭 환산 (map 축 아님), z = 클래스 기본값
builtin_interfaces/Time last_seen       # 마지막 관측 영상 시각
string[] cameras                        # 본 카메라 전부 (pole_front_cam | gripper_cam) — 오래 전에 본 것부터, 마지막 = 가장 최근
int32[2] px                             # 마지막 카메라(cameras 끝) 영상 속 박스 중심 픽셀 [u, v]
string color                            # 퍽: red | orange | yellow | green | blue | purple | white | black | unknown (영상 색 — 태그 번호와 무관) · 신호등 상태: red | yellow | green | off | all(두 색 이상 동시) · 그 외 빈 값 (박스 가운데 HSV)
diagnostic_msgs/KeyValue[] details      # 클래스별 속성 — puck: posture · tag(AprilTag 0~11 — 번호는 무작위, 색과 무관) · shape(box | cyl) · yaw_period(90 | 180 deg — 방향 대칭 주기, 방향을 알 때만) · distance(로봇 중심~물체 바닥 위 m, /tracked_objects만) / location_box: tag(12~14) · yaw_period(360 태그 법선 · 못 구했으면 90 윗면 변) / start_box: yaw_period 90 / barcode: content(작업 지시) / traffic_light: state · 그 외 빈 배열
