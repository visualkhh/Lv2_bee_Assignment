# urdf

BEE 로봇(TurtleBot3 Waffle Pi 베이스 + RM-X52 팔 + 그리퍼 + 카메라 2개)의 모델 정의 폴더다. Fusion 모델에서 뽑은 URDF와 메시, 그리고 이를 만들고 검증하는 스크립트가 들어 있다.

## 모델 고르기 (폴더 하나 = 모델 하나)

`urdf/<이름>/urdf.xacro` (+ 메시는 그 폴더 `meshes/`). ROS는 launch 인자 `urdf:=<이름>`으로 고르고(기본 `beebot_mesh`), 메시 경로는 launch가 `mesh_uri:=package://ros_system/urdf/<이름>/meshes`로 넘긴다 — 메시를 쓰는 모델은 `mesh_uri` 인자를 받아야 한다(`beebot_mesh`처럼). 시뮬레이터 [로봇 모델]에서 폴더를 고르면 확인 후 그 모델로 ROS를 다시 띄운다. 빌드(`build-ws`)가 `urdf/` 전체를 설치하고, 폴더마다 `test_urdf_<이름>`(xacro → check_urdf)이 돈다.

## 폴더 구조

```
urdf/
├── beebot.urdf.xacro      # ROS용 xacro (인자: mock_hardware·mock_effort·port·mesh_uri). make_xacro.py 생성본
├── beebot.urdf            # Isaac Sim URDF Importer용. xacro를 펼치고 메시 경로를 상대 경로로 바꾼 것
├── meshes/                # 링크별 STL 22개 (단위 mm, URDF에서 scale 0.001로 m 변환)
└── scripts/               # 생성·검증 스크립트
```

`beebot.urdf.xacro`와 `beebot_mesh/beebot.urdf`는 서로 변환된다. 한쪽을 고쳤으면 다른 쪽을 다시 만들고 검수까지 돌린다.

```bash
# beebot.urdf를 고쳤을 때
python3 scripts/make_xacro.py
# beebot.urdf.xacro를 고쳤을 때
scripts/make_urdf.sh
# 검수 (둘 다 고친 뒤 항상)
scripts/check_urdf.sh
```

### xacro 인자

| 인자 | 기본값 | 용도 |
| --- | --- | --- |
| `mock_hardware` | `false` | `true`면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌린다 (`ros_system.launch.py`, `faker.launch.py`가 넘김) |
| `mock_effort` | `0.0` | mock일 때 팔 관절 effort [A] (접촉 감지 테스트용) |
| `port` | `/dev/opencr` | OpenCR 시리얼 포트 (udev 고정 이름 — `ros2_ws/99-opencr.rules`) |
| `mesh_uri` | `package://ros_system/urdf/meshes` | 메시 경로 앞부분. `make_urdf.sh`는 이 값을 `beebot_mesh/meshes`로 바꿔 `beebot_mesh/beebot.urdf`를 만든다 |

```bash
xacro beebot.urdf.xacro mock_hardware:=true > /tmp/beebot.urdf && check_urdf /tmp/beebot.urdf
```

기본값은 `make_xacro.py`가 돌 때 `beebot_mesh/beebot.urdf`의 `<ros2_control>` 값을 그대로 가져온다. 없는 인자를 넘기면 xacro는 에러 없이 무시하므로, 인자를 추가하면 `make_xacro.py`와 `check_xacro.py`도 같이 고친다.

## 모델 파일

### 링크 구조

```
base_footprint
└─ base_link                      바퀴 축 아래 10 mm. x 앞, y 왼쪽, z 위
   ├─ wheel_left_link / wheel_right_link   continuous, 축 y
   ├─ pole_cam_link                        뒤 카메라 (고정)
   │   └─ pole_cam_color_optical_frame
   ├─ imu_link                             (고정)
   └─ arm_base_link                        팔 받침 (고정)
      └─ shoulder_link      joint1  z축  ±3.14 rad
         └─ upper_arm_link  joint2  ±1.92 rad
            └─ forearm_link joint3  -1.57 ~ 1.40 rad
               └─ wrist_link  joint4  -1.70 ~ 1.97 rad
                  └─ gripper_link  joint5  ±3.14 rad
                     ├─ finger_left_link / finger_right_link   prismatic 0 ~ 0.0265 m
                     ├─ gripper_cam_link
                     │   └─ gripper_cam_color_optical_frame
                     └─ end_effector_link
```

구동 관절은 6개(joint1~5, gripper_left_joint)와 바퀴 2개다. 오른쪽 손가락은 왼쪽을 따라 움직이는 mimic이다. 그리퍼는 0이 닫힘, 0.0265가 열림이다.

### 메시와 링크 매핑 (`beebot_mesh/meshes`)

| STL | 링크 | 설명 |
| --- | --- | --- |
| `base_link.stl` | base_link | 와플 판, 바구니, 기둥 등 베이스 전체를 한 덩어리로 합친 메시. 55만 삼각형을 3 mm 격자로 간략화한 것 (약 6 MB) |
| `wheel_left_link.stl`, `wheel_right_link.stl` | wheel_left/right_link | 바퀴 |
| `camera_rear_link.stl` | pole_cam_link | 뒤 카메라 |
| `arm_base_1.stl` | arm_base_link | 팔 받침 |
| `shoulder_1.stl` | shoulder_link | 어깨 |
| `link1_1.stl` | upper_arm_link | 위팔 |
| `link2_1.stl` | forearm_link | 아래팔 |
| `wrist_1.stl` | wrist_link | 손목 |
| `gripper_body_1.stl` | gripper_link | 그리퍼 몸체 |
| `finger_R_1.stl` | finger_left_link | 왼쪽 손가락 (이름이 교차되어 있음) |
| `finger_L_1.stl` | finger_right_link | 오른쪽 손가락 (이름이 교차되어 있음) |
| `camera_wrist_link.stl` | gripper_cam_link | 손목 카메라 |
| `storage_collision/storage_0~8.stl` | base_link | 바구니 충돌체 9개 (collision 전용, visual 없음) |

손가락 STL의 L/R은 CAD(Fusion) 기준이고 URDF 링크의 left/right는 로봇 기준이라 서로 반대다.

### 질량 (보정 후)

| 링크 | 질량 (g) |
| --- | --- |
| base_link | 2400.6 |
| wheel_left/right_link | 28.5 × 2 |
| pole_cam_link | 72.0 |
| arm_base_link | 81.1 |
| shoulder_link | 85.8 |
| upper_arm_link | 128.9 |
| forearm_link | 117.3 |
| wrist_link | 90.2 |
| gripper_link | 175.7 |
| finger_left/right_link | 53.6 × 2 |
| gripper_cam_link | 88.4 |
| **합계** | **3404.1** |

베이스는 Fusion 재질(대부분 Steel) 그대로면 약 8.3 kg이 나와서 공식 Waffle Pi 1.8 kg 등을 기준으로 줄였다. 팔은 Fusion 값을 그대로 썼다.

## 스크립트 (`beebot_mesh/scripts`)

| 스크립트 | 용도 | 입력 → 출력 |
| --- | --- | --- |
| `make_urdf.sh` | `beebot.urdf.xacro`를 펼쳐 `beebot_mesh/beebot.urdf`를 만들고 `check_urdf`로 확인한다 | xacro → `beebot_mesh/beebot.urdf` |
| `check_urdf.sh` | 검수 묶음. `check_urdf`, `verify.py`, `check_xacro.py`를 차례로 돌리고 하나라도 실패하면 종료 코드 1을 낸다 (ROS 환경을 알아서 불러옴) | `beebot_mesh/beebot.urdf`, `beebot.urdf.xacro` → 콘솔 |
| `build.py` | 질량을 목표값으로 보정하고 관성을 계산해 베이스 URDF를 조립한다. 팔과 합쳐 최종 `bee.urdf`를 만들고 베이스 메시를 간략화한다 | `raw/`, `rm_x52.urdf` → `out/bee.urdf` 등 |
| `make_xacro.py` | `beebot_mesh/beebot.urdf`를 xacro로 바꾼다. 메시 경로와 ros2_control 하드웨어 값을 xacro 인자로 되돌린다 | `beebot_mesh/beebot.urdf` → `beebot.urdf.xacro` |
| `merge_raw.py` | Fusion 추출 결과(batch)를 `build.py`가 읽는 형태로 합친다 | `raw/bodies_<tag>.json` → `raw/bodies.json` |
| `verify.py` | URDF를 검증한다 (트리, 관성 유효성, visual·collision 메시 전부 존재, 팔 조인트 위치, `raw/`가 있으면 메시 왕복 위치) | `beebot_mesh/beebot.urdf` → 콘솔 |
| `check_xacro.py` | `beebot.urdf.xacro`를 펼쳐 `beebot_mesh/beebot.urdf`와 비교한다 (이름, 그리퍼 방향과 mimic, 영점 형상 일치, end_effector·optical frame, ros2_control 조인트, xacro 인자 반영). 어긋나면 `make_urdf.sh`로 다시 만든다 | `beebot.urdf.xacro`, `beebot_mesh/beebot.urdf` → 콘솔 (ROS 환경 필요) |
| `stlio.py` | 바이너리 STL 읽기·쓰기 도우미 | 라이브러리 |
| `torque_check.py` | 팔 joint2~5의 정적 중력 토크를 계산하고 XM430-W350 정지 토크와 비교한다 | `beebot_mesh/beebot.urdf` → 콘솔 |
| `bracket_check.py` | 롤 브라켓의 강도·강성을 손계산한다 (FEA 아님) | `beebot_mesh/beebot.urdf` → 콘솔 |

`torque_check.py`와 `bracket_check.py`는 판정 없는 분석 리포트라 `check_urdf.sh`에 넣지 않았다. 설계(질량, 관절 범위, 브라켓)가 바뀌었을 때 따로 돌린다.

### Fusion에서 처음부터 다시 만들 때

`fusion_extract.py`(Fusion 안에서 실행, 이 폴더에는 없음)로 링크별 바디 데이터를 뽑고 `merge_raw.py` → `build.py` 순서로 돌린다. 이 두 스크립트는 예전 이름 체계(`bee.urdf`, 링크 `*_1`, 조인트 `J1_Base_Yaw` 등)로 `out/bee.urdf`를 만든다. 지금의 `beebot_mesh/beebot.urdf`는 그 결과에 이름 정리·그리퍼 mimic·end_effector·optical frame·ros2_control을 더한 것이라, `build.py` 결과를 바로 `beebot_mesh/beebot.urdf`로 쓸 수는 없다. `raw/`(Fusion 추출 결과)와 `rm_x52.urdf`(팔)는 이 폴더에 없다.

### make_xacro.py가 만드는 xacro

`beebot_mesh/beebot.urdf`에서 역변환한 단순본이라 매크로가 없다 (ros2_control 관절 블록도 관절마다 펼쳐져 있음). 첫 줄 근처의 `<!-- generated by make_xacro.py -->` 주석이 생성 표시이며, 이 주석이 없는 xacro(손으로 고친 파일)가 그 자리에 있으면 덮어쓰지 않고 멈춘다. xacro를 손으로 고쳐서 계속 쓰려면 이 주석을 지우고 `make_urdf.sh` 방향으로만 변환한다.

## Isaac Sim에서 쓰기

`beebot_mesh/beebot.urdf`를 Isaac Sim URDF Importer로 가져오면 USD가 만들어진다. 질량과 관성은 URDF 값이 USD로 한 번 변환되고, 시뮬은 USD만 읽는다. URDF를 고쳤다면 다시 가져와야 반영된다.
