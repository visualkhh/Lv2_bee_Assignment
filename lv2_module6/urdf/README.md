# urdf

BEE 로봇(TurtleBot3 Waffle Pi 베이스 + RM-X52 팔 + 그리퍼 + 카메라 2개)의 모델 정의 폴더다. Fusion 모델에서 뽑은 URDF와 메시, 그리고 이를 변환·검증하는 스크립트가 들어 있다. Fusion에서 URDF를 새로 뽑는 생성 스크립트는 여기 없다(아래 "Fusion에서 처음부터 다시 만들 때").

## 모델 고르기 (폴더 하나 = 모델 하나)

`urdf/<이름>/` 폴더 하나가 모델 하나다. 메시는 그 폴더의 `meshes/`에 둔다. ROS는 launch 인자 `urdf:=<이름>`으로 모델을 고른다(기본 `default`). 메시 경로는 launch가 `mesh_uri:=package://ros_system/urdf/<이름>/meshes`로 넘긴다. 메시를 쓰는 모델은 `mesh_uri` 인자를 받아야 한다. 시뮬레이터 [로봇 모델]에서 폴더를 고르면 확인 후 그 모델로 ROS를 다시 띄운다. 빌드(`build-ws`)가 `urdf/` 전체를 설치하고, 폴더마다 `test_urdf_<이름>`(xacro → check_urdf)이 돈다.

| 폴더 | xacro 파일 | 내용 |
| --- | --- | --- |
| `default` | `urdf.xacro` | 기본 모델(팀 배포본). `beebot_mesh`를 `deploy_default.sh`로 복사한 것 + Gazebo 블록(RGBD 카메라 2대, IMU, DiffDrive, 팔 JointPositionController). 직접 고치지 않는다 |
| `beebot_mesh` | `urdf.xacro` | 작업용 원본. 실측 CAD 메시 모델 (Gazebo 블록 없음). 변환·검수·배포 스크립트가 이 폴더에 있다 |
| `beebot_normal` | `urdf.xacro` | 기본 도형으로 만든 더미 치수 모델 (메시 없음). TurtleBot3 Waffle Pi + OM-X + 손목 roll + 그리퍼 |
| `beebot_slam` | `urdf.xacro` | SLAM 전용 더미 모델. 팔·그리퍼·gripper_cam을 빼고 폴 뒤쪽 카메라 `pole_rear_cam`을 더했다 |

작업은 `beebot_mesh/`에서만 하고, 팀에 배포할 때 `default/`로 덮어쓴다:

```bash
beebot_mesh/scripts/deploy_default.sh
```

- `beebot_mesh/urdf.xacro` → `default/urdf.xacro`, `beebot_mesh/meshes/` → `default/meshes/`로 덮어쓴다.
- `default/urdf.xacro`의 Gazebo 블록(`<!-- Gazebo` 주석부터 `</robot>` 앞까지)은 떼어 두었다가 새 xacro 끝에 다시 붙인다. Gazebo 블록이 없으면 덮어쓰지 않고 멈춘다.
- 덮어쓰기 전에 xacro를 펼쳐 `check_urdf`로 확인한다. 실패하면 `default/`는 그대로다.
- `default/meshes/`에만 있는 파일은 지우지 않는다. `default/meshes/maze.stl`은 xacro가 쓰지 않는 파일이다.

`default/`를 직접 고치면 다음 배포 때 Gazebo 블록 말고는 모두 덮어써진다. Gazebo 설정만 `default/urdf.xacro`에서 고친다.

## 폴더 구조

```
urdf/
├── default/                   # 팀 배포본 (deploy_default.sh가 덮어씀)
│   ├── urdf.xacro             # beebot_mesh/urdf.xacro + Gazebo 블록
│   └── meshes/                # beebot_mesh/meshes 복사본 (+ maze.stl)
├── beebot_mesh/               # 작업용 원본
│   ├── urdf.xacro             # ROS용 xacro (인자: mock_hardware·mock_effort·port·mesh_uri)
│   ├── beebot.urdf            # Isaac Sim용 순수 URDF. make_urdf.sh 생성본, 레포에 넣지 않음
│   ├── meshes/                # 링크별 STL + storage_collision/ 9개 (단위 mm, URDF에서 scale 0.001로 m 변환)
│   └── scripts/               # 변환(urdf ↔ xacro)·검증·분석·배포 스크립트
├── beebot_normal/urdf.xacro   # 더미 치수 모델
└── beebot_slam/urdf.xacro     # SLAM 전용 더미 모델
```

`beebot_mesh/urdf.xacro`와 `beebot_mesh/beebot.urdf`(Isaac Sim URDF Importer용, 메시 경로가 상대 경로 `meshes/`)는 서로 변환된다. `beebot.urdf`는 레포에 없으므로 처음에는 `make_urdf.sh`로 만든다. 한쪽을 고쳤으면 다른 쪽을 다시 만들고 검수까지 돌린다. 스크립트는 어디서 실행해도 `beebot_mesh/`에서 동작한다.

```bash
# beebot.urdf를 고쳤을 때
python3 beebot_mesh/scripts/make_xacro.py
# urdf.xacro를 고쳤을 때
beebot_mesh/scripts/make_urdf.sh
# 검수 (둘 다 고친 뒤 항상)
beebot_mesh/scripts/check_urdf.sh
# 팀 배포: urdf.xacro·meshes/ 를 default/ 에 덮어쓰기 (default 의 Gazebo 블록은 유지)
beebot_mesh/scripts/deploy_default.sh
```

`.sh`는 bash 스크립트라 그대로 실행한다(`python3`로 돌리면 `SyntaxError`). `.py`는 `python3`로 돌린다.

### xacro 인자

`beebot_mesh` 기준이다. `default`는 배포본이라 같다. `beebot_normal`·`beebot_slam`은 `mesh_uri`가 없다.

| 인자 | 기본값 | 용도 |
| --- | --- | --- |
| `mock_hardware` | `false` | `true`면 BeebotSystemHardware가 실기 대신 명령을 상태로 되돌린다 (`ros_system.launch.py`, `faker.launch.py`가 넘김) |
| `mock_effort` | `0.0` | mock일 때 팔 관절 effort [A] (접촉 감지 테스트용) |
| `port` | `/dev/opencr` | OpenCR 시리얼 포트 (udev 고정 이름 — `ros2_ws/99-opencr.rules`) |
| `mesh_uri` | `package://ros_system/urdf/meshes` | 메시 경로 앞부분. `make_urdf.sh`는 기본값을 상대 경로 `meshes/`로 바꿔 `beebot_mesh/beebot.urdf`를 만든다 |

```bash
xacro beebot_mesh/urdf.xacro mock_hardware:=true > /tmp/beebot.urdf && check_urdf /tmp/beebot.urdf
```

기본값은 `make_xacro.py`가 돌 때 `beebot_mesh/beebot.urdf`의 `<ros2_control>` 값을 그대로 가져온다. 없는 인자를 넘기면 xacro는 에러 없이 무시하므로, 인자를 추가하면 `make_xacro.py`와 `check_xacro.py`도 같이 고친다.

## 모델 파일

### 링크 구조

```
base_footprint
└─ base_link                      바퀴 축 아래 10 mm. x 앞, y 왼쪽, z 위
   ├─ wheel_left_link / wheel_right_link   continuous, 축 y
   ├─ front_pole_cam_link                  폴 앞쪽 카메라 (고정)
   │   └─ front_pole_cam_color_optical_frame
   ├─ imu_link                             (고정)
   ├─ basket_slot_0 ~ 11                   바구니 칸 바닥 중심 (고정, 방향은 base_link와 같음)
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

팔은 `arm_mount_joint`로 `base_link`에 고정돼 있고, 위치는 (47.985, −0.23, 130.175) mm다. Fusion `rm-x52:1` 조인트 원점 (51.045, 0, 134.795) mm에 팔 메시 보정 (−3.06, −0.23, −4.62) mm를 더한 값이다. 2026-10-19 수정: 팔 링크 메시가 조인트째 그 양만큼 밀려 있어서(팔 받침 4.6 mm 틈, 손목 카메라 브라켓이 그리퍼에 박혀 보임) 메시와 조인트를 함께 옮겼다. 메시만 옮기면 joint1~5 축이 서보 회전 중심에서 3~5.5 mm 벗어난다. 손목 카메라(`gripper_cam_joint`)는 Fusion 위치에 그대로 있도록 그리퍼 기준값을 같이 보정했다. 집는 점(`end_effector_link`)은 영점에서 (431.94, −0.23, 340.93) mm다.

바구니 칸 프레임 `basket_slot_k`는 칸(45 × 45 mm, 높이 35 mm) 바닥 중심이다. 3행 × 4열, 중심 간격은 x·y 모두 65 mm다. x는 −169.84 / −104.84 / −39.84 mm(기둥 → 팔), y는 −97.5 / −32.5 / 32.5 / 97.5 mm, z는 91.25 mm다. 번호는 `k = 행 × 4 + 열`이라 0번이 기둥 쪽 −y 끝, 11번이 팔 쪽 +y 끝이다. 값은 `storage_collision` 벽 위치와 맞춰 확인했다. 놓기·집기 목표 자세와 Isaac Sim 성공 판정의 기준으로 쓴다. 칸이 찼는지 같은 상태는 TF에 싣지 않고 노드가 가진다.

구동 관절은 6개(joint1~5, gripper_left_joint)와 바퀴 2개다. 오른쪽 손가락은 왼쪽을 따라 움직이는 mimic이다. 그리퍼는 0이 닫힘, 0.0265가 열림이다.

### 메시와 링크 매핑 (`beebot_mesh/meshes`, `default/meshes`도 같음)

| STL | 링크 | 설명 |
| --- | --- | --- |
| `base_link.stl` | base_link | 와플 판, 바구니, 기둥 등 베이스 전체를 한 덩어리로 합친 메시. 55만 삼각형을 3 mm 격자로 간략화한 것 (약 6 MB) |
| `wheel_left_link.stl`, `wheel_right_link.stl` | wheel_left/right_link | 바퀴 |
| `camera_rear_link.stl` | front_pole_cam_link | 폴 앞쪽 카메라 |
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
| front_pole_cam_link | 72.0 |
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

아래 `beebot.urdf`·`urdf.xacro`는 모두 `beebot_mesh/` 안의 파일이다.

| 스크립트 | 용도 | 입력 → 출력 |
| --- | --- | --- |
| `make_urdf.sh` | `urdf.xacro`를 펼쳐 `beebot.urdf`를 만들고 `check_urdf`로 확인한다 | xacro → `beebot.urdf` |
| `check_urdf.sh` | 검수 묶음. `check_urdf`, `verify.py`, `check_xacro.py`를 차례로 돌리고 하나라도 실패하면 종료 코드 1을 낸다 (ROS 환경을 알아서 불러옴) | `beebot.urdf`, `urdf.xacro` → 콘솔 |
| `make_xacro.py` | `beebot.urdf`를 xacro로 바꾼다. 메시 경로와 ros2_control 하드웨어 값을 xacro 인자로 되돌린다 | `beebot.urdf` → `urdf.xacro` |
| `verify.py` | URDF를 검증한다 (트리, 관성 유효성, visual·collision 메시 전부 존재, 팔 조인트 위치. Fusion 추출본 `raw/`가 있을 때만 메시 왕복 위치도 보는데, 이 레포에는 `raw/`가 없어 건너뛴다) | `beebot.urdf` → 콘솔 |
| `check_xacro.py` | `urdf.xacro`를 펼쳐 `beebot.urdf`와 비교한다 (이름, 그리퍼 방향과 mimic, 영점 형상 일치, end_effector·optical frame, ros2_control 조인트, xacro 인자 반영). 어긋나면 `make_urdf.sh`로 다시 만든다 | `urdf.xacro`, `beebot.urdf` → 콘솔 (ROS 환경 필요) |
| `deploy_default.sh` | `urdf.xacro`·`meshes/`를 `default/`에 덮어쓴다. `default`의 Gazebo 블록과 `default/meshes/`에만 있는 파일은 남긴다. 덮어쓰기 전에 `check_urdf`로 확인한다 | `urdf.xacro`, `meshes/` → `default/` |
| `stlio.py` | 바이너리 STL 읽기·쓰기 도우미 (`verify.py`가 씀) | 라이브러리 |
| `torque_check.py` | 팔 joint2~5의 정적 중력 토크를 계산하고 XM430-W350 정지 토크와 비교한다 | `beebot.urdf` → 콘솔 |
| `bracket_check.py` | 롤 브라켓의 강도·강성을 손계산한다 (FEA 아님) | `beebot.urdf` → 콘솔 |

`torque_check.py`와 `bracket_check.py`는 판정 없는 분석 리포트라 `check_urdf.sh`에 넣지 않았다. 설계(질량, 관절 범위, 브라켓)가 바뀌었을 때 따로 돌린다.

### Fusion에서 처음부터 다시 만들 때

설계(형상·카메라 위치·브라켓)를 바꿔 Fusion에서 다시 뽑는 일은 Windows(Fusion이 있는 PC)에서 한다. 그 생성 스크립트(`fusion_extract.py`, `merge_raw.py`, `build.py`, `to_beebot.py`, `rm_x52.urdf`)와 절차는 이 레포에 두지 않고 Drive `KANT/BEE/model/v00/urdf/beebot/scripts/`에서만 관리한다. 결과로 나온 `beebot.urdf`와 바뀐 메시를 `beebot_mesh/`에 넣은 뒤 `python3 beebot_mesh/scripts/make_xacro.py` → `beebot_mesh/scripts/check_urdf.sh`를 돌린다.

### make_xacro.py가 만드는 xacro

`beebot_mesh/beebot.urdf`에서 역변환한 단순본이라 매크로가 없다 (ros2_control 관절 블록도 관절마다 펼쳐져 있음). 첫 줄 근처의 `<!-- generated by make_xacro.py -->` 주석이 생성 표시이며, 이 주석이 없는 xacro(손으로 고친 파일)가 그 자리에 있으면 덮어쓰지 않고 멈춘다. xacro를 손으로 고쳐서 계속 쓰려면 이 주석을 지우고 `make_urdf.sh` 방향으로만 변환한다.

## Isaac Sim에서 쓰기

`beebot_mesh/beebot.urdf`(`make_urdf.sh`로 만든 것)를 Isaac Sim URDF Importer로 가져오면 USD가 만들어진다. 질량과 관성은 URDF 값이 USD로 한 번 변환되고, 시뮬은 USD만 읽는다. URDF를 고쳤다면 다시 가져와야 반영된다.
