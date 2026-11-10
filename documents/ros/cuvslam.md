# cuVSLAM — 위치 (`map→odom`) 레퍼런스

> 설치·빌드·실기 시험 기록은 [`CUVSLAM.md`](../jetson-orin-nano/CUVSLAM.md), 지도는 [`nvblox.md`](nvblox.md).
> 이 문서는 **`cuvslam_slam` 노드 자체** — 입출력, TF, 설정, initialpose, 장애 이력.
> 표기: ✅ 실기에서 확인 · 📄 코드만 읽음 · ❓ 미확인.

## 1. 한눈에

```text
pole_front_cam 컬러 + 정렬 depth + camera_info ─▶ cuvslam_slam ─┬▶ TF  map→odom           (~10 Hz, 0.5 s 미래 시각)  ─▶ Nav2 · nvblox · state_manager
                                                    (RGBD 모드,   ├▶ /cuvslam/odom          (속도만)                    ─▶ EKF(odom1) ─▶ odom→base_footprint
                                                     GPU)         └▶ /cuvslam/path          (최근 3000개)
/initialpose ─────────────────▶ (map 프레임 보정, 추적은 계속)
TF odom→base_footprint (EKF) ─▶ (map→odom 계산에 씀)
```

| 항목 | 값 |
|---|---|
| 노드 | `cuvslam_slam` (파이썬, [`scripts/cuvslam_slam.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/cuvslam_slam.py)) · PyCuVSLAM v17.0.1 ✅ |
| 모드 | **RGBD** (D435는 IMU가 없어 VIO 불가) · 루프 클로저 + 평면 제약(`planar_constraints`) |
| 입력 | `/pole_front_cam/color/image_raw` · `/pole_front_cam/aligned_depth_to_color/image_raw` · `…/camera_info` (`ApproximateTimeSynchronizer` 슬롭 0.02 s) |
| 지도 원점 | **출발 자리의 base_footprint**, x·y·yaw만 사용(경기장 바닥은 평평) |
| 띄우는 곳 | `ros_system.launch.py` `slam_nodes()` — `use_slam`일 때만 (지도 모드 NONE이면 안 뜸) |
| 카메라 마스크 | 카메라에서 `min_depth`(0.5 m) 안 픽셀은 추적에서 뺌 (자기 팔이 로봇과 같이 움직여 회전을 덜 잡게) |
| 자원 | CPU 코어 약 60~65%(파이썬 노드 하나는 코어 1개를 못 넘음) · GPU 메모리 약 211 MB · RSS 약 640 MB(PSS 331 MB) ✅ (정밀 노드별 측정은 미완) |

## 2. 출력 (위쪽이 받는 계약)

| 출력 | 내용 | 비고 |
|---|---|---|
| **TF `map→odom`** | `map_from_base · odom_from_base⁻¹` (평면). 스탬프 = **영상 시각 + `map_tf_tolerance`(0.5 s)** | 이게 없으면 Nav2·nvblox가 전부 멈춤 (`Initial robot pose is not available` 등) ✅ |
| `/cuvslam/odom` | **속도(vx, vy, ωz)만** | 자세는 일부러 안 냄 — 추적을 잃으면 cuVSLAM 좌표가 새로 시작해서 자세를 EKF에 넣으면 튐 |
| `/cuvslam/path` | 최근 3000개 자세(약 3분 20초) | 3 Hz로 발행 |

EKF 연결 ([`ekf.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/ekf.yaml)): `odom1: /cuvslam/odom` — vx·vy만 사용(`odom1_config`), yaw는 IMU만. 분산은 `linear_variance`·`angular_variance`(0.0025). 바퀴 `/odom`(vx·vy)과 IMU(ωz)와 합쳐 `odom→base_footprint` TF와 `/odometry/filtered`를 **EKF 하나만** 냄.

## 3. `map→odom`을 만드는 순서 (그래서 약한 고리)

1. 영상이 오면 cuVSLAM이 추적 → 카메라 자세.
2. `publish_map`이 **그 영상 시각의 `odom→base_footprint`(EKF)** 를 TF 버퍼에서 찾는다.
3. 못 찾으면 **예외를 삼키고 이 프레임의 `map→odom`을 건너뛴다** (코드 `except: return`, 로그 없음) 📄.
4. 찾으면 `map→odom`을 계산해 **0.5 s 미래 스탬프**로 낸다.

→ EKF가 늦거나(CPU 포화) 영상이 밀리면 `map→odom` 간격이 벌어진다. 관찰: 평균 간격 0.07~0.10 s, **최대 0.33~0.47 s**, 목표 이동을 시작하는 순간에는 **1.1~1.6 s까지** 비었다 ✅ (그때 Nav2 `Failed to transform end pose`, error 102).

### 왜 0.5 s 미래로 내나 (`map_tf_tolerance`)
Nav2 컨트롤러가 경로 끝점을 `map→odom`으로 바꿀 때 **요청 시각이 최신 TF보다 미래면 실패**한다. 영상 지연과 발행 간격 때문에 최신 `map→odom`이 늘 약간 과거라서, 미래 시각으로 내서 여유를 둔다 (map→odom은 천천히 바뀌어 안전).

| 값 | 결과 |
|---|---|
| 0.2 s (예전) | 실제 여유는 영상 지연 때문에 평균 +0.07 s뿐 → 목표 이동이 error 102로 중단 |
| **0.5 s (현재, 2026-11-10)** | 평균 +0.38 s, 최소 +0.18 s ✅. 단 목표 시작 순간의 1 s 넘는 빈틈은 이것만으로는 못 막음 |

## 4. 설정 (`cuvslam_slam.py` 파라미터)

| 파라미터 | 기본 | 뜻 |
|---|---|---|
| `publish_map_tf` | true | false면 시각 오도메트리만 (`map→odom`을 다른 곳이 낼 때) |
| `map_tf_tolerance` | **0.5** [s] | 3절 |
| `min_depth` | 0.5 [m] | 이 안 픽셀(자기 팔) 추적에서 제외 |
| `depth_scale_factor` | 1000 | 16UC1 mm → m |
| `linear_variance` · `angular_variance` | 0.0025 | EKF에 주는 속도 분산 (σ 0.05 m/s · 0.05 rad/s에서 시작, 실측으로 맞출 값) |
| `base_frame` · `odom_frame` · `map_frame` | base_footprint · odom · map | |
| `initialpose_topic` · `odom_topic` | /initialpose · /cuvslam/odom | |

## 5. initialpose (위치 보정)

- RViz **2D Pose Estimate**나 TUI(4→8 위치 지정)가 `/initialpose`(프레임 `map`)를 보내면 `map` 프레임을 **보정**한다 (`map_offset`). cuVSLAM 자체는 리셋하지 않고 추적은 계속된다.
- 첫 추적 프레임 전에 받으면 **대기**했다가 첫 프레임 뒤에 적용 (`initialpose 대기 — 첫 추적 프레임 뒤 적용`).
- **메모리에만 저장된다 — 파일에 안 남는다.** 노드나 스택을 재시작하면 사라지고, 저장 지도로 위치추정(`map:=saved`)을 올릴 때마다 cuVSLAM이 **원점부터 새로** 시작하므로 **출발 자리·방향이 저장할 때와 달라지면 매번 다시 찍어야** 한다 ✅. (자동 적용·저장은 아직 없음 — TODO)
- 안 찍어도 `map→odom`은 나온다. 필요한 것은 "지도와 실제 위치를 맞추는 것"이지 TF 발행의 조건이 아니다.

## 6. 장애 이력 (2026-11-09~10) — 같은 일을 다시 겪지 않게

| 증상 | 원인 | 확인 | 조치 |
|---|---|---|---|
| **initialpose를 보내면 `map→odom`이 사라짐** (TF 없음, 목표 이동 거절) | `apply_initialpose`의 `get_logger().warn(...)` — `RcutilsLogger`에 `warn`이 없어 `AttributeError`가 콜백 밖으로 나가 노드가 죽음 (`main()`의 `spin_once` 트레이스). 프로세스는 남지만 TF는 0개 | ✅ 로그 트레이스 + TF 5초 0개 | `warning`으로 수정 (`4790699`). 수정 후 적용 로그 확인 ✅ |
| **추적이 4프레임에서 멈춤** (`추적 4 · 잃음 0`이 계속) → `map→odom` 없음 | 스택을 `run-cli restart`로 띄울 때 **카메라 serial을 안 줘서** 두 RealSense 노드가 같은 장치를 잡음 (`Device or resource busy`) → 영상이 안 옴 | ✅ 로그 `VIDIOC_S_FMT … busy`, 경고 `RealSense 2대인데 serial 미지정` | `POLE_FRONT_CAM_SERIAL`·`GRIPPER_CAM_SERIAL`을 주고 재시작. 정상 추적 시 `추적 N · 잃음 0`, 초당 약 15 |
| 목표 이동이 `Failed to transform end pose` (error 102) | 최신 `map→odom`이 요청 시각보다 과거 (0.2 s 미래 여유로는 부족, 목표 시작 순간 1 s+ 빈틈) | ✅ 로그 + TF 타이밍 | `map_tf_tolerance` 0.5 · 몸 필터 CPU 절감 (빈틈 자체는 ❓ 근본 미해결) |
| 시작 후 `odom→base_footprint`가 **약 500~600 m** | EKF 위치 적분 발산 (원인 모름). `map` 기준 위치는 정상 (map→odom이 상쇄) | ✅ tf2_echo 2회 | ❓ 원인 조사 필요 |
| 같은 노드를 `run-cli restart cuvslam_slam`으로 못 띄움 | 새 프로세스가 스택을 못 봄 → `/cmd/restart 구독자 없음` | ✅ | 스택 전체 재시작으로 우회 |

## 7. 운영 규칙 · 주의

- **출발 자리·방향을 저장 지도와 같게 한다.** cuVSLAM은 저장 지도를 불러오지 않는다(지도는 nvblox만 저장·불러옴). 위치추정 때마다 원점부터 새로 시작한다. cuVSLAM 자체 지도 저장·재위치추정은 아직 없다.
- 카메라 serial을 항상 지정한다 (`start.sh`는 환경변수 `POLE_FRONT_CAM_SERIAL`·`GRIPPER_CAM_SERIAL` 또는 launch 인자). state_manager의 launch 재시작은 `start.sh`를 거치지 않고 **기록된 인자 그대로** 다시 띄운다.
- 그립퍼 캠은 `use_gripper_cam:=auto`라서, 안 꽂혀 있으면 관련 노드가 안 뜬다. cuVSLAM은 pole_front_cam만 쓰므로 영향 없음.
- 추적을 잃으면(`잃음 N` 증가) cuVSLAM 좌표가 새로 시작해 `map→odom`이 튈 수 있다. 로그 `추적 잃음`.
- 상태 로그: 10초마다 `추적 N · 잃음 M · 경로 K` ✅ — 정상은 N이 초당 약 15씩 늘고 M은 0.

## 8. 확인하는 법

**cuVSLAM 전용 RViz `cuvslam.rviz`** (TUI 4) 지도/위치 → `e`, 또는 `RVIZ_CONFIG=cuvslam ./run-rviz`): 위에서 내려다보는 시점에 `cuVSLAM Path`(`/cuvslam/path`, 최근 3분) · **odom 3종 비교** — EKF 융합 `/odometry/filtered`(초록, Nav2가 쓰는 것, 켜짐) · 바퀴 `/odom`(주황, 꺼짐) · `/cuvslam/odom`(보라, 꺼짐; 속도만 내므로 자세 화살표는 의미 없음) · `/map` 배경. ❓ **실기에서 열어 본 적은 없다** (YAML 파싱·속성 이름 대조까지만). `/odometry/filtered`가 `map`에서 500 m 떨어져 보이면 6절의 EKF 발산이다.

```bash
# 정상 추적 (bee 로그, 10초마다)
sed 's/\x1b\[[0-9;]*m//g' /tmp/bringup.log | grep -a 'cuvslam_slam\]' | tail -3
# map→odom이 나오는지 (TF 한 번)
ros2 run tf2_ros tf2_echo map odom
# map→odom 간격·스탬프 지연 (스크립트는 대화 기록의 tfgap.py — 10초 동안 /tf를 받아 간격 평균·최대와 스탬프 앞섬을 계산)
```
- 새 프로세스가 스택을 못 보는 현상 때문에 **스택 시작 후 1분 안에** 재야 안정적이다 ([`DEBUG.md`](DEBUG.md)).

## 9. 열린 질문 · TODO (이 문서에 계속 채움)

- [ ] 목표 시작 순간 `map→odom`이 1 s 넘게 비는 **근본 원인** (CPU 포화 추정, 확정 못 함)
- [ ] EKF odom 500~600 m 발산 원인 (cuVSLAM 속도 스파이크? 바퀴 odom? 초기 전이?)
- [ ] initialpose 저장·자동 적용 (지도 폴더에 저장하고 위치추정 시작 때 적용)
- [ ] cuVSLAM 노드별 정밀 CPU·GPU·메모리 (측정기를 돌리다 bee가 멈춰 못 얻음)
- [ ] `publish_map`이 TF 못 찾아 건너뛰는 횟수를 로그·진단으로 노출 (지금은 조용히 건너뜀)
- [ ] 추적 잃음 복구 동작 정리

## 10. 관련 코드

- 노드: [`ros_system/scripts/cuvslam_slam.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/cuvslam_slam.py)
- EKF: [`ros_system/config/ekf.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/ekf.yaml) · Nav2 TF 허용: [`nav2_params.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/nav2_params.yaml) (`transform_tolerance`)
- 위치 지정 UI: [`tools/cli/op/pages/ab/core.py`](../../lv2_module6/tools/cli/op/pages/ab/core.py) `_publish_initialpose`
