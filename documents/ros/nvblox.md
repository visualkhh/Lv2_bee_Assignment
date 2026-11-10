# nvblox — 지도(3D → 2D 점유 격자) 레퍼런스

> 설치·실행 기록은 [`CUVSLAM.md`](../jetson-orin-nano/CUVSLAM.md), 위치(cuVSLAM)는 [`cuvslam.md`](cuvslam.md), 부하 분석은 `temp/report.html`(저장소에 안 올림).
> 이 문서는 **nvblox 노드 자체**를 다룬다 — 무엇을 받고 무엇을 내는지, 우리가 어떤 값으로 쓰는지, 얼마나 먹는지, 알려진 문제.
> 표기: ✅ 실기에서 확인 · 📄 소스·설정만 읽음(실행 중 노드에서 직접 확인 안 함) · ❓ 미확인.

## 1. 한눈에

```text
pole_front_cam 정렬 depth ─▶ pole_depth_gate    (카메라 0.5 m 안 = 자기 몸 → 0) ─┐
gripper_cam    정렬 depth ─▶ gripper_depth_gate (스캔 자세일 때만 · 0.5 m 안 → 0) ─┴▶ nvblox_node ─┬▶ ~/mesh                 (3D 메시, 색 없음)
TF  map→카메라 (cuVSLAM의 map→odom + URDF)  ─────────────────────────────────────────▶            ├▶ ~/static_esdf_pointcloud
                                                                                                     ├▶ ~/combined_esdf_pointcloud ─▶ rl_nav_controller
                                                                                                     └▶ ~/static_occupancy_grid ─▶ nvblox_map ─▶ /map (latched)
```

| 항목 | 값 |
|---|---|
| 노드 | `/nvblox_node` (Isaac ROS nvblox **5.0.0**, 소스 `~/isaac_ros_ws/src/isaac_ros_nvblox` @ `79654d2`) ✅ |
| 우리 설정 | [`ros_system/config/nvblox.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/nvblox.yaml) (적힌 것만 덮어씀, 나머지는 nvblox 기본값) |
| 띄우는 곳 | [`ros_system.launch.py`](../../lv2_module6/ros2_ws/src/ros_system/launch/ros_system.launch.py) `slam_nodes()` — `use_slam`일 때만 (지도 모드 NONE이면 안 뜸) |
| 지도 프레임 | `map` (원점 = 출발 자리의 base_footprint) |
| 카메라 | 2대 (`num_cameras: 2`) — camera_0 = pole_front_cam(늘), camera_1 = gripper_cam(스캔 자세일 때만; **뽑혀 있으면 노드째 안 뜸**) |
| 복셀 | 0.05 m (Nav2 costmap 해상도와 같게) |
| 모드 | `mapping_type: dynamic` · `esdf_mode: 2d` · 색 안 씀(`use_color: false`) |

## 2. 토픽 (nvblox_node가 내는 것)

> 출처 📄: `isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp`의 `"~/…"` 정의 + 우리 코드의 사용처.
> 이 표를 **실행 중인 노드에서** 확인한 적은 아직 없다 (새 프로세스가 스택을 못 보는 현상 때문에 `ros2 topic list`가 비었음 — [`DEBUG.md`](DEBUG.md) 참고). 확인되면 ✅로 바꿀 것.

### 지도로 쓰는 것
| 토픽 | 타입 | 내용 | 누가 쓰나 |
|---|---|---|---|
| `/nvblox_node/static_occupancy_grid` | `nav_msgs/OccupancyGrid` | 2D 점유 격자(정적) | **`nvblox_map`** → `/map`으로 다시 냄 |
| `/nvblox_node/combined_occupancy_grid` | OccupancyGrid | 정적 + 동적 | (안 씀) |
| `/nvblox_node/dynamic_occupancy_grid` | OccupancyGrid | 움직이는 것만 | (안 씀) |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | **3D 메시 전체** (색 없으면 회색) | RViz nvblox 플러그인만 |

### ESDF (거리장)
| 토픽 | 타입 | 누가 쓰나 |
|---|---|---|
| `/nvblox_node/static_esdf_pointcloud` | PointCloud2 | 보기용 (지도 전체) |
| `/nvblox_node/combined_esdf_pointcloud` | PointCloud2 | **`rl_nav_controller`** (`nav2_params.yaml` `points_topic`) — 정적+동적, 높이 0.03~0.30 m 슬라이스 |
| `/nvblox_node/dynamic_esdf_pointcloud` · `pessimistic_static_esdf_pointcloud` | PointCloud2 | (안 씀) |
| `/nvblox_node/{static,dynamic,combined,pessimistic_static}_map_slice` | `nvblox_msgs/DistanceMapSlice` | (안 씀) |

### 그 밖에 (진단·디버그)
`~/esdf_slice_bounds` · `~/workspace_bounds` · `~/dynamic_points` · `~/groundplane_estimator_*` · `~/save_rates` · `~/save_timings` · `~/*_frame_overlay`

### 서비스
| 서비스 | 타입 | 쓰는 곳 |
|---|---|---|
| `/nvblox_node/save_map` | `nvblox_msgs/FilePath` | CLI/TUI 지도 저장 (`bus._save_nvblox_map`) ✅ |
| `/nvblox_node/load_map` | `nvblox_msgs/FilePath` | `nvblox_map`이 `map:=saved`로 시작할 때 ✅ |
| `~/save_ply` · `~/get_esdf_and_gradient` | — | (안 씀) |

### 입력
`camera_0/depth/image`, `camera_0/depth/camera_info`, `camera_0/color/image`, `camera_0/color/camera_info`, 그리고 `camera_1/…` — 모두 **게이트를 거친 depth**(`/…/aligned_depth_to_color/image_nvblox`)로 리맵 ✅ (`ros_system.launch.py` `camera_remaps`). QoS는 `SENSOR_DATA`(`input_qos`).

## 3. 지도 파일 (저장·불러오기)

지도 한 벌 = 폴더 하나 `<map_dir>/<이름>/` (`map_dir` 실기 = `lv2_module6/ros2_ws/map`).

| 파일 | 만든 쪽 | 용도 | 크기(예: `m100`) |
|---|---|---|---|
| `nvblox.nvblx` | nvblox `~/save_map` | **위치추정에 쓰는 정본** (3D). 지도 목록은 이 파일이 있는 폴더만 인식 | 12.5 MB ✅ |
| `map.pgm` + `map.yaml` | `map_saver` (`/map` 저장) | 사람이 보는 미리보기, TUI 지도 화면 | 28 KB ✅ |
| `nvblox.nvblx.png` + `.yaml` | nvblox가 저장할 때 같이 씀 | `map.pgm`과 같은 2D 격자(해상도 0.05, 원점 같음) — **사실상 중복**, 지워도 다시 생김 | 작음 ✅ |

- **매핑(`map:=slam`)**: 새 폴더를 만든다(이미 있으면 덮어쓰지 않고 중단). 저장은 TUI 지도 저장(4→6) 또는 끌 때. TUI 저장은 `map_saver` → `nvblox save_map` 순서로 둘 다 한다.
- **위치추정(`map:=saved map_name:=…`)**: `nvblox_map`이 시작할 때 `load_map`으로 불러온다. 폴더에 `nvblox.nvblx`가 없으면 NONE(지도 없음)으로 시작한다 ✅.
- **좌표 = 저장할 때의 출발 자리.** 위치추정 때 cuVSLAM은 **원점부터 새로** 시작하므로, 출발 자리·방향이 같아야 지도와 맞는다 (안 맞으면 initialpose로 보정 — [`cuvslam.md`](cuvslam.md) 5절).
- 지도 이름을 입력하면 `<map_dir>/<이름>/`에 저장된다 (경로 문자를 넣으면 그 경로). 이름만 넣었을 때 폴더가 없으면 만든다 — 예전엔 폴더가 없어 `Unable to open file`로 실패했음.

## 4. 설정 (`nvblox.yaml`)

| 파라미터 | 값 | 뜻 · 비고 |
|---|---|---|
| `global_frame` | `map` | cuVSLAM이 `map→odom`을 냄 |
| `voxel_size` | 0.05 | Nav2 해상도와 같게 |
| `mapping_type` / `esdf_mode` | `dynamic` / `2d` | 동적 물체 구분 + 2D ESDF |
| `use_color` | `false` | 색은 메시 보기용 — GPU·메모리 절약. **그래서 RViz 메시가 회색** |
| `integrate_depth_rate_hz` | 15 | 카메라 15 fps에 맞춤 |
| `update_mesh_rate_hz` / `update_esdf_rate_hz` / `publish_layer_rate_hz` | 2 / 5 / 2 | |
| `map_clearing_radius_m` | -1 | 멀어져도 지우지 않음 (경기장 약 4.4×6.3 m 통째 기억) |
| `static_mapper.esdf_slice_min/max_height` | **0.08** / 0.3 | 2D로 자르는 높이. **0.03 → 0.08로 올림(2026-11-10)** — depth 오차로 바닥이 3 cm 위로 찍혀 앞이 트여도 검은 부채꼴(가짜 장애물)이 생겼음. 8 cm 미만 낮은 물체는 안 남음 |
| `static_mapper.projective_integrator_max_integration_distance_m` | **4.5** | **3.0 → 4.5로 올림(2026-11-10)** — 더 멀리까지 지도에 넣음(nvblox 기본 7.0). 멀수록 depth 잡음이 커짐 |
| `workspace_bounds_*_height_m` | -0.1 / 0.8 | |

> ❓ **시야 4.5 m · 슬라이스 0.08은 아직 실기에서 효과를 확인하지 못했다.** 바닥 잡음이 오히려 늘면 4.0 → 3.0으로 되돌릴 것. 값을 바꾸면 nvblox를 다시 띄워야 반영된다 (지도 모드 전환·재시작).

## 5. 입력 depth를 거르는 곳 (`scan_cam_gate`)

nvblox는 점구름이 아니라 **depth 영상**을 받으므로, 자기 몸을 영상에서 뺀다.

- `pole_depth_gate`: 카메라에서 0.5 m 안 픽셀 → 0 (자기 팔·폴대). 늘 켜짐.
- `gripper_depth_gate`: 팔이 **스캔 자세일 때만** 통과, 아니면 전부 0 (팔이 움직이며 본 것은 지도에 안 넣음). 그립퍼 캠이 안 꽂혀 있으면 이 노드도 안 뜸 (`use_gripper_cam:=auto`).
- `skill_pose`의 자동 스캔 자세는 **기본 꺼짐**(`scan_pose_enabled=false`) — 팔을 돌리면 무게중심이 쏠려 매핑 중엔 stow를 유지.
- 0.5 m 안의 실제 장애물은 nvblox에 안 들어간다. 대신 Nav2 local costmap(몸 뺀 점구름 `body_filter`)이 피한다.

## 6. 자원 사용 (Jetson Orin Nano, 6코어 · RAM 7.4 GiB · GPU 공유 메모리)

| 항목 | 값 | 근거 |
|---|---|---|
| GPU 메모리 | **약 808 MB** (cuVSLAM 211 · rl_nav_controller 83 · 데스크톱 약 120 — 합계 약 1.2 GB) ✅ | `nvmap` 2026-11-10 |
| CPU | 약 33~45% (코어 1개 = 100%) ✅ | `top` 스냅샷·녹화 |
| RAM(PSS) | 약 218 MB (프로세스 RSS는 약 1 GB, 가상 12 GB) ✅ | `smaps_rollup` |
| GPU 사용률 | 시스템 전체 평균 25%, **순간 최대 94%** (nvblox + cuVSLAM 합) ✅ | tegrastats(매핑 녹화) |

- **메모리가 가장 큰 제약이다.** SLAM 매핑 중 남은 free 메모리가 약 100 MB(최소 70 MB), 스왑 사용, 연속 빈 메모리 4 MB까지 줄었다. 시야 4.5 m로 키우면 nvblox 메모리가 늘 수 있다 ❓.
- 시스템 전체 분석: `temp/report.html` (저장소에는 없음 — `temp/`는 `.gitignore`).

## 7. 보는 법

| 보고 싶은 것 | 방법 |
|---|---|
| 2D 점유 지도 | TUI 4) 지도/위치 → 7) 지도 보기, 또는 RViz `/map` (`nav.rviz`·`mapping.rviz`가 이미 그림) |
| **3D 메시 전체** | RViz에서 nvblox 플러그인(`nvblox_rviz_plugin`, bee에 설치됨)으로 `/nvblox_node/mesh`. 색이 꺼져 있어 회색 |
| ESDF 점구름 | RViz PointCloud2 → `static_esdf_pointcloud` (전체) / `combined_esdf_pointcloud` (로컬) |

- **nvblox 전용 `nvblox.rviz`** (TUI 4) 지도/위치 → `d`, 또는 `RVIZ_CONFIG=nvblox ./run-rviz`): `/map` 원본 · **3D 메시**(`/nvblox_node/mesh`, 플러그인 `nvblox_rviz_plugin/NvbloxMesh`) · ESDF 점구름 2개(static은 꺼짐 / combined는 켜짐) · Global·Local 코스트맵(꺼짐, 체크로 켬). 시점은 3D(Orbit). 메시 QoS는 NVIDIA 예제와 같게 Reliable. ❓ **실기에서 열어 본 적은 없다** — 설정 파일은 YAML 파싱과 기존 `.rviz`와의 속성 이름 대조까지만 했고, `NvbloxMesh`·`Odometry` 표시는 비교할 기존 파일이 없어 로드 여부 미확인.
- 기존 `nav.rviz`·`mapping.rviz`는 **메시를 안 그린다.**
- 주의: RViz는 Jetson CPU를 쓰고(코어 약 20%) 메모리 여유가 없어서, 이미 떠 있는 RViz에 표시만 더하는 것이 좋다.

## 7-1. Nav2가 실제로 보는 지도 (`/map`과 다르다)

`/map`은 nvblox 격자를 그대로 다시 낸 **원본**이다. Nav2는 이걸 그대로 쓰지 않고 **코스트맵**으로 가공해서 쓴다. 📄 `nav2_params.yaml`을 읽은 것 (실행 중 확인 ❓).

| 누가 | 지도 | 토픽 | 구성 |
|---|---|---|---|
| **전역 플래너** (`NavfnPlanner`, A\*, `allow_unknown: true`, `tolerance: 0.20`) | global costmap | **`/global_costmap/costmap`** | `static_layer`(= `/map`) + `inflation_layer`(0.18 m). 1 Hz · `map` 프레임 · `track_unknown_space: true` |
| **컨트롤러** (RL/DWB) | local costmap | **`/local_costmap/costmap`** | `voxel_layer`(`points_filtered` — 몸 뺀 깊이 점구름) + `inflation_layer`. 3×3 m가 로봇을 따라감 · 5 Hz · `odom` 프레임 |
| RL 컨트롤러 입력 | ESDF 점 구름 | `/nvblox_node/combined_esdf_pointcloud` | nvblox 2D 거리장(높이 0.03~0.30 m) |

- **`/map`은 동적이다.** `nvblox_map`이 nvblox `static_occupancy_grid`를 **새 격자가 올 때마다** 1 Hz로 다시 낸다(latched). 매핑 중엔 쌓이는 대로 바뀐다.
- ⚠ **위치추정(LOCALIZATION)에서도 nvblox는 계속 depth를 넣는다.** 📄 nvblox 소스에 매핑을 멈추는 파라미터(freeze·localization-only 등)가 **없고**, `load_map`은 저장 지도를 불러올 뿐 이후 입력을 막지 않는다. 우리 launch는 위치추정에서 **저장만 안 한다**(`save_path=''`, "불러온 지도를 안 덮어씀") — 즉 **메모리 안의 지도는 계속 바뀌고 디스크에만 안 쓴다.** ❓ 실제로 `/map`이 위치추정 중 바뀌는 것은 아직 직접 측정하지 않았다.
- **그래서 "저장 지도"는 고정된 정답 지도가 아니라 위치추정을 시작할 때 nvblox에 한 번 불러오는 *초기값*이다.**

```text
저장 지도(nvblx) ─(시작 시 1회 load_map)─▶ nvblox 메모리 ◀─ 새 depth를 계속 쌓음 (멈추는 설정 없음)
                                                │ static_occupancy_grid
                                       nvblox_map ──▶ /map (갱신될 때마다)
                                                │ global costmap의 static_layer가 /map을 다시 그림
                                  /global_costmap/costmap ──▶ 플래너(NavfnPlanner)가 이걸로 길을 찾음
```

- **`combined_esdf_pointcloud`는 플래너가 아니라 컨트롤러(`rl_nav_controller`)의 입력이다.** 플래너는 전역 코스트맵만 본다.
- **Nav2의 일반 구성(`map_server`가 파일을 한 번 읽어 고정)과 다르다.** 플래너가 보는 "정적 지도"가 사실은 정적이 아니다. 일어날 수 있는 일(추정, 재현 ❓): 바닥 잡음 같은 **가짜 장애물이 `/map`에 굳음** · 출발 자리가 저장 때와 다르면 cuVSLAM이 원점부터 새로 시작해 nvblox가 **어긋난 위치에 새 벽을 덧그림** · 움직이는 물체(`mapping_type: dynamic`이 일부만 거름) · 같은 목표인데 지도가 달라져 경로가 달라짐.
- ⚠ **전역 코스트맵 플러그인은 `["static_layer", "inflation_layer"]` 뿐이다.** `obstacle_layer` 설정은 파일에 있지만 `plugins` 목록에 없어서 **꺼져 있다.** → 전역 계획은 `/map`에 팽창만 더한 **정적 지도**로 세우고, 새로 나타난 장애물은 로컬 코스트맵(컨트롤러)만 피한다. 의도된 설계인지는 ❓.
- 팽창 0.18 m는 로봇 반경 0.16 m보다 2 cm 클 뿐이라 좁은 곳도 길이 열린다 (`robot_radius: 0.16`).
- 값이 의미하는 것: 코스트맵 0~100, 99 = 내접, 100 = 치명, -1 = 모름 (`tools/cli/op/views/map/grid.py`).

### 플래너가 보는 지도를 보는 법
| 방법 | 보이는 것 | 상태 |
|---|---|---|
| TUI 4) 지도/위치 → 7) 지도 보기 → **`c`** | 코스트맵 겹침 **GLOBAL → LOCAL → OFF**. 주황 = 팽창, 빨강 = 내접·치명 | 📄 코드만 확인 |
| TUI 4) → **`c) 목표 이동 전용 RViz`** (`nav.rviz`) | `/map` + Global Costmap + Local Costmap + `/plan` + footprint | 📄 설정 문법까지만 확인 |
| 한 번 확인 | `ros2 topic echo /global_costmap/costmap --once --field info` | ❓ |

## 8. 알려진 문제 · 이력

| 날짜 | 증상 | 원인 | 상태 |
|---|---|---|---|
| 11-09 | 지도 저장이 `tools/`에 떨어지고 위치추정이 `NONE` | 이름이 경로로 처리됨 · 저장 뒤 전환이 작업 중이던 폴더(nvblx 없음)로 감 | ✅ 고침 (`bus._map_save`, `localize_saved`) |
| 11-09 | 이름 저장 시 `map_saver가 저장하지 못했습니다` | 새 이름 폴더가 없음 | ✅ 고침 (폴더 생성) |
| 11-10 | 앞이 트인 곳이 검은 부채꼴로 나옴 | depth 바닥 잡음이 슬라이스 3 cm를 넘음 (추정) | ❓ 슬라이스 0.08로 대응, 효과 미검증 |
| 11-10 | 그립퍼 캠이 있으면 CPU·USB 부담 | 미연결 시 respawn 루프 / 연결 시 depth·게이트·인지 | ✅ `use_gripper_cam:=auto` |
| 11-10 | 매핑 중 SLAM 부하로 주행 끊김 | CPU·메모리 포화 (nvblox 자체는 약 40% 코어, GPU 808 MB) | 부분 대응 — `temp/report.html` |

## 9. 열린 질문 · TODO (이 문서에 계속 채움)

- [x] nvblox 전체 맵(3D 메시)을 보는 방법 — `nvblox.rviz` 만들어 TUI 4) `d`에 넣음 (2026-11-10). [ ] 실기에서 열어 메시가 실제로 나오는지 확인
- [ ] **전역 코스트맵 `obstacle_layer`가 꺼져 있는 게 의도인지 확인** (새 장애물이 전역 계획에 안 들어감)
- [ ] **위치추정 중 `/map`이 실제로 바뀌는지 측정** (몇 분간 격자 수신 횟수·점유 칸 수 비교) — 소스상 바뀔 가능성이 높음
- [ ] **(결정 필요) 위치추정에서 `/map` 고정**: `nvblox_map`이 처음 불러온 격자만 한 번 `/map`에 내고 이후 새 격자는 무시 (Nav2 표준과 같음). 새 장애물은 로컬 코스트맵 담당. 대안인 nvblox depth 입력 차단은 RL 컨트롤러 입력(`combined_esdf_pointcloud`)도 멈춰 비추천
- [ ] 실행 중 노드에서 토픽 목록·Hz를 직접 확인해 2절 표를 ✅로 (`ros2 topic hz` 등)
- [ ] 시야 4.5 m · 슬라이스 0.08 효과 확인 (앞이 트인 곳의 검은 부채꼴이 사라지는지, GPU·메모리 증가폭)
- [ ] `use_color: false`인 채 메시를 보기 좋게 할 방법 (색 켜면 GPU·메모리 증가 — 메모리 여유 없음)
- [ ] 지도 크기·복셀 수와 메모리의 관계 (큰 지도에서 nvblox가 얼마나 느는지)
- [ ] 실행 중 지도 전환 시 nvblox가 launch째 다시 뜨는 시간·비용

## 10. 관련 코드

- 설정: [`ros_system/config/nvblox.yaml`](../../lv2_module6/ros2_ws/src/ros_system/config/nvblox.yaml)
- 노드 구성: [`ros_system.launch.py`](../../lv2_module6/ros2_ws/src/ros_system/launch/ros_system.launch.py) `slam_nodes()` · [`nvblox_map.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/nvblox_map.py) (격자 → `/map`, 저장 지도 불러오기) · [`scan_cam_gate.py`](../../lv2_module6/ros2_ws/src/ros_system/scripts/scan_cam_gate.py)
- 지도 저장 흐름(TUI): [`tools/cli/op/bus.py`](../../lv2_module6/tools/cli/op/bus.py) `_map_save` · `_save_nvblox_map` · `localize_saved`
- 테스트베드 대역: [`faker/src/fake_nvblox.cpp`](../../lv2_module6/ros2_ws/src/faker/src/fake_nvblox.cpp)
