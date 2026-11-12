[목차](README.md) · [◀ USD](usd.md)

# 실기 측정 (Real to Sim)

이 문서의 경로·명령은 `scripts/calib/` 기준이다.

실제 beebot을 측정해 시뮬레이션과 제어에 쓸 값을 구한다. 관절 응답(drive·지연), 중력 처짐(비틀림 강성), 차체 무게중심, 세우기 동작을 다룬다.
(`real_test` → `R2S` → 지금의 `scripts/calib/` 로 이름이 바뀌었다. 측정 결과와 해석은 `logs/20261024_worklog.md`. `logs/`는 git 제외라 로컬에만 있다.)

## 테스트 목록

한 번에 launch 하나만 띄운다 (OpenCR 포트는 한 프로세스만 열 수 있다). 자세한 내용은 아래 [테스트 방법](#테스트-방법)을 본다.

| 테스트 | 구하는 것 | 터미널 1 launch | 터미널 2 실행 |
|---|---|---|---|
| [관절 0°로](#1-set_center--0-로-이동) | 시작 자세 | `hw.launch.py` | `ros2 run sine_test set_center` |
| [사인 응답](#2-sin_test--사인-응답-기본-configsineyaml) | drive·지연 | `hw.launch.py` | `ros2 run sine_test sin_test joint3` |
| [주기별 응답](#3-sin_test--주기별-응답-configsine_pyaml) | 주파수 응답 | `hw.launch.py` | `ros2 run sine_test sin_test joint3 --ros-args -p config:=...` |
| [hold·step·ramp](#4-profile_test--hold--step--ramp-configprofileyaml) | 마찰·히스테리시스 | `hw.launch.py` | `ros2 run sine_test profile_test` |
| [정지 상태](#5-idle_record--정지-상태) | 노이즈·갱신 주기 | `hw.launch.py` | `python3 idle_record.py 60` |
| [차체 무게중심](#6-tip_balance--차체-무게중심) | `base_link` 무게중심 x | `hw.launch.py` | `python3 tip_balance.py` |
| [관절 옮기기](#7-joint_goto--관절-옮기기) | 측정 전 자세 잡기 | `arm_cam.launch.py` | `python3 joint_goto.py joint4=0` |
| [처짐 측정](#8-sag_measure--처짐-측정) | 줄자 높이 vs TF | `arm_cam.launch.py` | `python3 sag_measure.py` |
| [강성 계산](#9-sag_fit--sag_table--강성처짐-표-계산) | 비틀림 강성·처짐 표 | 없음 | `python3 sag_fit.py --bias --joints 2 3` |
| [처짐 동선 시뮬](#10-sag_sim_check--처짐-동선-시뮬-점검) | 충돌·속도 사전 점검 | 없음 | Isaac Lab python |
| [앞먹임 시험](#11-앞먹임-시험--강성-값-검증) | 강성 값 검증 | `arm_cam.launch.py` | `python3 sag_measure.py --sag off` (앞먹임 켬 · 끔 비교) |
| [퍽 회전 전류](#12-pivot_test--퍽-회전-전류) | 세우기용 그리퍼 전류 | `arm_cam.launch.py` | `python3 pivot_test.py --shape box` |
| [세우기 반복](#13-stand_loop--세우기-반복) | `stand` skill 실기 확인 | `stand_test.launch.py` | `python3 stand_loop.py` |

## 구성

```
set_center              ──/arm_controller/joint_trajectory (topic)──┐
sin_test / profile_test ──follow_joint_trajectory (action)──────────┴─> arm_controller (JTC, 50 Hz)
   ↑                                                                       │
   └─ /joint_states ── joint_state_broadcaster                             └─> control/BeebotSystemHardware ──USB──> OpenCR ──> 서보
```

| 파일 | 역할 |
|---|---|
| `ros2_ws/src/sine_test/launch/hw.launch.py` | 하드웨어·컨트롤러만 띄우고 유지. 테스트 노드는 다른 터미널에서 `ros2 run` |
| `ros2_ws/src/sine_test/config/` | `sine.yaml`(기본 사인), `sine_p{0.5,1,2,4}.yaml`(주기별), `profile.yaml`(hold/step/ramp), `grip.yaml`(쥐는 힘), `controllers.yaml` |
| `set_center` · `sin_test` · `profile_test` | 테스트 노드 (`sine_test` 패키지) |
| `idle_record.py` | 정지 상태 `/joint_states` 기록 (ROS 노드, 일반 python 으로 실행) |
| `tip_balance.py` | 기울기 시작 자세로 차체 무게중심 계산 |
| `arm_cam.launch.py` · `joint_goto.py` | 팔 + 카메라 bringup · 관절 옮기기 |
| `sag_measure.py` · `sag_fit.py` · `sag_table.py` · `sag_sim_check.py` · `arm_model.py` | 처짐 측정·강성 계산·표·시뮬 점검·기구학 |
| `pivot_test.py` · `stand_loop.py` | 세우기 시험 (퍽 회전 전류 · `stand` 반복) |
| `plot_logs.py` · `freq_response.py` · `profile_analysis.py` · `compare_runs.py` | 도표·분석 (ROS 불필요, `profile_analysis.py` 만 xacro 사용) |
| `logs/` | 결과. 사인 `logs/<ts>_<joint>/`, 주기별 `logs/freq/`, profile `logs/profile/`, 정지 `logs/<ts>_idle/` |

- 메인 ws 에서 쓰는 것: `control` 패키지의 `BeebotSystemHardware` 플러그인(소스만 읽어 이 ws 에 빌드), `urdf/beebot_mesh/urdf.xacro`
- `hw.launch.py`에서 쓰지 않는 것: `ml`(rl_arm_correction), `ros_system`/`bringup` launch, diff_drive·imu·gripper action 컨트롤러
- `sine_test/sag_calib_node.py`는 `ros2 run`에 등록돼 있지 않다 (`setup.py`). 지금 처짐 측정은 `sag_measure.py`로 한다.
- 명령 없는 인터페이스: 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (플러그인 동작)

## 빌드 (이 폴더의 ros2_ws 에서)

```bash
cd ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src ../../../../ros2_ws/src/control --packages-select control sine_test
```

build/·install/·log/ 는 이 ws 안에만 생긴다. 메인 ws 는 건드리지 않는다.

- `ros2 launch`/`ros2 run` 은 `src/` 가 아니라 `install/` 에 복사된 yaml 을 읽는다. yaml 을 고치면 다시 빌드하거나, 처음부터 `--symlink-install` 로 빌드해 둔다.
- 폴더 이름을 바꾸면(`real_test` → `R2S`) `install/` 의 setup 스크립트에 예전 절대 경로가 남는다. `build/ install/ log/` 를 지우고 다시 빌드한다.

---

## 테스트 방법

### 0. 공통 준비

메인 bringup 은 꺼 둔다 (OpenCR 포트는 한 프로세스만 열 수 있다). 1~6번은 `hw.launch.py`로 띄운다. 각 터미널에서 `cd ros2_ws && source install/setup.bash`. `python3 <스크립트>`는 `scripts/calib/`로 돌아와 실행한다.

```bash
ros2 launch sine_test hw.launch.py        # 터미널 1: 실기 (/dev/opencr). 끝날 때까지 켜 둔다
                                          #   인자: port, urdf, mesh_uri (기본 package://ros_system/urdf/beebot_mesh/meshes)
                                          #   mock_hardware:=true → 로봇 없이 (명령 = 측정, 오차 ≈ 0)
ros2 run sine_test set_center             # 터미널 2: 모든 테스트 전에 팔을 0° 로
```

- 테스트 노드 공통: **`q`** (또는 Ctrl+C) → 그 자리에서 정지하고 정상 종료. 키 입력을 받아야 하므로 launch 가 아니라 터미널에서 실행한다.
- 다른 설정 파일·출력 위치: `--ros-args -p config:=/path/my.yaml -p out_dir:=/path/logs` (기본 out_dir = 이 폴더의 `logs/`).
  패키지 안의 설정은 `config:=$(ros2 pkg prefix sine_test)/share/sine_test/config/<파일>.yaml`.
- yaml 의 `out_subdir` 는 out_dir 아래 하위 폴더(`freq`, `profile`), `tag` 는 폴더 이름 뒤에 붙는 꼬리표다.
- `arm_controller` 가 active 가 아니면 10 초 기다린 뒤 아무것도 움직이지 않고 종료한다.

hw.launch 시작 순서: 하드웨어를 inactive 로 띄워 OpenCR 상태를 먼저 읽고(2초), active 로 바꾼 뒤 컨트롤러를 띄운다.
`BeebotSystemHardware::on_activate` 가 첫 시리얼 읽기 전의 상태(0)를 명령으로 잡아 시작 순간 모든 관절에 0 을 보내는 문제를 피하기 위해서다
(가짜 OpenCR 로 확인: 바로 active 면 시작 1.4 초 동안 팔 전체 0 명령, inactive 경유면 0 명령 없음).

### 1. set_center — 0° 로 이동

```bash
ros2 run sine_test set_center             # joint5 → joint4 → … → joint1 순서로 한 관절씩 0°(서보 수직, 2048)
ros2 run sine_test set_center joint3      # joint3 만 (팔 joint1~5 만, 없는 이름이면 로그만 남기고 종료)
```

관절당 최소 `move_s`, 큰 이동은 URDF 속도의 0.3 배 이하로 천천히 옮긴다. 나머지 관절은 JTC 의 현재 명령을 유지하고 그리퍼는 개도를 유지한다.
관절마다 이동이 끝나면 남은 오차를 출력한다.

### 2. sin_test — 사인 응답 (기본, `config/sine.yaml`)

```bash
ros2 run sine_test sin_test joint1        # joint1 ~ joint5, gripper_left_joint 를 하나씩
```

결과: `logs/<YYYYmmdd_HHMMSS>_<joint>/`. 진폭은 Gain Tuner 기본값(joint1 ±54° / 2.5 s, joint2 ±22°, joint3 ±25.5°, joint4 ±42.1°, joint5 ±54°, 그리퍼 ±6.6 mm / 2 s)이다.
한 세션(같은 날 6 개 run)을 다 돌려 두면 아래 도표 2·5 로 정리·비교할 수 있다.

흐름:
1. 검사 (걸리면 움직이지 않고 종료): 스윙이 URDF 한계 안쪽 `limit_margin` 을 넘음 / 최고 속도가 URDF velocity 의 `speed_margin` 배를 넘음
2. 다른 팔 관절이 0° 에서 2° 넘게 떨어져 있으면 경고 (set_center 먼저)
3. 해당 관절을 사인 center 로 천천히 이동 → 사인 (fade-in → cycles → fade-out) → center 에서 `settle_s` 기록 후 **그 자리에 정지**하고 종료

### 3. sin_test — 주기별 응답 (`config/sine_p*.yaml`)

모든 주기에서 진폭을 ±10°(그리퍼 ±3 mm)로 같게 두어 주기만 바뀌게 했다. 결과는 `logs/freq/<ts>_<joint>_p<주기>/` 에 쌓이고 logs/ 요약 그림에는 섞이지 않는다.

```bash
C=$(ros2 pkg prefix sine_test)/share/sine_test/config
for p in 0.5 1 2 4; do ros2 run sine_test sin_test joint3 --ros-args -p config:=$C/sine_p$p.yaml; done
```

관절마다 4 주기를 모두 돌려야 도표 3 에서 지연 직선을 맞출 수 있다. 같은 주기를 다시 돌리면 최신 run 이 쓰인다.

### 4. profile_test — hold / step / ramp (`config/profile.yaml`)

```bash
ros2 run sine_test profile_test joint2 hold   # joint2·3·4, 그리퍼
ros2 run sine_test profile_test joint2 step   # joint1~5, 그리퍼
ros2 run sine_test profile_test joint2 ramp   # joint1~5, 그리퍼
```

결과: `logs/profile/<ts>_<joint>_<mode>[_<tag>]/` (형식은 sin_test 와 같음). 흐름(시작 위치로 천천히 이동 → 한 goal → center 에서 정지)과 `q` 정지도 sin_test 와 같다.

| 모드 | 명령 | 보는 것 |
|---|---|---|
| `hold` | 각도 목록마다 천천히 이동해 `dwell_s` 유지 | 자세별 처짐·유지 전류 → 중력 토크, 링크 질량 검증 |
| `step` | +size → 0 → −size → 0 으로 명령이 바로 바뀜 (속도 검사 안 함, 서보 최고 속도) | 최고 속도, 오버슈트, 정착 시간 → damping |
| `ramp` | 일정 속도(`speed` deg/s)로 ±amplitude 왕복 | 방향이 바뀔 때 멈춰 있는 폭(백래시)과 전류 변화(마찰) |

hold 는 마찰 때문에 **접근 방향**에 따라 결과가 달라진다. 중력만 분리하려면 각 각도에 양방향으로 도착하도록 목록을 짜서 평균을 낸다.

쥐는 힘 시험(30 mm 퍽, `config/grip.yaml`):

```bash
ros2 run sine_test profile_test gripper_left_joint hold --ros-args -p config:=$(ros2 pkg prefix sine_test)/share/sine_test/config/grip.yaml
```

2 mm(열림, 퍽 넣기) → 9 mm(쥐기) → 2 mm(빼기). 접촉점(약 7.8 mm)을 크게 넘겨 명령하지 않는다 (17 mm 에서 손가락이 휨).

### 5. idle_record — 정지 상태

```bash
python3 idle_record.py 60                 # hw.launch 만 켜 둔 채 (테스트 노드 없이) 60 초
```

결과: `logs/<ts>_idle/log.csv` (`t_recv`, `t_stamp`, 관절별 `_pos`·`_eff`). 엔코더·전류 노이즈, 상태 갱신 주기, 처짐을 본다.
펌웨어 통신을 바꾼 뒤 실제 상태 갱신 주기를 확인할 때도 쓴다 (sin_test 로그는 50 Hz 로 샘플링되어 20 ms 보다 빠른 갱신을 볼 수 없다).

### 6. tip_balance — 차체 무게중심

팔을 stow에서 0 자세까지 천천히 뻗다가, 뒤 캐스터가 뜨는 순간 Enter를 누른다. 그때 자세를 기록하고 팔은 바로 stow로 돌아온다. 기울기 시작할 때 전체 무게중심이 바퀴 축 위에 있다고 보고, URDF의 나머지 링크 질량으로 `base_link` 무게중심 x를 역산한다.

```bash
python3 tip_balance.py                    # hw.launch 켜 둔 채. 기본 3회, 빠른 관절 2 deg/s
python3 tip_balance.py --trials 5 --speed 1
python3 tip_balance.py --self-test        # ROS·로봇 없이 계산만
```

결과: `logs/tip_balance/<ts>/log.csv`. **팔은 Enter를 눌러야 돌아온다.** 로봇이 넘어질 수 있으니 손을 가까이 둔다. 팔 질량 오차는 이 값에 그대로 들어간다. `hw.launch.py` 전용이다 (`arm_cam.launch.py`의 `arm_controller`는 관절 수가 달라 목표가 거절된다).

### 7~12번 공통 준비 — `arm_cam.launch.py`

```bash
source ../../../ros2_ws/install/setup.bash                       # scripts/calib/에서. 메인 ws
ros2 launch ./arm_cam.launch.py                                  # 팔 + 카메라 (주행·인지 없음)
ros2 launch ./arm_cam.launch.py use_moveit:=true                 # + move_group
ros2 launch ./arm_cam.launch.py mock_hardware:=true use_cameras:=false   # 로봇 없이
```

`rl_arm_correction`이 함께 떠서 `gravity_enabled: true`(`ros_system/config/controllers.yaml`) 앞먹임이 켜져 있다. 이 값은 켤 때 한 번만 읽으므로 런타임에 못 바꾼다. **8·9번 강성 측정과 11번을 할 때는 `false`로 바꾸고 launch를 다시 띄운다.** 안 그러면 처짐이 이미 보정된 값을 재거나 앞먹임이 두 번 들어간다. 같은 측정에서는 `twist_state:=false`도 준다 (기본 `true`는 `/joint_states`에 모델 비틀림을 더해 엔코더 각이 아니다).

### 7. joint_goto — 관절 옮기기

```bash
python3 joint_goto.py                     # 지금 각도 출력
python3 joint_goto.py joint4=0            # joint4 → 0°, 나머지 유지 (최대 15 deg/s)
python3 joint_goto.py joint2=0 joint3=0
python3 joint_goto.py tip=0.19,0.07       # 손 수직 아래, 손가락 끝면 중심 앞 0.19 m · 바닥 위 0.07 m (URDF IK)
```

`arm_cam.launch.py` 전용이다 (`hw.launch.py`에서는 `goal rejected`).

### 8. sag_measure — 처짐 측정

자세마다 멈추면 손가락 끝면 높이를 줄자로 재서 입력한다. 값은 TF 높이와 함께 기록된다.

```bash
python3 sag_measure.py --dry                          # 계획만 출력
python3 sag_measure.py --sag off --rounds 2           # 자세 목록 순서대로
python3 sag_measure.py --sag off --grid --span 30     # 강성 계산용 격자, 앞뒤 두 점 측정 (점 간격 30 mm)
```

입력: 앞뒤 높이 `131.5 128` [mm]. `--span 0`이면 한 값. Enter만 누르면 측정을 건너뛰고, `q`는 끝낸다. 결과: `logs/sag_measure/<ts>/log.csv`.

### 9. sag_fit · sag_table — 강성·처짐 표 계산

로봇 없이 실행한다. `sag_table.py`는 시스템 python에 없는 scipy를 쓰므로 Isaac Lab python으로 실행한다.

```bash
python3 sag_fit.py --bias --joints 2 3                                  # 최신 logs/sag_measure 사용
python3 sag_fit.py logs/sag_measure/<ts>/log.csv --bias --joints 2 3 --exclude 3
~/pa-course/IsaacLab-6.1/.venv/bin/python sag_table.py                  # sag_table.csv 다시 씀
```

### 10. sag_sim_check — 처짐 동선 시뮬 점검

Isaac Lab에서 측정 동선의 충돌과 속도를 미리 본다. 접촉이나 속도 초과가 있으면 종료 코드 1이다.

```bash
~/pa-course/IsaacLab-6.1/.venv/bin/python sag_sim_check.py                 # 창 없이, 선반 자세
~/pa-course/IsaacLab-6.1/.venv/bin/python sag_sim_check.py --poses ros2_ws/src/sine_test/config/sag_poses_basket.yaml
~/pa-course/IsaacLab-6.1/.venv/bin/python sag_sim_check.py --viz kit       # 창으로 보기
```

### 11. 앞먹임 시험 — 강성 값 검증

중력 앞먹임은 `rl_arm_correction` 안에 들어 있다 (`gravity_enabled`, `controllers.yaml` 기본 켬 · 강성 `gravity_compliance_j2`·`j3`).
`arm_cam.launch.py`가 `ros_system`과 함께 `rl_arm_correction`을 올리므로 따로 스크립트를 돌리지 않는다
(예전 `gravity_comp.py`는 `rl_arm_correction`의 `~/delta`로 보냈는데, 지금 컨트롤러는 그 토픽을 받지 않아 지웠다 — 같이 돌리면 앞먹임이 두 번 들어감).

```bash
ros2 launch .../arm_cam.launch.py twist_state:=false            # 강성 측정 때와 같게 /joint_states = 엔코더 각
python3 sag_measure.py --sag off --rounds 2                      # 앞먹임 켬 (기본)
ros2 param set /rl_arm_correction gravity_enabled false          # 끄고 같은 자세로 다시 재서 비교
python3 sag_measure.py --sag off --rounds 2
```

강성 값이 맞으면 앞먹임을 켰을 때 처짐이 편향값(약 −4.9 mm)만 남는다. 강성 값을 바꿔 시험하려면
`ros_system/config/controllers.yaml`의 `gravity_compliance_j2`·`j3`(= 1/강성)를 고치고 다시 띄운다
(컨트롤러는 실행 중에 `gravity_enabled`·`twist_state`만 바꿀 수 있고 강성 값은 시작할 때 읽음).
하드웨어 `twist_state`(상태 쪽)의 강성은 `urdf.xacro`의 `twist_compliance_j2`·`j3` — 값을 확정하면 두 곳을 같이 고친다.

### 12. pivot_test — 퍽 회전 전류

한쪽 끝을 쥔 퍽이 손가락 사이에서 도는 그리퍼 전류를 찾는다. 팔은 움직이지 않으므로 먼저 손을 아래로 향하게 두고, 손가락 중심을 바닥에서 약 10 cm 위에 둔다.

```bash
python3 pivot_test.py --shape box --trials 5
python3 pivot_test.py --shape cyl --currents 0.00538,0.00807
```

입력: `d` 떨어짐, `h` 돌아서 매달림(좋음), `p` 일부 회전, `n` 안 돔, `s` 건너뜀, `x` 다음 전류, `q` 끝.

### 13. stand_loop — 세우기 반복

같은 자리에서 `stand` skill을 반복한다. 가짜 누운 퍽을 `/tracked_objects`로 내므로 인지 노드는 꺼야 한다.

```bash
source ../../../ros2_ws/install/setup.bash
ros2 launch bringup stand_test.launch.py                 # 터미널 1
pkill -f perception_node                                 # 터미널 2: launch가 뜬 뒤
python3 stand_loop.py --x 0.22 --y 0.0 --preview         # 먼저 손 위치만 확인
python3 stand_loop.py --x 0.22 --y 0.0 --rounds 0        # 0 = Ctrl+C까지
```

라운드 사이 `--pause`(기본 6초) 동안 표시한 자리에 퍽을 다시 눕혀 둔다.

### 움직이지 않는 관절의 자세 유지 기준
`allow_partial_joints_goal: false` 라 궤적마다 6 개 관절 값을 모두 보낸다. 움직이지 않는 관절에는 **측정값(`/joint_states`)이 아니라
JTC 가 지금 명령하고 있는 값**(`/arm_controller/controller_state` 의 `reference`)을 그대로 넣는다.

실기에서는 팔 무게 때문에 서보가 명령보다 조금 처진다 (예: 명령 0.0°, 측정 −1.0°). 측정값을 새 명령으로 보내면 목표가 −1.0° 가 되고
거기서 또 처져, 테스트를 반복할수록 자세가 조금씩 내려간다. 명령값을 다시 보내면 목표가 그대로라 지금 버티던 상태가 유지된다.
(mock 은 명령 = 측정이라 차이가 없다.) `q` 정지(set_center)도 이 명령값을 다시 보내 그 자리를 유지한다.

---

## 도표 만드는 방법

ROS 없이 이 폴더(`scripts/calib/`)에서 실행한다 (`profile_analysis.py` 만 URDF 중력 계산에 xacro 가 필요해 `source /opt/ros/lyrical/setup.bash`).
결과는 PNG 와 yaml 로 run 폴더나 `logs/` 에 쓴다.

| 순서 | 명령 | 입력 | 출력 |
|---|---|---|---|
| 1 | `python3 plot_logs.py logs/<run>...` | 사인 run (meta.yaml 있는 폴더) | run 마다 `plot.png`, `plot_lag_removed.png`, `analysis.yaml` + 요약 3 장 |
| 2 | `python3 compare_runs.py <날짜A> <날짜B>` | 두 세션의 `analysis.yaml` (1 먼저) | `logs/compare_<A>_<B>.png`, `.yaml` |
| 3 | `python3 freq_response.py` | `logs/freq/*/analysis.yaml` (1 먼저) | `logs/freq/freq_response.png`, `.yaml` |
| 4 | `python3 profile_analysis.py` | `logs/profile/` run | run 마다 `plot.png`, `analysis.yaml` + `profile_summary.yaml`, `ramp_hysteresis.png` |

### 1. 사인 run 하나하나 + 요약 — `plot_logs.py`

```bash
python3 plot_logs.py logs/20261026_*      # 한 세션만
python3 plot_logs.py                      # logs/ 아래 사인 run 전부 (idle 은 meta.yaml 이 없어 건너뜀)
```

- `plot.png`: 명령·측정 위치, 오차, 전류 (fade-in/out·settle 구간은 회색)
- `plot_lag_removed.png`: 명령을 맞춘 지연만큼 늦춰 비교 → 남는 것은 서보 자체 오차(gain, 중력 오프셋, 엔코더 계단)
- `analysis.yaml`: 전 진폭 구간에 사인을 맞춘 gain, lag(ms), 위상, 평균 오프셋, RMS·peak 오차 (보정 전 / 지연 보정 / 지연+오프셋 보정), 최대 전류
- 요약 (인자로 준 run 들의 **관절별 최신 run**, 첫 run 의 상위 폴더에 씀):
  `summary.png`(lag·gain·RMS), `summary_lag_removed.png`(오프셋·지연 보정 RMS·peak), `joints_lag_removed.png`(모든 관절 한 장)

> **주의:** 요약 3 장은 같은 이름으로 덮어쓴다. `logs/summary*.png`·`joints_lag_removed.png` 는 10-24 worklog 가 쓰는 그림이므로,
> 새 세션은 run 별 그림만 만들고 요약은 다른 이름으로 저장한다 (예: `logs/joints_lag_removed_20261026.png`):
>
> ```bash
> python3 -c "
> import sys, yaml, plot_logs as P; from pathlib import Path
> runs = sorted(Path('logs').glob('20261026_*'))
> items = []
> for run in runs:
>     m, a = P.load(run); r = P.analyse(m, a); P.plot_run(run, m, a, r); P.plot_run(run, m, a, r, compensate=True)
>     (run / 'analysis.yaml').write_text(yaml.safe_dump({k: float(v) if isinstance(v, float) else v for k, v in r.items()}, sort_keys=False))
>     items.append((m, a, r))
> P.plot_joints(items, Path('logs/joints_lag_removed_20261026.png'))"
> ```

### 2. 세션끼리 비교 — `compare_runs.py`

```bash
python3 compare_runs.py 20261024 20261026  # 두 세션 모두 plot_logs.py (또는 위 스니펫) 를 먼저 돌려 analysis.yaml 이 있어야 함
```

같은 `sine.yaml` 로 돌린 두 세션(run 폴더 날짜 접두어)을 관절별로 나란히 그린다. 세션마다 관절별 최신 run 을 쓴다.
패널: lag, 상태 갱신 주기(log.csv 에서 위치 값이 바뀌는 간격의 중앙값), RMS (보정 전), RMS (지연+오프셋 보정), peak (지연 보정), gain.
펌웨어·설정을 바꾼 전후 비교에 쓴다. 두 세션에 모두 있는 팔 관절만 그리고, 한쪽에만 있는 관절(예: 그리퍼)은 yaml 에만 남긴다.

### 3. 주기별 응답 — `freq_response.py`

```bash
python3 plot_logs.py logs/freq/2026*      # run 별 analysis.yaml (요약 그림은 logs/freq/ 에 생김)
python3 freq_response.py                  # → logs/freq/freq_response.png, freq_response.yaml
```

관절별 gain·lag vs 주기. lag 는 `lag(T) = 순수 지연 + 일정 위상/360 × T` 로 맞춘다.
순수 지연은 주기와 무관한 ms(통신·측정 경로), 일정 위상은 주기에 비례해 커지는 몫(마찰·백래시)이다.
`logs/` 의 큰 진폭 run 은 속이 빈 점으로 함께 그린다 (진폭 의존성 확인). 같은 주기가 여러 번이면 최신 run 을 쓴다.

### 4. hold / step / ramp — `profile_analysis.py`

```bash
source /opt/ros/lyrical/setup.bash        # xacro
python3 profile_analysis.py               # 관절·모드별 최신 완료 run
```

- `hold`: 자세마다 dwell 뒤 절반의 평균 처짐·전류, 접근 방향, 실측 토크(전류 × 1.78 Nm/A) vs URDF 중력 토크
- `ramp`: 반전 직후를 뺀 일정 속도 구간에서 +/− 방향 오차·전류 → 히스테리시스 폭, 마찰 전류 `|I+ − I−|/2`, 중력 전류 `(I+ + I−)/2`
- `step`: 명령 가장자리마다 최고 속도, dead time, 10→90 % 상승 시간, 오버슈트, 최종 오차
- 요약: `logs/profile/profile_summary.yaml`, `logs/profile/ramp_hysteresis.png` (joint2~4 오차 vs 명령)

전류 → 토크 계수 1.78 Nm/A 는 XM430-W350 정격(4.1 Nm / 2.3 A)으로 잡은 대략값이다.

---

## 출력 파일 형식 (sin_test · profile_test 공통)

- `log.csv`: `t`(사인/프로파일 시작 기준 s), `joint`, `cmd_pos`, `cmd_vel`, `pos`, `vel`(rad·rad/s, 그리퍼 m·m/s), `effort_A`(서보 전류 A — Nm 아님)
- `meta.yaml`: 모드, 관절 설정, 전체 설정, 유지 자세, center·진폭, URDF 한계
- `summary.yaml`: Peak / RMS 오차(deg 또는 mm, Gain Tuner 와 같은 정의), 최대 전류, `interrupted`(q 로 중단했는지)

## 설정 (`config/sine.yaml`)

진폭·주기 기본값은 Isaac Gain Tuner "Joints to test" 표와 같은 값·정의다 (`gains_tuner.py` sinusoidal_step):
- `amplitude_pct`: 관절 범위 **절반**에 대한 % (Gain Tuner Amplitude %). 절대값으로 주려면 `amplitude`(팔 deg · 그리퍼 m)
- `offset`: 팔은 **0°(서보 수직, 2048)** 기준 deg, 그리퍼는 URDF 범위 **가운데**(13.25 mm) 기준 m
  (Gain Tuner 는 팔도 범위 가운데가 중심이다 — joint3 −5°, joint4 +7.7°. 비교할 때는 Gain Tuner 에 같은 중심을 준다)
- `period`[s], `cycles`, `ramp_cycles`(fade 길이, 주기 수), `settle_s`, `move_s`, `speed_margin`, `limit_margin`, `sample_hz`

명령 = center + env(t) · amplitude · sin(2πt/period). Gain Tuner 와 다른 점: env 가 첫·마지막 `ramp_cycles` 주기 동안 0↔1 로 부드럽게 바뀐다 (실기가 튀지 않게).
±10° 를 Gain Tuner Amplitude % 로 바꾸면 joint1 6 %, joint2 9 %, joint3 7 %, joint4 5.5 %, joint5 6 % 다.

## 실기 전 주의

**펌웨어 통신**
- 2026-10-24: 서보 8 개를 개별 패킷으로 읽고 써서, C(명령) 50 Hz 일 때 S(상태)가 약 17 Hz(60 ms 계단)로 밀렸다. 명령 → 측정 지연 약 100 ms.
- 2026-10-26: 펌웨어가 SyncRead·BulkWrite 로 바뀌어(`STATE_PERIOD_MS = 10`, S 100 Hz) 같은 `sine.yaml` 에서
  상태 갱신 20 ms 이하(로그 샘플 주기 한계), 지연 45~62 ms 로 줄었다 (`logs/compare_20261024_20261026.png`).
  주기별(`sine_p*`)·profile 측정은 아직 10-24 값뿐이라 다시 측정해야 한다.
- `controllers.yaml` 의 `update_rate` 는 아직 **50 Hz** 다. 올리려면 먼저 `idle_record.py` 로 실제 상태 갱신 주기를 확인한다.

**관절·그리퍼**
- 팔 관절 부호·영점은 아직 실측 중이다 (config.h TODO). 첫 실행은 작은 진폭으로 방향부터 확인한다.
- **그리퍼 방향이 URDF 와 반대다**: 값이 커질수록 닫힌다. 손가락 간격 ≈ 40.1 − 1.22 × 측정값 [mm], 실물 간격 범위 약 20~40 mm.
  16.78 mm 에서 구조상 완전히 닫히고, 그 위로 명령하면 약 18.1 mm 에서 멈춰 약 2 A 로 버틴다. **그리퍼 명령은 17 mm 이하로** 준다 (`profile.yaml` 이 그렇게 되어 있다).
- 그리퍼 사이에 물체가 없어야 한다 (쥐는 힘 시험 제외 — 전류 기반 위치 모드라 쥐면 멈춘다).
- 시작 자세가 URDF 한계 밖이면 `arm_controller` 가 바로 꺼진다 (`Joint position is out of bounds`). 토크가 꺼진 상태에서 손으로 한계 안으로 옮긴 뒤 다시 띄운다.
- 팔 주변을 비우고, 비상 시 전원을 끌 수 있게 둔다.

**연결**
- `ros2 topic list` 가 비고 hw.launch 가 activate 에서 멈추면: 터미널에 남은 `ROS_DISCOVERY_SERVER` 를 `unset` 하고 `ros2 daemon stop`. activate 단계에는 timeout 이 없어 에러 없이 멈춘다.
- PC 의 RViz 에서 팔이 떨리면 PC 와 Jetson 시계 차이다: PC 에서 `sudo timedatectl set-ntp true`, `ros2 topic delay /joint_states` 가 0 근처인지 확인한다.
- RViz 메시는 `package://` 로 찾으므로 RViz 를 띄운 PC 에도 `ros_system` 패키지가 있어야 한다.
- OpenCR 전원 스위치는 12V(모터)만 끊는다. MCU 는 USB 5V 로 계속 켜져 있어 재부팅되지 않고, 다시 켠 모터는 **토크가 꺼진 상태**다.
  토크를 다시 켜려면 OpenCR RESET 버튼을 누르거나 펌웨어를 다시 올린다.
