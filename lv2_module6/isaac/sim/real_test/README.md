# real_test — 실기 관절 사인파 응답 측정

실제 beebot 팔(joint1~5)과 그리퍼(gripper_left_joint)에 사인파 위치 명령을 주고, 명령값과 관측값(`/joint_states`)을 기록한다.
Isaac Sim Gain Tuner 결과와 비교해 시뮬레이션 drive(stiffness·damping)를 실기에 맞추는 용도.

## 구성

```
set_center ──/arm_controller/joint_trajectory (topic)──┐
sin_test   ──follow_joint_trajectory (action)──────────┴─> arm_controller (JTC, 100 Hz)
   ↑                                                           │
   └─ /joint_states ── joint_state_broadcaster                 └─> control/BeebotSystemHardware ──USB──> OpenCR ──> 서보
```

- `hw.launch.py`: 하드웨어·컨트롤러만 띄우고 계속 유지. 테스트 노드는 다른 터미널에서 `ros2 run` 으로 하나씩 실행
- 메인 ws 에서 쓰는 것: `control` 패키지의 `BeebotSystemHardware` 플러그인(소스만 읽어 이 ws 에 빌드), `urdf/beebot_mesh/urdf.xacro`
- 쓰지 않는 것: `ml`(rl_arm_correction), `ros_system`/`bringup` launch, diff_drive·imu·gripper action 컨트롤러
- 명령 없는 인터페이스: 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (플러그인 동작)

## 빌드 (이 폴더의 ros2_ws 에서)

```bash
cd ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src ../../../../ros2_ws/src/control --packages-select control sine_test
```

build/·install/·log/ 는 이 ws 안에만 생긴다. 메인 ws 는 건드리지 않는다.

## 실행

메인 bringup 은 꺼 둔다 (OpenCR 포트는 한 프로세스만 열 수 있다). 각 터미널에서 `source install/setup.bash`.

```bash
ros2 launch sine_test hw.launch.py                      # 터미널 1: 실기 (/dev/opencr) — mock_hardware:=true 면 로봇 없이 (오차 ≈ 0)
ros2 run sine_test set_center                           # 터미널 2: joint5 → joint1 순서로 0° 로
ros2 run sine_test set_center joint3                    # 터미널 2: joint3 만 0° 로
ros2 run sine_test sin_test joint3                      # 터미널 2: joint3 사인 테스트 (설정은 config/sine.yaml)
ros2 run sine_test sin_test joint3 --ros-args -p config:=/path/my.yaml -p out_dir:=/path/logs
```

두 노드 공통: **`q`** (또는 Ctrl+C) → 그 자리에서 정지하고 정상 종료. 키 입력을 받아야 하므로 launch 가 아니라 터미널에서 실행한다.

hw.launch 시작 순서: 하드웨어를 inactive 로 띄워 OpenCR 상태를 먼저 읽고(2초), active 로 바꾼 뒤 컨트롤러를 띄운다.
`BeebotSystemHardware::on_activate` 가 첫 시리얼 읽기 전의 상태(0)를 명령으로 잡아 시작 순간 모든 관절에 0 을 보내는 문제를 피하기 위해서다
(가짜 OpenCR 로 확인: 바로 active 면 시작 1.4 초 동안 팔 전체 0 명령, inactive 경유면 0 명령 없음).

### set_center (topic)
인자 없음: joint5 → joint4 → … → joint1 순서로 한 관절씩 0°(서보 수직, 2048)로 천천히 옮긴다
(관절당 최소 `move_s`, 큰 이동은 URDF 속도의 0.3 배 이하).
`set_center joint3` 처럼 관절 이름을 주면 그 관절만 옮긴다 (팔 joint1~5 만, 없는 이름이면 로그만 남기고 종료).
나머지 관절은 JTC 의 현재 명령을 유지하고, 그리퍼는 개도를 유지한다. 관절마다 이동이 끝나면 남은 오차를 출력한다.

### sin_test (action)
인자는 관절 이름 하나. 진폭·offset·주기·cycles 는 yaml 의 `joints.<이름>` 에서 읽는다 — 없는 이름이면 로그를 남기고 아무것도 움직이지 않는다.
1. 검사 (걸리면 움직이지 않고 종료): 스윙이 URDF 한계 안쪽 `limit_margin` 을 넘음 / 최고 속도가 URDF velocity 의 `speed_margin` 배를 넘음
2. 다른 팔 관절이 0° 에서 2° 넘게 떨어져 있으면 경고 (set_center 먼저)
3. 해당 관절을 사인 center 로 천천히 이동 → 사인 (fade-in → cycles → fade-out) → center 에서 `settle_s` 기록 후 **그 자리에 정지**하고 종료

### 움직이지 않는 관절의 자세 유지 기준
`allow_partial_joints_goal: false` 라 궤적마다 6 개 관절 값을 모두 보낸다. 움직이지 않는 관절에는 **측정값(`/joint_states`)이 아니라
JTC 가 지금 명령하고 있는 값**(`/arm_controller/controller_state` 의 `reference`)을 그대로 넣는다.

실기에서는 팔 무게 때문에 서보가 명령보다 조금 처진다 (예: 명령 0.0°, 측정 −1.0°). 측정값을 새 명령으로 보내면 목표가 −1.0° 가 되고
거기서 또 처져, 테스트를 반복할수록 자세가 조금씩 내려간다. 명령값을 다시 보내면 목표가 그대로라 지금 버티던 상태가 유지된다.
(mock 은 명령 = 측정이라 차이가 없다.)

이 값을 받아야 시작하므로 `arm_controller` 가 active 가 아니면 두 노드 모두 10 초 기다린 뒤 아무것도 움직이지 않고 종료한다.
`q` 정지(set_center)도 이 명령값을 다시 보내 그 자리를 유지한다.

## 설정 (`config/sine.yaml`)

진폭·주기 기본값은 Isaac Gain Tuner "Joints to test" 표와 같은 값·정의다 (`gains_tuner.py` sinusoidal_step):
- `amplitude_pct`: 관절 범위 **절반**에 대한 % (Gain Tuner Amplitude %). 절대값으로 주려면 `amplitude`(팔 deg · 그리퍼 m)
- `offset`: 팔은 **0°(서보 수직, 2048)** 기준 deg, 그리퍼는 URDF 범위 **가운데** 기준 m
  (Gain Tuner 는 팔도 범위 가운데가 중심이다 — joint3 −5°, joint4 +7.7°. 비교할 때는 Gain Tuner 에 같은 중심을 준다)
- `period`[s], `cycles`

명령 = center + env(t) · amplitude · sin(2πt/period). Gain Tuner 와 다른 점: env 가 첫·마지막 `ramp_cycles` 주기 동안 0↔1 로 부드럽게 바뀐다 (실기가 튀지 않게).

## 출력 (`<out_dir>/<YYYYmmdd_HHMMSS>_<joint>/`, 기본 out_dir = 이 폴더의 logs/)

- `log.csv`: `t`(사인 시작 기준 s), `joint`, `cmd_pos`, `cmd_vel`, `pos`, `vel`(rad·rad/s, 그리퍼 m·m/s), `effort_A`(서보 전류 A — Nm 아님)
- `meta.yaml`: 관절 설정, 전체 설정, 유지 자세, 사인 center·진폭, URDF 한계
- `summary.yaml`: Peak / RMS 오차(deg 또는 mm, Gain Tuner 와 같은 정의), 최대 전류, `interrupted`(q 로 중단했는지)

## 실기 전 주의

- 관측은 40 Hz 다 (펌웨어 `STATE_PERIOD_MS = 25`). joint_states 는 100 Hz 로 나오지만 값은 25 ms 마다 바뀐다.
- 펌웨어의 관절 부호·영점·그리퍼 m→rad 변환은 아직 실측 전(config.h TODO). 첫 실행은 작은 진폭으로 방향부터 확인한다.
- 그리퍼 사이에 물체가 없어야 한다 (전류 기반 위치 모드 — 쥐면 멈춘다).
- 팔 주변을 비우고, 비상 시 전원을 끌 수 있게 둔다.
