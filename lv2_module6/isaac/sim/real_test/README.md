# real_test — 실기 관절 사인파 응답 측정

실제 beebot 팔(joint1~5)과 그리퍼(gripper_left_joint)에 사인파 위치 명령을 주고, 명령값과 관측값(`/joint_states`)을 기록한다.
Isaac Sim Gain Tuner 결과와 비교해 시뮬레이션 drive(stiffness·damping)를 실기에 맞추는 용도.

## 구성

```
sine_test 노드 ──FollowJointTrajectory──> arm_controller (JTC, 100 Hz)
   ↑                                            │
   └─ /joint_states ── joint_state_broadcaster  └─> control/BeebotSystemHardware ──USB──> OpenCR ──> 서보
```

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

메인 bringup 은 꺼 둔다 (OpenCR 포트는 한 프로세스만 열 수 있다).

```bash
source install/setup.bash
ros2 launch sine_test sine_test.launch.py mock_hardware:=true     # 로봇 없이 흐름 확인 (오차 ≈ 0)
ros2 launch sine_test sine_test.launch.py                         # 실기 (/dev/opencr)
ros2 launch sine_test sine_test.launch.py config:=/path/my.yaml port:=/dev/ttyACM0 out_dir:=/path/logs
```

시작 순서: 하드웨어를 inactive 로 띄워 OpenCR 상태를 먼저 읽고(2초), active 로 바꾼 뒤 컨트롤러·테스트 노드를 띄운다.
`BeebotSystemHardware::on_activate` 가 첫 시리얼 읽기 전의 상태(0)를 명령으로 잡아 시작 순간 모든 관절에 0 을 보내는 문제를 피하기 위해서다
(가짜 OpenCR 로 확인: 바로 active 면 시작 1.4 초 동안 팔 전체 0 명령, inactive 경유면 0 명령 없음). 메인 bringup 은 이 문제가 그대로 있다.

시험이 끝나면 launch 가 스스로 종료한다. Ctrl+C 는 진행 중 goal 을 취소하고(그 자리에서 정지, 시작 자세로 돌아가지 않음) 그때까지의 로그를 저장한다.

## 설정 (`config/sine.yaml`)

기본값은 Isaac Gain Tuner "Joints to test" 표와 같은 값·정의다 (`gains_tuner.py` sinusoidal_step):
- `amplitude_pct`: 관절 범위 **절반**에 대한 % (Gain Tuner Amplitude %). 절대값으로 주려면 `amplitude`(팔 deg · 그리퍼 m)
- `offset`: URDF 관절 범위 **가운데** 기준 (팔 deg · 그리퍼 m). 절대 위치로 주려면 `center`
- `period`[s], `cycles`, `test`

명령 = center + env(t) · amplitude · sin(2πt/period). Gain Tuner 와 다른 점: env 가 첫·마지막 `ramp_cycles` 주기 동안 0↔1 로 부드럽게 바뀌고,
시작 자세에서 center 까지 먼저 천천히 이동한다 (실기가 튀지 않게). 한 번에 한 관절만 움직이고 나머지는 시작 자세를 유지한다.

보내기 전 검사 — 걸리면 그 관절은 건너뛴다:
- 진폭이 URDF 한계 안쪽 `limit_margin` 을 넘음
- 최고 속도(A·2π/T 근처)가 URDF velocity 의 `speed_margin` 배를 넘음

## 출력 (`<out_dir>/<YYYYmmdd_HHMMSS>/`, 기본 out_dir = 이 폴더의 logs/)

- `log.csv`: `t`(사인 시작 기준 s), `joint`, `cmd_pos`, `cmd_vel`, `pos`, `vel`(rad·rad/s, 그리퍼 m·m/s), `effort_A`(서보 전류 A — Nm 아님)
- `meta.yaml`: 설정, 시작 자세, URDF 한계, mock 여부
- `summary.yaml`: 관절별 Peak / RMS 오차(deg 또는 mm, Gain Tuner 와 같은 정의), 최대 전류

## 실기 전 주의

- 관측은 40 Hz 다 (펌웨어 `STATE_PERIOD_MS = 25`). joint_states 는 100 Hz 로 나오지만 값은 25 ms 마다 바뀐다.
- 펌웨어의 관절 부호·영점·그리퍼 m→rad 변환은 아직 실측 전(config.h TODO). 첫 실행은 작은 진폭으로 방향부터 확인한다.
- 그리퍼 사이에 물체가 없어야 한다 (전류 기반 위치 모드 — 쥐면 멈춘다).
- 팔 주변을 비우고, 비상 시 전원을 끌 수 있게 둔다.
