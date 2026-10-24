# R2S (Real to Sim) — 실기 관절 응답 측정

실제 beebot 팔(joint1~5)과 그리퍼(gripper_left_joint)에 사인파·hold·step·ramp 위치 명령을 주고, 명령값과 관측값(`/joint_states`)을 기록한다.
Isaac Sim Gain Tuner 결과와 비교해 시뮬레이션 drive(stiffness·damping·friction)와 지연 모델을 실기에 맞추는 용도.
(2026-10-24 `real_test` → `R2S` 로 이름 변경. 측정 결과와 해석은 `logs/20261024_worklog.md`.)

## 구성

```
set_center              ──/arm_controller/joint_trajectory (topic)──┐
sin_test / profile_test ──follow_joint_trajectory (action)──────────┴─> arm_controller (JTC, 50 Hz)
   ↑                                                                       │
   └─ /joint_states ── joint_state_broadcaster                             └─> control/BeebotSystemHardware ──USB──> OpenCR ──> 서보
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

- `ros2 launch`/`ros2 run` 은 `src/` 가 아니라 `install/` 에 복사된 yaml 을 읽는다. yaml 을 고치면 다시 빌드하거나, 처음부터 `--symlink-install` 로 빌드해 둔다.
- 폴더 이름을 바꾸면(`real_test` → `R2S`) `install/` 의 setup 스크립트에 예전 절대 경로가 남는다. `build/ install/ log/` 를 지우고 다시 빌드한다.

## 실행

메인 bringup 은 꺼 둔다 (OpenCR 포트는 한 프로세스만 열 수 있다). 각 터미널에서 `source install/setup.bash`.

```bash
ros2 launch sine_test hw.launch.py                      # 터미널 1: 실기 (/dev/opencr) — mock_hardware:=true 면 로봇 없이 (오차 ≈ 0)
                                                        #   인자: port, urdf, mesh_uri (기본 package://ros_system/urdf/beebot_mesh/meshes)
ros2 run sine_test set_center                           # 터미널 2: joint5 → joint1 순서로 0° 로
ros2 run sine_test set_center joint3                    # 터미널 2: joint3 만 0° 로
ros2 run sine_test sin_test joint3                      # 터미널 2: joint3 사인 테스트 (설정은 config/sine.yaml)
ros2 run sine_test sin_test joint3 --ros-args -p config:=/path/my.yaml -p out_dir:=/path/logs
```

테스트 노드 공통: **`q`** (또는 Ctrl+C) → 그 자리에서 정지하고 정상 종료. 키 입력을 받아야 하므로 launch 가 아니라 터미널에서 실행한다.

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

### profile_test (action)
`profile_test <관절> <hold|step|ramp>`. 설정은 `config/profile.yaml` 의 `joints.<이름>.<모드>`, 결과는 `logs/profile/<ts>_<관절>_<모드>/` (형식은 sin_test 와 같음).
흐름(시작 위치로 천천히 이동 → 한 goal → center 에서 정지)과 `q` 정지도 sin_test 와 같다.
yaml 에 `tag` 가 있으면 폴더 이름 뒤에 붙는다 (`<ts>_<관절>_<모드>_<tag>/`).
- `hold`: 각도 목록마다 천천히 이동해 `dwell_s` 유지 → 자세별 처짐·유지 전류 (중력 토크 → 링크 질량 검증). joint2·3·4 만
- `step`: +size → 0 → −size → 0 으로 명령이 바로 바뀜 → 속도 포화, 오버슈트, 정착 시간. 속도 검사를 하지 않는다 (서보가 최고 속도로 움직임)
- `ramp`: 일정 속도(`speed` deg/s)로 ±amplitude 왕복 → 방향이 바뀔 때 멈춰 있는 폭(백래시)과 전류 변화(마찰)

쥐는 힘 시험(30 mm 퍽): `profile_test gripper_left_joint hold --ros-args -p config:=$(ros2 pkg prefix sine_test)/share/sine_test/config/grip.yaml`
— 2 mm(열림, 퍽 넣기) → 9 mm(쥐기) → 2 mm(빼기). 접촉점(약 7.8 mm)을 크게 넘겨 명령하지 않는다 (17 mm 에서 손가락이 휨).

분석: `python3 profile_analysis.py` (ROS source 필요 — URDF 중력 토크 계산에 xacro 사용) → `<run>/plot.png`·`analysis.yaml`, `logs/profile/profile_summary.yaml`·`ramp_hysteresis.png`.

### 움직이지 않는 관절의 자세 유지 기준
`allow_partial_joints_goal: false` 라 궤적마다 6 개 관절 값을 모두 보낸다. 움직이지 않는 관절에는 **측정값(`/joint_states`)이 아니라
JTC 가 지금 명령하고 있는 값**(`/arm_controller/controller_state` 의 `reference`)을 그대로 넣는다.

실기에서는 팔 무게 때문에 서보가 명령보다 조금 처진다 (예: 명령 0.0°, 측정 −1.0°). 측정값을 새 명령으로 보내면 목표가 −1.0° 가 되고
거기서 또 처져, 테스트를 반복할수록 자세가 조금씩 내려간다. 명령값을 다시 보내면 목표가 그대로라 지금 버티던 상태가 유지된다.
(mock 은 명령 = 측정이라 차이가 없다.)

이 값을 받아야 시작하므로 `arm_controller` 가 active 가 아니면 테스트 노드 모두 10 초 기다린 뒤 아무것도 움직이지 않고 종료한다.
`q` 정지(set_center)도 이 명령값을 다시 보내 그 자리를 유지한다.

## 설정 (`config/sine.yaml`)

진폭·주기 기본값은 Isaac Gain Tuner "Joints to test" 표와 같은 값·정의다 (`gains_tuner.py` sinusoidal_step):
- `amplitude_pct`: 관절 범위 **절반**에 대한 % (Gain Tuner Amplitude %). 절대값으로 주려면 `amplitude`(팔 deg · 그리퍼 m)
- `offset`: 팔은 **0°(서보 수직, 2048)** 기준 deg, 그리퍼는 URDF 범위 **가운데** 기준 m
  (Gain Tuner 는 팔도 범위 가운데가 중심이다 — joint3 −5°, joint4 +7.7°. 비교할 때는 Gain Tuner 에 같은 중심을 준다)
- `period`[s], `cycles`

주기별 응답 (`config/sine_p0.5.yaml`, `sine_p1.yaml`, `sine_p2.yaml`, `sine_p4.yaml`): 모든 주기에서 진폭을 ±10°(그리퍼 ±3 mm)로 같게 두어
주기만 바뀌게 했다. `tag`(폴더 이름 뒤에 붙음)와 `out_subdir: freq` 때문에 결과는 `logs/freq/<ts>_<joint>_p<주기>/` 에 쌓이고 logs/ 요약 그림에는 섞이지 않는다.

```bash
ros2 run sine_test sin_test joint3 --ros-args -p config:=$(ros2 pkg prefix sine_test)/share/sine_test/config/sine_p0.5.yaml
```

명령 = center + env(t) · amplitude · sin(2πt/period). Gain Tuner 와 다른 점: env 가 첫·마지막 `ramp_cycles` 주기 동안 0↔1 로 부드럽게 바뀐다 (실기가 튀지 않게).

## 출력 (`<out_dir>/<YYYYmmdd_HHMMSS>_<joint>/`, 기본 out_dir = 이 폴더의 logs/)

- `log.csv`: `t`(사인 시작 기준 s), `joint`, `cmd_pos`, `cmd_vel`, `pos`, `vel`(rad·rad/s, 그리퍼 m·m/s), `effort_A`(서보 전류 A — Nm 아님)
- `meta.yaml`: 관절 설정, 전체 설정, 유지 자세, 사인 center·진폭, URDF 한계
- `summary.yaml`: Peak / RMS 오차(deg 또는 mm, Gain Tuner 와 같은 정의), 최대 전류, `interrupted`(q 로 중단했는지)

## 분석

```bash
python3 plot_logs.py                      # logs/ 아래 sin_test run 전부 (일부만: python3 plot_logs.py logs/2026..._joint2)
python3 plot_logs.py logs/freq/2026*      # 주기별 run 각각 분석 → 이어서
python3 freq_response.py                  # logs/freq/freq_response.png, freq_response.yaml
python3 profile_analysis.py               # profile run (ROS source 필요)
```

- `plot_logs.py`: run 마다 `plot.png`(명령·측정·오차·전류), `plot_lag_removed.png`(명령을 맞춘 지연만큼 늦춰 비교), `analysis.yaml`(gain, lag, 오프셋, 지연 보정 RMS).
  `logs/summary.png`, `summary_lag_removed.png`, `joints_lag_removed.png` 는 관절별 최신 run 으로 만든다.
- `freq_response.py`: 관절별 gain·lag vs 주기. lag 는 `lag(T) = 순수 지연 + 일정 위상/360 × T` 로 맞춘다.
  순수 지연은 주기와 무관한 ms(통신·측정 경로), 일정 위상은 주기에 비례해 커지는 몫(마찰·백래시)이다.

## 정지 기록 (`idle_record.py`)
`python3 idle_record.py 60`: 테스트 노드 없이 팔이 멈춰 있는 동안 `/joint_states` 를 60 초 기록한다 → `logs/<ts>_idle/log.csv`.
엔코더·전류 노이즈, 상태 갱신 주기, 처짐을 본다 (meta.yaml 이 없어 plot_logs.py 는 건너뛴다).

## 실기 전 주의

**펌웨어 통신 (2026-10-24 측정)**
- OpenCR 은 들어온 C(명령)를 전부 처리한 뒤에 S(상태)를 보낸다. 서보 8 개를 개별 패킷으로 읽고 쓰므로 C 가 잦을수록 S 가 밀린다.

  | C 주기 | S (상태 갱신) |
  |---|---|
  | 없음 | 40 Hz (`STATE_PERIOD_MS = 25`) |
  | 50 Hz | 움직이는 동안 약 17 Hz (60 ms 계단) |
  | 100 Hz | 끊김 (`OpenCR 상태(S) 수신 없음` 경고) |

- 그래서 `controllers.yaml` 의 `update_rate` 는 **50 Hz** 다. `/joint_states` 는 50 Hz 로 나오지만 값은 약 60 ms 마다 바뀐다.
- 명령 → 측정 지연은 약 100 ms (joint1 115 ms). 측정 peak 오차의 상당 부분은 60 ms 계단 때문이라 Isaac 비교에는 gain·지연 보정 RMS 를 쓴다.
- 펌웨어 SyncRead/SyncWrite 개선 후에는 `python3 idle_record.py 60` 의 상태 갱신 주기와 `sine_p*` 측정으로 다시 확인한다.

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
