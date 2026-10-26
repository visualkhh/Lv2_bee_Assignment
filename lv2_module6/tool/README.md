# tool

운영 중 수동 조작을 위한 CLI입니다. CLI를 고치는 개발자는 [cli/DEVELOP.md](cli/DEVELOP.md)를 봅니다. 별도 ROS 노드를 새로 만들지 않고, 이미 떠 있는
`state_manager`·`manual_control`의 토픽과 `map_saver` 서비스를 사용합니다.

```text
tool/
├── README.md
├── run-cli          # start|stop|restart 실행기 (기본 start)
├── run-rviz         # start|stop|restart 실행기 (기본 start)
├── rviz/
│   ├── start.sh
│   ├── stop.sh
│   ├── restart.sh
│   └── operator.rviz
└── cli/
    ├── start.sh
    ├── stop.sh
    ├── restart.sh
    ├── DEVELOP.md         # 개발자용 — TUI를 어떻게 만들었나 · 화면 흐름 규약 · 메뉴 추가 · 시험
    ├── operator_tool.py   # 얇은 App: 배선 + argparse + main
    └── op/                # 실제 구현
        ├── terminal.py    # 터미널 원자 연산 (크기·색·표시너비)
        ├── defs.py        # 메뉴 내비게이션 상수
        ├── consts.py      # 로봇 상수 + 파라미터/인자 변환
        ├── state.py       # StateStore — ROS 상태 보관
        ├── dialogs.py     # 키 읽기 · 확인/입력/결과창 · 메뉴 루프 (키 규약)
        ├── ui.py          # Ui — 4분할 프레임 + 상태바 + 다이얼로그 진입점
        ├── bus.py         # Bus — 발행·전송·대기·명령 실행
        └── pages/         # Page → Menu → 각 메뉴 화면 (MenuItem)
```

`run-cli`/`run-rviz`는 `start|stop|restart`를 받아 각 폴더의 스크립트를 실행한다.
인자가 없으면 `start`다.

```bash
cd tool
./run-cli            # CLI 시작 (= start)
./run-cli stop       # 실행 중인 CLI 종료
./run-cli restart    # CLI 프로세스 재시작
```

```bash
cd tool
./run-rviz          # RViz 시작 (= start)
./run-rviz stop
./run-rviz restart
```

주의: `./run-cli restart <대상> --yes`처럼 뒤에 인자가 있으면
CLI 프로세스 재시작이 아니라 state_manager에 보내는 재시작 '요청' 명령이다.

전체 로봇은 먼저 띄워 둡니다.

```bash
cd ../ros2_ws
./start.sh
```

## 실행

```bash
cd tool
./run-cli
```

`run-cli`가 `/opt/ros/lyrical`과 `../ros2_ws/install/setup.bash`를 source합니다.
다른 워크스페이스를 쓰면 `ROS_DISTRO_NAME` 또는 `BEEBOT_MAP_DIR`를 지정합니다.

터미널에서 인자 없이 실행하면 **대화형 메뉴**가 열립니다.

```text
  1) 상태
  2) 장치
  3) 조작 모드
  4) 수동 주행 / 텔레오퍼
  5) 팔
  6) 그리퍼
  7) 지도
  8) 비상정지
  9) 재시작
 10) RViz 열기 (Best Effort QoS)
  q) 종료
```

모든 화면이 같은 키 규약을 씁니다 (`cli/op/dialogs.py` 맨 위에 정리):

| 화면 | 키 |
|---|---|
| 메뉴 | `↑↓` 이동 · `Enter` 실행 · 항목 키(숫자·글자)로 포커스 · `Esc`/`◀`/`b` 뒤로(한 단계) · `m` 메인 · `r` 새로고침 · `q` 종료 |
| 확인창 | `y` 예 · `n`/`Esc` 아니오 · `Enter` 기본값 |
| 입력창 | `Enter` 확인 · `Esc`/`◀` 취소. 값이 틀리면 창이 닫히지 않고 사유를 보여 주므로 고쳐서 다시 Enter |
| 결과창 | 실행 출력은 결과창에 모입니다. `↑↓`/`PgUp`/`PgDn` 스크롤 · `Enter`/`Esc` 닫기(같은 메뉴로) |

- 경고·현재값처럼 결정 전에 알아야 할 내용은 확인창·입력창 안에 나옵니다. 취소하면 아무것도 출력하지 않습니다
- 수동 명령(주행·팔·그리퍼)은 MANUAL이 아니면 먼저 `MANUAL 모드로 전환할까요?`를 묻습니다
- AUTO·EXPLORE 전환, 비상정지 **해제**, 재시작, 지도 전환은 확인을 받습니다. 비상정지 **ON**은 확인 없이 바로 적용됩니다
- 상단 상태바(조작·지도·E-STOP·장치 + 인지·임무·진단 2단)는 입력 없이 2초마다 자동 새로고침됩니다
- 화면은 4분할입니다: 상단 상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 키 안내 고정.
  터미널 크기에 맞춰 다시 계산하므로 창을 넓히면 다음 그리기부터 반영됩니다
  (폭 70 미만이면 세로로 쌓습니다)
- `1) 상태 → 5) 진단`: 진단 목록(문제 있는 것 먼저, ● 초록 OK · 노랑 WARN · 빨강 ERROR) → 고르면 그 항목의 값 전부를 2초마다 갱신해서 보여 줌 (신호등 미션 진행 · 모터 에러 등)
- `2) 장치`에서 장치를 고르면 종류·활성 여부·토픽·관련 진단을 봅니다.
  `1) 파라미터`는 그 장치 토픽을 내는 노드를 찾아 `11) 파라미터 (노드별)`과 같은 화면을 엽니다
- `11) 파라미터 (노드별)`: 노드 → 파라미터 목록(현재값 표시) → 고르면 값 입력 → 변경 내용을 확인창에서 보고 적용.
  읽기전용은 거부. `c` config 기본값 비교 · `d` yaml 기본값으로 되돌리기
- `4) 수동 주행`에서 `텔레오퍼`를 고르면 `w`/`↑`(전진) · `x`/`↓`(후진) · `a`/`←`·`d`/`→`(회전),
  `s`·`space`(정지), `e`·`c`(속도▲▼), `q`/`Esc`(종료)로 바로 주행합니다

인자 없이 실행했는데 TTY가 아니면 자동화를 위해 `status`만 출력합니다.

한 번만 실행하는 기존 명령도 그대로 사용할 수 있습니다.

```bash
./run-cli status
./run-cli status --diagnostics
```

## 명령

### 상태

```bash
./run-cli status
./run-cli status --diagnostics                    # 진단 요약 (항목마다 레벨 · 한 줄 메시지)
./run-cli status --diagnostics signal_decoder     # 이름에 그 글자가 든 항목 상세 — 레벨 · 메시지 · 값 전부
./run-cli status --diagnostics opencr             # 예: OpenCR 연결 · 모터 에러 · 받은 S 줄
./run-cli status --topics
```

`/state/operation_mode`, `/state/map_mode`, `/state/map_name`, `/state/emergency`,
장치별 `/state/devices/<이름>`을 읽습니다.

### 자동/수동 전환

```bash
./run-cli mode manual
./run-cli mode auto
./run-cli mode explore       # MAPPING 중일 때만
```

터미널에서 수동 명령(`drive`·`teleop`·`arm`·`gripper`)을 내렸는데 MANUAL이 아니면
전환할지 물어보고 `y`면 자동으로 바꾼 뒤 명령을 보냅니다 (시뮬레이터 "마지막 행동이 모드"와 같은 사용감).
자동화(비TTY)에서는 묻지 않고 거부하므로 스크립트는 `mode manual`을 먼저 넣으세요.

### 비상정지

```bash
./run-cli estop on
./run-cli estop off
./run-cli estop status
```

`estop on`은 바퀴 명령을 막고, 현재 수동 주행 명령도 0으로 보냅니다.

### 수동 주행

먼저 `mode manual`이어야 합니다.

```bash
./run-cli drive forward --speed 0.15 --duration 1.0
./run-cli drive back --speed 0.10 --duration 0.5
./run-cli drive left --angular 0.6 --duration 0.5
./run-cli drive right --angular 0.6 --duration 0.5
./run-cli drive stop
```

터미널 키보드 조작:

```bash
./run-cli teleop
# w/s 전후진 · a/d 좌우회전 · space 정지 · q 종료
```

### 팔·그리퍼

```bash
./run-cli arm show
./run-cli arm set joint1=0.0 joint2=0.4 joint3=-0.2
./run-cli arm delta joint2=+0.05
./run-cli arm home
./run-cli arm reboot          # 과부하 등 Hardware Error 난 모터만 재부팅 (메뉴: 팔 → 5)

./run-cli gripper show
./run-cli gripper open
./run-cli gripper close
./run-cli gripper position 0.01
```

- `arm set`은 지정한 관절만 보내고, 나머지는 `manual_control`이 현재 각으로 채웁니다.
- `MAPPING` 중에는 팔 명령이 스캔 자세 고정 때문에 거절됩니다. 그리퍼는 가능합니다.
- 관절 한계는 `controller_manager`의 joint limiter가 최종 적용합니다.
- `arm reboot`은 `/state/devices/<wheels·arm·gripper>`에서 모터 에러(`joint2:과부하`)를 보고, 있으면 `/cmd/manual/reboot_motors`를 보낸 뒤 풀릴 때까지(최대 8초) 기다립니다. 에러가 없으면 보내지 않습니다. MANUAL 모드에서만 동작하며, 팔 메뉴에도 에러가 빨간색으로 표시됩니다.

### 지도

```bash
./run-cli map status
./run-cli map list
./run-cli map mode mapping
./run-cli map mode localization
./run-cli map switch 20261019_093353
./run-cli map save
./run-cli map save --file /tmp/my_map
./run-cli map save --map-dir /path/to/maps
```

`map save`는 `/map_saver/save_map` 서비스를 호출해 `.pgm`·`.yaml`을 저장합니다.
기본 경로는 `ros2_ws/map/<현재 지도 이름>/map`입니다.

### 재시작

```bash
./run-cli restart all --yes
./run-cli restart perception --yes
```

`--yes` 없이는 실행되지 않습니다. 전체 스택이 재시작되므로 주의합니다.

## RViz

```bash
cd tool
./run-rviz
```

`rviz/operator.rviz`는 카메라·점구름을 **Best Effort, queue 1**로 구독합니다.
이 설정이 없으면 RealSense의 best effort 발행자와 RViz의 reliable 구독자가
충돌해 `incompatible QoS` 경고가 나고 영상이 보이지 않습니다.

기본 표시:

| 표시 | 토픽 | QoS |
| --- | --- | --- |
| Front Camera | `/pole_front_cam/color/image_raw` | Best Effort, 1 |
| Gripper Camera | `/gripper_cam/color/image_raw` | Best Effort, 1 (기본 꺼짐) |
| Front Points | `/pole_front_cam/depth/color/points_filtered` | Best Effort, 1 (기본 꺼짐) |
| Gripper Points | `/gripper_cam/depth/color/points` | Best Effort, 1 (기본 꺼짐) |
| Map | `/map` | Reliable, Transient Local, 1 |

점구름과 gripper 영상은 네트워크를 많이 쓰므로 기본으로 꺼 두었습니다.
필요할 때 RViz Displays에서 켜세요.

Jetson에서 로컬로 실행하면 네트워크를 타지 않습니다. PC에서 원격으로 볼 때는
Jetson과 PC의 `ROS_DOMAIN_ID`를 맞추고, `ROS_LOCALHOST_ONLY`는 끄세요.

```bash
# Jetson과 PC 모두 같은 값
export ROS_DOMAIN_ID=42
```

느리면 RViz Image Display의 **Transport Hint**를 `compressed`로 바꾸거나
Jetson에서 `tool/run-rviz`를 실행하세요.

## 안전 규칙

- 수동 주행·팔·그리퍼는 `MANUAL` 모드에서만 동작합니다.
- 비상정지 중에는 수동 명령을 거부합니다.
- `drive`는 지정 시간이 끝나거나 Ctrl+C로 종료될 때 0 속도를 보냅니다.
- `teleop`은 종료 시 터미널 설정을 복원하고 0 속도를 보냅니다.
- `--force`는 상태 확인만 건너뛰며, 실제 `manual_control`이 명령을 무시할 수 있습니다.

## 문제 확인

```bash
# 상태 토픽이 안 보이면
ros2 node list | grep -E 'state_manager|manual_control'
ros2 topic echo /state/operation_mode --once \
  --qos-reliability reliable --qos-durability transient_local

# 수동 명령이 안 먹으면
ros2 topic info /cmd/manual/cmd_vel -v
ros2 topic echo /state/emergency --once \
  --qos-reliability reliable --qos-durability transient_local
```

`state_manager`가 없으면 `status`는 “상태 없음”을 출력합니다. 먼저 `bringup`을 띄우세요.

### 시험과 실기 분리

같은 네트워크에서 여러 로봇/시뮬레이터를 띄우면 `operator_tool`이 다른 스택의
`/state`를 읽거나 명령을 보낼 수 있습니다. 시험할 때는 `ROS_DOMAIN_ID`를 다르게
지정하고, 실행 전에 `ros2 node list`로 대상 스택이 하나인지 확인하세요.

```bash
export ROS_DOMAIN_ID=42
./run-cli status
```

## cli-test (ROS 없이 UI 시험)

가짜 ROS층(`cli-test/fake_ros.py`)으로 진짜 `operator_tool` 코드를 돌립니다. 발행은 `[PUB ...]`로 찍히고
mode/estop/map 요청은 가짜 state_manager가 받아 상태를 바꿉니다.

```bash
cd tool
./run-cli-test              # 대화형 메뉴
./run-cli-test status
python3 cli-test/scenarios.py                      # pty로 실제 키를 넣어 화면 흐름 전수 검증
python3 cli-test/scenarios.py teleop-hud           # 시나리오 하나만
```

실패한 시나리오는 화면 출력이 `cli-test/.scenario-<이름>.bin`에 남습니다 (git 무시).

