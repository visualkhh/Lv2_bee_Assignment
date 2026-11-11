# tools

운영 도구 — 수동 조작 CLI(`run-cli`) · RViz(`run-rviz`) · Isaac 지도 생성기(`course/`). CLI를 고치는 개발자는 [cli/DEVELOP.md](cli/DEVELOP.md)를 봅니다. 별도 ROS 노드를 새로 만들지 않고, 이미 떠 있는
`state_manager`·`manual_control`의 토픽과 `map_saver` 서비스를 사용합니다.

```text
tools/
├── README.md
├── setting-venv.sh  # tools/.venv 만들기 + requirements.txt (처음 한 번)
├── requirements.txt # ROS 밖 파이썬 의존성 (numpy · matplotlib · PyYAML)
├── run-cli          # 메뉴(기본) · start|stop|restart = 로봇 스택(bringup) · 그 밖은 CLI 명령
├── run-rviz         # start|stop|restart 실행기 (기본 start)
├── bag/             # rosbag 기록 (record.sh · snap_on_fail.py)
├── course/          # Isaac USD → faker/world 지도 (build_map_usd.py)
├── rviz/
│   ├── start.sh     # RVIZ_CONFIG=operator(기본)·light·mapping
│   ├── stop.sh
│   ├── restart.sh
│   ├── operator.rviz
│   ├── light.rviz   # Wi-Fi 너머 PC용 (지도·TF·로봇 모델만)
├── log/                 # 실행 결과 (raw.csv 등 · gitignore)
└── cli/
    ├── start.sh
    ├── DEVELOP.md         # 개발자용 — TUI를 어떻게 만들었나 · 화면 흐름 규약 · 메뉴 추가 · 시험
    ├── operator_tool.py   # 얇은 App: 배선 + argparse + main
    ├── calibration/       # wheel/{command.json,report.py} (sim과 같은 네모 · 비교 리포트)
    └── op/                # 실제 구현
        ├── terminal.py    # 터미널 원자 연산 (크기·색·표시너비)
        ├── defs.py        # 메뉴 내비게이션 상수
        ├── consts.py      # 로봇 상수 + 파라미터/인자 변환
        ├── state.py       # StateStore — ROS 상태 보관
        ├── keys.py        # 키 읽기 (cbreak · 방향키 · UTF-8)
        ├── component.py   # Component — 화면 부품 최상위 부모
        ├── window/        # 화면 틀 — Window (조합·키 루프·신호 · 키 규약)
        ├── menu_bar/      # 메뉴 막대 — MenuBar → LeftSideMenuBar · ChoiceItem
        ├── dialog/        # 창 — Dialog 최상위 부모 + Input·Key·Pager·Busy·Teleop (한 파일에 한 클래스)
        ├── state_bar/     # 상단 상태바 — State 칸들 + StateBar + TopStateBar
        ├── ui.py          # Ui — 창 진입점 · 터미널 모드 · ui.window
        ├── bus.py         # Bus — 발행·전송·대기·명령 실행
        └── pages/         # Page → Menu → 각 메뉴 화면 (MenuItem)
```

`run-cli`는 인자가 없으면 대화형 메뉴, `start|stop|restart`는 **로봇 스택(bringup)** 제어, 그 밖은 CLI 한 번 명령이다.

```bash
cd tools
./run-cli                     # 대화형 메뉴 (TUI)
./run-cli start               # bringup 시작 (백그라운드 · 로그 ./bringup.log(실행한 폴더, BRINGUP_LOG=경로로 바꿈) · 이미 떠 있으면 그대로)
./run-cli stop                # 떠 있는 ROS 전부 끄기 (ros2_ws/stop.sh)
./run-cli restart             # stop → start (= restart all)
./run-cli restart perception  # 그 노드만 (state_manager · 수명주기 노드는 active까지)
./run-cli status              # 그 밖은 CLI 명령 (status · mode manual · arm show · node list …)
./run-cli beep                # OpenCR 부저 기본음 · ./run-cli beep 4000 300 = 높이·길이 (메뉴 1 상태 → 삑)
```

### bag 녹화 + 시스템 부하 (CPU · 메모리 · GPU)

과부하·끊김을 분석하려면 토픽만으로는 모자란다 — **bag 과 같은 폴더에 부하 기록(`sys.csv`)을 같이** 남긴다. 메뉴 `8) 토픽 · bag`(`s` 선택 녹화 · `d` 진단 녹화 · `u` 부하 기록 켬/끔)과 아래 명령이 같은 모듈(`op/bagrec.py`)이다 — 다른 터미널·ssh에서 멈출 수도 있다.

```bash
./run-cli bag start --label asis_manual    # 진단 프리셋 토픽(명령 체인·odom·TF·지도·코스트맵·상태) + 부하 기록
./run-cli bag start --topics /map /tf      # 이 토픽만
./run-cli bag start --no-sys               # 부하 기록 끔
./run-cli bag status                       # 녹화 중인 폴더 · 경과 · 크기
./run-cli bag stop                         # SIGINT 로 정상 종료 (metadata 기록)
```

저장: `~/.cache/beebot/bags/<날짜시각>_<라벨>/` — `*.mcap` + `metadata.yaml` + **`sys.csv`**(1초마다: load1 · cpu_pct · mem_avail_mb · swap_used_mb · gpu_pct · 메모리 큰 프로세스 6개) + `tegrastats.log`(Jetson). 부하 기록기는 `tools/bag/sysmon.py`(표준 라이브러리만, 단독 실행 가능). 시험: `python3 tools/cli/tests/test_bagrec.py`.

`start`는 기본값으로 **`ROS_DOMAIN_ID=88` + `ros2_ws/fastdds_local_only.xml`(SHM 전용)** 을 쓴다 — DDS가 기기 밖으로 나가지도 들어오지도 않는다. 셸의 `FASTDDS_DEFAULT_PROFILES_FILE` · `ROS_DOMAIN_ID`를 지정하면 그 값이 우선한다 (PC에서 볼 때는 `fastdds.xml`).

```bash
cd tools
./run-rviz          # RViz 시작 (= start)
./run-rviz stop
./run-rviz restart
```

로그를 터미널에서 직접 보려면 `ros2_ws/start.sh`로 앞에서 띄워도 된다.

## 처음 셋팅 (venv)

```bash
cd tools
./setting-venv.sh      # tools/.venv 만들기 + requirements.txt (numpy · matplotlib · PyYAML)
```

- ROS와 같은 파이썬으로 `--system-site-packages` — rclpy·메시지(ROS setup.bash)와 apt 패키지는 그대로 보고, 없는 것만 venv에 깐다.
- `run-cli`가 `.venv`가 있으면 쓴다 (없으면 시스템 python3). `run-rviz`는 파이썬을 안 써서 무관.
- `.venv`는 git에 안 올라간다 (`.gitignore`). requirements가 바뀌면 다시 실행.
- 도커 개발 컨테이너는 `/ws/tools`가 읽기 전용 마운트라 venv를 안 만든다 — 이미지에 이미 설치돼 있음.

## 실행

```bash
cd tools
./run-cli
```

`run-cli`가 `/opt/ros/lyrical`과 `../ros2_ws/install/setup.bash`를 source합니다.
다른 워크스페이스를 쓰면 `ROS_DISTRO_NAME` 또는 `BEEBOT_MAP_DIR`를 지정합니다.

터미널에서 인자 없이 실행하면 **대화형 메뉴**가 열립니다.

```text
  1) 상태
  2) 장치
  3) 조작 (모드 · 수동 · 팔 · 그리퍼 · 바퀴 · E-stop · 비프음 · 충돌 감시)
  4) 지도/위치
  5) 노드 (상태 · 파라미터 · 진단 · 재시작 · 전체 재시작)
  6) RViz 열기 (Best Effort QoS)
  7) 칼리브레이션
  8) 토픽 · bag
  9) 미션 (시작 · 중지 · skill 직접 · 진행 · 실패 이유 — 실기 시험 디버그)
  q) 종료
```

모든 화면이 같은 키 규약을 씁니다 (`cli/op/window/window.py` 맨 위에 정리):

| 화면 | 키 |
|---|---|
| 메뉴 | `↑↓` 이동 · `Enter` 실행 · 항목 키(숫자·글자)로 포커스 · `Esc`/`◀`/`b` 뒤로(한 단계) · `m` 메인 · `r` 새로고침 · `q` 종료 |
| 상단 상태바 | `Tab` 들어가기 · `←→` 칸 이동 · `Enter` 열기(미션 칸 → 미션 화면 · 진단 칸 → 진단 화면) · `Tab`/`Esc` 메뉴로 |
| 확인창 | `y` 예 · `n`/`Esc` 아니오 · `Enter` 기본값 |
| 입력창 | `Enter` 확인 · `Esc`/`◀` 취소. 값이 틀리면 창이 닫히지 않고 사유를 보여 주므로 고쳐서 다시 Enter |
| 결과창 | 실행 출력은 결과창에 모입니다. `↑↓`/`PgUp`/`PgDn` 스크롤 · `Enter`/`Esc` 닫기(같은 메뉴로) |

- 경고·현재값처럼 결정 전에 알아야 할 내용은 확인창·입력창 안에 나옵니다. 취소하면 아무것도 출력하지 않습니다
- 수동 명령(주행·팔·그리퍼)은 MANUAL이 아니면 먼저 `MANUAL 모드로 전환할까요?`를 묻습니다
- AUTO 전환, 비상정지 **해제**, 재시작, 지도 전환은 확인을 받습니다. 비상정지 **ON**은 확인 없이 바로 적용됩니다
- 상단 상태바는 입력 없이 2초마다 자동 새로고침됩니다 — 1줄 조작·지도·위치·E-STOP·장치 · 2줄 **미션**(단계+경과 · 지금 skill+경과 · 마지막 실패 이유 빨강)·보이는 물체·주행 제어기(RL/DWB)·충돌 감시·SLAM(cuVSLAM Hz·점프)·진단 · 3줄 토픽 Hz(joint·odom·imu·카메라·점구름·인지) · 4~5줄 모터·전원
- 화면은 4분할입니다: 상단 상태바 고정 · 왼쪽 메뉴 · 중앙 내용 · 하단 키 안내 고정.
  터미널 크기에 맞춰 다시 계산하므로 창을 넓히면 다음 그리기부터 반영됩니다
  (폭 70 미만이면 세로로 쌓습니다)
- `1) 상태 → 5) 진단`: 진단 목록(문제 있는 것 먼저, ● 초록 OK · 노랑 WARN · 빨강 ERROR) → 고르면 그 항목의 값 전부를 2초마다 갱신해서 보여 줌 (신호등 미션 진행 · 모터 에러 등)
- `2) 장치`에서 장치를 고르면 종류·활성 여부·토픽·관련 진단을 봅니다.
  `1) 파라미터`는 그 장치 토픽을 내는 노드를 찾아 노드 파라미터 화면을 엽니다
- `5) 노드 → 노드 → 4) 파라미터`: 노드 → 파라미터 목록(현재값 표시) → 고르면 값 입력 → 변경 내용을 확인창에서 보고 적용.
  읽기전용은 거부. `c` config 기본값 비교 · `d` yaml 기본값으로 되돌리기
- `3) 조작 → 2) 수동 주행`에서 `텔레오퍼`를 고르면 `w`/`↑`(전진) · `x`/`↓`(후진) · `a`/`←`·`d`/`→`(회전),
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

상단 상태바와 `status`의 배터리 %는 **3S LiPo 전압 기반 대략 추정**입니다.
12V 아답터를 꽂으면 OpenCR 전원 입력값이라 배터리 잔량이 아니며, 12.45V 이상이면 `충전?`으로 표시합니다.
정확한 잔량이 필요하면 주행 부하를 뺀 상태에서 전압을 보세요.

### 자동/수동 전환

```bash
./run-cli mode manual
./run-cli mode auto
```

터미널에서 수동 명령(`drive`·`teleop`·`arm`·`gripper`)을 내렸는데 MANUAL이 아니면
전환할지 물어보고 `y`면 자동으로 바꾼 뒤 명령을 보냅니다 (시뮬레이터 "마지막 행동이 모드"와 같은 사용감).
자동화(비TTY)에서는 묻지 않고 거부하므로 스크립트는 `mode manual`을 먼저 넣으세요.

### 미션 (실기 시험 디버그)

```bash
./run-cli mission status   # 미션 화면과 같은 줄 (단계 · skill · 실패 이력 · 적재함 · 주행 · SLAM · 진단)
./run-cli mission start    # /cmd/mission start (mission_manager)
./run-cli mission abort    # /cmd/mission abort
./run-cli mission skill pick 5   # skill 한 단계만 (/task/sub_goals → coordinator) — AUTO에서만
./run-cli mission stop     # 진행 중 skill 전부 취소
```

메뉴 **9) 미션** 한 화면에서 RViz·`ros2` 명령 없이 "왜 멈췄나"를 봅니다 (2초마다 갱신):

| 줄 | 내용 | 출처 |
|---|---|---|
| [미션] | 단계 + 단계 경과 + note · 바코드 · 신호 R/Y/G · 배정 · 놓음/전체 · 결과 6개 · 오류 | `/mission/state` |
| [skill] | step/전체 · skill → 대상 · RUNNING/FAILED + 경과 · 이유 | `/task/status` + `/task/sub_goals` |
| [실패 이력] | 최근 10건 (시각 · skill 실패 이유 · 미션 ERROR 이유) — 이유 문자열은 그대로 표시 | 위 둘 |
| [적재함] | 칸:tag (`?` 모름 · `!` 확인 필요) · 손에 든 퍽 · 실은 순서 (장부를 못 받으면 /mission/state의 tray 사본) | `/coordinator/basket` |
| [주행] | 남은 거리 · 회복 횟수 · 경과 · 제어기 RL/DWB(이유) · cloud 나이 · scan_min · v·w · 충돌 감시 켬/끔 + 지금 정지·감속 | Nav2 feedback · `/controller_server/FollowPath/status` · manual_control 진단 · `/collision_monitor_state` |
| [SLAM] | cuVSLAM Hz · map→odom 점프 횟수·크기 (TF를 0.5초마다 보고 5 cm/3° 넘게 바뀌면 1회) | `/cuvslam/odom` · TF |
| [보이는 물체] | 클래스별 지금 보이는 수 | `/tracked_objects` |
| [팔 skill] | skill · 단계(내려가기·쥐기/놓기…) · 붙듦 · 보정 live | `/skill_context` |
| [신호 해독] | 확정/오류 이유 · R/Y/G · 연속 사이클 · raw | `/mission/signal_status` |
| [진단] | WARN/ERROR 항목 (배터리 20% 미만이면 [전원] 경고) | `/diagnostics_agg` |

- `1) 미션 시작` · `2) 미션 중지`: 확인 뒤 `/cmd/mission`. 적재함 장부에 퍽이 남아 있으면 mission_manager가 시작을 거부합니다 (note에 이유).
- `3) 세부 명령`: navigate · pick · place … skill 한 단계를 직접 보냄 (AUTO가 아니면 전환을 물음) · `s` 전부 취소.
- `4) 적재함 장부 비우기`: 적재함을 손으로 비웠을 때 `/coordinator/basket_clear`.
- `5) 실패 이력 지우기`.
- 상단바 미션 칸(Tab → Enter)으로도 이 화면이 열립니다.

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

### 칼리브레이션 (바퀴)

바퀴 응답(gain · 지연)을 재서 sim에 맞추는 측정입니다. 메뉴: **7) 칼리브레이션 → 1) 바퀴**. 팔(`calibrate arm`)은 나중에 같은 메뉴에 붙입니다.
계획(JSON)의 `(v, w)`를 수동 경로로 재생하며 응답을 `raw.csv`로 기록합니다.
계획 파일은 **설정**이라 `tools/cli/calibration/wheel/command.json`(sim과 같은 네모 54 s)에 두고,
결과는 **실행 로그**라 `tools/artifacts/calibration/<시각>_wheel/raw.csv`에 남깁니다.

```bash
./run-cli calibrate wheel --check-only          # 안 움직이고 준비만 확인 (MANUAL · /odom · /imu · /joint_states)
./run-cli calibrate wheel                       # 기본 네모 재생 + 기록 → tools/artifacts/calibration/<시각>_wheel/raw.csv
./run-cli calibrate wheel plan.json             # 다른 계획: {"dt": 0.05, "label": "real", "steps": [[v, w], ...]}  (v ≤ 0.26 m/s · |w| ≤ 1.82 rad/s · 180 s 이하)

./run-cli calibrate report                      # 최근 raw.csv 1~2개 → report.png (2개면 비교)
./run-cli calibrate report sim.csv real.csv --labels SIM REAL --out compare.png
```

메뉴 7 → 바퀴: `1) 준비 확인` → (공간 비움) → `2) 재생 + 기록` · `3) 계획 파일 바꾸기` · `4) 리포트/비교`.

- 명령은 `/cmd/manual/cmd_vel` → `manual_control` → `/cmd_vel_manual`이라 twist_mux 수동 경로로 바로 간다 (순수 응답).
- 기록: `/odom` · `/imu` · `/joint_states`(바퀴) · `/cmd_vel`(smoother · collision_monitor를 거쳐 실제로 나간 명령) — isaac sim의 `raw.csv`와 같은 열.
- 끝나거나 비상정지 · 모드 이탈 · Ctrl+C면 바로 정지 명령을 내고(저장 안 함), 로봇이 움직이므로 사방 약 1.5 m를 비웁니다. 팔이 시작 자세(stow)가 아니면 재생 전에 `arm initial`로 먼저 보내고 도달을 기다립니다(10 s 안에 못 가면 거부 · 충돌 회피 없음).
- `report.py`는 궤적 · 명령 vs 엔코더/자이로 · 추종 오차 · 바퀴 속도 · 메시지 지연을 그리고,
  두 CSV를 주면 두 궤적의 평균 거리까지 비교한다. 결과는 `tools/artifacts/calibration/<시각>_wheel/report.png`.
- 결과를 PC로: `scp bee@bee.local:<폴더>/raw.csv …` 후 isaac의 `calibration/report.py`로 sim과 비교.

### 팔·그리퍼

```bash
./run-cli arm show
./run-cli arm set joint1=0.0 joint2=0.4 joint3=-0.2
./run-cli arm delta joint2=+0.05
./run-cli arm home
./run-cli arm initial       # 시작 자세로 — 떠 있는 skill_pose의 initial_pose·initial_gripper (없으면 skills.yaml) (메뉴: 팔 → 7)
./run-cli arm reboot          # 과부하 등 Hardware Error 난 모터만 재부팅 (메뉴: 팔 → 5)
./run-cli arm capture         # 지금 팔·그리퍼 자세를 skills.yaml(skill_pose) initial_pose·initial_gripper 줄로 출력 (파일은 안 고침)

./run-cli gripper show
./run-cli gripper open
./run-cli gripper close
./run-cli gripper position 0.01
```

- `arm set`은 지정한 관절만 보내고, 나머지는 `manual_control`이 현재 각으로 채웁니다.
- `MAPPING` 중에는 팔 명령이 스캔 자세 고정 때문에 거절됩니다. 그리퍼는 가능합니다.
- 관절 한계는 `controller_manager`의 joint limiter가 최종 적용합니다.
- 시작 자세 다시 잡기: 팔 조작 화면(관절 막대)으로 자세를 맞춘 뒤 `arm capture` → 출력된 `initial_pose`·`initial_gripper` 줄을 `ros2_ws/src/skills/config/skills.yaml`(`skill_pose`)에 붙여 넣고 bringup을 재시작합니다. 조작 모드는 AUTO·MANUAL 둘뿐이며 MANUAL은 토크가 늘 켜져 있습니다 (TEACH 모드는 state_manager가 받지 않아 TUI에서 뺐습니다).
- `arm reboot`은 `/state/devices/<wheels·arm·gripper>`에서 모터 에러(`joint2:과부하`)를 보고, 있으면 `/cmd/manual/reboot_motors`를 보낸 뒤 풀릴 때까지(최대 8초) 기다립니다. 에러가 없으면 보내지 않습니다. MANUAL 모드에서만 동작하며, 팔 메뉴에도 에러가 빨간색으로 표시됩니다.

### 지도

```bash
./run-cli map status
./run-cli map list
./run-cli map mode mapping
./run-cli map mode localization
./run-cli map switch 20261019_093353
./run-cli map rename 20261019_093353 --to arena_a   # 폴더 이름 바꾸기
./run-cli map delete arena_a                         # 폴더째 삭제
./run-cli map save                 # 저장 후 LOCALIZATION 전환을 물어봄 (TTY)
./run-cli map save --localize      # 저장 후 바로 LOCALIZATION (비대화형)
./run-cli map save --no-localize   # 미리보기만 저장하고 MAPPING 유지
./run-cli map save --file /tmp/my_map
./run-cli map save --map-dir /path/to/maps
```

지도 한 벌 = 폴더 하나 `<map_dir>/<이름>/` (`nvblox.nvblx` 정본 · `map.pgm`·`map.yaml` 미리보기). 목록은 `nvblox.nvblx`가 있는 폴더입니다.

- `map mode` · `map switch`는 **launch 전체를 다시 띄웁니다** (cuVSLAM·nvblox는 실행 중 전환이 없음 — state_manager가 `all map:=slam` · `all map:=saved map_name:=<이름>`).
  CLI는 `/state/restart`의 첫 응답(`all: 재시작 중 …` · `거절: …`)만 보여 주고 끝납니다. 다시 뜨는 데 수십 초(nvblox 지도 저장 → SLAM 시작) — 끝났는지는 `map status`.
- `map rename` · `map delete`는 지도 폴더를 직접 바꿉니다 (state_manager가 2초마다 다시 읽어 목록 갱신). **지금 쓰는 지도**(모드가 NONE이 아닐 때의 `map_name`)는 거절 — 다른 지도로 전환하거나 NONE에서 하세요.
  TUI **7) 지도/위치 → 2) 지도 목록**에서 지도를 고르면 보기 · 이름 바꾸기 · 삭제(확인창) 메뉴가 열립니다.
- 없는 지도 이름 · 지도 폴더 없이 `localization`은 CLI가 먼저 거절합니다 (state_manager는 거절 이유를 `/diagnostics`에만 남김).
- COURSE는 없어졌습니다. 테스트베드(faker)는 fake SLAM(fake_cuvslam + fake_nvblox)으로 GPU 없이 MAPPING(센서로 지도 작성)/LOCALIZATION(저장 지도)이 됩니다.
- `map save`는 `/map_saver/save_map`(map:=slam·saved일 때만 뜸)을 불러 `.pgm`·`.yaml`을 저장합니다.
  저장할 때 `/nvblox_node/save_map`으로 `nvblox.nvblx`도 명시적으로 저장한 뒤,
  **LOCALIZATION으로 전환할지 물어보고** 위치추정으로 다시 뜹니다.
  (`after_shutdown_map_save_path`는 png/yaml만 쓰므로 nvblx 저장은 이 서비스가 담당)
  TUI **7) 지도 → 6) 지도 저장**도 같은 확인창을 띄웁니다.
  기본 경로는 `<state_manager의 map_dir>/<현재 지도 이름>/map` — map_saver가 도는 쪽(로봇·컨테이너) 경로라 원격 PC에서 CLI를 써도 맞습니다.
  `map_dir`을 못 읽으면 이 컴퓨터의 `ros2_ws/map`(또는 `BEEBOT_MAP_DIR`). `--file` · `--map-dir`는 그대로 씁니다.
- **테스트 지도 폴더는 따로** — `--source fake`·`--source docker`는 `tools/cli/ros/fake/map/`을 씁니다 (도커는 compose가 `/ws/map`으로 바인드 · `BEEBOT_MAP_DIR`을 주면 그게 우선). 실기 지도(`ros2_ws/map`)와 섞이지 않습니다.
  `default` = `tools/course/build_world.py`가 Isaac 미로 USD로 만든 **완벽한 지도**(벽 100 · 미로 안 0 · 밖 미지)라 매핑 없이 바로 `map switch default` → LOCALIZATION 주행 시험. 커밋되는 건 `default`만입니다.

### 노드 (상태 · 파라미터 · 진단 · 재시작)

```bash
./run-cli node list                          # 노드 전부 — ● 보임 / ✗ 안 보임 · pid · 수명주기 노드는 [active] 등
./run-cli node status velocity_smoother      # 한 노드 상세 (pid · 수명주기 상태 · 발행/구독 수 · 진단)
./run-cli node activate velocity_smoother    # 수명주기 노드 unconfigured·inactive → active
./run-cli node restart perception            # 그 노드만 다시 띄움 (수명주기 노드면 다시 뜬 뒤 active까지)
./run-cli restart                            # 전체 재시작 — stop → start (메뉴 5 a)는 state_manager가 같은 launch 명령으로)
```

- 메뉴 **5) 노드**: 맨 위 `a)` 전체 재시작(RESTART 입력), 그 아래 노드 목록. 노드를 고르면 상태·진단이 **1초마다 갱신**되고, 1) 상태 자세히 · 2) 재시작 · 3) 활성화(수명주기 노드만) · 4) 파라미터·진단 live(파라미터 서비스가 있는 노드만)를 고를 수 있습니다.
- **4) 파라미터 · 진단 live** 화면은 파라미터 현재값과 그 노드의 `/diagnostics` 항목을 1초마다 다시 읽습니다. 값 변경은 `ros2 param set` 또는 메인 **12) 파라미터**에서.
- 수명주기 노드(velocity_smoother · collision_monitor · Nav2 서버)는 `lifecycle_manager_base`·`_navigation`이 기동 때 켭니다. 기동 중 응답 시간 초과 등으로 멈춰 `[unconfigured]`·`[inactive]`로 남으면 바퀴 명령이 `/cmd_vel`까지 못 가 텔레오퍼가 안 움직입니다 → 그 노드를 **3) 활성화**.
- 재시작은 state_manager 재시작 목록(이 launch 아래 노드)에 있는 것만 됩니다. 명령줄 재시작은 확인 없이 바로 실행합니다.

## RViz

```bash
cd tools
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

Jetson 기본은 `ROS_DOMAIN_ID=88` + `fastdds_local_only.xml`(SHM만)이라 네트워크를 타지 않습니다. PC에서 원격으로 보려면 양쪽을 `fastdds.xml`(UDP 포함)로 바꾸고, PC도 같은 `ROS_DOMAIN_ID=88`로 맞추고 `ROS_LOCALHOST_ONLY`는 끄세요.

```bash
# PC — Jetson과 같은 값
export ROS_DOMAIN_ID=88
export FASTDDS_DEFAULT_PROFILES_FILE=/path/to/ros2_ws/fastdds.xml
```

느리면 RViz Image Display의 **Transport Hint**를 `compressed`로 바꾸거나
Jetson에서 `tools/run-rviz`를 실행하세요.

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

## CLI 모드 (real · fake · docker)

`run-cli --source`로 CLI가 쓸 ROS 층을 고릅니다. 기본은 `real`(진짜 rclpy·bringup)입니다.

- `fake` — 프로세스 안 가짜 ROS(`cli/ros/fake/FakeRos.py`). ROS·도커 없이 화면을 시험합니다.
  발행은 `[PUB ...]`로 찍히고 mode/estop/map 요청은 가짜 state_manager가 받아 상태를 바꿉니다.
- `docker` — 도커 `test-faker 0`이 쓰는 `lv2_module6/debug` 파일을 폴링합니다. faker가 꺼져 있으면 자동으로 띄우고 준비될 때까지 기다립니다.
  구독은 `output/topic`, 발행은 `input/topic`, 서비스는 `input/service`↔`output/service`.
  faker의 `service_action_bag_input` 화이트리스트 밖 서비스와 파라미터 변경은 아직 미지원입니다.

`start/stop/restart/build`는 `real` 전용입니다.

```bash
cd tools
./run-cli --source fake                   # 대화형 메뉴 (가짜 ROS)
./run-cli --source fake status
./run-cli --source docker status          # 도커 스택을 debug 파일로 (뷰어 없음)
./run-cli --source docker menu            # 메뉴
./run-cli --source docker --debug-dir ../debug menu
python3 cli/tests/scenarios.py            # pty로 실제 키를 넣어 화면 흐름 전수 검증
python3 cli/tests/scenarios.py teleop-hud # 시나리오 하나만
```

실패한 시나리오는 화면 출력이 `cli/tests/.scenario-<이름>.bin`에 남습니다 (git 무시).

