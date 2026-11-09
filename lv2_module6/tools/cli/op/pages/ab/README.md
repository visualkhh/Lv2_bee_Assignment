# tools/cli/op/pages/ab — A/B 자동 주행·분석

노드별 A/B 동작을 `command.json`으로 정의하고, CLI가 rosbag 기록과 Nav2 주행을 직접 수행한다.
사용자가 주행할 필요는 없다.

```text
tools/cli/op/pages/ab/
└── rl_nav_controller/
    ├── command.json                # A/B 시작 자세·route·반복·timeout
    └── report.py                   # rosbag2 → report.png (횡오차·속도·궤적·정책 행동)
```

리포트는 같은 폴더 `report.py`가 만든다.

## 산출물 (tools/artifacts/ab/<노드>/)

```text
tools/artifacts/ab/rl_nav_controller/
└── 20261107_124659/                # guide 한 번 = 그룹 폴더 (시작 시각)
    ├── map.pgm · map.yaml          # 사전 점검 때 받은 /map — 리포트 배경
    ├── 20261107_124659_A/          # A leg: bag(.mcap) + raw.csv + plan.csv + pose.csv + result.json + report.png
    ├── 20261107_124726_B/          # B leg: (같음)
    └── compare.png                 # A vs B 비교 리포트 (고정명)

리포트 (compare.png · leg별 report.png): 지도 위 궤적(출발 ▲ · 목표 ★ · 진행 방향 화살표) · 선속도 · 각속도(leg 하나면 정책 행동) ·
경로 횡오차 · 핵심 지표(판정 · 정책 실행 · 시간 · 거리 · 경로 진행 · 도착 오차 · 횡오차 · 벽 최소 거리 · 후진 시간).
둘 다 유효·도착일 때만 더 나은 값을 초록으로 칠한다. 무효·실패 이유는 제목 아래 빨간 줄.

pose.csv 열: t,x,y,yaw — map 기준 자세 (guide가 TF map→base_footprint를 10Hz로 기록).
  궤적·횡오차·도착 오차는 이걸로 잰다 — odom은 시뮬 텔레포트·SLAM 보정으로 map(=웨이포인트 좌표)과 어긋난다.
  plan.csv는 guide가 쓴 route(출발 + goal, map 좌표)이고 export가 덮지 않는다.

result.json (guide가 leg마다 남김):
  arrived       Nav2가 route를 끝까지 성공했나 (실패면 error에 Nav2 error_code·error_msg)
  rl_ratio      컨트롤러가 돈 주기 중 정책이 돈 비율 (/controller_server/FollowPath/status 집계)
  reasons       바이패스 이유별 횟수 (정책 · 모델 없음 · enable=false(DWB) · 센서 없음/stale · 추론 실패)
  valid         비교에 쓸 수 있나 — A는 rl_ratio ≥ 0.9, B는 정책 0회, 둘 다 컨트롤러가 한 번이라도 돌아야 함
  (arrived는 성능 결과, valid는 측정이 맞게 됐나 — 도착 못 한 A도 valid면 '정책이 실패한' 정상 데이터)
```

raw.csv 열: t,cmd_v,cmd_w,act_v,act_w,odom_v,odom_w,odom_x,odom_y,odom_yaw,
a0,a1,scan_min,dbg_v_odom,dbg_w_odom,on (20Hz 공통 격자) — calibration/wheel raw.csv와 같은 결.

## TUI로 A/B 하는 법 (한눈에)

**13) A/B 테스트 → rl_nav_controller**:

```text
1) ① 경로 정하기   — RViz에서 goal 찍기 · 텔레오퍼로 몰고 가며 g로 찍기 · 좌표 (한 번만, 녹화 안 함)
2) ② 자동 A/B 실행 — 사전 점검 → A(정책 ON) 녹화·주행 → 시작점 복귀 → B(정책 OFF = DWB) 녹화·주행 → 리포트
                     주행 중 s = 이 leg 중지하고 다음으로 · q = 전체 중단
3) ③ 최근 결과 보기 — leg별 판정 · 도착한 목표 수 · compare.png 위치
4) 수동 (고급)      — 상태 · 정책 ON/OFF · 녹화 시작/종료 · 리포트 (사람이 직접 몰며 기록할 때만)
```

정책 ON/OFF와 녹화 시작/종료는 ②가 leg마다 알아서 한다 — 사람이 누를 일은 없다.

**시작점 복귀는 누가?** leg 사이(A 끝 → B 시작)에 **Nav2가 DWB(정책 OFF)로 자동 주행**한다 — 정책으로 돌아오면 못 오는 경우가 많아서.
- 시뮬(faker): 주행 뒤 텔레포트로 출발점에 정확히 맞춘다 (복귀 주행이 실패해도 같은 출발점).
- 실기: 자동 복귀가 실패하면 TUI가 **"로봇을 출발점 (x, y, yaw)에 놓고 Enter"** 창을 띄운다 —
  텔레오퍼로 몰거나 손으로 옮긴 뒤 Enter (s = 그 leg 건너뛰기 · q = 전체 중단). 명령줄 실행이면 멈추지 않고 실패로 끝난다.

## 경로 지정 — 실기에서 사람이 goal을 정하는 법

A·B는 **같은 경로를 그대로** 주행해야 비교가 된다 — leg마다 따로 찍지 않고, 한 번 정한 경로를 둘 다 쓴다.
TUI **13) A/B → rl_nav_controller → 1) ① 경로 정하기**:

| 항목 | 쓰는 법 |
|---|---|
| 1) 지도에서 찍기 | **터미널 지도 화면**(`/map`)이 열린다 — RViz 없이. 중앙 = 지도(처음에 포커스), 왼쪽 = 같은 동작 메뉴(`Tab`/`Esc`로 오감). 화면 한가운데 **십자선(`+`)** 이 찍을 자리, 로봇은 **빨간 점(●)**. 지도 포커스 키: `↑↓←→` 지도 밀기(로봇 따라가기가 풀림) · `+`/`-` 십자선 기준 확대·축소 · `f` 전체 보기 · `r` 로봇 따라가기 · `[` `]` 방향 ∓15° · `Enter` 십자선 위치에 goal 추가(벽 위는 거부) · `Backspace` 마지막 취소. 왼쪽 메뉴 9) 저장하고 나가기 · Esc = 저장 않고 나가기(고친 게 있으면 확인) |
| 2) RViz로 지정 | `RVIZ_CONFIG=ab ./run-rviz`로 연 RViz에서 2D Goal Pose를 순서대로 찍는다 — TUI에 G1·G2…가 바로 보임. Enter 저장 · Backspace 마지막 취소 · Esc 취소. 출발점 = 지금 로봇 위치 |
| 3) 지금 위치를 출발점으로 | 텔레오퍼로 출발 자리에 두고 |
| 4) 텔레오퍼로 이동하여 goal 추가 | 텔레오퍼 화면이 뜬다 → 몰고 가서 **g/Enter** → "지금 위치를 G2로 추가할까요?" y → 계속 몰고 가서 g … → Esc. 로봇은 그 자리에 둬도 됨 (②가 시작할 때 출발점으로 데려감). 경로가 없으면 텔레오퍼를 시작한 자리가 출발점 |
| 5) 좌표 입력 | `x y yaw°` (map 기준) |
| 6)·7) | 마지막 goal 삭제 · 전부 지우기(→ command.json 기본) |
| 8) 경로 점검 | Nav2 planner로 구간마다 경로가 나오는지 (guide 사전 점검과 같음) |

> 터미널 지도는 색(24bit)이 되면 반블록 `▀`(한 글자에 두 칸)로, 아니면 `# . `문자로 그린다. 로봇 `R` · 출발 `S` · goal `1 2 …` · 커서는 방향 화살표 · 경로는 노란 점.

- 저장: `tools/artifacts/ab/rl_nav_controller/route.json` (컨테이너 `/debug/tools/artifacts/…` — 실기·도커 모두 쓰기 가능, 컨테이너를 다시 만들어도 남음).
  있으면 start·route를 이걸로, 나머지 설정(timeout·반복 등)은 command.json. 상태 화면에 어느 쪽인지 표시.
- `ab.rviz`의 2D Goal Pose는 `/ab/goal_pose` — 기본 RViz(`/goal_pose`)는 Nav2가 받자마자 주행하므로 경로를 찍는 동안 로봇이 움직인다.

### 주행 중 키 (TUI 자동 A/B)

- `s` — 지금 leg만 멈추고 다음 leg로 (그 leg는 '중지됨', 비교에서 제외)
- `q` — 전체 중단 (녹화 정리 · 정책 ON 복원 · 그때까지 리포트)

## command.json

| 키 | 뜻 |
|---|---|
| `frame_id` | goal 좌표계 (기본 `map`) |
| `start` | A/B 시작 자세 `{x, y, yaw_deg}`. `null`이면 현재 `map→base_footprint` 자세를 시작점으로 잡는다 |
| `route` | 주행할 goal 목록 `{x, y, yaw_deg}`. A/B 모두 같은 순서로 주행 |
| `repeats` | A/B 쌍을 몇 번 반복할지 (기본 1) |
| `goal_timeout_s` | goal 하나당 제한 시간 [s] |
| `settle_s` | goal 도착 뒤 다음 goal 전 대기 [s] |
| `auto_mode` | true면 시작 전에 조작 모드를 AUTO로 전환 |
| `move_to_start` | `start`가 있을 때 시작 전에 그 자세로 이동 |
| `return_to_start` | A 주행 뒤 B를 위해 시작 자세로 복귀 |

## 실행

```bash
cd tools
./run-cli ab rl_nav_controller guide --label test1
```

순서:

1. E-STOP OFF · AUTO 확인 (필요하면 AUTO로 전환)
2. **사전 점검** — Nav2 수명주기 노드(planner·controller·bt_navigator·behavior·velocity_smoother·collision_monitor,
   코스 지도면 map_server)가 active · `/map` 수신 · start→각 goal 경로 계획(`/compute_path_to_pose`)이 되는지.
   하나라도 안 되면 녹화하지 않고 멈춘다 (복구 동작만 녹화된 A/B가 리포트로 나오지 않게).
   active가 아니면 `run-cli node activate <이름>`.
3. leg마다(A → B): 시작 자세로 주행(설정 시) → 시뮬이면 같은 자세로 텔레포트 → costmap 지우기 →
   정책 ON(A)/OFF(B) → 녹화 → `route` 주행 → 녹화 종료 → `result.json` 판정
4. 무효 leg가 있으면 반복을 멈춘다 · 정책 ON 복원 → leg별 report.png + compare.png

`command.json` 키: `frame_id` · `start` · `route` · `repeats` · `goal_timeout_s` · `settle_s` ·
`auto_mode` · `move_to_start` · `return_to_start`.

> 지도·로컬라이제이션이 없으면 `map→base_footprint` TF와 `/navigate_to_pose`가 없어 자동 주행이
> 실패한다. 먼저 지도를 로드하고 LOCALIZATION이 active인지 확인한다.

### 테스트베드(faker)에서

정책을 학습한 Isaac 미로로 띄운다 — 지도·충돌·가상 센서가 모두 같은 코스 지도를 쓴다.

```bash
docker compose exec lyrical bash
test-faker 0 course:=isaac_maze use_monitor:=false     # 다른 터미널에서 ↓
cd /ws/tools && ./run-cli ab rl_nav_controller guide --label isaac1
```

한 번에 (기동 → A/B → 정리, 종료 코드 = PASS/FAIL): `test-ab-nav-policy` (`docker/lv2_module6/test-ab-nav-policy.sh`).
이 스크립트는 `test-faker 0`으로 띄우고 위 guide를 부를 뿐이다 — 점검·주행·판정·리포트는 전부 run-cli.
goal은 `command.json`(다른 파일은 `NAV_AB_CONFIG=<경로>`), 코스는 인자(`course:=dice_maze` 등, 기본 isaac_maze).

시뮬레이터 없이(headless)도 센서가 비지 않는다: fake_nvblox = 코스 지도 ESDF(정책 가상 스캔),
fake_camera = 코스 지도 ray-casting depth(costmap·collision_monitor — B(DWB)도 벽을 봄).

## 수동 A/B — 녹화만

자동 주행 대신 RViz/Nav2로 직접 같은 goal을 보내며 기록할 수도 있다.

```bash
./run-cli ab rl_nav_controller status                # 정책 ON/OFF · 모델 · 계획 · 녹화 상태
./run-cli ab rl_nav_controller record --label 정책   # A 녹화 시작 (백그라운드)
# RViz 2D Goal Pose 또는 ros2 action send_goal /navigate_to_pose 로 주행 후
./run-cli ab rl_nav_controller stop

./run-cli ab rl_nav_controller off                   # B는 inner DWB
./run-cli ab rl_nav_controller record --label DWB
# A와 같은 출발점·같은 goal로 주행 후
./run-cli ab rl_nav_controller stop
./run-cli ab rl_nav_controller on

./run-cli ab rl_nav_controller report --last         # 최근 두 녹화 → report.png
./run-cli ab rl_nav_controller report tools/artifacts/ab/<A> tools/artifacts/ab/<B> --labels 정책 DWB
```

- 기본 저장: `tools/artifacts/ab/<시각>_<라벨>` (rosbag2 · mcap · 컨테이너: `/debug/tools/artifacts/ab`)
- TUI 메뉴 **13) A/B 테스트 (노드별)** → **rl_nav_controller** → **4) 수동 (고급)**:
  상태 · 정책 ON/OFF · 녹화 시작/종료 · 리포트 — 위 명령줄과 같은 일을 메뉴로 (자동 A/B는 이걸 전부 알아서 하므로 보통은 필요 없음).
- 공정한 비교: 같은 출발 자세·방향, 같은 goal, 같은 배터리·바닥에서 ON/OFF를 번갈아 여러 번.
