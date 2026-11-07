# tool/cli/op/pages/ab — A/B 자동 주행·분석

노드별 A/B 동작을 `command.json`으로 정의하고, CLI가 rosbag 기록과 Nav2 주행을 직접 수행한다.
사용자가 주행할 필요는 없다.

```text
tool/cli/op/pages/ab/
└── rl_nav_controller/
    ├── command.json                # A/B 시작 자세·route·반복·timeout
    └── report.py                   # rosbag2 → report.png (횡오차·속도·궤적·정책 행동)
```

리포트는 같은 폴더 `report.py`가 만든다.

## 산출물 (tool/artifacts/ab/<노드>/)

```text
tool/artifacts/ab/rl_nav_controller/
└── 20261107_124659/                # guide 한 번 = 그룹 폴더 (시작 시각)
    ├── 20261107_124659_A/          # A leg: bag(.mcap) + raw.csv + plan.csv + report.png
    ├── 20261107_124726_B/          # B leg: bag(.mcap) + raw.csv + plan.csv + report.png
    └── compare.png                 # A vs B 비교 리포트 (고정명)
```

raw.csv 열: t,cmd_v,cmd_w,act_v,act_w,odom_v,odom_w,odom_x,odom_y,odom_yaw,
a0,a1,scan_min,dbg_v_odom,dbg_w_odom,on (20Hz 공통 격자) — calibration/wheel raw.csv와 같은 결.

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
cd tool
./run-cli ab rl_nav_controller guide --label test1
```

순서:

1. E-STOP OFF · AUTO 확인 (필요하면 AUTO로 전환)
2. `start`로 이동 (설정한 경우)
3. A 녹화 시작 (정책 ON) → `route` 주행 → 녹화 종료
4. 시작 자세로 복귀 → 정책 OFF (inner DWB)
5. B 녹화 시작 → 같은 `route` 주행 → 녹화 종료
6. 정책 ON 복원 → 최근 A/B 리포트 생성

`command.json` 키: `frame_id` · `start` · `route` · `repeats` · `goal_timeout_s` · `settle_s` ·
`auto_mode` · `move_to_start` · `return_to_start`.

> 지도·로컬라이제이션이 없으면 `map→base_footprint` TF와 `/navigate_to_pose`가 없어 자동 주행이
> 실패한다. 먼저 지도를 로드하고 LOCALIZATION이 active인지 확인한다.

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
./run-cli ab rl_nav_controller report tool/artifacts/ab/<A> tool/artifacts/ab/<B> --labels 정책 DWB
```

- 기본 저장: `tool/artifacts/ab/<시각>_<라벨>` (rosbag2 · mcap · 컨테이너: `/debug/tool/artifacts/ab`)
- TUI 메뉴 **14) A/B 테스트 (노드별)** → **rl_nav_controller**:
  상태 · ON/OFF · 녹화 시작/종료 · 최근 리포트 · **자동 A/B(command.json)** 를 한 화면에서 실행.
- 공정한 비교: 같은 출발 자세·방향, 같은 goal, 같은 배터리·바닥에서 ON/OFF를 번갈아 여러 번.
