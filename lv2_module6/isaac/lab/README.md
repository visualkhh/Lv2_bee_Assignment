# isaac/lab — 팔 RL (주사위 칸 놓기 잔차)과 한 사이클 시연

Isaac Lab 6.1 (`~/pa-course/IsaacLab-6.1/.venv`)에서 돈다. 로봇·퍽·상자는 `../sim/usda`, 기구학은 `../../urdf/beebot_mesh/beebot.urdf`를 쓴다. ROS는 쓰지 않는다.

## 시연

```bash
cd isaac/lab && ~/pa-course/IsaacLab-6.1/.venv/bin/python demo_cycle.py --device cpu --viz kit --hold
```

창이 뜨고 `usda/demo_cycle.usda`에서 한 사이클을 돈다. 순서는 다음과 같다.

1. 시작 박스에서 퍽을 집는다.
2. 바구니 칸 5에 넣는다.
3. 주사위 상자 앞으로 주행한다.
4. 바구니에서 퍽을 꺼낸다.
5. 가운데 칸(주사위 1)에 놓는다.

끝나면 창이 그대로 남는다. 터미널에는 단계별 줄(`[demo] 1 …`부터 `9 …`까지), 놓기 결과(중심 오차·기울기·2초 정지), 사이클 시간이 찍힌다.

| 옵션 | 뜻 |
|---|---|
| `--runs 3` | 연속 3회 (사이마다 장면 초기화) |
| `--no_policy` | 놓기를 잔차 0(스크립트 동작만)으로. 비교용 |
| `--fast` | 놓기를 2배 속도(0.6 × 팀 한계, settle 0.15 s)로, 속도 무작위화로 학습한 `place_residual_fast.pt` 사용 |
| `--puck b` | 원기둥 퍽 |
| `--cell 0..5` | 놓을 칸 (0~2 가까운 줄, 3~5 가운데 줄, 4 = 가운데) |
| `--seed -1` | 인식 잡음 무작위 (기본 0) |
| `--viz`·`--hold` 빼고 `--runs 20` | 창 없이 성공률만 |

## 무엇이 학습됐나

RL 태스크 분할의 구간 9(place_pip)만 학습했다. 나머지 구간은 `feature/isaac-pick`의 `cycle_sm.py` 규칙을 옮긴 스크립트다. 오프라인 IK, 관절 공간 이동(0.3 × 팀 한계), 실측 지연, 유지 중 처짐 적분이 그 내용이다.

- **환경** `place_env.py` (`Bee-Place-Residual`): 손에 퍽을 쥔 채 칸 위에서 시작한다. 이어서 유지 → 내려가기 → 안정 → 36 mm로 열기 → 올리기 → 대기 순으로 진행한다.
  - 기본 동작은 칸 위치 ±5 mm, 손 안 퍽 ±2 mm 오차를 가진 IK 계획이다.
  - 정책은 손가락을 열기 전까지 관절 잔차(|Δ| ≤ 0.05 rad)를 더한다.
  - 정책은 그리퍼 카메라 대용 관측(칸 − 퍽, 편향 2 mm + 잡음 1 mm)을 받는다.
  - critic은 참값을 받는다(asymmetric).
  - 가까운·가운데 줄만 다룬다. 먼 줄은 넘어짐(기구 문제)이라 뺐다.
- **학습** `train_place.py`: rsl_rl PPO, env 512개, 50 Hz 정책, 200 Hz 물리.

```bash
cd isaac/lab && PY=~/pa-course/IsaacLab-6.1/.venv/bin/python
$PY train_place.py --mode train --num_envs 512 --iters 1500                       # → policies/place_residual.pt (약 35분)
$PY train_place.py --mode eval --num_envs 512 --episodes 2048 --policy policies/place_residual.pt
$PY train_place.py --mode zero --num_envs 512 --episodes 2048                     # 잔차 0 기준선
```

## 결과 (2026-10-31, seed 7, 에피소드 2048개씩)

| 조건 | 잔차 0 | 정책 |
|---|---|---|
| 기본 속도 (0.3, settle 0.3 s) | 96.3 %, 중심 오차 p90 8.4 mm | **99.0 %, p90 4.1 mm** |
| 2배 속도 (0.6, settle 0.15 s), v1 정책 | 96.1 %, p90 7.2 mm | 98.2 %, p90 5.2 mm |
| 2배 속도, v2 정책 (`place_residual_fast.pt`, 속도 무작위화로 v1에서 1000 iter 더) | 96.1 %, p90 7.2 mm | **98.9 %, p90 4.1 mm** |
| 기본 속도, v2 정책 | 96.3 %, p90 8.4 mm | 98.4 %, p90 5.2 mm |

v2는 놓기를 2배 빠르게 해도(내려가기·올리기 속도 2배, settle 절반) v1의 기본 속도 수준을 유지한다.

시연 한 사이클(창 없이 여러 번):
- 놓기는 정책을 켜고 성공한 사이클 전부가 중심 오차 1.4~5.9 mm다.
- 사이클 시간은 시뮬 시간 약 42 s다. 대부분 0.3 × 팀 한계 이동과 주행이다.
- 실패는 대부분 시작 박스 직육면체 퍽 집기에서 났다. 연속 실행에서 약 1/3이었다.
  - 원인은 손이 닫힘 방향으로 목표보다 2~3 mm 더 나가는 것이다(처짐·추종).
  - 여기에 인식 오차 +3 mm가 겹치면 손가락이 퍽 윗면에 얹혀 퍽을 넘어뜨린다.
  - 접근 자세에서 0.3 s 멈춰 처짐 적분이 먼저 보정하게 해서 3/5 실패를 1/5로 줄였다.
  - 남은 실패는 집기 구간 잔차(RL 태스크 분할 B3, pick_box)가 맡을 몫이다.
- 넣기 실패 1회는 퍽이 링 모따기에 걸려 덜 들어간 경우다.

성공 기준은 `cycle_sm.py --place`와 같다(중심 10 mm 이내, 기울기 5° 이하, 손에서 빠짐, 상자 위, 정지). 학습 환경의 대기는 0.5 s, 시연은 2 s다.

## 한계

- 인식은 참값에 학습 때와 같은 잡음을 더한 대용이다. YOLO·태그 인식은 붙어 있지 않다.
- 주행은 참 위치를 쓰는 P 제어다. cuVSLAM·Nav2를 쓰지 않는다.
- 기본 계획은 cuRobo가 아니라 pinocchio IK와 관절 공간 보간이다. 충돌 회피는 없고 경로는 `cycle_sm.py` 규칙의 경유점이다.
- 실기 처짐 오프셋(B0-0)은 아직 없다. 시뮬의 처짐은 유지 중 적분(`cycle_sm.py`와 같음)이 받는다.
- 시작 박스 직육면체 퍽 집기가 가장 약하다(위 참고).
  - 넘어지지 않았으면 다시 집기를 2회까지 한다.
  - 넘어졌으면 사이클을 멈춘다. 위에서 잡는 방식으로는 누운 퍽을 집을 수 없다.
  - 기본 `--seed 0` 첫 사이클은 넘겨주기 전에 창 모드로 성공을 확인했다.

## skill별 학습 환경 (skills.yaml 이름 기준)

미션 skill 목록(`lv2_module6/learning/skills.yaml`, 브랜치 `feature/skill-learning`)의 움직이는 skill을 모두 이 폴더의 잔차 강화학습 환경 하나씩에 연결했다. 모두 `train_place.py`로 같은 방식(rsl_rl PPO, 정책 = 기준 동작 + 잔차)으로 학습·평가한다.

```bash
cd isaac/lab && PY=~/pa-course/IsaacLab-6.1/.venv/bin/python
$PY train_place.py --task <task> [--cfg k=v,...] --mode zero  --num_envs 64  --episodes 128   # 기준선 (잔차 0)
$PY train_place.py --task <task> [--cfg k=v,...] --mode train --num_envs 512 --iters 1500 --run <skill> --out <skill>.pt
$PY train_place.py --task <task> [--cfg k=v,...] --mode eval  --num_envs 256 --episodes 1024 --policy policies/<skill>.pt
#   창으로 보기: --num_envs 16 --viz newton (가벼움) 또는 --viz kit
#   8 GB GPU에는 학습 하나만 (둘이면 PhysX GPU 메모리 부족). GPU가 바쁘면 --device cpu
#   여러 skill 차례로: ./train_queue.sh [skill ...]  → logs/queue_summary.tsv (기준선·학습 뒤 성공률)
#   새 환경(load·unload·reach·drive)은 행동 표준편차를 로그 공간으로 (그냥 두면 dock에서 0.00으로 무너져 학습이 멈췄음)
```

| skill | `--task` | `--cfg` | 환경 파일 | 기준선 성공 (잔차 0, 2026-11-03 pa27) |
|---|---|---|---|---|
| `place_cell` | `pip` | | pip_env.py (팀원) | 96.3 % (place, 10-31) |
| `pick_standing_puck` | `pick_standing` (예전 `pick`) | | pick_env.py (팀원) | |
| `stand_lying_puck` | `stand_lying` (예전 `stand` · `right`) | | right_env.py (팀원) | |
| `regrasp_after_stand` | `stand_lying` (예전 `stand` · `right`) | `regrasp=1` | right_env.py (팀원, 세우기 + 다시 집기 한 번에) | |
| `load_slot` | `load` | | slot_env.py | 93.8 % (192판; 처음 71.1 %는 뒤 칸에서 손이 20° 기운 자세에 퍽을 똑바로 넣은 시작 상태 오류였음) |
| `unload_slot` | `unload` | | slot_env.py | 90.6 % |
| `move_pose` | `reach` | `target=pose` | reach_env.py | 100 % (오차 ≤ 0.3 mm) |
| `camera_clear_view` | `reach` | `target=clear` | reach_env.py | 100 % (scan 자세가 정해질 때까지 stow) |
| `pregrasp` | `reach` | `target=above` | reach_env.py | 88.3~91.0 % — 먼 줄은 맞아도 판정 시간 안에 안 멈춤(흔들림), 목표 5번(손목 −2.35)은 18 mm 어긋남 |
| `camera_look_down` | `reach` | `target=look` | reach_env.py | 100 % (높이 0.08 m, 아래 참고) |
| `drive_straight` | `drive` | `task=straight` | drive_env.py | 68.0 % (허용 10 mm, 중앙값 8.3 mm) |
| `drive_turn` | `drive` | `task=turn` | drive_env.py | 100 % |
| `drive_arc` | `drive` | `task=arc` | drive_env.py | 94.5 % |
| `dock_box` | `drive` | `task=dock` | drive_env.py | 31.4 % (p90 50.8 mm) |
| `park_home` | `drive` | `task=park` | drive_env.py | 87.5 % |

이름 (11-03 바꿈 — `upright`·`right`·`stand`가 헷갈려서):
- `pick_standing_puck` (예전 `pick_upright`): **서 있는** 퍽을 집기 — `--task pick_standing`, 팀원 `pick_env.py`
- `stand_lying_puck` (예전 `stand_up`): **누운** 퍽을 세우기 — `--task stand_lying`, 팀원 `right_env.py` (right a puck = 세우다)
- `regrasp_after_stand` (예전 `regrasp_stood`): 세운 바로 그 자리에서 다시 쥐기 — `stand_lying` + `regrasp=1`
- `stand_puck_in_cell` (예전 `stand_in_place`): 칸 안에서 누운 퍽 세우기 (환경 아직 없음)
- 예전 `--task` 이름(`pick` · `stand` · `right`)도 그대로 됨

아직 환경이 없는 것: `nudge_cell`·`stand_puck_in_cell`(기준 동작부터 설계 필요), `navigate`(Nav2와 장애물 장면 필요).

- **camera_look_down 높이**: `plan.yaml` coordinator `inspect_height_m = 0.15`는 시작 박스(70 mm) 위 퍽에서 손끝 아래 방향으로 닿지 않는다(pinocchio IK: 박스 가운데 ≤ 0.10 m, 박스 전체 0.08 m). 학습은 0.08 m, 카메라 기울기 25°까지 허용.
- **보상 틀** (환경마다 같은 모양): 걸음마다 목표 추종 + · 급한 동작 − · 잔차 크기 − / 끝에 성공 +10 · 최종 오차 + / 떨굼·넘어짐·충돌 −5. 가중치는 각 `*EnvCfg`의 `w_*`.
