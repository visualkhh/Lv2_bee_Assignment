[목차](README.md) · [학습 ▶](lab.md)

# 시연

Isaac Sim이 beebot 하드웨어 역할을 하고, 실기와 같은 ROS 노드가 로봇을 움직인다. 명령은 모두 `lv2_module6`에서 실행한다.

## 시연 목록

| 시연 | 장면 + ROS | 임무 |
|---|---|---|
| [퍽 집어 적재](#퍽-집어-적재) | `pick_load_demo.sh` | `pick_load_play.py` |
| [전체 미션](#전체-미션) | `isaac_sim.sh` | `mission_manager` |

## 퍽 집어 적재

바닥, 시작 상자 1개, 퍽 2개, 상자에서 30 cm 떨어진 로봇으로 시작한다. 기둥 카메라로 두 퍽을 찾아 상자 앞으로 접근한 뒤, 퍽마다 손 카메라로 분류(inspect) → 집기(pick) → 적재(load)를 한다.

1. 장면과 ROS 노드를 띄운다.

   ```bash
   isaac/scripts/demo/pick_load_demo.sh
   ```

2. Isaac 창이 뜨면 새 터미널에서 임무를 시작한다. `ROS_DOMAIN_ID`는 1번 터미널 값을 물려받으니 같게 두고, `FASTDDS_DEFAULT_PROFILES_FILE`은 `ros2_ws/fastdds_local_only.xml`이어야 토픽이 보인다 (`isaac_sim.sh`가 쓰는 파일).

   ```bash
   source ros2_ws/install/setup.bash && python3 -u isaac/scripts/demo/pick_load_play.py --clear-basket
   ```

3. 다시 돌릴 때는 먼저 처음 자세로 되돌린 뒤 2번을 실행한다. ROS 노드도 재시작된다.

   ```bash
   isaac/scripts/standalone/reset.sh
   ```

`--clear-basket`은 적재함 장부를 비운다. 적재함이 실제로 비어 있을 때만 붙인다.

### 인자

```bash
## demo 씬 생성
### --puck:모양:색상:태그id
isaac/scripts/demo/pick_load_demo.sh --puck red:cyl:stand:3 --puck green:box:lie:7 --seed 42

## demo 실행
### basket 비우기, 적재 슬롯
python3 -u isaac/scripts/demo/pick_load_play.py --clear-basket --storage 0,1
```

| 인자 | 받는 곳 | 형식 |
|---|---|---|
| `--puck` | `pick_load_demo.sh` | `색:모양:상태:태그`, 두 번. `red\|green`, `box\|cyl`, `stand\|lie`, `0~11`. 뒤쪽·빈 항목은 seed로 정한다 (`green::lie`) |
| `--seed` | `pick_load_demo.sh` | 비운 항목과 퍽 방향을 정한다. 자리는 두 곳 고정 |
| `--headless` | `pick_load_demo.sh` | 창 없이 실행 |
| `--no-play` | `pick_load_demo.sh` | Play를 자동으로 누르지 않음 |
| `--no-policy` | `pick_load_demo.sh` | 그 skill 정책을 끄고 기준 동작만. `pick_standing_puck,load_slot` (이름 = `ros2_ws/model/skills/` 폴더) · `all` |
| `--no-twist-state` | `pick_load_demo.sh` | `/joint_states`에 관절 비틀림을 더하지 않음 (엔코더 각 그대로). 기본은 더함 = TF가 실제 팔 (`twist_state`) |
| `--storage` | `pick_load_play.py` | 적재함 칸 2개를 싣는 순서대로. 없으면 `next` |
| `--self-test` | `pick_load_play.py` | ROS 없이 계산만 확인 |

누운 퍽은 `mission_manager`와 같이 `stand` 후 `stand_pick`으로 집는다.

## pick 성공률 시험

정책을 끄고, 접근 없이 `inspect`에서 시작해 인식은 Isaac 참값으로 pick을 잰다(`--no-inspect`로 inspect를 뺄 수 있음). 시행마다 상자를 차체 앞 2~12 cm에(y ±1.5 cm · yaw ±4°), 서 있는 퍽 2개를 상자 위 아무 데나(서로 6 cm 이상, 가장자리 3.5 cm 안쪽) 새로 뽑는다. 퍽 모양·색·yaw·질량(×0.8~1.2)·마찰(×0.7~1.3)도 무작위다. 차체는 고정이다.

```bash
isaac/scripts/demo/pick_load_demo.sh --random-layout --trial-seed 0 --no-policy pick_standing_puck --truth-perception
```

```bash
source ros2_ws/install/setup.bash && python3 -u isaac/scripts/demo/pick_test.py --trials 30 --seed 0
```

- `--truth-perception`은 YOLO `perception_node` 대신 `standalone/truth_perception.py`가 참값(`/tmp/beebot_slot_pucks.json`)으로 `/tracked_objects`를 낸다. 팀 launch를 고치지 않으려고 YOLO 노드를 뜬 뒤 60초 동안 1초마다 `pkill -9`로 끈다(한 번으로는 모델 로딩 중이면 안 죽어서 더미 객체와 YOLO 트랙이 섞였다). 오차는 `--truth-noise "<옵션>"`으로 얹는다(기본 0 = 참값). 편향은 퍽·시행(seed)마다 한 번 뽑고(같은 seed면 같음) 잡음은 프레임마다 뽑는다.

  | 옵션 | 뜻 |
  |---|---|
  | `--bias-mm` | 위치 편향 σ [mm], x·y 각각 |
  | `--bias-x-mm` | 모든 퍽 x 평균 이동 [mm] (기둥 카메라 depth 축 편향) |
  | `--jitter-mm` | 프레임 위치 잡음 σ [mm] |
  | `--yaw-bias-deg`, `--yaw-jitter-deg` | yaw 편향·프레임 잡음 σ [deg] |

  예: `pick_load_demo.sh --random-layout --no-policy pick_standing_puck --truth-perception --truth-noise "--bias-mm 5 --jitter-mm 1"`
- 시행마다 `inspect` → `pick` 한 번. 대상은 닿는 퍽(base_footprint에서 0.30 m 이내) 중 seed로 고른 하나다. 둘 다 밖이면 '도달 불가'로 세고 성공률에서 뺀다.
- 성공 판정은 Isaac 퍽 높이다: 상자 윗면에서 20 mm 이상 들리고 손 안(닫힘 방향 25 mm · 손가락 방향 40 mm 이내). skill의 성공 보고와 다르면 `pick_false_ok`.
- 결과 CSV(`/tmp/pick_test_*.csv`)에 손가락 끝면의 퍽 바닥 위 높이를 남긴다. 설계값은 40 mm(퍽 중앙 30 mm, 윗면 60 mm)다.
- 시행 사이에는 `reset.sh`가 장면과 ROS 노드를 다시 띄운다. 다음 seed는 `/tmp/beebot_trial_seed`, 현재 배치는 `/tmp/beebot_trial.json`.

## inspect 인식 시험

팔을 정한 자세로 시작하고 ROS는 `perception_node` · 카메라 어댑터 · 고정 자세 TF만 띄워(`standalone/isaac_perception_only.launch.py`) gripper_cam 한 장에서 퍽 태그를 몇 개 읽는지 잰다. 배치는 pick 시험과 같다(`--random-layout`).

```bash
isaac/scripts/demo/pick_load_demo.sh --headless --fix-base --random-layout --trial-seed 1999 --perception-only cover
```

```bash
source ros2_ws/install/setup.bash && export FASTDDS_DEFAULT_PROFILES_FILE=$PWD/ros2_ws/fastdds_local_only.xml && python3 -u isaac/scripts/demo/inspect_test.py --modes static --trials 20 --seed 2000 --save-images ~/inspect_img
```

- `--perception-only <자세>`: `0,-96.4,18.6,110.4,0`처럼 joint1~5 [deg] · `box` = 상자 윗면 가운데를 보는 자세(`tools/inspect_pose.py look_at`) · `cover` = 간격별로 상자 위를 가장 많이 덮는 자세(`tools/inspect_cover.py --table` → `tools/inspect_cover_table.json`).
- 지금 자세는 `/tmp/beebot_arm_q`에 남고 launch의 `joint_state_publisher`가 그대로 낸다. perception 파라미터 덮어쓰기는 `/tmp/beebot_perception_params`에 `tag_gamma:=0.0 body_box:=[0.0]`처럼 적는다(리셋마다 다시 읽음).
- `--save-images`: 시행마다 디버그 영상. 버린 퍽 박스 아래에 빨간 `drop: <이유>`가 찍힌다(`self` = 자기 몸 필터, `no depth` · `크기` 등 = 퍽 판단).
- `tools/inspect_pose.py`의 FK는 카메라 위치가 y로 약 3 cm 어긋나 있다(URDF FK와 비교, 화면에서 약 70 px). 자세를 새로 구할 때는 URDF를 그대로 쓰는 `inspect_cover.py`를 쓴다.

## 전체 미션

`scene_sim.usda` 장면과 전체 ROS 시스템(`isaac_system.launch.py`)을 띄운다.

```bash
isaac/scripts/standalone/isaac_sim.sh
```
