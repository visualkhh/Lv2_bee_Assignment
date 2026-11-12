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
| `--no-policy` | `pick_load_demo.sh` | 그 skill 정책을 끄고 기준 동작만. `pick_standing_puck,load_slot` (이름 = `models/skills/` 폴더) · `all` |
| `--no-twist-state` | `pick_load_demo.sh` | `/joint_states`에 관절 비틀림을 더하지 않음 (엔코더 각 그대로). 기본은 더함 = TF가 실제 팔 (`twist_state`) |
| `--storage` | `pick_load_play.py` | 적재함 칸 2개를 싣는 순서대로. 없으면 `next` |
| `--self-test` | `pick_load_play.py` | ROS 없이 계산만 확인 |

누운 퍽은 `mission_manager`와 같이 `stand` 후 `stand_pick`으로 집는다.

## 전체 미션

`scene_sim.usda` 장면과 전체 ROS 시스템(`isaac_system.launch.py`)을 띄운다.

```bash
isaac/scripts/standalone/isaac_sim.sh
```
