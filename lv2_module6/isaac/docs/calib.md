[목차](README.md) · [◀ USD](usd.md)

# 실기 측정 · 시험 반복

이 문서의 경로·명령은 `loops/` 기준이다. 실기와 Isaac Sim(`use_sim:=true`) 모두 같은 스크립트를 쓴다. `logs/`는 git 제외라 로컬에만 있다.

| 파일 | 역할 |
|---|---|
| `manipulator-launcher.sh` | 팔·카메라·인지·MoveIt·skill 노드 bringup (SLAM · Nav2 없음). `use_sim:=true`면 Isaac 대상 |
| `manipulator-record.sh` | 팔 전용 rosbag 기록 → `logs/bag_<시각>/` |
| `pick_loop.py` · `place_loop.py` · `stand_loop.py` | 집기 · 놓기 · 세우기 반복 시험, 결과는 `logs/*.csv` |
| `arm_model.py` | URDF 기구학·중력 토크 (ROS 쪽 `rl_arm_correction`의 기준값) |
| `ros2_ws/src/sine_test` (빌드: `cd ros2_ws && colcon build`) | 사인·프로파일 응답 측정용 ROS 패키지 (`hw.launch.py`) |

### stand_loop — 세우기 반복

같은 자리에서 `stand` skill을 반복한다. 가짜 누운 퍽을 `/tracked_objects`로 내므로 인지 노드는 꺼야 한다.

```bash
source ../../../ros2_ws/install/setup.bash
ros2 launch bringup stand_test.launch.py                 # 터미널 1
pkill -f perception_node                                 # 터미널 2: launch가 뜬 뒤
python3 stand_loop.py --x 0.22 --y 0.0 --preview         # 먼저 손 위치만 확인
python3 stand_loop.py --x 0.22 --y 0.0 --rounds 0        # 0 = Ctrl+C까지
```

라운드 사이 `--pause`(기본 6초) 동안 표시한 자리에 퍽을 다시 눕혀 둔다.

### pick_loop — 집기 오차

stow → inspect box → pick을 실제 인식(YOLO)으로 반복하고, 라운드마다 인식 오차와 파지 오차를 `logs/pick_loop_<시각>.csv`에 남긴다.
`manipulator-launcher.sh`는 `stand_test.launch.py` 스택(팔·카메라 2대·인지·MoveIt·coordinator·skill 노드, SLAM·Nav2 없음)을 띄운다.

```bash
./manipulator-launcher.sh perception_device:=cuda                              # 터미널 1
python3 pick_loop.py --box-x 0.22 --box-y 0.0 --rounds 0                       # 터미널 2 (0 = q · Ctrl+C까지)
./manipulator-record.sh                                                        # 터미널 3 (선택): 팔 전용 bag → logs/bag_<시각>/ (nocam = 영상 뺌)
```

- 상자 가운데를 로봇 중심(base_footprint)에서 `--box-x`·`--box-y` 자리에 둔다. 퍽은 로봇 중심 0.30 m 안이어야 한다.
- 인식 뒤 스크립트가 퍽 위치(로봇 기준 mm)를 보여 주면, 자로 잰 퍽 가운데 `x y`를 넣는다 → `perc_err_mm`.
- pick 중 손 카메라가 마지막으로 본 퍽 − 손끝(`/hand_relative`)을 `hand_rel_*`로 남기고, 쥔 뒤 손 안 어긋남(닫힘 방향 · 손가락 방향 mm)을 넣는다.
- 정책 켜고 끄기는 여기서 하지 않는다: `models/skills/pick_standing_puck/meta.yaml`의 `use:` + `colcon build`.

### place_loop — 놓기 오차

퍽을 손에 쥔 상태에서 `/skills/place`로 정한 자리(`--to x y z`, base_footprint 기준)에 놓고, 자로 잰 위치와 서 있는지를 `logs/place_loop_<시각>.csv`에 남긴다. 먼저 `./manipulator-launcher.sh`를 띄운다.

```bash
python3 place_loop.py --from slot --slot 0 --shape box box cyl --to 0.22 -0.05 0.07   # 적재함 슬롯에서 꺼내서
python3 place_loop.py --from pick --to 0.22 0.05 0.07                                # 상자에서 집어서
```

- 팔 동작 bag은 라운드마다 `logs/place_loop_<시각>_bags/`에 남는다 (`--bag off`로 끔, `--auto` = 전체 한 개).
- 정책 켜고 끄기는 `models/skills/<skill>/meta.yaml`의 `use:` + `colcon build`.

### Isaac Sim 대상

`isaac_sim.sh`를 먼저 띄운 뒤 `./manipulator-launcher.sh use_sim:=true`, 스크립트는 `--use_sim true`(또는 launcher와 같은 `ROS_DOMAIN_ID`)로 실행한다.
