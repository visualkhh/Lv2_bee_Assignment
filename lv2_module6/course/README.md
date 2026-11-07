# 대회 코스 — Twin Closed Maze

미로 4.464 × 6.364m, 벽 높이 0.39m. 가운데(y=0) 벽이 위·아래 두 미로로 완전히 나눈다 — 로봇 시작 칸(`spawn_xy_m`)이 있는 쪽만 오갈 수 있다.

| 파일 | 내용 |
| --- | --- |
| `twin_closed_maze/walls.json` | **우리가 쓰는 코스** — 기둥 83 + 벽판 80 상자 (세로 벽 포함), 크기 · 벽 높이 · 시작 위치. 시뮬레이터가 읽는다 |
| `twin_closed_maze/model.sdf` · `model.config` | 같은 상자로 만든 Gazebo 모델 (`model://twin_closed_maze`) |
| `worlds/twin_closed_maze.sdf` | Gazebo 월드 (원본 그대로 — 바닥 · 조명 · 위 모델 include) |
| `twin_closed_maze/original/` | 원본 그대로 (model.sdf · source.json · LICENSE · NOTICE) |
| `tools/build_walls.py` | 원본 메시(STL)에서 위 파일들을 다시 만든다 — 코스가 바뀔 때만 |

- 원본: [SeongminJaden/waffle-lyrical-slam](https://github.com/SeongminJaden/waffle-lyrical-slam/tree/7fc1ef486f3fc2513b06ca9bc76cd00f4df4fb03/src/waffle_slam_course) @ `7fc1ef4`, Apache-2.0 (`original/LICENSE`, `original/NOTICE`). 미로 형상의 출처는 `original/source.json`.
- **원본 `model.sdf`의 충돌 상자에는 세로 벽(바깥 테두리 · 안쪽 4개)이 빠져 있다** — 그대로 Gazebo에 쓰면 로봇이 세로 벽을 통과한다. 그래서 메시를 위에서 본 모양으로 기둥 사이 벽판을 판정해 다시 만들었다.
- 원본 메시(29MB STL)는 저장소에 넣지 않았다 — `build_walls.py`를 돌릴 때만 받는다 (`tools/.cache/`, gitignore).

## Isaac 학습 미로 — `isaac_maze`

Isaac Lab 장면(`isaac/usda/scene_sim.usda`)이 쓰는 미로 `isaac/usda/maze/maze.usda`를 faker 코스로 쓴다
(`course:=isaac_maze`). 미로 4.57 × 6.46m, 벽 0.39m, 바깥 테두리 + L자 벽 조각 9개.

| 파일 | 내용 |
| --- | --- |
| `tools/build_map_usd.py` | USD collision 메시를 위에서 본 모양 그대로 칠해 `faker/maps/isaac_maze.pgm·yaml`을 만든다. 원점 = scene_sim.usda의 beebot 위치(미로 (−0.185, −2.370)) |

- `walls.json`(축에 평행한 상자)이 없다 — 지도는 메시에서 바로 만든다. 그래서 호스트 시뮬레이터(vazebo)로는 아직 못 띄우고, faker headless(A/B 등)용이다.
- 바깥 테두리 collision에 빈 곳이 몇 군데 있다 (USD 그대로 — Isaac 물리도 같은 collision을 쓴다).
- 미로가 바뀌면: `pip install usd-core` 후 `python3 lv2_module6/course/tools/build_map_usd.py` → 지도 두 파일 커밋.
