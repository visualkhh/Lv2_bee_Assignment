# docker SIL 뷰어 자산

`cli/ros/docker/assets/`는 도커 모드 SIL 뷰어(`simulators/vazebo/simple.py`)가 **런타임에 직접 쓰는 사본**입니다.
`lv2_module6/isaac`·`lv2_module6/simulators` 원본을 실행 중에 참조하지 않습니다.

| 파일 | 원본 | 만든 방법 |
|---|---|---|
| `course/dice_maze/walls.json` | `lv2_module6/course/dice_maze/walls.json` | Isaac USD에서 `tools/course/build_map_usd.py`로 만든 코스 벽 데이터 |
| `dice/q1.json` | `lv2_module6/simulators/vazebo/dice/q1.json` | 주사위 미션 문제 |
| `background/isaac_class_room.jpg` | `lv2_module6/isaac/source/HDRI/class_room_03_4k.exr` | 2048×1024, Reinhard 톤매핑 + JPEG |
| `floor/isaac_class_room_floor.jpg` | `lv2_module6/isaac/source/texture/class_room_floor/class_room_floor_albedo.png` | 1024×1024 JPEG |
| `wall/wall_painted.jpg` | Poly Haven CC0 (`simulators/vazebo/images/wall/`) | 그대로 복사 |
| `wall/WALL_CREDITS.txt` | 위와 같음 | 그대로 복사 |

- 렌더러(`simulators/vazebo`)는 USDA를 직접 읽지 않습니다. 코스 벽은 Isaac USD에서 파생한 `walls.json`, 배경/바닥은 Isaac 텍스처를 변환한 사본을 씁니다.
- 원본을 갱신하면 이 폴더의 사본도 다시 만들어 커밋합니다. **런타임 경로를 원본으로 돌리지 않습니다.**
- `BEEBOT_SIM_ASSETS`가 이 폴더를 가리키면 `simple.py`가 여기서 읽고, 없으면 개발용으로 `simulators/vazebo/images`·`course`로 폴백합니다.
