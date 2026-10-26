# ManualActors — Gazebo GUI 수동 액터 버튼

Gazebo GUI에 **사람 소환 / 박스 장애물 소환 / 수동 모델 전체 삭제** 버튼을 붙이는 작은 GUI 플러그인입니다.

## 동작

- **사람/박스/퍽/신호등**: `/tmp/beebot_scene.json`의 `dynamic.spawn_points` 중 빈 지점에 바로 생성
- **수동 사람**: 타이머로 주변 빈 지점을 향해 계속 이동합니다.
- **퍽 모델**: `BEEBOT_SPAWN_DIR`의 `puck_red.sdf`·`puck_orange.sdf`·`puck_green.sdf`를 사용하며, 6면 AprilTag가 포함됩니다.
- **신호등 모델**: 같은 디렉터리의 `traffic_light.sdf`를 사용합니다. 수동 모델은 빨강 렌즈가 켜진 정적 모델이며, 자동 점등 주기는 `traffic_lights.py`가 관리하는 생성 신호등에만 적용됩니다.
- **수동 모델 전체 삭제**: 이름이 `manual_`로 시작하는 모델을 모두 `remove`
- **환경 탭**: `BEEBOT_ENV_MODEL_DIR`의 전체 바닥/벽/파노라마 모델을 `remove`/`create`로 교체
- **임의 위치에 직접 배치하고 싶을 때**: Gazebo 기본 **Spawn** 기능(SDF 드래그/드롭, Spawn 플러그인)을 사용
- 수동으로 소환한 모델은 `dynamic_obstacles.py`의 자동 수명/정리 대상이 **아닙니다**. 삭제 버튼으로 정리하세요.

## 빌드

```bash
lv2_module6/develop/gazebo/gui_plugin/build.sh
```

`start.sh`는 `build/libManualActors.so`가 없으면 자동으로 빌드하고, `GZ_GUI_PLUGIN_PATH`에 넣은 뒤
`gazebo/gui.config`를 `gz sim --gui-config`로 전달합니다. QML만 고친 경우에는 `build.sh`를 다시 실행하세요.

퍽/신호등 SDF는 `generate_world.py`가 `BEEBOT_SPAWN_DIR`(기본 `/tmp/beebot_spawn_models`)에 매 시작마다 생성합니다.

## 구조

| 파일 | 역할 |
| --- | --- |
| `ManualActors.cc` / `.hh` | gz-gui `Plugin` + gz-transport create/remove |
| `ManualActors.qml` | 버튼 패널 UI |
| `CMakeLists.txt` | Qt6 + gz-gui/gz-transport/gz-msgs/gz-plugin 빌드 |
| `build.sh` | 빌드 편의 스크립트 |
| `../gui.config` | Gazebo 기본 GUI 설정 + 이 플러그인 등록 |

## 플러그인 원리

Gazebo GUI 플러그인은 **`gz::gui::Plugin`을 상속한 공유 라이브러리**와 같은 이름의 QML 파일로 구성됩니다.

- `GZ_ADD_PLUGIN(beebot::ManualActors, gz::gui::Plugin)`로 등록
- `_ManualActors`라는 QML context property로 C++의 `Q_INVOKABLE` 메서드 호출
- 소환은 gz-transport의 `/world/default/create` 서비스, 삭제는 `/world/default/remove` 서비스 사용
- 버튼을 누르면 `beebot_scene.json`의 미션/장애물과 겹치지 않는 후보 지점에 바로 생성

Gazebo 기본 GUI 설정을 그대로 쓰면서 플러그인만 추가하려면 `gui.config`처럼 `--gui-config`에 기본 설정 + 새 `<plugin>`을 넣은 파일을 넘기면 됩니다.
