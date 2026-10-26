# Gazebo 개발 도구 README

이 디렉터리는 호스트에서 Gazebo를 띄우고, 미션 월드·퍽·AprilTag·신호등·동적 사람·수동 액터를 관리하는 스크립트 모음입니다.

```text
gazebo/
├── start.sh                 # 클린 스타트: 월드 생성 + Gazebo + 브리지/릴레이 + 런타임
├── kill.sh                  # Gazebo·브리지·릴레이·신호등·동적 장애물 종료
├── restart.sh               # kill → start
├── reset.sh                 # 로봇 위치 초기화
├── generate_world.py        # 미션/퍽/태그/신호등/사람 월드 생성
├── generate_tags.py         # 태그36h11 PNG/OBJ/MTL 생성
├── publish_poses.py         # 실제 pose / RViz 마커 / /gz_mission/info
├── relay.py                 # Gazebo 카메라·IMU → 표준 ROS 토픽
├── traffic_lights.py        # 신호등 주기 런타임
├── dynamic_obstacles.py     # 이동 중에만 나타나는 사람 장애물
├── bridge.yaml              # ros_gz_bridge 설정
├── gui.config               # Gazebo GUI 설정 + 수동 액터 버튼 등록
├── gui_plugin/              # Gazebo GUI 플러그인 (ManualActors)
└── assets/apriltags/        # 생성된 태그 자산
```

## 빠른 실행

```bash
# ROS 2 환경을 source한 뒤
lv2_module6/develop/gazebo/start.sh

# 종료
lv2_module6/develop/gazebo/kill.sh

# 로봇만 원위치
lv2_module6/develop/gazebo/reset.sh
```

`start.sh`는 기존 Gazebo를 정리하고, 매번 월드를 새로 생성한 뒤 Gazebo를 띄웁니다.

- 미션: `SORT_COLOR`, `MOVE_TO_B`, `STAND_UP`, `LAY_DOWN`이 지도에 분산
- 퍽: 9개 기본, `30×30×60 mm`, 6면 AprilTag
- 신호등: 상단 방 통행로 위 갠트리 2개
- 동적 사람: 로봇 이동 중에만 생성, 정지 시 제거
- 수동 액터: Gazebo GUI 버튼으로 사람/박스 소환·삭제

---

# 수동 액터 GUI 플러그인

Gazebo GUI에 다음 버튼 패널을 붙입니다.

| 버튼 | 동작 |
| --- | --- |
| **사람 소환** | `/tmp/beebot_scene.json`의 빈 후보 지점에 사람형 모델 생성 |
| **박스 장애물 소환** | 같은 빈 후보 지점에 박스 모델 생성 |
| **빨강/주황/초록 퍽** | 6면 AprilTag가 붙은 퍽 생성 |
| **가로형 신호등 소환** | 갠트리형 신호등 생성 (수동 모델, 자동 점등 런타임 미연결) |
| **수동 모델 전체 삭제** | 이름이 `manual_`로 시작하는 모델을 모두 제거 |

**마우스 오른쪽 클릭**으로 3D 화면의 원하는 지점에 바로 소환할 수도 있습니다.
사람·박스·퍽·신호등 메뉴가 뜨고, 클릭한 지점의 x/y 위치에 생성됩니다.
임의의 다른 SDF 모델은 Gazebo 기본 **Spawn / SDF 드래그 앤 드롭**을 사용합니다.

## 어떻게 연결되는가

```text
start.sh
  │
  ├─ GZ_GUI_PLUGIN_PATH = gazebo/gui_plugin/build
  │
  └─ gz sim --gui-config gazebo/gui.config ...
          │
          └─ gui.config 안의:
             <plugin filename="ManualActors" name="Manual Actors">
                     │
                     └─ libManualActors.so
                          ├─ C++ : gz::gui::Plugin
                          └─ QML  : 버튼 화면
```

플러그인은 **월드 시스템 플러그인이 아니라 GUI 플러그인**입니다.

- `start.sh:37` — `gui_plugin/build/libManualActors.so`가 없으면 `build.sh`로 자동 빌드
- `start.sh:42` — `GZ_GUI_PLUGIN_PATH`에 플러그인 디렉터리 추가
- `start.sh:58` — `gz sim --gui-config gui.config`로 GUI 설정 전달
- `gui.config:255` — `<plugin filename="ManualActors">` 등록

## 어떻게 만드는가

Gazebo GUI 플러그인은 **C++ 공유 라이브러리 + QML UI**입니다.

### 1. C++ 클래스

`gui_plugin/ManualActors.hh`

```cpp
class ManualActors : public gz::gui::Plugin
{
  Q_OBJECT
public:
  Q_INVOKABLE void SpawnHuman();
  Q_INVOKABLE void SpawnBox();
  Q_INVOKABLE void ClearManualModels();
};
```

`Q_INVOKABLE`로 선언한 함수만 QML에서 호출할 수 있습니다.

### 2. C++ 구현

`gui_plugin/ManualActors.cc`

- 사람/박스: `gz::transport::Node`로 `/world/default/create` 서비스 호출
- 삭제: `/world/default/scene/info`에서 `manual_` 모델을 찾아 `/world/default/remove` 호출
- 등록: `GZ_ADD_PLUGIN(beebot::ManualActors, gz::gui::Plugin)`

### 3. QML 버튼

`gui_plugin/ManualActors.qml`

```qml
Button {
  text: "사람 소환"
  onClicked: _ManualActors.SpawnHuman()
}
```

플러그인 파일명이 `ManualActors`이므로 Gazebo가 QML에 `_ManualActors` context property를 자동으로 넣습니다.

### 4. CMake

`gui_plugin/CMakeLists.txt`

- `gz-gui`, `gz-transport`, `gz-msgs`, `gz-plugin-register`, Qt6 링크
- `ManualActors.qml`을 `:/ManualActors/ManualActors.qml` 리소스로 포함
- `libManualActors.so` 생성

### 5. 빌드

```bash
lv2_module6/develop/gazebo/gui_plugin/build.sh
```

결과:

```text
lv2_module6/develop/gazebo/gui_plugin/build/libManualActors.so
```

`start.sh`는 `.so`가 없을 때만 자동 빌드합니다. **C++/QML을 수정한 뒤에는 `build.sh`를 다시 실행**해야 합니다.

## 새 버튼 추가 방법

1. `ManualActors.hh`에 `Q_INVOKABLE` 함수 추가
2. `ManualActors.cc`에 서비스 호출/모델 생성 구현
3. `ManualActors.qml`에 `Button { ... }` 추가
4. `./build.sh`
5. `start.sh`로 Gazebo 재시작

## 문제 확인

패널이 안 보이면:

```bash
# 1. .so가 있는지
ls -l lv2_module6/develop/gazebo/gui_plugin/build/libManualActors.so

# 2. Gazebo가 GUI 설정을 받았는지
pgrep -af gz-sim-gui-client

# 3. 플러그인 직접 확인
gz plugin -i -p lv2_module6/develop/gazebo/gui_plugin/build/libManualActors.so
```

직접 실행할 때도 GUI 설정과 플러그인 경로가 필요합니다.

```bash
export GZ_GUI_PLUGIN_PATH="$PWD/lv2_module6/develop/gazebo/gui_plugin/build"
gz sim --gui-config lv2_module6/develop/gazebo/gui.config /tmp/beebot_world.sdf
```

## 주의

- 수동 소환 모델은 `dynamic_obstacles.py`의 자동 수명/정리 대상이 **아닙니다**.
- 정리하려면 **수동 모델 전체 삭제** 버튼을 누르거나 Gazebo Entity Tree에서 삭제하세요.
- 모델을 생성하는 서비스는 Gazebo가 완전히 뜬 뒤에만 동작합니다.

---

# 카메라·점구름 성능

현재 Gazebo RGBD 설정은 RealSense D435 운용 모드에 맞춰져 있습니다.

- 해상도: **640×360**
- 갱신: **15 Hz**
- 적용 위치: `lv2_module6/urdf/default/urdf.xacro:579`
- `body_filter`는 `stride: 2`로 점구름을 솎아 Nav2/collision_monitor에 전달
  - 설정: `lv2_module6/ros2_ws/src/perception/config/perception.yaml (`body_filter:`)`
  - 구현: `lv2_module6/ros2_ws/src/perception/src/body_filter.cpp`

더 줄이고 싶으면:

1. `body_filter`의 `stride`를 3~4로 올리기
2. `urdf.xacro`의 RGBD `update_rate`/해상도 낮추기
3. `ros_system`을 `colcon build --symlink-install --packages-select ros_system`로 재설치

자세한 플러그인 설명은 [`gui_plugin/README.md`](gui_plugin/README.md)를 참고하세요.

---

# 환경 텍스처 선택

`../../vazebo/images`의 Poly Haven CC0 파노라마·바닥·벽 텍스처를 Gazebo 쪽
`assets/environment/`로 복사했습니다. 원본 크레딧도 함께 들어 있습니다.

```text
assets/environment/
├── background/   # pano_*.jpg (4096×2048 equirectangular)
├── floor/        # floor_*.jpg (1024×1024, tileable)
└── wall/         # wall_*.jpg (1024×1024, tileable)
```

## Gazebo GUI에서 선택

`수동 액터` 패널의 **환경** 탭에서 바닥·벽·파노라마를 고른 뒤 **환경 적용**을 누르면 즉시 현재 월드 모델을 교체합니다.

- 최초 기본값: **none** (기존 단색 바닥/벽, 파노라마 없음)
- 적용은 현재 세션에만 반영됩니다.
- 시작 시 기본값을 바꾸려면 아래 환경변수를 사용합니다.

## 시작할 때 선택

Gazebo 시작 전에 환경변수를 지정합니다. `none`이면 기존 단색 재질을 그대로 씁니다.

```bash
GZ_FLOOR_TEXTURE=floor_wood \
GZ_WALL_TEXTURE=wall_painted \
GZ_BACKGROUND=pano_wooden_lounge \
lv2_module6/develop/gazebo/start.sh
```

값은 파일명 또는 stem(`floor_wood`, `pano_wooden_lounge`)으로 줄 수 있습니다.

| 변수 | 가능한 값 |
| --- | --- |
| `GZ_FLOOR_TEXTURE` | `floor_concrete`, `floor_tiles`, `floor_wood`, `floor_garage`, `none` |
| `GZ_WALL_TEXTURE` | `wall_concrete`, `wall_painted`, `none` |
| `GZ_BACKGROUND` | `pano_hospital_room`, `pano_industrial_wooden_attic`, `pano_lebombo`, `pano_photo_studio_loft_hall`, `pano_unfinished_office`, `pano_vintage_measuring_lab`, `pano_wooden_lounge`, `none` |

예:

```bash
# 바닥만
GZ_FLOOR_TEXTURE=floor_concrete lv2_module6/develop/gazebo/start.sh

# 파노라마만
GZ_BACKGROUND=pano_unfinished_office lv2_module6/develop/gazebo/start.sh
```

## 구현 방식

- **GUI 적용**: `generate_world.py`가 모든 바닥·벽·파노라마 변형을 `/tmp/beebot_environment_models/`에 미리 만들고, GUI 플러그인이 `/world/default/remove` + `/world/default/create`로 현재 모델을 교체한다.
- **바닥**: 14×14 m plane mesh에 선택한 바닥 텍스처를 입힌다.
- **벽**: `walls.json`의 163개 박스를 OBJ mesh로 만들고, 벽 텍스처를 입힌다. 충돌은 기존과 같은 박스 collision으로 유지한다.
- **파노라마**: equirectangular 이미지를 큰 구(sphere) 안쪽 면에 매핑해 배경처럼 보여 준다. 별도 DDS 큐브맵 변환은 하지 않는다.
- 시작 시 선택한 OBJ/MTL은 `/tmp/beebot_environment/`에, GUI 선택용 전체 모델은 `/tmp/beebot_environment_models/`에 놓인다.
- 선택하지 않은 항목은 기존 Gazebo 월드의 재질/모델을 그대로 둔다.

> 벽/바닥은 Gazebo OGRE1에서 확실히 보이도록 OBJ/MTL 경로를 사용합니다.
> 파노라마는 Gazebo 기본 sky cubemap이 아니라 안쪽 면 sphere 방식입니다.
