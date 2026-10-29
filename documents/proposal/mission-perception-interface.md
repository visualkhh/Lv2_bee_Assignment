# 주사위 미션 — 인지(perception) ↔ 계획(plan) 인터페이스 제안

> **상태: 제안 (아직 구현 전).** 발제 「주사위 기억 로봇」(2026-11-04) 기준으로 인지가 무엇을 어디까지 만들어서 넘기고,
> 계획(plan)이 그걸 어떻게 받아 쓰는지 정한다. 합의되면 `.msg` 파일과 [`API.md`](../API.md)가 정본이 된다.
> 코드 예시는 "이렇게 만들면 된다"는 설명용이다 — 그대로 붙여 넣기 전에 각 패키지 구조에 맞춰 옮긴다.

관련: [이전 미션 문서](../mission/README.md) (주사위 미션으로 갱신됨) · [API.md](../API.md) · 이슈 #11 #20 #21 #24 #27

## 목차

1. [한 문장 요약](#1-한-문장-요약)
2. [왜 경계를 정해야 하나](#2-왜-경계를-정해야-하나)
3. [좌표계 — 경기장(arena) 프레임](#3-좌표계--경기장arena-프레임)
4. [인지가 내는 것 3가지](#4-인지가-내는-것-3가지)
5. [skill별로 받는 값](#5-skill별로-받는-값)
6. [판단이 갈리는 3곳 — 코드로 보기](#6-판단이-갈리는-3곳--코드로-보기)
7. [인지 쪽 신규 기능 — 코드 예시](#7-인지-쪽-신규-기능--코드-예시)
8. [정해 둔 값 (주사위 칸 · 적재함)](#8-정해-둔-값-주사위-칸--적재함)
9. [인지 작업 목록 (이슈 후보)](#9-인지-작업-목록-이슈-후보)
10. [합의 체크리스트](#10-합의-체크리스트)

---

## 1. 한 문장 요약

**인지는 "무엇이 · 어디에 · 어떤 속성으로 · 얼마나 확실하게 · 언제" 보였는지(사실)까지 만들고,
"그래서 어떻게 할지"(판단)는 계획이 한다.**

```text
 카메라 픽셀 · 점구름                   사실 (물체 단위)                         판단
┌──────────────────┐   인지가 함   ┌──────────────────────────┐  계획이 함  ┌────────────────────────┐
│ 검출 · 태그 해독  │ ──────────▶ │ 퍽 tag=7 · red · cylinder │ ─────────▶ │ 7번은 R 그룹 → L2 칸 4  │
│ HSV · 점멸 시간   │             │ map (1.42, 0.95) ±1cm     │            │ 놓은 뒤 오차 6mm → 성공 │
└──────────────────┘             │ 0.2초 전에 봄 (지금 보임)  │            └────────────────────────┘
                                 └──────────────────────────┘
```

## 2. 왜 경계를 정해야 하나

경계가 흐리면 같은 일을 두 번 하거나, 아무도 안 한다. 발제에는 경계를 정해 주는 규칙이 세 개 있다.

| 발제 규칙                                                           | 뜻하는 것                         | 그래서                                                                      |
| ------------------------------------------------------------------- | --------------------------------- | --------------------------------------------------------------------------- |
| "누락·불명 색은 **임의 추정 금지**" (p3)                     | 모르면 모른다고 해야 함           | 인지는 `unknown`을 그대로 낸다. 오류로 볼지는 계획이 미션 규칙으로 정한다 |
| "순서/횟수/바코드는 **실제 센서 관측 기록**으로 평가" (p7)    | 무엇을 봤는지 원본이 남아야 함    | 인지는 해석 전 사실(점멸 순서·시각, 바코드 원문)을 낸다                    |
| "목표 상태를 **그대로 복사한 로그는 재검증으로 불인정**" (p7) | 기억이 아니라 새로 본 것이어야 함 | 인지는 "지금 보이는지 / 기억만 하는지"를 구분해서 낸다                      |

**예외 하나 — 집기(pick).** 손가락이 퍽을 잡는 자세는 mm 단위 형상이 필요해서, 물체 단위로 줄이면 정보가 사라진다.
그래서 지금 구조 그대로 ML(`grasp_scorer`)이 gripper_cam 점구름을 직접 보고 파지 자세를 낸다.

## 3. 좌표계 — 경기장(arena) 프레임

발제의 좌표(L1~L3, 시작 박스, 관측 위치)는 전부 **경기장 좌표**(왼쪽 아래 기둥 = 원점, mm, 지도 위쪽 = +Y)다.
RTAB-Map의 `map`은 **로봇이 켜진 자리**가 원점이다. 둘을 잇는 TF `arena → map`을 하나 둔다.

- 운영 규칙: 로봇은 늘 **출발 관측 위치 (734, 950) mm에서 −X를 보게(yaw = π)** 놓고 켠다.
- 값은 한 곳: `mission.yaml`의 `start_pose_arena: [0.734, 0.950, 3.14159]` (m, rad).
  현장에서 바꿀 땐 launch 인자 `start_pose:=0.740,0.945,3.12` 또는 실행 중 `ros2 param set`.
- 인지는 지금처럼 `map`으로 낸다. **arena로 바꾸는 건 계획**이 한다 (인지는 경기장 배치를 몰라도 된다).

```text
arena (경기장 원점, 기둥)                map (로봇 시작 자리 = 출발 관측 위치)
  ┌───────────────────────────┐             ▲ x_map (로봇 앞 = arena −X)
  │ +Y                        │             │
  │ ▲      L2 (2.202, 3.700)  │      y_map ◀─┘ (로봇 왼쪽 = arena −Y)
  │ │                         │
  │ │  ◀── 로봇 (0.734, 0.950) yaw π
  │ └──────▶ +X               │
  └───────────────────────────┘
```

```python
# arena 점 → map 점 (계획이 쓰는 변환. TF로 하면 이 계산을 tf2가 해 준다)
import math
def arena_to_map(xa, ya, start=(0.734, 0.950, math.pi)):
    x0, y0, yaw0 = start
    dx, dy = xa - x0, ya - y0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    return c * dx - s * dy, s * dx + c * dy

# L1 (3.670, 0.950)은 로봇 정면 반대쪽(+X) 2.936 m → map에선 x = −2.936
assert all(abs(a - b) < 1e-9 for a, b in zip(arena_to_map(3.670, 0.950), (-2.936, 0.0)))
```

## 4. 인지가 내는 것 3가지

| 토픽                 | 타입                                                            | QoS                        | 무엇                                                   |
| -------------------- | --------------------------------------------------------------- | -------------------------- | ------------------------------------------------------ |
| `/tracked_objects` | `perception/TrackedObjectArray` (구조 그대로)                 | best effort, 5~10 Hz       | 지금까지 본 물체 전부 — 퍽·장소 태그·신호등·바코드 |
| `/mission/signal`  | `perception/MissionSignal` (**새로 정의**)         | reliable + transient_local | 신호등 점멸 관측 결과                                  |
| `/mission/barcode` | `std_msgs/String` (**이름만 `/mission/qr`에서 변경**) | reliable + transient_local | 바코드 원문 (`P2-01` …)                             |

### 4-1. `/tracked_objects` — 구조는 그대로, 퍽 속성의 **뜻**이 바뀐다

```text
# 퍽 하나 (ros2 topic echo 모양으로)
id: 23                          # 추적 번호 — 인지 내부 번호, 바뀔 수 있음
class_name: puck
pose: {pose: {position: {x: -1.42, y: 0.31, z: 0.10}}, covariance: [...]}   # map, 공분산 포함
last_seen: {sec: 1793094000, nanosec: 0}
cameras: [gripper_cam]          # 지금 보고 있는 카메라. 빈 배열 = 기억만 하는 중
color: red                      # ★ HSV로 본 색만 — red | green | unknown
details:
  - {key: tag,   value: '7'}         # 퍽 ID 0~11 (윗면 AprilTag). 아직 못 읽었으면 이 줄이 없음
  - {key: shape, value: cylinder}    # ★ 신규: cylinder | cuboid | unknown
  - {key: posture, value: standing}
```

| 필드              | 지금                                                               | 바뀜                                     | 이유                                           |
| ----------------- | ------------------------------------------------------------------ | --------------------------------------- | ---------------------------------------------- |
| `color`         | 태그를 읽으면 **태그 번호대 색으로 덮어씀** (0~3 → 빨강 …) | **HSV로 본 색만**                 | 발제 p5 "ID 0~11은 색상을 영구 지정하지 않음"  |
| `details.shape` | 없음                                                               | `cylinder` / `cuboid` / `unknown` | Y 그룹은 한 모양으로 통일 (색·모양 선택 18점) |
| `details.tag`   | 0~11                                                               | 그대로 — **퍽 ID**               | 할당·재검증의 열쇠                            |
| `id`            | 추적 번호                                                          | 그대로                                  | 계획은 퍽을 **tag로** 부른다 (아래 6절)  |

장소 태그(`class_name: location`, tag 12~14)는 **방향(orientation)까지** 낸다 — 지금은 `orientation.w = 1.0` 고정
([`perception_node.py:420`](../../lv2_module6/ros2_ws/src/perception/scripts/perception_node.py)). 7-1절.

### 4-2. `/mission/signal` — 주사위 미션 형식

```text
# perception/msg/MissionSignal.msg (제안)
std_msgs/Header header
uint8 WAITING=0          # 신호 시작 전
uint8 OBSERVING=1        # 세는 중 — 점멸마다 갱신 (진행 상황)
uint8 DONE=2             # 12회 + 마지막 3초 연속 소등 확인
uint8 ERROR=3            # 관측 실패 — 추정하지 않고 멈춤
uint8 state
string[] sequence                  # 관측 순서 "R" | "Y" | "G" — DONE이면 정확히 12개
builtin_interfaces/Time[] onsets   # 각 점멸이 켜진 시각 (기록·재현용)
uint8 count_r
uint8 count_y
uint8 count_g
string error             # "" | too_many | unknown_color | timeout
```

```text
# 발제 공개 예제 Q1을 다 본 뒤 (DONE)
state: 2
sequence: [R, G, R, R, G, R, G, G, Y, G, Y, G]
count_r: 4
count_y: 2
count_g: 6
error: ''
```

> 12회 **미만**은 "아직 세는 중"과 구별이 안 되므로 인지는 `OBSERVING`을 유지하고, 계획이 `signal_timeout_s`로 끊는다.

## 5. skill별로 받는 값

| skill                      | 인지에서 받는 것                                                                   | 가공 수준              | 필요 정확도                         | 지금 상태                                  |
| -------------------------- | ---------------------------------------------------------------------------------- | ---------------------- | ----------------------------------- | ------------------------------------------ |
| **navigate**         | 없음 (Nav2가 RTAB-Map 위치 + 몸 뺀 장애물 점구름을 씀). 목표는 설정 좌표           | 인프라                 | ~5 cm                               | 자기 팔이 장애물로 잡힘 (#20)              |
| **look** (퍽 확인)   | 퍽 `details.tag` · `color` · `details.shape` · `last_seen`              | 물체 단위              | 태그 해독 정확                      | 태그 ✅ · 색 덮어쓰기 ❌ · 모양 없음 ❌  |
| **pick**             | ⓐ 다가갈 때 퍽 `pose` ⓑ 집을 때 `/score_grasps` 파지 자세 (ML이 점구름 직접) | ⓐ 물체 단위 ⓑ 점구름 | ⓐ 2~3 cm ⓑ mm                     | ⓐ ✅ ⓑ ✅ (gripper 점구름 필터 없음 #24) |
| **place**            | 장소 태그 **6자유도 자세** — 놓기 직전 새로 본 것                           | 물체 단위 (자세까지)   | **±3 mm** (채점: 중심 10 mm) | 방향 없음 ❌                               |
| **verify** (놓은 뒤) | 장소 근처 퍽들의 `tag` · `pose` · **이번에 새로 봤는지**               | 물체 단위              | ~5 mm                               | 위에서 보는 동작 ✅ · 정밀 위치 ❌        |
| **watch_signal**     | `/mission/signal`                                                                | 시간축 정리            | 점멸 하나도 틀리면 안 됨            | 주사위 미션 규칙으로 새로 ❌                        |
| **read_barcode**     | `/mission/barcode` 원문                                                          | 해독까지               | 정확                                | ✅ (이름만 변경)                           |
| **return_home**      | 없음 (TF 위치 + 출발 구역 다각형 설정)                                             | 인프라                 | 차체 전체 구역 안                   | 계획이 판정                                |

## 6. 판단이 갈리는 3곳 — 코드로 보기

### 6-1. 퍽은 추적 번호가 아니라 **tag로** 찾는다

추적 번호(`id`)는 인지가 물체를 잠깐 놓쳤다 다시 잡으면 바뀔 수 있다. 퍽 ID(`details.tag`)는 바뀌지 않는다.
그래서 계획은 "7번 퍽"을 tag로 찾고, 그 순간의 추적 번호로 skill을 보낸다.

```cpp
// plan 쪽 — /tracked_objects에서 tag로 퍽 찾기
#include <optional>
#include <string>
#include "perception/msg/tracked_object_array.hpp"
using perception::msg::TrackedObject;
using perception::msg::TrackedObjectArray;

// details에서 key 값 꺼내기 (없으면 빈 문자열)
std::string detail(const TrackedObject & o, const std::string & key) {
  for (const auto & kv : o.details) if (kv.key == key) return kv.value;
  return "";
}

// 퍽 ID(tag)로 지금 추적 중인 물체 찾기
const TrackedObject * find_puck(const TrackedObjectArray & a, int tag) {
  for (const auto & o : a.objects)
    if (o.class_name == "puck" && detail(o, "tag") == std::to_string(tag)) return &o;
  return nullptr;   // 아직 못 봤거나 잃어버림 → look으로 다시 보러 감
}
```

**인지가 지킬 것:** 한 번 읽은 tag는 그 물체(트랙)에 계속 붙어 있어야 한다. 두 트랙이 같은 tag를 가지면 안 된다.

### 6-2. 장소 중심은 인지가 아니라 **계획이** 계산한다 (place)

인지는 **장소 태그의 자세(사실)**만 준다. 태그는 장소 박스의 접근 방향 쪽 **수직면**에 붙어 있고 중심 높이 35 mm다 (발제 p8).
"태그 → 박스 중심 → 3×3 칸"은 박스 크기·격자 간격 같은 **미션 설정값**이 필요하므로 계획이 한다.
(인지에 넣으면 경기장 배치가 바뀔 때 인지 코드를 고쳐야 한다.)

```text
 위에서 본 장소 L1 (205 × 205 mm 박스)               태그 좌표축 (OpenCV aruco 관례)
   ┌─────────────────────┐                             z: 태그 면에서 바깥(로봇 쪽)으로
   │  ●0   ●1   ●2       │  ← 3×3 칸, 간격 60 mm        x: 태그 오른쪽 · y: 태그 위쪽
   │  ●3   ●4   ●5       │
   │  ●6   ●7   ●8       │
   └─────────▣───────────┘  ▣ = 장소 태그 (수직면)
             │ z (바깥)
             ▼  로봇이 여기서 다가옴
```

```python
# plan 쪽 — 태그 자세(map)에서 장소 중심과 칸 위치 계산 (numpy만)
import numpy as np

BOX_HALF = {"L1": 0.1025, "L3": 0.1025}   # 박스 깊이의 절반 [m] — L2(820 받침대 북쪽)는 현장 실측 후 추가
TAG_HEIGHT = 0.035                         # 태그 중심 높이 (바닥 기준)
PUCK_BOTTOM_Z = 0.070                      # 모든 퍽 바닥 Z (발제 p4)
GRID = 0.060

CELL = {i: ((i % 3 - 1) * GRID, (1 - i // 3) * GRID) for i in range(9)}   # 칸 → (dx, dy) arena: +Y가 위(북)

def location_center(tag_pos, tag_z_axis, loc):
    """태그 위치·바깥 방향(map) → 박스 윗면 중심 (map). 태그 바깥 방향의 반대로 박스 절반만큼 들어감"""
    n = np.array(tag_z_axis[:2]) / np.linalg.norm(tag_z_axis[:2])   # 수평 성분만
    c = np.array(tag_pos[:2]) - n * BOX_HALF[loc]
    return np.array([c[0], c[1], PUCK_BOTTOM_Z])

def cell_target(center, cell, arena_to_map_rot):
    """칸 오프셋은 경기장(arena) 축 기준 (모든 장소에서 같은 방향 — 발제 p4) → map 축으로 돌려 더함"""
    dx, dy = CELL[cell]
    return center + np.r_[arena_to_map_rot @ np.array([dx, dy]), 0.0]

# 예: 로봇(map 원점)이 +x를 보고 있고, 태그가 정면 0.5 m에서 로봇 쪽(−x)을 보고 있다
center = location_center(tag_pos=(0.5, 0.0, TAG_HEIGHT), tag_z_axis=(-1.0, 0.0, 0.0), loc="L1")
assert np.allclose(center, (0.6025, 0.0, 0.070))
```

> **왜 태그를 꼭 다시 보나:** 지도 위치 추정은 수 cm 오차라 10 mm 기준을 못 맞춘다. 놓기 직전 태그를 보고
> **로봇 기준 상대 위치**로 맞추면 위치 추정 오차가 빠진다.

### 6-3. 재검증은 **새로 본 것만** 센다 (verify)

발제는 "목표 상태를 복사한 로그는 재검증으로 불인정"이다. 인지는 각 퍽의 `last_seen`과 `cameras`(지금 보이는지)를 정확히 내고,
계획은 **재검증 동작을 시작한 시각 이후에 실제로 관측된 것만** 센다.

```cpp
// plan 쪽 — verify: 이번 관측만 골라서 장소별 기록
#include <cmath>
#include <vector>
#include "rclcpp/time.hpp"

struct Observed { int tag; double x, y; rclcpp::Time seen; };

std::vector<Observed> fresh_pucks_near(const TrackedObjectArray & a, const rclcpp::Time & look_started,
                                       double cx, double cy, double radius) {
  std::vector<Observed> out;
  for (const auto & o : a.objects) {
    if (o.class_name != "puck") continue;
    const rclcpp::Time seen(o.last_seen);
    if (seen < look_started) continue;          // 기억만 하던 것 — 재검증 증거가 아님
    const std::string tag = detail(o, "tag");
    if (tag.empty()) continue;                  // ID를 못 읽음 → "불명"으로 따로 기록
    const double x = o.pose.pose.position.x, y = o.pose.pose.position.y;
    if (std::hypot(x - cx, y - cy) > radius) continue;
    out.push_back({std::stoi(tag), x, y, seen});
  }
  return out;
}
```

### 6-4. (보너스) 신호 오류는 **두 종류**로 나눈다

| 종류                  | 예                                         | 누가 판단                                      | 어디에 기록                               |
| --------------------- | ------------------------------------------ | ---------------------------------------------- | ----------------------------------------- |
| 관측 실패 (센서 문제) | 불명 색, 13번째 점멸, 시간 초과            | **인지** → `state = ERROR`, `error` | `/mission/signal`                       |
| 규칙 위반 (미션 문제) | R이 7회, 합이 12가 아님, 퍽 구성으로 못 풂 | **계획** (`mission_solver`)            | `/mission/state.error`, `signal.json` |

```cpp
// plan 쪽 — 규칙 검증은 mission_solver (인지 결과가 DONE일 때만 부름)
bool valid_counts(int r, int y, int g, std::string & why) {
  for (int c : {r, y, g}) if (c < 1 || c > 6) { why = "count_out_of_range"; return false; }   // 각 1~6 (발제 p3)
  if (r + y + g != 12) { why = "sum_not_12"; return false; }
  return true;
}
```

## 7. 인지 쪽 신규 기능 — 코드 예시

지금 `perception_node.py`는 OpenCV(aruco)·numpy를 쓴다. 아래 예시도 **새 라이브러리 없이** 같은 것만 쓴다.

### 7-1. 장소 태그 6자유도 자세 (solvePnP)

태그 한 변 길이(48 mm)를 알고 있으니, 네 모서리 픽셀로 카메라 기준 자세를 바로 구할 수 있다.

```python
import cv2
import numpy as np

TAG_SIZE = 0.048   # 장소 태그 검은 사각형 한 변 [m] (흰 여백 제외)

# aruco 모서리 순서(좌상 → 우상 → 우하 → 좌하)에 맞춘 태그 평면 위 점 — 태그 중심이 원점, z = 0 (바깥이 +z)
OBJ = np.array([[-1,  1, 0], [1,  1, 0], [1, -1, 0], [-1, -1, 0]], np.float32) * (TAG_SIZE / 2)

def tag_pose(corners_px, K, dist):
    """모서리 4개 픽셀 → 카메라 광학 프레임 기준 (위치 t [m], 회전 R 3×3). 실패하면 None"""
    ok, rvec, tvec = cv2.solvePnP(OBJ, corners_px.reshape(4, 2).astype(np.float32), K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)   # 정사각형 마커 전용 (빠르고 안정적)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return tvec.ravel(), R          # R[:, 2] = 태그 바깥 방향(z축)

# 사용: K = camera_info.k (3×3), dist = camera_info.d — 결과를 TF(광학 프레임 → map)로 바꿔 TrackedObject.pose에
#       position과 orientation(R → 쿼터니언)을 모두 채운다. 지금처럼 orientation.w = 1.0 고정이면 안 됨.
```

- 정확도 확인 방법: 태그를 고정해 두고 같은 자리에서 10번 → 위치 표준편차 ≤ 3 mm, 방향 ≤ 2°.
- 멀면 오차가 커지므로 **놓기 직전(0.3~0.6 m)** 에 다시 본 값을 쓴다 (계획이 look → place 순서로 보냄).

### 7-2. 퍽 모양 (원기둥 / 직육면체)

위에서 보면 원기둥은 **원**, 직육면체는 **정사각형**이다. 윤곽의 "원형도" 4πA/P²로 구별한다 (원 = 1.0, 정사각형 ≈ 0.785).
윗면 태그 라벨(흰 사각형)이 안쪽에 있어도 **가장 바깥 윤곽**만 보면 된다.

```python
import math
import cv2
import numpy as np

def classify_shape(mask):
    """퍽 색 마스크(위에서 본, uint8 0/255) → 'cylinder' | 'cuboid' | 'unknown'"""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)   # 바깥 윤곽만 (안쪽 라벨 무시)
    if not cnts:
        return "unknown"
    c = max(cnts, key=cv2.contourArea)
    area, peri = cv2.contourArea(c), cv2.arcLength(c, True)
    if area < 200 or peri == 0:          # 너무 작으면(멀거나 가려짐) 추정하지 않음
        return "unknown"
    circularity = 4 * math.pi * area / peri ** 2
    if circularity > 0.86:
        return "cylinder"
    if circularity < 0.82:
        return "cuboid"
    return "unknown"                     # 애매하면 모른다고 (발제: 추정 금지)

# 자체 확인: 그린 원과 사각형
img = np.zeros((200, 200), np.uint8); cv2.circle(img, (100, 100), 50, 255, -1)
assert classify_shape(img) == "cylinder"
img = np.zeros((200, 200), np.uint8); cv2.rectangle(img, (50, 50), (150, 150), 255, -1)
assert classify_shape(img) == "cuboid"
```

- 비스듬히 보면 원이 타원이 돼 원형도가 떨어진다 → **위에서 볼 때(gripper_cam, look 자세)만** 판정하고, 한 번 확정되면 트랙에 기억한다.
- 경계값(0.82 / 0.86)은 실물 사진으로 맞추는 파라미터로 둔다.

### 7-3. 퍽 색 — HSV만, 태그 번호로 덮어쓰지 않기

```python
import cv2
import numpy as np

def puck_color(bgr_roi, min_ratio=0.3):
    """퍽 윗면 ROI → 'red' | 'green' | 'unknown'. 빨강은 H가 0과 180 양쪽에 걸쳐 있어 두 구간을 합친다"""
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    sat = (hsv[..., 1] > 80) & (hsv[..., 2] > 50)            # 흰 라벨·그림자 제외
    h = hsv[..., 0]
    red = sat & ((h < 10) | (h > 170))
    green = sat & (h > 45) & (h < 85)
    n = max(1, int(sat.sum()))
    if red.sum() / n > min_ratio and red.sum() > green.sum():
        return "red"
    if green.sum() / n > min_ratio:
        return "green"
    return "unknown"
```

> 지금 코드의 "태그를 읽으면 태그 번호대 색으로 덮어쓰기" ([`mission/README.md`](../mission/README.md) 126행)는 **지운다.**
> 주사위 미션에선 같은 ID가 회차마다 다른 색일 수 있다.

### 7-4. 신호 디코더 — 주사위 미션 규칙

입력은 지금처럼 신호등 물체의 프레임별 상태(`red | yellow | green | off`)다.
"켜짐 구간"을 하나의 점멸로 세고, 12번째 뒤 **3초 연속 꺼짐**이면 끝이다.

```python
class SignalCounter:
    """프레임마다 (시각 s, 상태) 를 넣으면 점멸을 센다 — 발제 p3: 1초 켜짐 + 1초 소등, 12회 뒤 3초 소등"""
    COLORS = {"red": "R", "yellow": "Y", "green": "G"}

    def __init__(self, min_on_s=0.3, end_off_s=3.0, total=12):
        self.min_on_s, self.end_off_s, self.total = min_on_s, end_off_s, total
        self.sequence, self.onsets = [], []
        self.on_color, self.on_since, self.off_since = None, None, None
        self.state, self.error = "WAITING", ""

    def feed(self, t, lamp):
        if self.state in ("DONE", "ERROR"):
            return
        if lamp in self.COLORS:
            if self.on_color is None:                     # 꺼짐 → 켜짐: 새 점멸 시작
                self.on_color, self.on_since = lamp, t
                self.state = "OBSERVING"
            elif lamp != self.on_color:                   # 켜진 채 색이 바뀜 — 1초 켜짐 사이엔 없어야 함
                self._fail("unknown_color")
            self.off_since = None
        elif lamp == "off":
            if self.on_color is not None:                 # 켜짐 → 꺼짐: 점멸 하나 끝
                if t - self.on_since >= self.min_on_s:    # 너무 짧은 켜짐은 잡음
                    self.sequence.append(self.COLORS[self.on_color])
                    self.onsets.append(self.on_since)
                    if len(self.sequence) > self.total:
                        self._fail("too_many")
                self.on_color = None
            if self.off_since is None:
                self.off_since = t
            if len(self.sequence) == self.total and t - self.off_since >= self.end_off_s:
                self.state = "DONE"
        else:                                             # 'all'(두 색 이상) · 판단 불가 → 추정 금지
            self._fail("unknown_color")

    def _fail(self, why):
        self.state, self.error = "ERROR", why

    def counts(self):
        return {c: self.sequence.count(c) for c in "RYG"}


# 자체 확인: 발제 Q1 시퀀스를 30 Hz 프레임으로 흉내 (1초 켜짐 · 1초 꺼짐, 마지막 3초 이상 꺼짐)
q1 = "R G R R G R G G Y G Y G".split()
name = {"R": "red", "Y": "yellow", "G": "green"}
sc, t = SignalCounter(), 0.0
for c in q1:
    for _ in range(30): sc.feed(t, name[c]); t += 1 / 30
    for _ in range(30): sc.feed(t, "off"); t += 1 / 30
for _ in range(100): sc.feed(t, "off"); t += 1 / 30
assert sc.state == "DONE" and sc.sequence == q1 and sc.counts() == {"R": 4, "Y": 2, "G": 6}
```

- 실제 노드에선 이 결과를 `MissionSignal`로 옮겨 점멸마다(`OBSERVING`) · 끝날 때(`DONE`/`ERROR`) 발행한다.
- 신호등이 화면에서 잠깐 안 보이는 것(가림)은 `off`와 다르다 — 1초 넘게 안 보이면 `timeout`으로 오류 처리할지 실물 시험 후 정한다.

## 8. 정해 둔 값 (주사위 칸 · 적재함)

**주사위 칸** — 위가 +Y(북), 칸 번호 행 우선 (0 = 북서, 8 = 남동). 2·3은 ↘ 대각선, 6은 좌우 두 열. 회전 대체 불가.

```text
 0 1 2      1: (4)          2: (0, 8)        3: (0, 4, 8)
 3 4 5      4: (0,2,6,8)    5: (0,2,4,6,8)   6: (0,3,6, 2,5,8)
 6 7 8
```

```cpp
// plan/mission_solver — 주사위 눈 n → 칸 번호
std::vector<int> dice_cells(int n) {
  static const std::vector<int> table[7] = {{}, {4}, {0, 8}, {0, 4, 8}, {0, 2, 6, 8}, {0, 2, 4, 6, 8}, {0, 3, 6, 2, 5, 8}};
  return (n >= 1 && n <= 6) ? table[n] : std::vector<int>{};
}
```

**적재함** — URDF(`urdf/default`)의 칸막이 충돌체 `storage_0~8` 기준 **3 × 4 = 12칸** (칸 안쪽 약 45 × 45 mm, base_link 기준):

```text
            y = +0.0975  +0.0325  −0.0325  −0.0975
x −0.040      [ 0 ]      [ 1 ]    [ 2 ]    [ 3 ]    ← 앞줄
x −0.105      [ 4 ]      [ 5 ]    [ 6 ]    [ 7 ]
x −0.170      [ 8 ]      [ 9 ]    [10 ]    [11 ]    ← 뒷줄 — MoveIt 도달 시험 필요 (실측: x −0.16 이후 거의 안 닿음)
```

지금 `plan.yaml`의 4칸 값은 개발용 임시값이다. 12칸 좌표로 바꾼 뒤 칸별 도달 시험을 하고, 안 닿는 칸은 설정에서 뺀다.
한 번에 나르는 개수는 설정값으로 둔다 (하나씩 ↔ 적재함 활용을 실측으로 비교 — 도전 C와 같은 주제).

## 9. 인지 작업 목록 (이슈 후보)

| #  | 작업                                                       | 왜 (배점)        | 완료 기준                                                  | 관련    |
| -- | ---------------------------------------------------------- | ---------------- | ---------------------------------------------------------- | ------- |
| P1 | 장소 태그 **6자유도 자세** (7-1)                     | 주사위 배치 30   | 고정 태그 10회 반복 위치 σ ≤ 3 mm, 방향 ≤ 2°           | 신규    |
| P2 | 퍽 **모양** `details.shape` (7-2)                  | 색·모양 선택 18 | 실물 12개 위에서 보고 12/12, 애매하면 `unknown`          | 신규    |
| P3 | 퍽 **색 HSV만** (7-3)                                | 색·모양 선택 18 | 태그 번호와 무관하게 실물 색 12/12                         | #27     |
| P4 | `/mission/signal` 새 형식 (4-2, 7-4)                          | 순서 12 + 횟수 9 | Q1 포함 녹화 5개 시퀀스 전부 일치, 소등을 점멸로 세지 않음 | #27     |
| P5 | `/mission/qr` → `/mission/barcode`, `qr_board` 정리 | 바코드 4         | 바코드 원문이 새 토픽으로                                  | #27     |
| P6 | 태그 ↔ 트랙 유지 (6-1)                                    | 할당·재검증     | 퍽을 집어 옮긴 뒤에도 같은 tag, tag 중복 0                 | #11 #21 |
| P7 | gripper 점구름 필터 · 자기 몸 제거                        | 집기 · 주행     | 팔이 장애물로 안 잡힘                                      | #20 #24 |

## 10. 합의 체크리스트

출력마다 아래 다섯 칸을 채우면 인터페이스가 확정된다.

| 출력                      | 좌표계          | 주기 · QoS                            | 모를 때 값                                              | 정확도 목표 · 측정법    | 원본 기록 위치                     |
| ------------------------- | --------------- | -------------------------------------- | ------------------------------------------------------- | ------------------------ | ---------------------------------- |
| 퍽 (`/tracked_objects`) | map             | 5~10 Hz · best effort                 | `color: unknown` · `shape: unknown` · tag 줄 없음 | 위치 ±2 cm (다가가기용) | (계획) `allocation.json`의 pucks |
| 장소 태그                 | map (자세 포함) | 같음                                   | 안 보이면 `cameras: []`                               | ±3 mm · 2° (P1)       | (계획) place 로그                  |
| `/mission/signal`       | —              | 점멸마다 · reliable + transient_local | `ERROR` + `error`                                   | 시퀀스 완전 일치         | (계획) `signal.json`             |
| `/mission/barcode`      | —              | 읽을 때 · reliable + transient_local  | 안 냄                                                   | 원문 일치                | (계획) `signal.json`의 barcode   |

계획 쪽 기록 파일(`signal.json` · `allocation.json` · `verify.json`)의 형식은 mission_manager 설계 문서에서 따로 정한다.

## 11. 통합 설계 결정 (2026-10-28 · 검토용 초안)

인지 ↔ 계획 경계 위에서, 미션을 실제로 어떻게 진행할지 정한 것. 숫자는 시뮬레이터·Isaac 도달 지도(`isaac/lab/scripts/dice_reach.py`)로 확인한 값이다.

| # | 항목 | 결정 | 근거 |
|---|---|---|---|
| D1 | 실패 정책 | 동작마다 재시도 → 안 되면 그 퍽·장소만 실패로 기록하고 **다음 장소 계속** (재검증·복귀까지). 신호·바코드 오류처럼 발제가 정지를 요구하는 것만 즉시 정지 | 발제: 점수는 그때까지 유효 — 재검증 8 · 복귀 10을 지킴 |
| D2 | 퍽 조사 | 시작 박스 6곳 **전부 먼저** 조사 → 12개 한 번에 할당 | Y(한 모양)를 못 만드는 실수 방지 (발제 p5 · 도전 B) |
| D3 | 운반 | 조사하면서 **12개 전부 적재함에 싣기** (칸 → tag 대응표). 도달 안 되는 칸은 설정에서 빼고, 싣기 실패한 퍽은 1개씩 직접 운반 | 12개를 모두 쓰므로 할당 전에 실어도 손해 없음 · 이동 감소. Isaac 10/27: 칸 2·8·9 성공률 낮음 → 먼저 적재·꺼내기 성공률 확인 |
| D4 | 놓기 기준 | 놓기 직전 **장소 태그로 칸 위치 보정**(P1) + 팔 추종 오차는 `rl_arm_correction`·관절 보정 | 위치 추정 오차(수 cm)는 태그 보정만, 팔 처짐은 팔 보정만 줄인다 — 서로 대신 못 함 |
| D5 | 놓은 뒤 확인 | 놓을 때마다 내려다보고 10 mm 밖·쓰러짐이면 다시 놓기 | 주사위 배치 30점 |
| D6 | 재시도 | 동작마다 2번 (접근은 다른 변으로) — 설정값 | |
| D7 | 박스 앞 접근 | Nav2로 변 앞 0.25 m → `approach` skill로 저속 직선 **0.085 m**까지 (물러날 때도 직선). collision_monitor 감시 영역 = 로봇 외곽 직사각형 | 도달 지도: 변 0.08이면 3×3 칸 9개 · 0.20이면 가까운 줄 3칸뿐. Nav2·감시 원(0.18/0.20)으로는 못 붙음 |
| D8 | L2 먼 줄 | **미해결 위험** — L2 격자가 820 받침대 북쪽 변에서 약 0.2 m 안쪽(그림 추정)이라 가까운 줄만 닿을 수 있음. 현장 실측 + Isaac(손 기울이기)로 확인 | |
| D9 | 상태 머신 | 단계 enum + 전이 함수(ROS 없는 순수 함수) + gtest | 재시도·장소 단위 계속·적재함 관리가 들어가면 콜백 연쇄로는 검증이 어려움 |
| D10 | 상태 출력 | `MissionState` 메시지 (단계 · 설명 · 바코드 · 신호 횟수 · 배정 · 적재함 칸→tag · 진행 · 장소별 결과 · 오류) | UI·폰 상태 화면이 구독 |
| D11 | 재검증 | 장소마다 **새 관측만** · 칸별 tag·위치 오차 · 빈 칸 퍽 · 12개 전체 중복·누락 판정 · 사진 → `verify.json` | 검증 8점 (장소별 2 × 3 + 전체 2) |
| D12 | 복귀 | 출발 구역 가운데로 → 외곽 직사각형이 구역(600 × 600) 안인지 확인, 밖이면 보정 → 3초 정지 기록 | 복귀 10점: 차체 전체가 구역 안 |
| D13 | 기록 | json + 단계별 카메라 사진 + rosbag(미션·신호·skill·위치 토픽, 영상·점구름 제외). 전체 영상은 외부 촬영 | 발제 제출물 |

**인지 쪽 문제 (인지 담당 결정)**
- I1 깊이 없는 퍽: gripper_cam을 퍽 윗면 위 8 cm(팔이 아래를 보며 닿는 한계)에 두면 카메라~퍽이 약 11 cm로 D435 최소 거리(약 17 cm) 안 → 깊이 0 → 퍽 위치를 못 구해 tag를 붙일 트랙이 없음. 제안: 깊이가 없을 때만 퍽 윗면 높이(Z = 0.13) 평면과 광선 교차 (`puck_plane_z`, 기본 끔).
