[목차](README.md) · [◀ 학습](lab.md) · [실기 측정 ▶](calib.md)

# USD

## USD 구조

`usda/` 안. 화살표는 어느 파일이 어느 파일을 불러오는지(payload·sublayer·asset 참조).

```
scene_sim.usda                     전체 미션 장면 진입점
├── /World/beebot                  → beebot/beebot.usda
├── /World/Environment/DomeLight   → ../source/HDRI/class_room_03_4k.exr
├── /World/Environment/GroundPlane → ../source/texture/class_room_floor/*.png
├── /World/maze                    → maze/maze.usda
├── /World/sign                    → sign/sign.usda
├── /World/start_boxes/start_1~6   → shelf/shelf.usda
├── /World/pucks/puck_<모양>_<색>_<태그> (12개) → puck/box · puck/cyl
└── /World/shelf_L1~L3             → shelf/shelf_L1~L3.usda

beebot/beebot.usda                 로봇 진입점, 임포터 생성물 (**재임포트하면 beebot/ 통째로 새로 써짐**)
│                                  Physics variant: physx / physics / mujoco / none
└── payloads/
    ├── base.usda                  링크 계층 (kinematic tree)
    ├── geometries.usd             메시 데이터 (바이너리)
    ├── instances.usda             visual·collision 메시 조립
    ├── robot.usda                 Isaac robot 스키마
    └── Physics/
        ├── physics.usda           공통 물리: 질량, 관절, drive K·D·maxForce, 물리 재질
        ├── physx.usda             PhysX 전용: armature, maxJointVelocity, articulation 설정 (physics.usda를 sublayer로 가짐)
        └── mujoco.usda            MuJoCo 전용

```

ctrl+S 하지 말것

- `physics.usda` 로봇의 drive 값들
- `physx.usda` 물리 모델 Physx 파라미터

## 장면 열기
Isaac Sim에서 `usda/scene_sim.usda`를 연다. 로봇만 볼 때는 `usda/beebot/beebot.usda`.

## 에셋

`scene_sim.usda`가 미로, 표지판, 시작 상자 6개, 선반 상자 3개, 퍽 12개, 카펫 바닥을 물리 설정까지 포함해 불러온다. 경로는 모두 상대 경로다. 집기 시연 장면은 파일이 아니라 코드(`rl/bee_isaac/scenes/manipulator_scene.py`)로 만들고 같은 에셋을 쓴다.

```
usda/
├─ scene_sim.usda          전체 미션 장면
├─ scene_lab.usda          옛 시연 장면 (지금은 scripts/tools/stand_plan.py만 읽음)
├─ beebot/    로봇 (위 구조)
├─ maze/      maze.usda, geo.usd
├─ sign/      sign.usda, geo.usd, blink.py
├─ puck/box/  puck_box_green_0.usda, puck_box_red_1.usda, geo.usd
├─ puck/cyl/  puck_cyl_green_2.usda, puck_cyl_red_3.usda, geo.usd
├─ shelf/     shelf.usda, shelf_L1~L3.usda, geo.usd
└─ scripts/   convert.py
(텍스처는 ../source/HDRI, ../source/texture/{class_room_floor,puck,shelf})
```

각 에셋은 `*.usda`(머티리얼·물리 설정)가 같은 폴더의 `geo.usd`(지오메트리)를 서브레이어로 불러오는 구조입니다. 장면에서는 `*.usda`를 payload로 참조합니다.



## 퍽

| 에셋 | 파일 | 기본 태그 | 모양 | 크기 (W×D×H) | 색상 (sRGB) | 질량 |
|---|---|---|---|---|---|---|
| `puck_box_green_0` | `puck/box/puck_box_green_0.usda` | 0 | 직육면체 | 3 × 3 × 6 cm | 초록 `#00AE42` (0, 0.6824, 0.2588) | 0.025 kg |
| `puck_box_red_1` | `puck/box/puck_box_red_1.usda` | 1 | 직육면체 | 3 × 3 × 6 cm | 빨강 `#9D2235` (0.6157, 0.1333, 0.2078) | 0.025 kg |
| `puck_cyl_green_2` | `puck/cyl/puck_cyl_green_2.usda` | 2 | 원기둥 | Ø3 × 6 cm | 초록 `#00AE42` | 0.02 kg |
| `puck_cyl_red_3` | `puck/cyl/puck_cyl_red_3.usda` | 3 | 원기둥 | Ø3 × 6 cm | 빨강 `#9D2235` | 0.02 kg |

- 태그 ID는 베이스컬러 텍스처 번호(`puck_*_basecolor_id<N>.png`)와 같습니다.
- 에셋은 모양·색 조합별 4개이고, 장면의 퍽 12개는 이 에셋에 텍스처만 바꿔 태그를 정합니다. 직육면체 빨강 0·1·2·8, 초록 6·7, 원기둥 빨강 3·4·5, 초록 9·10·11.
- 색상은 MaterialX 그래프의 상수(`mtlxmultiply2.inputs:in1`)이고, 그래프 안에서 power 2.2로 선형 변환됩니다. 태그 영역은 `puck_*_tagmask.png`로 구분됩니다.
### 퍽 물리 설정

| 항목 | 직육면체 퍽 | 원기둥 퍽 |
|---|---|---|
| 강체 | `PhysicsRigidBodyAPI` (루트 prim) | `PhysicsRigidBodyAPI` (루트 prim) |
| 콜라이더 | `box/collision` — 8점 박스 메시, `convexHull` | `cyl/collision` — `Cylinder` 기본 도형 |
| 선형 / 회전 감쇠 | 기본값 (0 / 0.05) | 0.1 / 1.5 |
| 물리 재질 | `PhysicsMaterials/PLA` | `PhysicsMaterials/PLA` |

원기둥 퍽의 회전 감쇠는 굴림 저항 대용입니다. PhysX에는 굴림 저항이 없어서, 감쇠가 없으면 바닥에 떨어진 원기둥이 멈추지 않고 계속 굴러갑니다.

## 종이상자 (`shelf`)

| 항목 | 값 |
|---|---|
| 파일 | `shelf/shelf.usda` |
| 크기 | 20.7 × 20.7 × 7 cm |
| 강체 / 질량 | `PhysicsRigidBodyAPI`, 0.4 kg (추정값) |
| 콜라이더 | `collision` — 8점 박스 메시, `convexHull` |
| 감쇠 | 기본값 |
| 물리 재질 | `PhysicsMaterials/Cardboard` |

## 미로 (`maze`)

| 항목 | 값 |
|---|---|
| 파일 | `maze/maze.usda` |
| 크기 | 4.55 × 3.26 × 0.39 m |
| 강체 | 없음 (정적 콜라이더) |
| 콜라이더 | `collision/geo_0` — 벽 메시, `approximation = none`(삼각형 메시 그대로)<br>`collision/geo_1` ~ `geo_45` — 기둥 `Cylinder` 기본 도형 45개 |
| 물리 재질 | `PhysicsMaterials/PLA` |

렌더링 메시(`maze/geo/wall`, `PLA`, `profile`, `steel`)에는 콜리전이 없고, 충돌은 `collision` 아래 prim만 담당합니다.

## 바닥 (`/World/Environment/GroundPlane`)

| 항목 | 값 |
|---|---|
| 콜라이더 | `CollisionPlane` — 무한 평면 (Z축) |
| 시각 메시 | `CollisionMesh` (이름과 달리 콜리전 없음), 머티리얼 `class_room_floor` (OmniPBR) |
| 물리 재질 | `floor_PhysicsMaterial` (카펫) |
| 조명 | `DomeLight` — `../source/HDRI/class_room_03_4k.exr` |

## 물리 재질

| 재질 | 사용처 | staticFriction | dynamicFriction | restitution | restitution 합산 방식 |
|---|---|---|---|---|---|
| PLA | 퍽 4개, 미로 | 0.35 | 0.30 | 0.40 | average (기본) |
| Cardboard | 종이상자 | 0.55 | 0.45 | 0.15 | min |
| Carpet (`floor_PhysicsMaterial`) | 바닥 | 0.85 | 0.70 | 0.02 | min |

PhysX는 접촉한 두 재질의 마찰을 평균해서 씁니다. 그래서 실제 정지 마찰은 다음과 같습니다.

| 접촉 쌍 | 정지 마찰 | 반발 |
|---|---|---|
| PLA ↔ PLA | 0.35 | 0.40 |
| PLA ↔ 종이상자 | 0.45 | 0.15 |
| PLA ↔ 카펫 | 0.60 | 0.02 |
| 종이상자 ↔ 카펫 | 0.70 | 0.02 |

반발 계수는 Cardboard와 Carpet에 `min` 합산을 걸어 두었습니다. 그래서 이 두 재질과 닿을 때는 둘 중 작은 값이 쓰여서, 상자나 카펫 위에서 퍽이 거의 튀지 않습니다. 재질 값은 실측이 아닌 시작값입니다. 실측이 필요하면 판을 기울여 물체가 미끄러지기 시작하는 각도 θ를 재서 `μs = tan θ`로 구할 수 있습니다.

## 텍스처

모든 텍스처는 `../source/` 아래에 있고 (HDRI는 `source/HDRI/`, 나머지는 `source/texture/`) 8bit PNG입니다. HDRI만 EXR입니다. 미로는 텍스처 없이 머티리얼 상수 색만 씁니다.

| 파일 | 사용처 | 종류 | 해상도 | 채널 |
|---|---|---|---|---|
| `HDRI/class_room_03_4k.exr` | DomeLight | 환경광 HDRI (위경도 파노라마) | 4096 × 2048 | RGB, half float, PIZ 압축 |
| `class_room_floor/class_room_floor_albedo.png` | 바닥 (OmniPBR `diffuse_texture`) | 베이스컬러 | 2048 × 2048 | RGB |
| `class_room_floor/class_room_floor_normal.png` | 바닥 (OmniPBR `normalmap_texture`) | 노멀맵 | 2048 × 2048 | RGB |
| `puck/puck_box_basecolor_id0~11.png` | 직육면체 퍽 | 베이스컬러 (태그 ID) | 1024 × 1024 | 그레이스케일 |
| `puck/puck_box_tagmask.png` | 직육면체 퍽 공통 | 태그 마스크 | 1024 × 1024 | 그레이스케일 |
| `puck/puck_box_roughness.png` | 직육면체 퍽 공통 | 러프니스 | 1024 × 1024 | 그레이스케일 |
| `puck/puck_box_normal.png` | 직육면체 퍽 공통 | 노멀맵 | 1024 × 1024 | RGB |
| `puck/puck_cyl_basecolor_id0~11.png` | 원기둥 퍽 | 베이스컬러 (태그 ID) | 1024 × 1024 | 그레이스케일 |
| `puck/puck_cyl_tagmask.png` | 원기둥 퍽 공통 | 태그 마스크 | 1024 × 1024 | 그레이스케일 |
| `puck/puck_cyl_roughness.png` | 원기둥 퍽 공통 | 러프니스 | 1024 × 1024 | 그레이스케일 |
| `puck/puck_cyl_normal.png` | 원기둥 퍽 공통 | 노멀맵 | 1024 × 1024 | RGB |
| `shelf/box_basecolor.png` | 종이상자 | 베이스컬러 | 1024 × 1024 | RGB |
| `shelf/box_roughness.png` | 종이상자 | 러프니스 | 1024 × 1024 | 그레이스케일 |
| `shelf/box_normal.png` | 종이상자 | 노멀맵 | 1024 × 1024 | RGB |
| `shelf/box_tag_basecolor_L1~L3.png` | `shelf_L1~L3` | 베이스컬러 (번호판) | 1024 × 1024 | RGB |
| `shelf/box_tag_roughness.png` | `shelf_L1~L3` | 러프니스 | 1024 × 1024 | 그레이스케일 |
| `shelf/box_tag_normal.png` | `shelf_L1~L3` | 노멀맵 | 1024 × 1024 | RGB |

- 퍽 베이스컬러는 흰 바탕에 검은 태그 패턴이 있는 그레이스케일 이미지입니다. 머티리얼에서 태그 마스크(흰색 = 태그 영역)를 기준으로, 태그 영역에는 이 이미지를, 나머지에는 퍽 색상 상수를 섞어 최종 색을 만듭니다. 그래서 색이 다른 퍽도 마스크, 러프니스, 노멀은 모양별로 공유합니다.
- 노멀맵은 탄젠트 공간 노멀입니다. 퍽 노멀맵에는 FDM 적층 줄무늬(약 0.24mm 간격)가 들어 있습니다.
- 러프니스, 태그 마스크, 노멀맵은 색이 아닌 데이터라서 raw(선형)로 읽어야 합니다.

## 참고

- 퍽, 상자, 미로 머티리얼은 MaterialX(`standard_surface`)입니다.
