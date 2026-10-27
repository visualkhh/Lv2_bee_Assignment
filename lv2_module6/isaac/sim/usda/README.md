# lv_module6_usd

Isaac Sim용 module6 시뮬레이션 장면입니다. `scene.usda`를 열면 미로, 종이상자, 퍽 4개, 카펫 바닥이 물리 설정까지 포함된 상태로 로드됩니다. 폴더를 통째로 옮겨도 동작하도록 모든 경로는 상대 경로로 되어 있습니다. 로봇은 이 배포본에 포함되어 있지 않습니다.

## 폴더 구조

```
lv_module6_usd/
├─ scene.usda              전체 장면 (진입점)
├─ maze/      maze.usda, geo.usd
├─ puck/box/  puck_box_green_0.usda, puck_box_red_1.usda, geo.usd
├─ puck/cyl/  puck_cyl_green_2.usda, puck_cyl_red_3.usda, geo.usd
├─ shelf/     shelf.usda, geo.usd
(텍스처는 ../source/HDRI, ../source/texture/{class_room_floor,puck,shelf})
```

각 에셋은 `*.usda`(머티리얼·물리 설정)가 같은 폴더의 `geo.usd`(지오메트리)를 서브레이어로 불러오는 구조입니다. 장면에서는 `*.usda`를 payload로 참조합니다.



## 퍽

| prim | 파일 | 태그 ID | 모양 | 크기 (W×D×H) | 색상 (sRGB) | 질량 |
|---|---|---|---|---|---|---|
| `puck_box_green_0` | `puck/box/puck_box_green_0.usda` | 0 | 직육면체 | 3 × 3 × 6 cm | 초록 `#00AE42` (0, 0.6824, 0.2588) | 0.025 kg |
| `puck_box_red_1` | `puck/box/puck_box_red_1.usda` | 1 | 직육면체 | 3 × 3 × 6 cm | 빨강 `#9D2235` (0.6157, 0.1333, 0.2078) | 0.025 kg |
| `puck_cyl_green_2` | `puck/cyl/puck_cyl_green_2.usda` | 2 | 원기둥 | Ø3 × 6 cm | 초록 `#00AE42` | 0.02 kg |
| `puck_cyl_red_3` | `puck/cyl/puck_cyl_red_3.usda` | 3 | 원기둥 | Ø3 × 6 cm | 빨강 `#9D2235` | 0.02 kg |

- 태그 ID는 베이스컬러 텍스처 번호(`puck_*_basecolor_id<N>.png`)와 같습니다.
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
| 크기 | 25 × 27 × 7 cm |
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

## 바닥 (`Environment/GroundPlane`)

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
| `puck/puck_box_basecolor_id0.png` | `puck_box_green_0` | 베이스컬러 (태그 ID 0) | 4096 × 4096 | 그레이스케일 |
| `puck/puck_box_basecolor_id1.png` | `puck_box_red_1` | 베이스컬러 (태그 ID 1) | 4096 × 4096 | 그레이스케일 |
| `puck/puck_box_tagmask.png` | 직육면체 퍽 공통 | 태그 마스크 | 4096 × 4096 | 그레이스케일 |
| `puck/puck_box_roughness.png` | 직육면체 퍽 공통 | 러프니스 | 4096 × 4096 | 그레이스케일 |
| `puck/puck_box_normal.png` | 직육면체 퍽 공통 | 노멀맵 | 4096 × 4096 | RGB |
| `puck/puck_cyl_basecolor_id2.png` | `puck_cyl_green_2` | 베이스컬러 (태그 ID 2) | 4096 × 4096 | 그레이스케일 |
| `puck/puck_cyl_basecolor_id3.png` | `puck_cyl_red_3` | 베이스컬러 (태그 ID 3) | 4096 × 4096 | 그레이스케일 |
| `puck/puck_cyl_tagmask.png` | 원기둥 퍽 공통 | 태그 마스크 | 4096 × 4096 | 그레이스케일 |
| `puck/puck_cyl_roughness.png` | 원기둥 퍽 공통 | 러프니스 | 4096 × 4096 | 그레이스케일 |
| `puck/puck_cyl_normal.png` | 원기둥 퍽 공통 | 노멀맵 | 4096 × 4096 | RGB |
| `shelf/box_basecolor.png` | 종이상자 | 베이스컬러 | 4096 × 4096 | RGB |
| `shelf/box_roughness.png` | 종이상자 | 러프니스 | 4096 × 4096 | 그레이스케일 |
| `shelf/box_normal.png` | 종이상자 | 노멀맵 | 4096 × 4096 | RGB |

- 퍽 베이스컬러는 흰 바탕에 검은 태그 패턴이 있는 그레이스케일 이미지입니다. 머티리얼에서 태그 마스크(흰색 = 태그 영역)를 기준으로, 태그 영역에는 이 이미지를, 나머지에는 퍽 색상 상수를 섞어 최종 색을 만듭니다. 그래서 색이 다른 퍽도 마스크, 러프니스, 노멀은 모양별로 공유합니다.
- 노멀맵은 탄젠트 공간 노멀입니다. 퍽 노멀맵에는 FDM 적층 줄무늬(약 0.24mm 간격)가 들어 있습니다.
- 러프니스, 태그 마스크, 노멀맵은 색이 아닌 데이터라서 raw(선형)로 읽어야 합니다.

## 참고

- 퍽, 상자, 미로 머티리얼은 MaterialX(`standard_surface`)입니다.
