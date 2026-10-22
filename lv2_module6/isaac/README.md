# isaac

Isaac Sim 장면·로봇 USD(`sim/`)와 Isaac Lab 학습(`lab/`). 로봇은 `../urdf/beebot_mesh/beebot.urdf`를 Isaac Sim URDF Importer로 가져와 만든다.

## 폴더 구조

```
isaac/
├── sim/                Isaac Sim
│   ├── usda/           USD 파일 (아래 "USD 구조")
│   ├── source/         씬 구성에 필요한 이미지 소스
│   │   ├── HDRI/           HDRI
│   │   └── texture/        텍스쳐
│   └── scripts/        usd, isaacsim 관련 scripts
│       └── sim_drive.py    Script Editor용 바퀴 구동·전복 테스트 
└── lab/               Isaac Lab 학습 (준비 중, scripts/ 비어 있음)
```

## USD 구조

`sim/usda/` 안. 화살표는 어느 파일이 어느 파일을 불러오는지(payload·sublayer·asset 참조).

```
scene_sim.usda                     장면 진입점
├── /World/beebot         → beebot/beebot.usda
├── /World/puck_blue_box  → puck/puck_blue_box.usda
├── /World/shelf          → shelf/shelf.usda
├── /World/GroundPlane    → ../source/texture/class_room_floor/*.png
└── /Environment/DomeLight → ../source/HDRI/class_room_03_4k.exr

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

puck/puck_red_cyl.usda             빨간 원기둥 퍽 (아직 장면에 안 들어감)
```

ctrl+S 하지 말것

- `physics.usda` 로봇의 drive 값들
- `physx.usda` 물리 모델 Physx 파라미터

## 장면 열기
Isaac Sim에서 `sim/usda/scene_sim.usda`를 연다. 로봇만 볼 때는 `sim/usda/beebot/beebot.usda`.