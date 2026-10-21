# usd

Isaac Sim용 USD. 로봇은 `../urdf/beebot_mesh/beebot.urdf`를 Isaac Sim URDF Importer로 가져와 만든다.

```
usd/
├── beebot/                    임포터 생성물 (재임포트하면 통째로 새로 써짐)
│   ├── beebot.usda            진입점. 이 파일을 연다. Physics variant: physx / physics / mujoco / none
│   └── payloads/
│       ├── base.usda          링크 계층 (kinematic tree)
│       ├── geometries.usd     메시 데이터 (바이너리)
│       ├── instances.usda     visual·collision 메시 조립
│       ├── robot.usda         Isaac robot 스키마
│       └── Physics/
│           ├── physics.usda   공통 물리: 질량, 관절, drive K·D·maxForce, 물리 재질
│           ├── physx.usda     PhysX 전용: armature, maxJointVelocity, articulation 설정 (physics.usda를 sublayer로 가짐)
│           └── mujoco.usda    MuJoCo 전용
├── maze.usd                   미로 장면
└── scripts/
    └── carry_tuning.py        재임포트 후 튜닝 값을 git 직전 버전에서 다시 옮김
```

ctrl+S 하지 말것

- `physics.usda` 로봇의 drive 값들
- `physx.usda` 물리 모델 Physx 파라미터


## URDF 수정 후 가이드
1. 튜닝 값이 커밋돼 있는지 확인 (스크립트가 HEAD에서 읽는다)
2. `make_urdf.sh` → `check_urdf.sh`
3. Isaac에서 `beebot.urdf` 임포트, USD Output = `lv2_module6/usd/`, Merge fixed joints 끔
4. `python scripts/carry_tuning.py --dry-run` 으로 확인 후 `python scripts/carry_tuning.py` (pxr 필요: Isaac Sim python 또는 `pip install usd-core`)
5. `git diff --stat` 로 URDF에서 고친 부분만 바뀌었는지 확인

`SKIP ... (removed or renamed?)` 가 나오면 이름이 바뀐 prim이다. 그 값은 손으로 다시 넣는다.
