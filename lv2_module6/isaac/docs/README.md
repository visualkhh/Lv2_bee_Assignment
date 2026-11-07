# isaac

Isaac Sim 장면·로봇 USD(`usda/`)와 Isaac Lab 학습(`rl/bee_isaac/tasks/`). 로봇은 `../../urdf/beebot_mesh/beebot.urdf`를 Isaac Sim URDF Importer로 가져와 만든다.

## 목차

1. [시연](demo.md) — Isaac Sim + ROS 실행 명령, 인자
2. [학습](lab.md) — 팔 RL 잔차, skill별 학습 환경, 결과
3. [USD](usd.md) — 장면·로봇 USD 구조, 퍽·상자·미로 물리, 텍스처
4. [실기 측정](calib.md) — Real to Sim 관절 응답 측정과 도표
5. [v02 학습 계획](v02.md) — 시뮬에도 중력 앞먹임을 넣고 다시 학습

## 폴더 구조

```
isaac/
├── pyproject.toml      패키지 bee_isaac (설치: 아래)
├── rl/bee_isaac/      시연·학습 공용 코드
│   ├── constants.py    경로·배치 값 (표준 라이브러리만 — ROS 쪽도 씀)
│   ├── common.py       IK·궤적·치수 (pinocchio)
│   ├── mission_rules.py
│   ├── assets/         로봇 설정 (beebot.py)
│   ├── scenes/         코드로 만드는 장면 (manipulator_scene.py)
│   ├── tasks/          Isaac Lab 학습 환경 (*_env.py)
│   └── data/           slot_table
├── scripts/            실행 파일 — 공용 코드는 bee_isaac에서 import
│   ├── train.py · train_queue.sh
│   ├── demo/           시연 (pick_load_demo.sh · pick_load_play.py)
│   ├── standalone/     Isaac + ROS 실행 기반 (isaac_sim.sh · manipulator_env · reset.sh · opencr_bridge · launch)
│   ├── calib/          실기 측정 (Real to Sim, 예전 R2S — 결과 logs/는 git 제외)
│   └── tools/          slot_table · stand_plan · sim_drive
├── usda/               USD 파일 (usd.md)
├── source/             장면용 이미지 (HDRI · texture) — usda가 ../source로 참조
├── policies/           학습한 정책 (.pt · .onnx) · export_onnx.py — 새 결과물(*.pt · *.onnx)은 git 제외
└── docs/               문서 (README.md가 목차)
```

**실행 환경이 둘이다.** `tasks/`·`scenes/`·학습·시연은 Isaac venv (numpy 2), `camera_adapter.py`·`pick_load_play.py`·launch는 시스템 python + ROS (numpy 1.26). `constants.py`만 양쪽에서 import한다.

## 설치

쓰는 venv마다 한 번 (editable — 코드 수정은 바로 반영):

```bash
uv pip install --python <isaaclab venv python 경로> --no-deps -e lv2_module6/isaac
uv pip install --python ~/pa-course/IsaacLab-6.1/.venv/bin/python --no-deps -e lv2_module6/isaac
```

워크트리를 옮기거나 지우면 다시 설치한다.
