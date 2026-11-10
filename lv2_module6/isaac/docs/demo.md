[목차](README.md) · [학습 ▶](lab.md)

# 시연

Isaac Sim이 beebot 하드웨어 역할을 하고, 실기와 같은 ROS 노드가 로봇을 움직인다. 명령은 모두 `lv2_module6`에서 실행한다.

## 전체 미션

`scene_sim.usda` 장면과 전체 ROS 시스템(`isaac_system.launch.py`)을 띄운다.

```bash
isaac/standalone/isaac_sim.sh
```

Isaac을 끄지 않고 처음 자세로 되돌리고 ROS 노드를 재시작하려면:

```bash
isaac/standalone/runtime/reset.sh
```

## 팔만 시험 (SLAM · Nav2 없음)

같은 Isaac 위에 `isaac_manipulator_demo.launch.py`(stand_test 스택 + Isaac 카메라 어댑터)를 올린다. 실기 시험과 같은 스크립트를 `use_sim:=true`로 돌린다. 자세한 흐름은 [실기 측정](calib.md)의 시험 반복 절을 본다.

```bash
isaac/standalone/isaac_sim.sh                    # 터미널 1
isaac/loops/manipulator-launcher.sh use_sim:=true # 터미널 2
```
