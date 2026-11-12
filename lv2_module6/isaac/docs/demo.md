[목차](README.md) · [학습 ▶](lab.md)

# 시연

Isaac Sim이 beebot 하드웨어 역할을 하고, 실기와 같은 ROS 노드가 로봇을 움직인다. 명령은 모두 `lv2_module6`에서 실행한다.

## 전체 미션

`scene_sim.usda` 장면과 전체 ROS 시스템(`isaac_system.launch.py`)을 띄운다. 인지를 켜고 AUTO로 띄워 바로 미션을 돌릴 수 있다. 신호·바코드를 넣어 한 판 돌리기 · RViz · Nav2 벽 지도는 [`isaac/mission/`](../mission/README.md).

준비: `ros2_ws` 빌드(`install/`), `sudo apt install ros-lyrical-depth-image-proc`, Isaac Sim venv(기본 `~/pa-course/.venv-isaacsim`, 다르면 `ISAACSIM_VENV=<경로>`).

```bash
isaac/standalone/isaac_sim.sh
```

- Isaac이 `scene_sim.usda`를 열고 가상 OpenCR(`opencr_bridge.py`)를 띄운다. **Play를 눌러야** ROS가 뜬다. Play 전에 띄우면 `BeebotSystemHardware`가 시작 응답을 못 받아 컨트롤러가 전부 안 뜬다. 자동으로 Play하려면 `ISAAC_AUTOPLAY=1`.
- 실행 중 **Stop은 누르지 않는다.** 시뮬 시계가 0으로 돌아가 TF·EKF·cuVSLAM·nvblox가 꼬인다. 잠깐 멈추려면 Pause, 처음부터 하려면 아래 `reset.sh`.
- 관절 비틀림 모델(`opencr_bridge.py`)과 실기 보정(`rl_arm_correction` 앞먹임)은 늘 켜져 있다(`OPENCR_TWIST_SCALE=1`). 둘 다 끄려면 `OPENCR_TWIST_SCALE=0 isaac/standalone/isaac_sim.sh`.
- Fast DDS는 `ros2_ws/fastdds_local_only.xml`(이 머신 안에서만, SHM)을 쓴다. 다른 터미널의 `ros2` 명령도 같은 파일(`FASTDDS_DEFAULT_PROFILES_FILE`)이어야 토픽이 보인다.
- Isaac을 끄면 ROS 쪽도 같이 꺼진다.

Isaac을 끄지 않고 처음 자세로 되돌리고 ROS 노드를 재시작하려면:

```bash
isaac/standalone/runtime/reset.sh
```

## 팔만 시험 (SLAM · Nav2 없음)

같은 Isaac 위에 `isaac_manipulator_demo.launch.py`(stand_test 스택 + Isaac 카메라 어댑터)를 올린다. 실기 시험과 같은 스크립트를 `use_sim:=true`로 돌린다. 자세한 흐름은 [실기 측정](calib.md)의 시험 반복 절을 본다.

```bash
isaac/standalone/isaac_sim.sh                    # 터미널 1: Play까지 누른다
isaac/loops/manipulator-launcher.sh use_sim:=true # 터미널 2
```

`use_sim:=true`는 Isaac이 Play될 때까지(`/tmp/opencr`) 기다린 뒤, 떠 있는 ROS(전체 bringup 포함)를 정리하고 팔 스택(`isaac_manipulator_demo.launch.py`)을 띄운다. `ROS_DOMAIN_ID`는 떠 있는 `isaac_sim.sh`에서 가져오고 모든 노드가 `use_sim_time`이다. 정리 없이 띄우려면 `STAND_NO_CLEAN=1`(다른 ROS 작업이 같이 꺼지는 것을 피할 때).

처음 자세로 되돌린 뒤(`reset.sh`)에는 전체 bringup이 다시 뜨므로 이 명령을 한 번 더 실행한다. 시험 루프(`pick_loop.py` 등)는 `--use_sim true`로 돌린다.

`isaac_sim.sh` 한 번으로 팔 스택만 띄울 수도 있다:

```bash
ISAAC_ROS_LAUNCH=isaac_manipulator_demo.launch.py ISAAC_AUTOPLAY=1 isaac/standalone/isaac_sim.sh
```
