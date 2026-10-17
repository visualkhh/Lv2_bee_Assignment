# OpenCR 펌웨어 (beebot_opencr)

OpenCR 하나가 TurtleBot3 바퀴 2개, OpenMANIPULATOR-X(RM-X52-TNM) 4축 + 손목 roll + 그리퍼, 내장 IMU를 맡는다.
SBC(라즈베리파이·Jetson 등 로봇에 실린 컴퓨터)의 `control/BeebotSystemHardware`(ros2_control 하드웨어 플러그인)와 USB 시리얼로 짝을 이룬다.

```text
SBC /dev/opencr (udev → ttyACM*) ── USB CDC, ASCII 줄 ──> OpenCR USB Serial
OpenCR Serial3 ── DYNAMIXEL Protocol 2.0, 1,000,000 bps ──> 바퀴 ID 1·2 · 팔 ID 11~15 · 그리퍼 ID 16
OpenCR 내장 IMU (MPU9250, cIMU)
```

## 프로토콜

단위는 SI, 관절 0rad = 서보 중앙(2048 = 180°). 바꾸면 `ros2_ws/src/control/src/beebot_system_hardware.cpp`도 같이 바꾼다 (`test-hardware`가 확인).

| 방향 | 형식 | 주기 |
| --- | --- | --- |
| SBC → OpenCR | `C,<바퀴L rad/s>,<바퀴R rad/s>,<joint1>,<joint2>,<joint3>,<joint4>,<joint5 rad>,<그리퍼 m>\n` (8개) | 제어 주기 (100Hz) |
| OpenCR → SBC | `S,` 바퀴L pos·vel, 바퀴R pos·vel, joint1~5 pos·vel·전류A, 그리퍼 m·m/s·A, IMU qx·qy·qz·qw, gx·gy·gz rad/s, ax·ay·az m/s² `\n` (32개) | `STATE_PERIOD_MS` (25ms) |

- watchdog: 마지막 **유효한** `C`부터 `COMMAND_TIMEOUT_MS`(500ms)가 지나면 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (한 번).
  형식이 틀린 줄·빈 줄은 무시하고 타이머도 갱신하지 않는다.
- 부팅 때 모터 하나라도 ping이 안 되면 `C`를 받지 않는다 (상태 `S`는 계속 보내 SBC가 원인을 볼 수 있게).
- 그리퍼는 전류 기반 위치 모드 — 퍽을 쥐면 전류 한계에서 멈춘다.

## 준비

- 모터 ID (DYNAMIXEL Wizard로 미리): 바퀴 왼쪽 **1** · 오른쪽 **2**, 팔 joint1~4 **11~14**, 손목 roll **15**, 그리퍼 **16**
  (OM-X 기본은 그리퍼가 15 — 손목을 끼우면서 16으로 옮긴다). 전부 1,000,000 bps · Protocol 2.0.
- `config.h`에서 바퀴 방향, 관절 부호(`ARM_DIR`), 그리퍼 변환(`GRIPPER_RAD_PER_M`)을 실기에서 확인한다 (TODO).

## 컴파일·업로드

```bash
# 테스트베드 (도커) — 컴파일 + 가상 시리얼 업로드 검사 (test-all에 포함)
test-firmware

# 실기 PC·라즈베리파이 — 저장소에서
./upload.sh --setup                 # 최초 1회 (arduino-cli · OpenCR 코어 · 업로더)
./upload.sh [포트]                  # 컴파일 → 업로드 (기본 /dev/opencr, 없으면 /dev/ttyACM0)

# CI 배포 패키지에서 — 미리 컴파일된 .bin
../ros2_ws/firmware-upload.sh --setup && ../ros2_ws/firmware-upload.sh [포트]
```

업로드 중에는 ROS(`start.sh`, controller_manager)를 끈다 — 같은 포트를 잡으면 업로드가 깨진다.
안 되면 OpenCR의 SW2를 누른 채 RESET → 부트로더 모드에서 다시. 자세한 업로더 설명은 [upload.sh](upload.sh) 머리 주석.
