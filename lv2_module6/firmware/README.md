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
| SBC → OpenCR | `T,<8필드>` 또는 `T,0`·`T,1` — 토크 온/오프 (빈칸=유지, 관절별 가능) | 자세 실측·수동 정렬 |
| SBC → OpenCR | `R,<8필드>` 또는 `R,1` — 모터 재부팅 (1=재부팅, 빈칸=그대로). Hardware Error(과부하·과열 등)는 재부팅으로만 풀린다. 재부팅 후 설정·토크 켜기, 팔은 느린 복귀 | 에러 났을 때 |
| OpenCR → SBC | `E,<8 정수>` — 모터별 Hardware Error Status (0 정상 · −1 응답 없음 · 비트 1 입력전압 · 4 과열 · 8 엔코더 · 16 전기충격 · 32 과부하) | `MOTOR_CHECK_MS` (0.5 s) |
| OpenCR → SBC | `S,` 바퀴L pos·vel, 바퀴R pos·vel, joint1~5 pos·vel·전류A, 그리퍼 m·m/s·A, IMU qx·qy·qz·qw, gx·gy·gz rad/s, ax·ay·az m/s², **모터 8개 온도 °C** `\n` (40개 — 아래 칸 번호 표) | `STATE_PERIOD_MS` (10ms = 100Hz). 온도만 0.5 s(`MOTOR_CHECK_MS`)마다 읽은 마지막 값 |

### 줄 예시

한 줄 = 글자 하나 + `,`로 나눈 값 + `\n`. SBC가 보내는 `C`는 소수 넷째 자리(`%.4f`), OpenCR이 보내는 `S`는 다섯째 자리(온도는 정수).

```text
# SBC → OpenCR
C,0.3000,0.3000,0.0000,-1.0472,0.3491,0.6981,0.0000,0.0050   바퀴 0.3 rad/s 직진 · 팔 자세 · 그리퍼 손가락 5 mm(간격 30 mm)
C,0.3000,0.3000,,,,,,                                        바퀴만 — 팔·그리퍼 빈칸 = 지금 목표 유지
C,,,,,,,,                                                    전부 빈칸 = 바퀴 정지 + 팔 유지
T,0                                                          토크 전체 해제 (손으로 자세 잡기)
T,1                                                          토크 전체 켜기 (지금 자세를 목표로)
T,,,,0,,,,                                                   joint2만 해제, 나머지 그대로
R,,,,1,,,,                                                   joint2만 재부팅 (Hardware Error 풀기)
R,1                                                          전체 재부팅

# OpenCR → SBC
S,12.34567,0.00000,-3.21000,0.00000,0.00000,0.00000,0.05380,-1.04720,0.00000,0.86080,0.34910,0.00000,0.45730,0.69810,0.00000,0.16140,0.00000,0.00000,0.02690,0.00500,0.00000,0.18830,0.00000,0.00000,0.00000,1.00000,0.00000,0.00000,0.00000,0.00000,0.00000,9.81000,31,32,38,45,41,36,34,33
E,0,0,0,32,0,0,0,0                                           joint2 과부하(32) — 나머지 정상
E,0,-1,0,0,0,0,0,0                                           바퀴R 응답 없음(−1)
```

`S` 칸 번호 (위 예시 기준 — 바퀴L 누적 12.3 rad, joint2 −60°·중력 부하 0.86 A, 그리퍼 5 mm, 정지·수평, joint2 45 °C):

| 칸 | 값 | 단위 | 예시 |
| --- | --- | --- | --- |
| 1 · 2 | 바퀴L 위치 · 속도 | rad (부팅 후 누적) · rad/s | 12.34567 · 0 |
| 3 · 4 | 바퀴R 위치 · 속도 | rad · rad/s | −3.21 · 0 |
| 5 · 6 · 7 | joint1 위치 · 속도 · 전류 | rad · rad/s · A | 0 · 0 · 0.0538 |
| 8 · 9 · 10 | joint2 | rad · rad/s · A | −1.0472 · 0 · 0.8608 |
| 11 · 12 · 13 | joint3 | rad · rad/s · A | 0.3491 · 0 · 0.4573 |
| 14 · 15 · 16 | joint4 | rad · rad/s · A | 0.6981 · 0 · 0.1614 |
| 17 · 18 · 19 | joint5 (손목 roll) | rad · rad/s · A | 0 · 0 · 0.0269 |
| 20 · 21 · 22 | 그리퍼 (`gripper_left_joint`, 손가락 하나) | m · m/s · A | 0.005 · 0 · 0.1883 |
| 23 ~ 26 | IMU 자세 qx · qy · qz · qw | 쿼터니언 | 0 · 0 · 0 · 1 |
| 27 ~ 29 | IMU 각속도 gx · gy · gz | rad/s | 0 · 0 · 0 |
| 30 ~ 32 | IMU 가속도 ax · ay · az | m/s² | 0 · 0 · 9.81 |
| 33 ~ 40 | 모터 온도 바퀴L · 바퀴R · joint1~5 · 그리퍼 | °C 정수 (−1 = 모름) | 31 · 32 · 38 · 45 · 41 · 36 · 34 · 33 |

- 온도: Present Temperature(addr 146)를 0.5 s마다 읽어 둔 값. 부팅 후 첫 읽기 전·응답 없는 모터는 −1. SBC가 `/motor_status`의
  `temperature`·진단(70 °C 이상 WARN)으로 낸다. 80 °C(Temperature Limit)를 넘으면 과열 Hardware Error로 토크가 꺼진다.
- 온도 전 예전 펌웨어는 `S`가 32개(33~40 없음) — SBC는 그대로 받고 온도만 모름(−1)으로 둔다. `test-firmware`는 다시 올리라고 WARN.

`C` 필드별 단위와 펌웨어 변환:

| 필드 | 값 | 단위 | 펌웨어 변환 |
| --- | --- | --- | --- |
| 바퀴L · 바퀴R | 속도 | rad/s (바퀴 각속도) | `rpm(v × WHEEL_*_DIR) / 0.229` → Goal Velocity(104) — BulkWrite |
| joint1~5 | 위치 | rad (0 = 서보 중앙) | `raw = 2048 + ARM_DIR × rad × 4096/2π` → Goal Position(116) — BulkWrite |
| 그리퍼 | URDF `gripper_left_joint` (손가락 하나) | m, 0 = 닫힘 · + = 열림 | `config.h` 표(`GRIPPER_FINGER_M` ↔ `GRIPPER_SERVO_RAD`) 구간별 직선 → 서보 raw. 제원 스트로크 20~75 mm(손가락 0~27.5 mm, URDF 상한 26.5 mm) 전체. 간격 20~38 mm는 실측, 그 위는 실측 기울기로 늘린 값(TODO 끝까지 열어 실측) |

- 빈칸 = 유지: 팔·그리퍼는 명령 생략, 바퀴는 0(정지).
- 영점 오프셋은 펌웨어가 아니라 SBC가 처리한다: URDF `<ros2_control>` 관절 `<param name="offset">` → 보낼 때 `관절값 + offset`, 받을 때 `− offset` (단위 그대로 rad·m). 바퀴는 속도 명령이라 오프셋 없음.
- 한 번도 읽지 못한 모터(부팅 때 12V 꺼짐 등)는 `S`에 `nan`으로 보낸다. SBC는 그런 `S`를 상태로 쓰지 않으므로(해석 실패로 셈) 활성화가 실패한다 — 0 raw(−π)를 실제 자세로 믿고 팔을 보내지 않게.
- `S`도 같은 단위: 바퀴 누적 위치 rad·속도 rad/s, 관절 rad·rad/s·전류 A, 그리퍼 m·m/s·A.

- watchdog: 마지막 **유효한** `C`부터 `COMMAND_TIMEOUT_MS`(500ms)가 지나면 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (한 번).
  형식이 틀린 줄·빈 줄은 무시하고 타이머도 갱신하지 않는다.
- SBC는 활성화 때 첫 `S`를 받아 실제 각도로 팔 명령을 맞춘다. 3초 안에 `S`가 없으면 활성화를 실패시킨다(실제 각도를 모르니 띄우지 않음). 팔 명령이 없으면(NaN) 팔 칸을 **빈칸(유지)**으로 보낸다 — 예전엔 초기 상태 0을 그대로 보내 시작하자마자 팔이 0 rad로 최고 속도로 튀었음.
- 부팅 때 토크: 전원이 켜지면 모터 8개를 찾아 동작 모드·Return Delay를 맞추고 **토크를 전부 켠다** — 바퀴는 목표 속도 0,
  팔·그리퍼는 목표 = **지금 위치**(켜는 순간 튀지 않게 그 자세 유지). 첫 `C`가 오기 전에는 움직이지 않는다(watchdog이 걸린 채 시작).
  토크(`T`)는 bringup 없이 `test-firmware --torque`로 쓰거나, bringup 중이면 팔(joint1~5)·그리퍼만 `BeebotSystemHardware`가 보낸다 —
  서비스 `/beebotsystem/set_arm_torque` ← manual_control (운영 모드 TEACH에 들어가면 끔 · 나가면 켬) ← `run-cli mode teach`. 바퀴는 빈칸(유지).
- 부팅 때 모터 하나라도 ping이 안 되면 `C`를 받지 않는다 (상태 `S`는 계속 보내 SBC가 원인을 볼 수 있게).
- 그리퍼는 전류 기반 위치 모드 — 퍽을 쥐면 `GRIPPER_GOAL_CURRENT_MA`(600 mA, TODO 실측)에서 멈춘다 (닫힘 한계를 넘는 명령으로 2 A를 버티지 않게).
- 주기 구조 (2026-10-24 실기 시험: 개별 패킷이라 명령→측정 약 100 ms, S 60 ms 계단, C 100 Hz면 S 끊김 → 아래처럼 바꿈. **개선 후 실기 측정은 아직**)
  - 서보 8개를 **BulkWrite**(목표 — 바퀴 속도·팔/그리퍼 위치를 주소가 달라도 한 패킷, 모든 서보가 같은 순간 반영) · **SyncRead**(Present Current~Position 연속 10 byte) 패킷 하나로.
  - loop 한 번에 들어온 `C` 중 **마지막 하나만** 적용하고 S를 보낸다 — 밀린 C를 몰아 처리하던 것이 S를 굶기고 목표를 계단으로 튀게 했다(joint1 떨림 추정 원인).
  - 부팅 때 모든 모터의 **Return Delay Time을 0**으로 (EEPROM, 다를 때만 씀).
- 모터 에러: 펌웨어가 0.5 s마다 Hardware Error Status를 읽어 `E` 줄로 보낸다. 에러 난 모터는 토크가 안 켜지므로 자동 복구를 건너뛰고 재부팅을 기다린다(걸린 채 다시 힘을 주지 않게 자동 재부팅은 안 함). SBC는 `/motor_status`(control/MotorStatus — 다른 노드가 읽는 값, state_manager → `/state/devices/arm` error)와 진단 "OpenCR 연결"(사람이 보는 것)에 `joint2 과부하`처럼 내고, 서비스 `/beebotsystem/reboot_motors`(std_srvs/Trigger)를 부르거나 수동 모드에서 `/cmd/manual/reboot_motors`(std_msgs/Empty)를 보내면 **에러 난 모터만** `R`로 재부팅한다 (멀쩡한 관절까지 재부팅하면 팔이 떨어짐).
- 12V 복구: 12V를 껐다 켜면 OpenCR은 USB 5V로 켜져 있고 모터만 재부팅돼 토크가 풀린다. 펌웨어가 `MOTOR_CHECK_MS`(0.5 s)마다 Torque Enable을 읽어 풀린 모터를 다시 설정하고 켠다. 팔은 그 사이 명령 자세와 멀어졌을 수 있어 **느린 속도(`RECOVER_PROFILE_VELOCITY` ≈ 0.48 rad/s)로 돌아간 뒤** 속도 한계를 푼다. `T,1`로 켤 때도 같다. 부팅 때 모터가 없었으면(12V 꺼짐) 1 s마다 다시 찾는다 — RESET 버튼 불필요.

## 준비

- 모터 ID (DYNAMIXEL Wizard로 미리): 바퀴 왼쪽 **1** · 오른쪽 **2**, 팔 joint1~4 **11~14**, 손목 roll **15**, 그리퍼 **16**
  (OM-X 기본은 그리퍼가 15 — 손목을 끼우면서 16으로 옮긴다). 전부 1,000,000 bps · Protocol 2.0.
- `config.h`에서 바퀴 방향, 관절 부호(`ARM_DIR`), 그리퍼 실측표(`GRIPPER_FINGER_M`·`GRIPPER_SERVO_RAD`)·쥐는 전류를 실기에서 확인한다. 팔 영점은 펌웨어가 아니라 URDF `offset`.

## 설치·컴파일·업로드

스크립트는 단계별로 나뉜다.

| 스크립트 | 역할 |
| --- | --- |
| `setup.sh` | arduino-cli·OpenCR 코어·컴파일러·업로더 설치 (최초 1회) |
| `compile.sh [스케치]` | 컴파일만 → `build/<스케치>/*.ino.bin` |
| `upload.sh [포트] [파일]` | `.bin`/`.opencr` 업로드 (기본: 최신 `.bin`, `/dev/opencr`) |
| `setup_compile_upload.sh [포트] [스케치]` | 설치 + 컴파일 + 업로드를 한 번에 |

```bash
# 최초 1회 또는 툴이 바뀌었을 때
./setup.sh

# 컴파일만
./compile.sh
./compile.sh beebot_opencr

# 업로드만 (기본 포트 /dev/opencr → 없으면 /dev/ttyACM0)
./upload.sh
./upload.sh /dev/ttyACM0
./upload.sh /dev/ttyACM0 build/beebot_opencr/beebot_opencr.ino.bin

# 설치부터 한 번에
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```

CI 배포 패키지에서 미리 컴파일된 `.bin`을 올릴 때:

```bash
../ros2_ws/firmware-upload.sh --setup
../ros2_ws/firmware-upload.sh /dev/ttyACM0
```

업로드 중에는 ROS(`start.sh`, controller_manager)를 끈다 — 같은 포트를 잡으면 업로드가 깨진다.
안 되면 OpenCR의 SW2를 누른 채 RESET → 부트로더 모드에서 다시. 공통 경로·업로더 설정은 [common.sh](common.sh)를 본다.

## 실기 시리얼 테스트

업로드 후 ROS 없이 OpenCR의 `S` 상태와 `C` 명령을 확인한다.  
스크립트는 `test-firmware`이다.

### 사용법

```bash
cd lv2_module6/firmware

chmod +x test-firmware          # 최초 1회 (Permission denied 방지)

./test-firmware                  # S 상태만 읽음 — 로봇 안 움직임
./test-firmware --raw            # 받은 S 줄 원문 출력
./test-firmware --port /dev/ttyACM0

./test-firmware --hold --yes     # 현재 자세 유지 명령
./test-firmware --arm joint1 0.05 --yes
./test-firmware --gripper 0.005 --yes          # 손가락 간격 30 mm (= 20 + 2×5 mm)
./test-firmware --wheel 0.3 --yes       # 양쪽 바퀴 0.3 rad/s (팔·그리퍼 현재 유지)
./test-firmware --l-wheel 0.3 --yes     # 왼쪽만 (오른쪽 정지)
./test-firmware --r-wheel 0.3 --yes     # 오른쪽만 (왼쪽 정지)
./test-firmware --send 'C,0,0,0,0,0,0,0,0' --yes
./test-firmware --send 'C,0.3,0.3,,,,,,' --yes  # 빈칸 = 유지 (아래 규칙)
./test-firmware --torque off --yes              # 전체 토크 해제 (손으로 자세 잡기)
./test-firmware --torque on --yes               # 전체 토크 켜기 (지금 자세를 목표로)
./test-firmware --torque 1,0,,,,,, --yes        # 조인트별 (C와 같은 순서, 빈칸=유지)
./test-firmware --torque joint2=0,joint4=1 --yes
```

`sh ./test-firmware`로 실행하면 Ubuntu의 `/bin/sh`(dash)가 되어
`set: Illegal option -o pipefail` 오류가 난다. 반드시 `./` 또는 `bash`로 실행한다.

```bash
./test-firmware
# 또는
bash ./test-firmware
```

### 옵션

| 옵션 | 의미 |
| --- | --- |
| `--port <포트>` | 기본 `/dev/opencr`, 없으면 `/dev/ttyACM0` |
| `--raw` | `S` 줄 원문 출력 |
| `--lines <n>` | `--raw`에서 보여줄 줄 수 (기본 3) |
| `--hold` | 현재 자세를 유지하는 `C` 명령 전송 |
| `--arm <joint> <rad>` | 현재 자세에서 해당 관절만 delta만큼 이동 |
| `--gripper <m>` | 그리퍼를 `C`와 같은 값으로 이동 — 손가락 **하나**의 이동량 m (0 = 닫힘). 손가락 간격 = 20 mm + 2 × 값 (예: 0.005 → 30 mm) |
| `--wheel <vl> [vr]` | 바퀴 속도 rad/s (vr 생략 시 양쪽 동일, 팔·그리퍼는 S 현재값 유지) |
| `--l-wheel <v>` / `--r-wheel <v>` | 한쪽 바퀴만 속도 (반대쪽은 정지) |
| `--count <n>` | `C` 전송 횟수, 1회 = 0.05초 (기본: `--hold`·`--send` 5회, `--arm`·`--gripper`·바퀴 20회, 최대 1200) |
| `--send 'C,...'` | `C` 명령을 직접 전송 (빈 필드 `,,`는 유지 — 아래 규칙) |
| `--torque on\|off` | 전체 토크 켜기/끄기 (`T,1` · `T,0`). 끄면 손으로 자세를 잡을 수 있다 |
| `--torque <8필드>` | 조인트별 토크 (C와 같은 순서, 빈칸=지금 상태 유지, `1`=켜기 `0`=끄기) |
| `--reboot <모터,…\|all>` | 모터 재부팅 `R` (예: `joint2,gripper`). 상태 출력에 `E`(모터 에러)·모터 온도(`S` 끝)도 같이 나온다. `all`은 팔이 잠깐 힘이 빠지니 받치고 |
| `--yes` | 확인 프롬프트 없이 실행 |

- `--arm`, `--gripper`, `--wheel`, `--send`는 실제로 모터를 움직인다. 주변을 비우고 작은 값부터 사용한다.
- `C` 빈 필드는 **유지**다: 팔·그리퍼는 현재 목표 유지, 바퀴는 정지(0). 전부 빈칸이면 바퀴 정지 + 팔 유지. 예: `C,0.3,0.3,,,,,,` (바퀴만 구동), `C,1,,0.4,,,,,` (좌바퀴 1 · 우 정지 · joint1 0.4).
- 필드가 모자라면 **뒷부분을 빈칸(유지)으로 채운다** — `C,0.3,0.3`만 보내도 바퀴만 돌고 팔은 유지. 8개를 넘거나 숫자 뒤에 쓰레기가 있으면 줄 전체를 버린다. `--send`도 같은 규칙으로 자동 채운다.
- `--send 'C,0,0,...'`처럼 전부 숫자로 보내면 팔을 0 rad(중앙)으로 보내는 명령이므로, 현재 자세 유지는 `--hold`나 빈칸을 쓴다.
- 모터 8개(바퀴 1·2, 팔 11~15, 그리퍼 16)가 모두 1 Mbps / Protocol 2.0으로 연결되어 있지 않으면 `C`가 무시된다.
- 그리퍼는 Current-based Position의 Goal Current가 0이면 움직이지 않을 수 있다. (`setGoalCurrent()` 확인)

### 결과 해석

```text
S 40개 필드                 → beebot_opencr 정상 (32개면 모터 온도 없는 예전 펌웨어 — 다시 업로드)
S 필드가 9개/8개            → 다른 펌웨어이거나, 시리얼 timeout으로 마지막 줄이 잘린 경우
S가 안 나옴                 → 포트·업로드·권한 문제
S만 나오고 C가 무시됨       → 모터 일부 미연결 또는 ID/baud 오류
```

> 시리얼을 timeout으로 끊으면 마지막 `S` 줄이 중간에서 잘릴 수 있다.
> `test-firmware`는 **40개(예전 펌웨어 32개) 필드가 모두 있는 완전한 줄만** 사용하므로, 중간에 잘린 줄은 무시한다.

`S` 필드가 8~9개로 나오는 경우:

```bash
./test-firmware --raw
# 또는
stty -F /dev/ttyACM0 raw -echo 115200
timeout 2 cat /dev/ttyACM0 | head -3
```

`S,` 뒤 값이 8개뿐이면 `beebot_opencr`의 40값 프로토콜이 아니다.  
OpenCR에 올라간 펌웨어를 확인하고 `beebot_opencr`을 다시 업로드한다.

```bash
./setup_compile_upload.sh /dev/ttyACM0 beebot_opencr
```
