# DYNAMIXEL Wizard 2.0

- [ROBOTIS DYNAMIXEL Wizard 2.0 문서](https://docs.robotis.com/docs/software/dynamixel_wizard_2_0/introduction/)
- OpenCR에 `usb_to_dxl` 올리는 방법: [`../arduino/README.md`](../arduino/README.md)

## 실행

현재 이 PC에는 Wizard가 root 홈에 설치되어 있습니다.

```bash
sudo env DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" \
  "/root/ROBOTIS/DYNAMIXEL Wizard 2.0/DynamixelWizard2"
```

일반 계정으로 옮겨서 실행하려면:

```bash
sudo cp -a "/root/ROBOTIS/DYNAMIXEL Wizard 2.0" "$HOME/ROBOTIS/"
sudo chown -R "$USER:$USER" "$HOME/ROBOTIS/DYNAMIXEL Wizard 2.0"

"$HOME/ROBOTIS/DYNAMIXEL Wizard 2.0/DynamixelWizard2"
```

## OpenCR이 Wizard에서 안 잡히는 이유

OpenCR이 다음처럼 잡혀 있으면 일반 OpenCR 펌웨어가 동작 중인 상태입니다.

```text
ID_MODEL=OpenCR_Virtual_ComPort_in_FS_Mode
```

이 상태에서는 USB 시리얼이 Dynamixel 버스로 전달되지 않아서, Wizard가 `/dev/ttyACM0`를 열어도 모터를 찾지 못합니다.

해결:

1. **U2D2**를 PC에 직접 연결
2. OpenCR에 **`usb_to_dxl` 패스스루 예제** 업로드

2번은 [`../arduino/README.md`](../arduino/README.md)의 “OpenCR에 `usb_to_dxl` 업로드”를 따릅니다.

현재 이 환경은 `usb_to_dxl` 업로드가 완료된 상태입니다.  
ID 설정이 끝나면 원래 `beebot_opencr` 펌웨어로 복구해야 합니다.

## Wizard 검색 설정

```text
Port     : /dev/ttyACM0
Baudrate : 1,000,000
Protocol : 2.0
ID       : 1
```

- 포트는 `/dev/ttyACM0`입니다.
- `/dev/ttyS0`는 메인보드 내장 시리얼 포트이며 Dynamixel과 무관합니다.
- 검색 창에서 **ttyACM0를 목록에서 직접 선택한 뒤** 검색을 시작합니다.
- 이미 검색이 시작된 상태에서 포트를 바꾸면 진행 중인 검색은 이전 포트(예: ttyS0)를 계속 볼 수 있습니다.
- 로그에 `ttyS0`만 보이면 **중지 → ttyACM0 선택 → 다시 검색**합니다.

## 토크 / 리버스 옵션

Wizard의 체크박스 3개는 모터 내부 설정이며, 한 번 저장하면 전원을 껐다 켜도 유지됩니다.

| Wizard 항목 | 실제 설정 | 의미 | 권장 |
| --- | --- | --- | --- |
| 시작시 토크 켜기 | Startup Configuration(60) bit 0 | 전원 인가 시 Torque Enable(64)을 자동으로 1로 설정 | **끄기** |
| 목표지령시 토크켜기 | Drive Mode(10) bit 3 | Torque가 꺼져 있어도 Goal 명령을 받으면 자동으로 Torque On | **끄기** |
| 리버스 모드 | Drive Mode(10) bit 0 | CCW/CW 및 Goal/Present Position 방향을 반전 | **끄기** |

`beebot_opencr`은 모터의 자동 토크 기능에 의존하지 않고 직접 제어합니다.

```cpp
dxl.torqueOff(id);
dxl.setOperatingMode(id, mode);
dxl.setGoalPosition(...);
dxl.torqueOn(id);
```

`beebot_opencr.ino:33-41`

따라서:

- **시작시 토크 켜기**를 켜면 펌웨어가 모드를 설정하기 전에 모터가 힘을 줄 수 있으므로 끕니다.
- **목표지령시 토크켜기**를 켜면 펌웨어가 Torque Off 상태로 생각해도 Goal 명령에 의해 모터가 움직일 수 있으므로 끕니다.
- **리버스 모드**를 켜면 코드의 `WHEEL_LEFT_DIR`, `ARM_DIR`, 그리퍼 방향/영점 가정이 모두 반대로 바뀌므로 끕니다.

> 이 세 항목은 EEPROM 설정입니다. Wizard에서 켠 뒤 저장하면 `beebot_opencr`을 다시 올려도 유지됩니다.

## 목표 ID

`lv2_module6/firmware/beebot_opencr/config.h` 기준입니다.

| 모터 | ID |
| --- | ---: |
| 왼쪽 바퀴 | 1 |
| 오른쪽 바퀴 | 2 |
| 팔 joint1 | 11 |
| 팔 joint2 | 12 |
| 팔 joint3 | 13 |
| 팔 joint4 | 14 |
| 손목 roll joint5 | 15 |
| 그리퍼 | 16 |

### 15번 중복 주의

OM-X 기본 그리퍼 ID는 보통 **15**입니다.  
이 로봇은 손목 roll을 추가하면서:

- 손목 roll → **15**
- 그리퍼 → **16**

으로 옮긴 구성입니다. 같은 버스에 15가 두 개 있으면 안 됩니다.

## Wizard 없이 ID 변경

Wizard가 계속 ttyS0를 잡거나 검색이 불안정하면, Arduino CLI로 `c_ID_Change` 예제를 올려서 ID를 바꿀 수 있습니다.

예제와 명령은 [`../arduino/README.md`](../arduino/README.md)의 “Arduino만으로 ID 변경”을 참고합니다.

## 문제 해결

### ttyS0만 검색됨

- 검색 중지
- Port를 `/dev/ttyACM0`로 다시 선택
- Baud `1000000`, Protocol `2.0`으로 재검색
- 그래도 안 되면 Wizard를 완전히 종료 후 재실행

### 포트가 안 보임

```bash
ls -l /dev/ttyACM0
lsusb | grep -i -E 'OpenCR|STM'
```

OpenCR이 `0483:5740 STMicroelectronics Virtual COM Port`로 보이면 USB 인식은 된 상태입니다.

### 포트가 사용 중임

- Wizard
- Arduino 시리얼 모니터
- ROS 제어 노드

중 하나만 `/dev/ttyACM0`를 잡고 있도록 합니다.

### 모터가 안 보임

- OpenCR에 12V 전원이 들어왔는지 확인
- Dynamixel 3핀 케이블 방향/체결 확인
- `usb_to_dxl`이 실제로 업로드됐는지 확인
- 한 모터만 연결해 먼저 ping 확인
- 모터가 공장 초기 57600bps라면 baud를 바꿔 검색
