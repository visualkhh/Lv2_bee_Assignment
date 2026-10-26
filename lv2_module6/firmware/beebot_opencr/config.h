#pragma once

#include <stddef.h>
#include <stdint.h>

// ---- DYNAMIXEL 버스 (OpenCR Serial3, XM430-W350-T 전부 Protocol 2.0)
constexpr int DXL_DIR_PIN = 84;
constexpr float DXL_PROTOCOL_VERSION = 2.0f;
constexpr uint32_t DXL_BAUD = 1000000;

// ---- 모터 ID (실기에서 DYNAMIXEL Wizard로 미리 설정)
//   바퀴: TurtleBot3 기본 1·2 · 팔: OM-X(RM-X52-TNM) 기본 11~14 + 손목 roll 15(추가) · 그리퍼 16
//   (OM-X 기본은 그리퍼가 15 — 손목을 끼우면서 16으로 옮긴다)
constexpr uint8_t WHEEL_LEFT_ID = 1;
constexpr uint8_t WHEEL_RIGHT_ID = 2;
constexpr uint8_t ARM_IDS[] = {11, 12, 13, 14, 15};   // joint1~joint5
constexpr size_t ARM_COUNT = sizeof(ARM_IDS) / sizeof(ARM_IDS[0]);
constexpr uint8_t GRIPPER_ID = 16;

// ---- 방향·변환 (실기 확인: 바퀴 1번=좌·2번=우, 두 모터 같은 방향 장착.
//   TB3(거울 장착)와 달라 양쪽 DIR = +1. URDF 바퀴 축도 양쪽 "0 1 0"으로 동일.
//   TODO(김현하): 팔 관절마다 부호 확인 — 영점 오프셋은 SBC URDF <param name="offset">)
constexpr float WHEEL_LEFT_DIR = 1.0f;
constexpr float WHEEL_RIGHT_DIR = 1.0f;               // 같은 방향 장착 (TB3 거울 장착과 다름)
constexpr float ARM_DIR[ARM_COUNT] = {1, 1, 1, 1, 1};

// ---- 그리퍼: 프로토콜 값은 URDF gripper_left_joint 규약 (손가락 하나 [m], 0 = 닫힘, + = 열림)
//   제원 (ROBOTIS OpenMANIPULATOR-X RM-X52-TNM): 스트로크 20~75 mm → 손가락 하나 0~27.5 mm (URDF 상한 26.5 mm와 1 mm 이내)
//   링크 구조라 서보각과 비선형 → 실측표로 구간별 직선 (2026-10-24 캘리퍼 실측, Notion "OpenCR 지연, 그리퍼 방향 문제")
//   서보각 = 서보 중앙(2048)에서의 rad, 손가락 = (손가락 간격 − 20 mm) / 2
//     간격 20 mm(구조상 완전히 닫힘) · 27 · 30(30 mm 퍽) · 38 mm 는 실측
//     75 mm(제원 최대)는 실측 안 함 — 열림 쪽 실측 기울기(약 10.5 mm/rad)로 늘린 값 (TODO: 끝까지 열어 서보각 실측 후 교체)
//   표 밖 명령은 끝값으로 자른다. 기구 끝에 닿아도 전류 기반 위치 모드라 GRIPPER_GOAL_CURRENT_MA에서 멈춘다.
constexpr size_t GRIPPER_TABLE = 5;
constexpr float GRIPPER_FINGER_M[GRIPPER_TABLE] = {0.0f, 0.0035f, 0.005f, 0.009f, 0.0275f};      // 오름차순
constexpr float GRIPPER_SERVO_RAD[GRIPPER_TABLE] = {1.0068f, 0.6f, 0.4998f, 0.12f, -1.637f};     // 서보각이 클수록 닫힘
constexpr float GRIPPER_GOAL_CURRENT_MA = 600.0f;     // 쥐는 힘 한계 (전류 기반 위치 모드) — 퍽이 미끄러지면 올림 (TODO 실측)

// ---- 시리얼 프로토콜 (SBC control/BeebotSystemHardware와 짝 — README.md "프로토콜")
constexpr uint32_t HOST_BAUD = 115200;                // USB CDC라 실제 속도와 무관
constexpr size_t LINE_CAPACITY = 160;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;          // 마지막 유효 C 명령 후 바퀴 정지·팔 현재 자세 유지
constexpr uint32_t STATE_PERIOD_MS = 10;              // S 상태 송신 주기 — SyncRead 한 번(모터 8개 · 10 byte)이라 100 Hz (온도는 0.5 s마다 읽은 값)
constexpr uint32_t MOTOR_CHECK_MS = 500;              // 모터 토크 확인 주기 — 12V를 껐다 켜 토크가 풀린 모터를 다시 켠다 · E 줄 · 모터 온도 읽기도 이 주기
constexpr uint32_t MOTOR_RETRY_MS = 1000;             // 부팅 때 응답 없던 모터(12V 꺼짐 등) 다시 찾는 주기
constexpr uint32_t RECOVER_PROFILE_VELOCITY = 20;     // 토크를 다시 켠 팔 관절이 명령 자세로 돌아갈 때 속도 한계 (× 0.229 rpm ≈ 0.48 rad/s)
constexpr float RECOVER_DONE_RAD = 0.05f;             // 명령 자세에 이만큼 가까워지면 속도 한계 해제
constexpr uint32_t RECOVER_TIMEOUT_MS = 10000;        // 못 가도 이 시간 뒤엔 해제
constexpr uint32_t REBOOT_WAIT_MS = 1000;             // R(모터 재부팅) 후 응답 기다리는 최대 시간

// ---- DYNAMIXEL X 시리즈 컨트롤 테이블 (XM430-W350, Protocol 2.0)
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_HARDWARE_ERROR = 70;         // bit0 입력전압 · bit2 과열 · bit3 엔코더 · bit4 전기충격 · bit5 과부하 — Reboot로만 풀림
constexpr uint16_t ADDR_PROFILE_VELOCITY = 112;
constexpr uint16_t ADDR_GOAL_VELOCITY = 104;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;    // 1 byte, °C — S 끝 8칸 (Temperature Limit 기본 80 °C 넘으면 과열 Hardware Error)
constexpr uint16_t ADDR_PRESENT_CURRENT = 126;        // 126 전류(2) · 128 속도(4) · 132 위치(4) — 연속 주소라 한 번에 10 byte
constexpr float CURRENT_MA_PER_UNIT = 2.69f;
constexpr float VELOCITY_RPM_PER_UNIT = 0.229f;

// ---- OpenCR 내장 IMU (MPU9250) 변환 — TurtleBot3 OpenCR 펌웨어와 같은 값
constexpr float GYRO_FACTOR = 0.0010642f;             // ADC → rad/s (±2000 deg/s)
constexpr float ACCEL_FACTOR = 0.000598550415f;       // ADC → m/s² (±2 g)
