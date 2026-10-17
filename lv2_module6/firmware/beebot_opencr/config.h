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

// ---- 방향·변환 (TODO(김현하): 실기에서 관절마다 부호·영점 확인)
constexpr float WHEEL_LEFT_DIR = 1.0f;
constexpr float WHEEL_RIGHT_DIR = -1.0f;              // 오른쪽 바퀴는 반대로 장착 (TB3 관례)
constexpr float ARM_DIR[ARM_COUNT] = {1, 1, 1, 1, 1};
constexpr float GRIPPER_RAD_PER_M = 60.0f;            // 그리퍼 손가락 [m] → 서보 [rad] (TODO 실측)

// ---- 시리얼 프로토콜 (SBC control/BeebotSystemHardware와 짝 — README.md "프로토콜")
constexpr uint32_t HOST_BAUD = 115200;                // USB CDC라 실제 속도와 무관
constexpr size_t LINE_CAPACITY = 160;
constexpr uint32_t COMMAND_TIMEOUT_MS = 500;          // 마지막 유효 C 명령 후 바퀴 정지·팔 현재 자세 유지
constexpr uint32_t STATE_PERIOD_MS = 25;              // S 상태 송신 주기 (모터 8개 개별 읽기 기준 — SyncRead면 더 빠르게)

// ---- OpenCR 내장 IMU (MPU9250) 변환 — TurtleBot3 OpenCR 펌웨어와 같은 값
constexpr float GYRO_FACTOR = 0.0010642f;             // ADC → rad/s (±2000 deg/s)
constexpr float ACCEL_FACTOR = 0.000598550415f;       // ADC → m/s² (±2 g)
