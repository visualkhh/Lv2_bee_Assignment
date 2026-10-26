// beebot OpenCR 펌웨어 — SBC(control/BeebotSystemHardware) ↔ OpenCR ↔ DYNAMIXEL 8개 + 내장 IMU
//   USB 시리얼 ASCII 줄 프로토콜 (module5 opencr_pan_tilt와 같은 방식, 단위는 SI):
//     SBC → OpenCR  C,<바퀴L rad/s>,<바퀴R rad/s>,<j1>,<j2>,<j3>,<j4>,<j5 rad>,<그리퍼 m>\n      (매 제어 주기)
//                        빈 필드(`,,` ·trailing comma · 모자란 뒷필드)는 "유지" — 팔·그리퍼는 명령 생략(현재 유지),
//                        바퀴는 정지(0). 전부 빈칸이면 바퀴 정지 + 팔 유지. 예: C,0.3,0.3,,,,,,
//                        (빈칸 없이 전부 숫자였던 기존 C와 호환 — SBC는 팔 명령이 없을 때(NaN) 팔 칸을 빈칸으로 보냄)
//     OpenCR → SBC  S,<바퀴L pos,vel>,<바퀴R pos,vel>,<j1 pos,vel,A>…<j5 …>,<그리퍼 m,m/s,A>,
//                        <qx,qy,qz,qw>,<gx,gy,gz rad/s>,<ax,ay,az m/s²>,<모터 8개 온도 °C>\n          (STATE_PERIOD_MS마다, 40개)
//                        온도 = 바퀴L·R·joint1~5·그리퍼 순 정수 (−1 = 모름). 0.5 s(MOTOR_CHECK_MS)마다 읽은 마지막 값
//   관절 0rad = 서보 중앙(2048 = 180°). 바퀴 위치는 부팅 후 누적 rad.
//   그리퍼 = URDF gripper_left_joint 규약 (손가락 하나 m, 0 = 닫힘) — 서보각과는 config.h 실측표로 변환.
//   토크: T,0 = 전체 해제(손으로 자세 잡기) · T,1 = 전체 켜기(현재 자세 목표) ·
//         T,<8필드> = 관절별(빈칸=지금 상태 유지, 1=켜기 0=끄기). 예: T,1,0,,,,,,
//   재부팅: R,<8필드> = 관절별 DYNAMIXEL Reboot(1=재부팅) · R,1 = 전체. 과부하·과열 등 Hardware Error는 Reboot로만 풀린다.
//         재부팅 후 모드·Return Delay·전류를 다시 설정하고 토크를 켠다 (팔은 느린 속도로 명령 자세 복귀).
//     OpenCR → SBC  E,<8 정수>\n  모터별 Hardware Error Status (0 = 정상, -1 = 응답 없음) — MOTOR_CHECK_MS마다
//   watchdog: 마지막 유효 C부터 COMMAND_TIMEOUT_MS 지나면 바퀴 속도 0, 팔·그리퍼는 현재 자세 유지 (한 번)
//
// 주기 구조 (2026-10-24 실기 시험: 개별 패킷이라 S가 60 ms 계단·C 100 Hz면 S 끊김 → 아래로 바꿈)
//   - 서보 8개를 BulkWrite(목표 — 바퀴 Goal Velocity · 팔/그리퍼 Goal Position을 한 패킷) · SyncRead(전류·속도·위치 연속 10 byte) 패킷 하나로
//   - loop 한 번에 들어온 C 중 마지막 하나만 적용 (밀린 C를 몰아 처리하면 S가 굶고 목표가 계단으로 튐)
//   - Return Delay Time 0 (기본 250 µs가 응답마다 붙음)
//   - 12V를 껐다 켜 토크가 풀린 모터는 다시 찾아 켜고, 팔은 느린 속도로 명령 자세에 돌아간다
// TODO(김현하): 관절 부호 실측, 그리퍼 쥐는 전류 실측
#include <Dynamixel2Arduino.h>
#include <IMU.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

Dynamixel2Arduino dxl(Serial3, DXL_DIR_PIN);
cIMU imu;

// 모터 순서 = 프로토콜 순서: 바퀴L, 바퀴R, joint1~5, 그리퍼
constexpr size_t MOTOR_COUNT = 2 + ARM_COUNT + 1;
constexpr size_t GRIPPER_INDEX = 2 + ARM_COUNT;
constexpr float RAW_PER_RAD = 4096.0f / (2.0f * PI);

char line[LINE_CAPACITY];
size_t line_length = 0;
char pending_c[LINE_CAPACITY];  // 이번 loop에 들어온 마지막 C
bool has_pending_c = false;
bool motors_ready = false;
bool watchdog_tripped = true;  // 첫 C 전에는 움직이지 않음
uint32_t last_command_ms = 0;
uint32_t last_state_ms = 0;
uint32_t last_check_ms = 0;
bool torque_state[MOTOR_COUNT] = {true, true, true, true, true, true, true, true};
int32_t goal_raw[MOTOR_COUNT] = {};        // 마지막 위치 목표 (복구 완료 판정)
uint32_t recover_since[MOTOR_COUNT] = {};  // 0이 아니면 느린 속도로 복귀 중
int16_t hw_error[MOTOR_COUNT] = {};        // Hardware Error Status (−1 = 응답 없음) — E 줄로 보고
int16_t hw_temp[MOTOR_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1};  // Present Temperature °C (−1 = 모름) — S 끝 8칸

// ---- SyncRead: Present Current(126, 2) · Velocity(128, 4) · Position(132, 4)
struct __attribute__((packed)) Present
{
    int16_t current;
    int32_t velocity;
    int32_t position;
};
Present present[MOTOR_COUNT] = {};
bool seen[MOTOR_COUNT] = {};  // 한 번이라도 읽었나 — 못 읽은 모터는 S에 nan (0 raw = −π를 실제 자세로 믿지 않게)
Present present_rx[MOTOR_COUNT];
uint8_t sr_buf[64];
DYNAMIXEL::InfoSyncReadInst_t sr;
DYNAMIXEL::XELInfoSyncRead_t sr_xels[MOTOR_COUNT];

// ---- BulkWrite: 모터마다 주소가 달라도 한 패킷 — 바퀴 Goal Velocity(104, 4) · 팔·그리퍼 Goal Position(116, 4).
//      모든 서보가 같은 패킷을 같은 순간에 받아 반영 (응답 없음). 보낼 모터만 골라 매번 다시 만든다
int32_t bw_value[MOTOR_COUNT];
uint8_t bw_buf[128];
DYNAMIXEL::InfoBulkWriteInst_t bw;
DYNAMIXEL::XELInfoBulkWrite_t bw_xels[MOTOR_COUNT];

uint8_t motor_id(size_t i)
{
    if (i == 0)
        return WHEEL_LEFT_ID;
    if (i == 1)
        return WHEEL_RIGHT_ID;
    if (i < GRIPPER_INDEX)
        return ARM_IDS[i - 2];
    return GRIPPER_ID;
}

float rpm_to_rad_s(float rpm) { return rpm * 2.0f * PI / 60.0f; }
float rad_s_to_rpm(float rad_s) { return rad_s * 60.0f / (2.0f * PI); }
int32_t rad_to_raw(float rad, float dir) { return lroundf(2048.0f + dir * rad * RAW_PER_RAD); }
float raw_to_rad(int32_t raw, float dir) { return dir * (raw - 2048) / RAW_PER_RAD; }

// 단조 표 구간별 직선 (xs 오름·내림 둘 다, 표 밖은 끝값)
float interp(float x, const float *xs, const float *ys, size_t n)
{
    const bool up = xs[n - 1] > xs[0];
    if (up ? x <= xs[0] : x >= xs[0])
        return ys[0];
    for (size_t i = 1; i < n; ++i)
        if (up ? x <= xs[i] : x >= xs[i])
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[n - 1];
}
float gripper_m_to_servo_rad(float m) { return interp(m, GRIPPER_FINGER_M, GRIPPER_SERVO_RAD, GRIPPER_TABLE); }
float gripper_servo_rad_to_m(float rad) { return interp(rad, GRIPPER_SERVO_RAD, GRIPPER_FINGER_M, GRIPPER_TABLE); }

bool is_position_motor(size_t i) { return i >= 2; }

// 토크 켜기 — 목표 = 지금 위치(튀지 않게). recover면 팔이 명령 자세로 돌아갈 때까지 속도 한계
void enable_motor(size_t i, bool recover)
{
    const uint8_t id = motor_id(i);
    if (i == GRIPPER_INDEX)
        dxl.setGoalCurrent(id, GRIPPER_GOAL_CURRENT_MA, UNIT_MILLI_AMPERE);  // RAM이라 전원 복귀 때마다
    if (!is_position_motor(i))
    {
        dxl.setGoalVelocity(id, 0);
    }
    else
    {
        goal_raw[i] = dxl.getPresentPosition(id, UNIT_RAW);
        dxl.setGoalPosition(id, goal_raw[i], UNIT_RAW);
        if (recover && i != GRIPPER_INDEX)
        {
            dxl.write(id, ADDR_PROFILE_VELOCITY, reinterpret_cast<const uint8_t *>(&RECOVER_PROFILE_VELOCITY), 4);
            recover_since[i] = millis() | 1;
        }
    }
    dxl.torqueOn(id);
    torque_state[i] = true;
}

bool setup_motor(size_t i, bool recover)
{
    const uint8_t id = motor_id(i);
    if (!dxl.ping(id))
        return false;
    dxl.torqueOff(id);
    const uint8_t mode = i < 2 ? OP_VELOCITY : (i == GRIPPER_INDEX ? OP_CURRENT_BASED_POSITION : OP_POSITION);
    if (dxl.readControlTableItem(ControlTableItem::OPERATING_MODE, id) != mode)  // EEPROM — 다를 때만 씀 (복구가 반복돼도 닳지 않게)
        dxl.setOperatingMode(id, mode);
    if (dxl.readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, id) != 0)  // EEPROM — 다를 때만 씀
        dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, id, 0);
    enable_motor(i, recover);
    return true;
}

bool setup_motors()
{
    bool ok = true;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
        ok = setup_motor(i, false) && ok;
    return ok;
}

void setup_sync()
{
    sr.packet.p_buf = sr_buf;
    sr.packet.buf_capacity = sizeof(sr_buf);
    sr.packet.is_completed = false;
    sr.addr = ADDR_PRESENT_CURRENT;
    sr.addr_length = sizeof(Present);
    sr.p_xels = sr_xels;
    sr.xel_count = MOTOR_COUNT;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        sr_xels[i].id = motor_id(i);
        sr_xels[i].p_recv_buf = reinterpret_cast<uint8_t *>(&present_rx[i]);
    }
    sr.is_info_changed = true;

    bw.packet.p_buf = bw_buf;
    bw.packet.buf_capacity = sizeof(bw_buf);
    bw.packet.is_completed = false;
    bw.p_xels = bw_xels;
}

// 모아 둔 목표를 BulkWrite 하나로 (send[i]인 모터만 — 바퀴는 속도, 나머지는 위치 주소)
void bulk_write(const bool *send)
{
    bw.xel_count = 0;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (!send[i])
            continue;
        auto &x = bw_xels[bw.xel_count++];
        x.id = motor_id(i);
        x.addr = is_position_motor(i) ? ADDR_GOAL_POSITION : ADDR_GOAL_VELOCITY;
        x.addr_length = 4;
        x.p_data = reinterpret_cast<uint8_t *>(&bw_value[i]);
    }
    if (bw.xel_count == 0)
        return;
    bw.is_info_changed = true;  // 값이 패킷 안에 들어가므로 매번 다시 만든다
    dxl.bulkWrite(&bw);
}

void read_present()
{
    // 응답 없는 모터에서 멈추므로 앞에서부터 받은 개수만 반영 (나머지는 직전 값 유지)
    const uint8_t n = dxl.syncRead(&sr);
    for (uint8_t i = 0; i < n && i < MOTOR_COUNT; ++i)
    {
        present[i] = present_rx[i];
        seen[i] = true;
    }
}

void hold_present_pose()
{
    dxl.setGoalVelocity(WHEEL_LEFT_ID, 0);
    dxl.setGoalVelocity(WHEEL_RIGHT_ID, 0);
    for (size_t i = 2; i < MOTOR_COUNT; ++i)
    {
        goal_raw[i] = dxl.getPresentPosition(motor_id(i), UNIT_RAW);
        dxl.setGoalPosition(motor_id(i), goal_raw[i], UNIT_RAW);
    }
}

bool parse_fields(char *text, float *out, size_t count)
{
    // 필드 수는 유연하게: 모자란 뒷 필드는 빈칸(유지)으로 채운다.
    //   C,0.3,0.3            → 바퀴만 (팔·그리퍼 유지)
    //   C,0.3,0.3,,,,,       → 위와 같음 (빈칸 명시)
    //   C,0.3,0.3,+1(더 많음) → 형식 오류로 줄 전체 버림
    size_t i = 0;
    while (true)
    {
        if (*text == ',' || *text == '\0')
        {
            out[i] = NAN;  // 빈칸 = 유지 (바퀴는 0으로, 팔·그리퍼는 명령 생략)
        }
        else
        {
            char *end = nullptr;
            out[i] = strtof(text, &end);
            if (end == text || !isfinite(out[i]))
                return false;
            text = end;
        }
        ++i;
        if (*text == '\0')
            break;
        if (*text != ',')
            return false;
        ++text;
        if (i == count)
            return false;  // 필드가 count개를 넘음
    }
    for (; i < count; ++i)
        out[i] = NAN;  // 생략된 뒷 필드 = 유지
    return true;
}

// T 명령 — 토크 온/오프. 필드 8개 = C와 같은 순서(바퀴L, 바퀴R, joint1~5, 그리퍼).
//   T,0                     → 전체 해제
//   T,1                     → 전체 켜기 (지금 자세를 목표로)
//   T,1,0,,,,,,             → 왼바퀴만 켜고 오른바퀴 끔, 나머지 유지
//   T,,0,,,,,,             → joint1만 끄기 (빈칸 = 지금 상태 유지)
// 켤 때는 그 관절의 현재 위치를 목표로 잡고, 명령 자세로는 느린 속도로 돌아간다 (손으로 옮겨 둔 팔이 튀지 않게).
void set_torque_one(size_t i, bool on)
{
    if (on)
    {
        enable_motor(i, true);
    }
    else
    {
        dxl.torqueOff(motor_id(i));
        torque_state[i] = false;
    }
}

bool any_torque_on()
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
        if (torque_state[i])
            return true;
    return false;
}

void apply_torque_line(char *text)
{
    const bool whole = (strchr(text + 2, ',') == nullptr);  // T,0 / T,1 = 전체
    float v[MOTOR_COUNT];
    if (!parse_fields(text + 2, v, MOTOR_COUNT))
        return;
    if (whole)
    {
        if (isnan(v[0]))
            return;
        const bool on = v[0] != 0.0f;
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
            set_torque_one(i, on);
    }
    else
    {
        for (size_t i = 0; i < MOTOR_COUNT; ++i)
            if (!isnan(v[i]))
                set_torque_one(i, v[i] != 0.0f);
    }
    // 하나라도 토크가 켜져 있으면 watchdog 유지, 전부 꺼졌으면 watchdog 정지
    watchdog_tripped = !any_torque_on();
    last_command_ms = millis();
}

void apply_command(char *text)
{
    float v[MOTOR_COUNT];
    if (!parse_fields(text + 2, v, MOTOR_COUNT))
        return;
    bool send[MOTOR_COUNT] = {};
    // 바퀴: 빈칸·토크 꺼짐 = 정지
    const float wheel_dir[] = {WHEEL_LEFT_DIR, WHEEL_RIGHT_DIR};
    for (size_t i = 0; i < 2; ++i)
    {
        const float rad_s = (isnan(v[i]) || !torque_state[i]) ? 0.0f : v[i] * wheel_dir[i];
        bw_value[i] = lroundf(rad_s_to_rpm(rad_s) / VELOCITY_RPM_PER_UNIT);
        send[i] = true;
    }
    // 팔·그리퍼: 빈칸·토크 꺼짐 = 명령 생략 (현재 목표 유지)
    for (size_t i = 2; i < MOTOR_COUNT; ++i)
    {
        if (isnan(v[i]) || !torque_state[i])
            continue;
        bw_value[i] = i == GRIPPER_INDEX ? rad_to_raw(gripper_m_to_servo_rad(v[i]), 1.0f)
                                         : rad_to_raw(v[i], ARM_DIR[i - 2]);
        goal_raw[i] = bw_value[i];
        send[i] = true;
    }
    bulk_write(send);  // 바퀴·팔·그리퍼 한 패킷 — 동시에 반영
    last_command_ms = millis();
    if (any_torque_on())
        watchdog_tripped = false;
}

// 들어온 바이트를 줄로 — T는 바로 적용, C는 마지막 것만 남긴다 (적용은 loop에서 한 번)
void read_host()
{
    for (int n = Serial.available(); n > 0; --n)  // 지금 와 있는 만큼만 (계속 들어와도 loop가 막히지 않게)
    {
        const char ch = static_cast<char>(Serial.read());
        if (ch == '\n')
        {
            line[line_length] = '\0';
            if (strncmp(line, "C,", 2) == 0)
            {
                memcpy(pending_c, line, line_length + 1);
                has_pending_c = true;
            }
            else if (strncmp(line, "T,", 2) == 0 && motors_ready)
            {
                apply_torque_line(line);
            }
            else if (strncmp(line, "R,", 2) == 0 && motors_ready)
            {
                apply_reboot_line(line);
            }
            line_length = 0;
        }
        else if (ch != '\r' && line_length < LINE_CAPACITY - 1)
        {
            line[line_length++] = ch;
        }
        else if (line_length >= LINE_CAPACITY - 1)
        {
            line_length = 0;  // 너무 긴 줄은 버림
        }
    }
}

void send_errors()
{
    Serial.print('E');
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        Serial.print(',');
        Serial.print(hw_error[i]);
    }
    Serial.println();
}

void read_error(size_t i)
{
    uint8_t err = 0;
    hw_error[i] = dxl.read(motor_id(i), ADDR_HARDWARE_ERROR, 1, &err, 1) == 1 ? err : -1;
}

// 온도는 빨리 안 변해서 10 ms마다 읽지 않고 E와 같은 0.5 s에 읽어 두고(모터당 1 byte — 버스 부담 작음), S에는 마지막 값을 싣는다
void read_temperature(size_t i)
{
    uint8_t temp = 0;
    hw_temp[i] = dxl.read(motor_id(i), ADDR_PRESENT_TEMPERATURE, 1, &temp, 1) == 1 ? temp : -1;
}

// R 명령 — 고른 모터를 DYNAMIXEL Reboot → 다시 응답하면 설정·토크 켜기 (팔은 느린 복귀)
void apply_reboot_line(char *text)
{
    const bool whole = (strchr(text + 2, ',') == nullptr);  // R,1 = 전체
    float v[MOTOR_COUNT];
    if (!parse_fields(text + 2, v, MOTOR_COUNT))
        return;
    bool pick[MOTOR_COUNT] = {};
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
        pick[i] = whole ? (!isnan(v[0]) && v[0] != 0.0f) : (!isnan(v[i]) && v[i] != 0.0f);
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
        if (pick[i])
            dxl.reboot(motor_id(i));
    const uint32_t start = millis();
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        if (!pick[i])
            continue;
        while (!dxl.ping(motor_id(i)) && millis() - start < REBOOT_WAIT_MS)
            delay(20);
        setup_motor(i, true);
        read_error(i);
    }
    send_errors();
    last_command_ms = millis();
}

void check_command_timeout()
{
    if (!motors_ready || watchdog_tripped || millis() - last_command_ms < COMMAND_TIMEOUT_MS)
        return;
    hold_present_pose();
    watchdog_tripped = true;
}

// 12V를 껐다 켜면 OpenCR(USB 5V)은 그대로인데 모터만 재부팅돼 토크가 풀린다 → 찾아서 다시 켬.
// 부팅 때 모터가 없었으면(motors_ready=false) 주기적으로 다시 찾는다.
void check_motors()
{
    const uint32_t now = millis();
    if (!motors_ready)
    {
        if (now - last_check_ms < MOTOR_RETRY_MS)
            return;
        last_check_ms = now;
        motors_ready = setup_motors();
        return;
    }
    if (now - last_check_ms < MOTOR_CHECK_MS)
        return;
    last_check_ms = now;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
        const uint8_t id = motor_id(i);
        if (recover_since[i])
        {
            // 느린 복귀 끝: 명령 자세 근처이거나 시간 초과 → 속도 한계 해제 (0 = 무제한)
            const bool near = abs(present[i].position - goal_raw[i]) < RECOVER_DONE_RAD * RAW_PER_RAD;
            if (near || now - recover_since[i] > RECOVER_TIMEOUT_MS)
            {
                const uint32_t unlimited = 0;
                dxl.write(id, ADDR_PROFILE_VELOCITY, reinterpret_cast<const uint8_t *>(&unlimited), 4);
                recover_since[i] = 0;
            }
        }
        read_error(i);
        read_temperature(i);  // 과열(Hardware Error)로 멈추기 전에 SBC가 오르는 걸 보게
        if (!torque_state[i] || hw_error[i] != 0)
            continue;  // Hardware Error면 토크가 안 켜진다 — R로 재부팅해야 함 (E 줄로 SBC에 알림)
        uint8_t enabled = 0;
        if (dxl.read(id, ADDR_TORQUE_ENABLE, 1, &enabled, 1) == 1 && enabled == 0)
            setup_motor(i, true);  // 재부팅된 모터 — 모드·Return Delay·쥐는 전류 다시, 팔은 느린 복귀
    }
    send_errors();
}

void print_value(float value)
{
    Serial.print(',');
    Serial.print(value, 5);  // NAN은 "nan" — SBC strtod가 NaN으로 읽는다
}

float or_nan(size_t i, float value) { return seen[i] ? value : NAN; }

void send_state()
{
    read_present();
    Serial.print('S');
    const float wheel_dir[] = {WHEEL_LEFT_DIR, WHEEL_RIGHT_DIR};
    for (size_t i = 0; i < 2; ++i)
    {
        print_value(or_nan(i, wheel_dir[i] * present[i].position / RAW_PER_RAD));
        print_value(or_nan(i, wheel_dir[i] * rpm_to_rad_s(present[i].velocity * VELOCITY_RPM_PER_UNIT)));
    }
    for (size_t i = 2; i < GRIPPER_INDEX; ++i)
    {
        const float dir = ARM_DIR[i - 2];
        print_value(or_nan(i, raw_to_rad(present[i].position, dir)));
        print_value(or_nan(i, dir * rpm_to_rad_s(present[i].velocity * VELOCITY_RPM_PER_UNIT)));
        print_value(or_nan(i, present[i].current * CURRENT_MA_PER_UNIT / 1000.0f));
    }
    const Present &g = present[GRIPPER_INDEX];
    const float servo = raw_to_rad(g.position, 1.0f);
    const float slope = (gripper_servo_rad_to_m(servo + 0.01f) - gripper_servo_rad_to_m(servo - 0.01f)) / 0.02f;  // m/rad
    print_value(or_nan(GRIPPER_INDEX, gripper_servo_rad_to_m(servo)));
    print_value(or_nan(GRIPPER_INDEX, slope * rpm_to_rad_s(g.velocity * VELOCITY_RPM_PER_UNIT)));
    print_value(or_nan(GRIPPER_INDEX, g.current * CURRENT_MA_PER_UNIT / 1000.0f));

    imu.update();
    print_value(imu.quat[1]);  // cIMU quat = w, x, y, z
    print_value(imu.quat[2]);
    print_value(imu.quat[3]);
    print_value(imu.quat[0]);
    for (size_t i = 0; i < 3; ++i)
        print_value(imu.SEN.gyroADC[i] * GYRO_FACTOR);
    for (size_t i = 0; i < 3; ++i)
        print_value(imu.SEN.accADC[i] * ACCEL_FACTOR);
    for (size_t i = 0; i < MOTOR_COUNT; ++i)  // 모터 온도 °C (정수, −1 = 모름)
    {
        Serial.print(',');
        Serial.print(hw_temp[i]);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(HOST_BAUD);
    imu.begin();
    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    setup_sync();
    motors_ready = setup_motors();  // 하나라도 응답이 없으면 명령을 받지 않는다 (상태는 계속 보냄 · check_motors가 다시 찾음)
    last_check_ms = millis();
}

void loop()
{
    read_host();
    if (has_pending_c)
    {
        has_pending_c = false;
        if (motors_ready)
            apply_command(pending_c);
    }
    check_command_timeout();
    if (millis() - last_state_ms >= STATE_PERIOD_MS)
    {
        last_state_ms = millis();
        send_state();
    }
    check_motors();
}
