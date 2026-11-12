#pragma once

// BeebotSystemHardware (🛠 직접 작성, 담당: 김현하) — ros2_control hardware component (SystemInterface)
// controller_manager가 URDF의 <ros2_control> 태그를 보고 이 플러그인을 올린다. 설계: documents/project/ros/08-ctrl.md
//   관절·센서 (lv2_module6/urdf/beebot.urdf.xacro):
//     wheel_left/right_joint   command velocity · state position, velocity          (TurtleBot3 2륜)
//     joint1~5                 command position · state position, velocity, effort  (OM-X 4축 + 손목 roll)
//     gripper_left_joint       command position · state position, velocity, effort  (평행 그리퍼, 오른쪽은 mimic)
//     imu (sensor)             state orientation·angular_velocity·linear_acceleration
//   한 주기(100Hz): read() → 컨트롤러 update → write()
//
// mock:=true (테스트베드): 실기 없이 명령을 그대로 상태로 되돌린다 — 바퀴는 속도를 적분해 위치, 팔은 목표각 = 현재각,
//   effort는 mock_effort[A] 고정 (arm_safety_monitor 시험용), IMU는 정지 상태.
// mock:=false (실기): OpenCR 하나와 USB 시리얼 ASCII 줄 프로토콜 (lv2_module6/firmware/beebot_opencr 와 짝)
//   write(): C,<바퀴L rad/s>,<바퀴R rad/s>,<j1>…<j5 rad>,<그리퍼 m>[,<그리퍼 전류 A>]\n (9번째 = URDF 그리퍼 effort 명령이 있을 때)
//   read():  가장 최근의 완전한 S 줄 (43개 — 모터 8개 pos·vel·A + IMU 10 + 온도 8 + 배터리 V) → 상태. 개수가 다르면 예전 펌웨어.
//            순서는 STATE_* 와 firmware/README.md "프로토콜". E 줄(모터 에러, 0.5초) · S 온도 → /motor_status · 진단
//   OpenCR가 DYNAMIXEL 8개 + 내장 IMU를 맡고, 단위 변환(rad ↔ 서보 각도·rpm)도 펌웨어가 한다.
// 부저: 토픽 /cmd/beep (std_msgs/Int32MultiArray — [] 기본음 · [Hz, ms]) → write()가 B 줄 (모드와 무관)
// 팔 토크: 서비스 ~/set_arm_torque (std_srvs/SetBool) → write()가 T 줄 (joint1~5 + 그리퍼 — 바퀴는 그대로).
//   끄면 손으로 자세를 잡을 수 있다(S로 각은 계속 읽힘). 켜기 전에 JTC 목표를 지금 각으로 맞춰야 안 튄다 — manual_control이 함.
// 비틀림 상태 (twist_state, 기본 켜짐 — 강성 측정만 false): 서보 엔코더가 못 보는 joint2·3 중력 비틀림 d = c·τ(엔코더 각)를 상태 위치에 더해
//   /joint_states·TF·MoveIt·JTC·rl_arm_correction이 실제 관절 각을 보게 한다. 명령(C 줄)은 그대로 엔코더 기준 —
//   d는 앞먹임(rl_arm_correction gravity_enabled)이 빼고, rl_arm_correction twist_state도 같이 켠다 (ros_system launch가 함께 켬).
//   모델·강성은 ml/rl_arm_correction.hpp gravity와 같은 값.
// 상태: /diagnostics "BeebotSystem: OpenCR 연결" (mock · 마지막 S 수신 · 송수신 줄 수 · 해석 실패) — 컴포넌트 노드로 발행
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <memory>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <map>
#include <thread>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control/msg/motor_status.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace control {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class BeebotSystemHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void report(diagnostic_updater::DiagnosticStatusWrapper & s);

  CallbackReturn missing(const char * name);

  // ---------- mock
  return_type read_mock(double dt);

  // ---------- 실기: OpenCR 시리얼
  bool open_port();

  void close_port();

  return_type read_serial();

  bool apply_state(const std::string & text);

  bool apply_errors(const std::string & text);  // E 줄 — 모터별 Hardware Error Status

  void reboot_motors(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res);

  void set_arm_torque(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);

  return_type write_serial();

  bool mock_ = false;
  double mock_effort_ = 0.0;
  std::string port_, imu_;
  bool twist_state_ = true;                     // 위 "비틀림 상태"
  double twist_c2_ = 0.0817, twist_c3_ = 0.0532;  // [rad/Nm] joint2·3 = controllers.yaml gravity_compliance_j2·j3
  std::array<double, 6> twist_{};               // ARM 순서, 마지막으로 상태에 더한 d — on_activate 명령을 엔코더 각으로
  std::map<std::string, double> offset_;  // 팔·그리퍼 영점 오프셋 (URDF <joint><param name="offset">) — 서보 값 = 관절값 + offset
  int fd_ = -1;
  std::string rx_;
  std::atomic<std::chrono::steady_clock::time_point> last_state_{};
  std::atomic<size_t> sent_{0}, received_{0}, bad_{0};
  std::unique_ptr<diagnostic_updater::Updater> diag_;
  std::array<std::atomic<int>, 8> hw_error_{};  // 프로토콜 순서(바퀴L·R·joint1~5·그리퍼), −1 = 응답 없음
  std::array<std::atomic<int>, 8> hw_temp_{};   // 같은 순서, °C · −1 = 모름(응답 없음·온도 없는 예전 펌웨어·mock) — on_init에서 −1로
  std::atomic<uint32_t> reboot_mask_{0};         // 서비스가 고른 모터 — write()가 R 줄로 보냄
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_;
  std::atomic<int> torque_req_{-1};    // 서비스가 고른 팔·그리퍼 토크 (0 끄기 · 1 켜기 · −1 없음) — write()가 T 줄로 보냄
  std::atomic<bool> arm_nan_{false};    // 마지막 S에서 팔·그리퍼 값이 NaN (모터 응답 없음 · 팔을 뽑음)
  std::atomic<bool> arm_absent_{false}; // 활성화 때 팔이 없었음 → 팔 명령을 계속 막는다 (나중에 꽂혀도 0rad로 튀지 않게 — 스택 재시작해야 팔 사용)
  std::atomic<bool> arm_torque_{true}; // 마지막으로 보낸 팔 토크 (펌웨어는 부팅 때 켬) — 진단 표시
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_srv_;
  std::atomic<uint64_t> beep_req_{0};  // /cmd/beep — 최상위 비트 = 요청 있음 · Hz<<32 | ms (0 = 펌웨어 기본) — write()가 B 줄로
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr beep_sub_;
  rclcpp::Publisher<control::msg::MotorStatus>::SharedPtr motor_pub_;  // /motor_status — 바뀔 때만 (state_manager가 읽음)
  rclcpp::TimerBase::SharedPtr motor_timer_;                           // read()(실시간 루프)에서 발행하지 않으려고 따로 0.5초
  std::array<int, 8> motor_sent_{};
  std::array<int, 8> temp_sent_{};
  std::atomic<double> battery_v_{std::numeric_limits<double>::quiet_NaN()};  // S 끝 배터리 V (NaN = 모름 · 예전 펌웨어 · mock)
  double battery_sent_ = std::numeric_limits<double>::quiet_NaN();
  bool wheel_effort_ = false;        // URDF 바퀴에 effort 상태 인터페이스 — S 바퀴 3번째 칸(전류 A)
  bool gripper_effort_cmd_ = false;  // URDF gripper_left_joint에 effort 명령 인터페이스 — C 9번째(그리퍼 전류 A)
  std::atomic<size_t> wrong_size_{0};  // 마지막으로 받은 잘못된 S 값 개수 (예전 펌웨어 진단용)
  bool motor_published_ = false;
  void publish_motor_status();
};

}  // namespace control
