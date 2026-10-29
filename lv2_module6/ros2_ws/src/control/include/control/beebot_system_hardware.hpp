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
//   write(): C,<바퀴L rad/s>,<바퀴R rad/s>,<j1>…<j5 rad>,<그리퍼 m>\n
//   read():  가장 최근의 완전한 S 줄 (40개 값 — 끝 8개 = 모터 온도 °C · 예전 펌웨어 32개도 받음) → 상태.
//            순서는 STATE_* 와 firmware/README.md "프로토콜". E 줄(모터 에러, 0.5초) · S 온도 → /motor_status · 진단
//   OpenCR가 DYNAMIXEL 8개 + 내장 IMU를 맡고, 단위 변환(rad ↔ 서보 각도·rpm)도 펌웨어가 한다.
// 팔 토크: 서비스 ~/set_arm_torque (std_srvs/SetBool) → write()가 T 줄 (joint1~5 + 그리퍼 — 바퀴는 그대로).
//   끄면 손으로 자세를 잡을 수 있다(S로 각은 계속 읽힘). 켜기 전에 JTC 목표를 지금 각으로 맞춰야 안 튄다 — manual_control이 함.
// 상태: /diagnostics "BeebotSystem: OpenCR 연결" (mock · 마지막 S 수신 · 송수신 줄 수 · 해석 실패) — 컴포넌트 노드로 발행
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

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
#include <map>
#include <thread>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "control/msg/motor_status.hpp"
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
  std::atomic<bool> arm_torque_{true}; // 마지막으로 보낸 팔 토크 (펌웨어는 부팅 때 켬) — 진단 표시
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr torque_srv_;
  rclcpp::Publisher<control::msg::MotorStatus>::SharedPtr motor_pub_;  // /motor_status — 바뀔 때만 (state_manager가 읽음)
  rclcpp::TimerBase::SharedPtr motor_timer_;                           // read()(실시간 루프)에서 발행하지 않으려고 따로 0.5초
  std::array<int, 8> motor_sent_{};
  std::array<int, 8> temp_sent_{};
  bool motor_published_ = false;
  void publish_motor_status();
};

}  // namespace control
