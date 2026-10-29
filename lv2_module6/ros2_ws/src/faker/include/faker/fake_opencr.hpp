#pragma once

// OpenCR 대역: BeebotSystemHardware(mock:=false)가 가상 시리얼로 주고받는 펌웨어 프로토콜(firmware/README.md)을
// 파일로 흉내낸다. 테스트베드에서 OpenCR 역할은 이것 하나 (시뮬레이터는 시리얼에 쓰지 않음 — simulator/opencr.py가 읽기·요청만).
//   C 읽기: <serial_out> (debug/output/hardware/serial) — "C,<바퀴L·R rad/s>,<joint1~5 rad>,<그리퍼 m>" 8 필드,
//     빈 필드(`,,`)는 유지(NaN) — 팔·그리퍼는 명령 생략(현재 유지), 바퀴는 0(정지).
//     줄마다 엄격 검사 후 적용 (틀린 줄은 버림 — 결과 = 마지막 유효 줄). 예: C,1,,0.4,,,,,
//   S 쓰기: <serial_in> (debug/input/hardware/serial) — "S,<40값>"(끝 8개 = 모터 온도 °C) 25ms마다 항상 (펌웨어처럼 C와 무관 — 통신 끊김일 때만 멈춤)
//     바퀴 pos·vel ×2 · 관절 pos·vel·A ×6(joint1~5 + gripper_left) · 쿼터니언 · 자이로 · 가속도
// 물리 (한 걸음 dt):
//   바퀴: 명령 속도로 가속 (한계 ACCEL_MAX × friction) → 엔코더는 회전 그대로,
//         바닥 전달은 (1 − slip) → 차동 구동 자세 적분.
// 펌웨어(firmware/beebot_opencr)와 1:1: S 25ms 주기로 항상 · C는 줄마다 엄격 검사(8 필드·유한·뒤 쓰레기 없음,
//   빈 필드는 유지) · watchdog = 마지막 유효 C부터 0.5초 → 바퀴 0 + 팔·그리퍼 지금 자세 유지 (한 번) · 첫 C 전엔 안 움직임
//   충돌: 코스 지도 PGM(벽)과 로봇 외곽 직사각형(앞 0.069 · 뒤 −0.233 · 좌우 ±0.153)이 겹치면 그 걸음 이동·회전 차단 (bumped)
//   팔·그리퍼: 속도 한계로 목표 추종. arm_grabbed면 관절 고정 + 과전류 1.6A
//   IMU: 실제 yaw · yaw rate (+ 가우스 잡음), 가속도 z = 9.81
// 테스트베드의 유일한 OpenCR — 시뮬레이터는 시리얼을 읽기만 하고, 바꿀 것은 아래 ROS 인터페이스로 요청한다.
//   ✉️ ~/ground_truth (PoseStamped, map) 20Hz — 미끄럼까지 반영된 진짜 위치 (S에는 엔코더·IMU뿐) → 시뮬레이터 카메라 렌더
//   ✉️ ~/bumped (Bool) 20Hz — 이번 걸음에 벽에 막혔나
//   ✉️ /cmd/fake_opencr/set_pose (geometry_msgs/PoseStamped) 구독 (외부 명령은 /cmd/*) — 위치 옮기기·초기화 (바퀴 속도 0)
//   ⚙️ slip · friction · noise · comm_down · arm_grabbed — 실행 중 파라미터 변경 (ros2 param set · 시뮬레이터 버튼)
//   ⚙️ hw_error (int[8], 프로토콜 순서) — 모터 Hardware Error 주입 (>0이면 그 모터 토크 꺼짐, R로 재부팅해야 풀림 — 펌웨어와 같음)
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace faker {

class FakeOpenCR : public rclcpp::Node {
public:
  FakeOpenCR();

private:
  // 지도 PGM 읽기 (P5) — 실패하면 빈 지도(충돌 없음)
  bool load_map(const std::string & yaml_path);
  bool occupied(double x, double y) const;
  bool collide(double x, double y, double yaw) const;
  // C 파일 끝부분에서 가장 최근의 완전한 "C,..." 줄을 파싱 → true (8 필드, 빈칸=유지 NaN)
  bool read_command(std::array<double, 8> & cmd);
  // 펌웨어 parse_fields와 같은 규칙으로 C 한 줄 검사
  static bool parse_command(const std::string & line, std::array<double, 8> & v);
  // T 줄(토크 온/오프) 적용 — 전체(T,0) 또는 조인트별(T,1,0,,,,,,), 빈칸=유지
  void apply_torque_line(const std::string & line);
  // R 줄(모터 재부팅) — 고른 모터의 에러를 지우고 토크 켜기 (펌웨어 apply_reboot_line)
  void apply_reboot_line(const std::string & line);
  void write_errors();  // E 줄 — 모터별 Hardware Error Status
  void write_errors_locked();  // mutex_를 이미 잡은 쪽에서
  void write_state();
  void tick();

  std::string serial_in_, serial_out_;
  std::ifstream cmd_stream_;
  double slip_, friction_, noise_;
  bool comm_down_, arm_grabbed_;
  double x_, y_, yaw_, yaw_rate_;
  std::array<double, 2> wheel_cmd_{}, wheel_vel_{}, wheel_pos_{};
  std::array<double, 6> joint_cmd_{}, joint_pos_{}, joint_vel_{};
  std::array<bool, 8> torque_{true, true, true, true, true, true, true, true};  // 펌웨어 T 명령 상태
  std::array<int, 8> hw_error_{};  // 주입된 Hardware Error (hw_error 파라미터, R로 지움)
  std::chrono::steady_clock::time_point last_errors_{};
  std::chrono::steady_clock::time_point last_cmd_{}, last_state_{}, last_tick_{};
  long cmds_ = 0;
  bool bumped_ = false;
  bool watchdog_tripped_ = true;  // 펌웨어와 같음 — 첫 유효 C 전에는 움직이지 않음
  std::vector<uint8_t> map_;
  int map_w_ = 0, map_h_ = 0;
  double map_res_ = 0.05, map_ox_ = 0.0, map_oy_ = 0.0;
  std::mt19937 rng_{std::random_device{}()};
  rclcpp::TimerBase::SharedPtr timer_, pub_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bumped_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
  std::mutex mutex_;
};

}  // namespace faker
