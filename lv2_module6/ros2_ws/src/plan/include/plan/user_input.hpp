#pragma once

// user_input (🛠 직접 작성, 담당: 권형중) — 외부 입력 노드. 설계 도식의 voice_ui 자리 (음성 대신 텍스트로 받는다)
//   입력: 텍스트 명령 — command 파라미터(시작 후 한 번) 또는 터미널 표준입력(한 줄 = 명령 하나)
//         ✉️ /robot_say (coordinator가 운영자에게 알릴 말), ✉️ /task_status (진행 표시)
//   출력: ✉️ /cmd/user_command (std_msgs/String) → vlm_task_planner
//   ros2 run plan user_input                                    # 터미널에서 입력
//   ros2 run plan user_input --ros-args -p command:="빨간 퍽을 목표 구역으로 옮겨"
// 테스트베드 통제실은 faker input_manager로 /cmd/user_command를 직접 넣을 수도 있다.
// TODO(권형중): 통제실·웹 등 실제 입력 UI가 정해지면 여기서 받기
#include <unistd.h>

#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace plan {

using plan::msg::TaskStatus;
using std_msgs::msg::String;

class UserInput : public rclcpp::Node {
public:
  UserInput();

private:
  // vlm_task_planner가 붙기 전에 보내면 사라진다 (/cmd/user_command는 volatile) → 구독자가 생길 때까지 들고 있음
  void send_pending();

  std::mutex mutex_;
  std::deque<std::string> pending_;
  rclcpp::Publisher<String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<String>::SharedPtr say_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t sent_ = 0;
  std::string last_ = "-";
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
