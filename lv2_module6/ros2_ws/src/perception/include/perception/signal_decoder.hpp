#pragma once

// signal_decoder (🛠 직접 작성, 담당: 전승혜) — 주사위 기억 미션 신호(documents/mission/README.md §3)를 시간축으로 센다
//   입력: ✉️ /tracked_objects — class traffic_light의 color (perception: red | yellow | green | off | all(두 색 이상 동시에 켜짐))
//         ✉️ /mission/signal/reset (std_msgs/Empty) — 처음부터 다시 셈 (시뮬레이터 재시작 · 운영 장비 오류 재시도)
//   출력: ✉️ /mission/signal (perception/MissionSignal, reliable + transient_local) → mission_manager — 점멸마다 · 끝날 때
//         ✉️ /diagnostics
//   규칙 (발제 p3): 한 번 점멸 = 한 색 1초 켜짐 + 1초 소등 · 12번째 점멸 뒤 3초 연속 소등이면 끝 (DONE)
//   해석(정렬·배정)은 하지 않는다 — 본 순서·횟수·시각만 (plan의 mission_solver가 판단)
//   잡음 처리 (4Hz 추론 · YOLO가 한두 장 놓치는 것):
//     켜짐 중 min_off_s보다 짧은 꺼짐은 같은 점멸로 이어 붙인다 · min_on_s보다 짧은 켜짐은 버린다
//     한 점멸 안에서 두 번째 상태(다른 색 · all)도 min_on_s 넘게 보이면 색을 모름 → ERROR unknown_color (추정 금지)
//     13번째 점멸 → ERROR too_many. 12회 미만은 '아직 세는 중'과 구별이 안 되므로 OBSERVING 그대로 (끊는 건 plan의 시간 초과)
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/mission_signal.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

namespace perception {

using perception::msg::MissionSignal;

struct SignalParams {
  double min_on_s = 0.3;    // 이보다 짧은 켜짐은 잡음
  double min_off_s = 0.4;   // 이보다 짧은 꺼짐은 점멸 사이가 아님 (한 점멸 안에서 한 장 놓침)
  double end_off_s = 3.0;   // 12회 뒤 이만큼 꺼짐이면 끝
  int total = 12;
};

// 신호등 상태 샘플을 시간 순으로 받아 센다 (ROS 없이 — 테스트용으로 따로)
class SignalDecoder {
public:
  explicit SignalDecoder(SignalParams p = {});
  // t [s] · state = red | yellow | green | off | all | ""(안 보임 = off) → 내보낼 변화(점멸 하나 · 끝 · 오류)가 있으면 true
  bool feed(double t, const std::string & state);
  const MissionSignal & result() const { return out_; }  // stamp 빼고 채움 (onsets는 feed의 t 그대로)
  std::string progress() const;  // 진단

private:
  bool close_blink();
  void fail(const std::string & why);

  SignalParams p_;
  MissionSignal out_;
  bool on_ = false;               // 점멸 중 (켜짐 구간)
  double on_since_ = 0.0, off_since_ = 0.0, last_t_ = 0.0;
  std::string last_state_ = "off";
  std::map<std::string, double> dur_;  // 이번 점멸 안 상태별 시간
};

class SignalDecoderNode : public rclcpp::Node {
public:
  SignalDecoderNode();

private:
  void on_objects(const perception::msg::TrackedObjectArray & m);
  void publish();

  SignalParams params_;
  SignalDecoder dec_;
  std::string state_ = "off";
  rclcpp::Publisher<MissionSignal>::SharedPtr pub_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace perception
