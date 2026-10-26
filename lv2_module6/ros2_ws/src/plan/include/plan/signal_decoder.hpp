#pragma once

// signal_decoder (🛠 직접 작성) — 신호등 암호 미션(documents/mission/README.md) 1·2·3단계를 시간축으로 읽는다
//   입력: ✉️ /tracked_objects — class traffic_light의 color (perception: red | yellow | green | off | all(두 색 이상 동시에 켜짐))
//   출력: ✉️ /mission/signal (plan/MissionSignal, reliable + transient_local) → vlm_task_planner — 확정될 때마다
//         ✉️ /diagnostics
//   읽는 법 (MISSION §2):
//     1단계 한 색이 N번 → 그 색 퍽 N개 · 2단계 깜빡인 색 점수의 합 = 태그 번호 합 · 3단계 노랑 1·2·3번 → 목적지 태그 12·13·14
//   단계 구분 (MISSION §6 미정 — 제안값이 기본, plan.yaml에서 바꿈):
//     all(세 색 동시)이 separator_s 넘게 이어지면 단계 끝 · 꺼짐이 gap_s 넘게 이어져도 단계 끝 (한 바퀴 사이 쉬는 시간)
//     켜짐이 min_on_s보다 짧으면 잡음. 끝난 단계 셋이 [한 색만 · 아무거나 · 노랑만 1~3번] 모양이면 답 (반복 신호에서 어디가 1단계인지도 이걸로)
//     confirm번 같은 답이 연달아 나와야 확정 (신호를 반복하면 2로 — 깜빡임을 놓치거나 두 번 센 답을 거름)
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/mission_signal.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plan {

using plan::msg::MissionSignal;

struct SignalParams {
  double min_on_s = 0.15;       // 이보다 짧은 켜짐은 잡음
  double separator_s = 2.0;     // all이 이만큼 이어지면 단계 구분 (제안: 3초)
  double gap_s = 3.0;           // 꺼짐(신호등 안 보임 포함)이 이만큼 이어지면 단계 끝
  std::map<std::string, int> scores{{"red", 1}, {"yellow", 1}, {"green", 1}};  // 2단계 색 점수 (제안: 모두 1)
  std::vector<int> dest_tags{12, 13, 14};  // 3단계 노랑 1·2·3번 → 태그
  int confirm = 1;              // 같은 답이 이만큼 연달아 나와야 확정
};

// 신호등 상태 샘플을 시간 순으로 받아 3단계를 해석 (ROS 없이 — 테스트용으로 따로)
class SignalDecoder {
public:
  explicit SignalDecoder(SignalParams p = {});
  // t [s] · state = red | yellow | green | off | all | ""(신호등 안 보임 = off). 확정되면 true + out
  bool feed(double t, const std::string & state, MissionSignal & out);
  std::string progress() const;  // 지금까지 읽은 것 (진단)

private:
  void end_run(double t);
  bool boundary(MissionSignal & out);
  bool decode(MissionSignal & out) const;

  SignalParams p_;
  std::string cur_ = "off";
  double since_ = 0.0;
  bool marked_ = false;                       // 이번 all·꺼짐 구간이 이미 단계 끝으로 처리됨
  std::vector<std::string> seg_;              // 지금 단계의 깜빡임 (색 이름)
  std::vector<std::vector<std::string>> segs_;  // 끝난 단계들 (최근 3개)
  std::string last_raw_;
  int same_ = 0;                              // 같은 답 연속 횟수
};

class SignalDecoderNode : public rclcpp::Node {
public:
  SignalDecoderNode();

private:
  void on_objects(const perception::msg::TrackedObjectArray & m);

  SignalDecoder dec_;
  std::string state_ = "off";
  MissionSignal last_;
  bool have_ = false;
  rclcpp::Publisher<MissionSignal>::SharedPtr pub_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr sub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
