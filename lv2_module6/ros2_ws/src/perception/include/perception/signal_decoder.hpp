#pragma once

// signal_decoder (🛠 직접 작성, 담당: 전승혜) — 신호등 12회 점멸 (발제 v5 §02 「신호와 바코드 규칙」)을 시간축으로 읽는다
//   입력: ✉️ /tracked_objects — class traffic_light의 color (perception: red | yellow | green | off | all(두 색 이상 동시에 켜짐))
//   출력: ✉️ /mission/signal (perception/MissionSignal, reliable + transient_local) → 통합(판단) — 새 답이 확정될 때만 (ok=true)
//         ✉️ /mission/signal_status (같은 타입, reliable) — 사이클마다 결과 (ok · 오류 이유 count | unknown | range | unconfirmed)
//            → 통합이 "아직 안 옴"과 "읽기 실패"를 가려 대응(발제: 불명 색이면 오류 기록·안전 정지)하거나 로그로 남길 때
//   (인지 쪽이 깜빡임을 세서 미션 토픽으로 낸다 — 바코드 → /mission/qr 와 같은 자리. 정렬·배정·퍽 할당은 받는 쪽에서)
//         ✉️ /diagnostics
//   발제 신호: 한 번 = 한 색 1초 켜짐 + 1초 꺼짐 (같은 색 연속도 꺼짐으로 구분) · 12번이 한 사이클 · 끝나면 3초 꺼짐 후 반복 (사용자 10/28)
//              R·Y·G 각 1~6회, 합 12. 소등은 세지 않음. 누락·불명 색은 추정 금지 → 오류로 알림
//   읽는 법:
//     점멸 1번 = 켜진 구간 하나 (red·yellow·green·all 아무거나 켜짐 → min_off_s 넘는 꺼짐까지)
//       - 구간 안에서 모델이 잠깐 다른 색으로 봐도 쪼개지 않고, 구간에서 가장 많이 나온 색으로 정함 (실물 녹화 10/28: 빨강 1초 중
//         2~3프레임을 노랑으로 봐서 한 번이 두 번으로 쪼개졌음). 가장 많은 색이 min_share 미만이거나 all이면 unknown (추정 금지)
//       - min_off_s보다 짧은 꺼짐(모델이 한두 프레임 놓침)은 구간 안 — 진짜 점멸 사이 꺼짐은 1초
//       - min_on_s보다 짧은 켜짐 구간은 잡음 — 버리고 꺼짐 시간도 이어서 잼 (실물: 3초 꺼짐 중 잡음이 사이클 경계를 지워 두 사이클이 합쳐졌음)
//     마지막 켜짐 뒤 꺼짐(신호등 안 보임 포함)이 gap_s 넘게 이어지면 사이클 끝 — 점멸 사이 꺼짐(1초)과 사이클 사이 꺼짐(3초)을 가름
//     노드가 켜진 뒤 첫 사이클 끝 전의 조각은 버림 (사이클 중간부터 봤을 수 있음)
//     사이클마다 검사: blinks개인가 · 모르는 색 없나 · 각 색 min_count~max_count인가 → 맞으면 같은 답 연속 횟수를 세고
//     confirm번 연속 같으면 ok=true (반복 신호 — 한 번 놓치거나 두 번 센 답을 거름)
//     /mission/signal은 새 답이 확정될 때만 (planner가 받을 때마다 다시 계획함). 사이클마다 결과는 /mission/signal_status · 로그(WARN) · 진단
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "perception/msg/mission_signal.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception {

using perception::msg::MissionSignal;

struct SignalParams {
  double min_on_s = 0.3;        // 켜진 구간(첫~마지막 켜짐 샘플)이 이보다 짧으면 잡음 [s] (진짜 켜짐 1초 · 10Hz면 4프레임 이상)
  double min_off_s = 0.3;       // 이보다 짧은 꺼짐은 켜진 구간 안으로 봄 [s] (진짜 점멸 사이 꺼짐 1초 · 10Hz 2프레임 / 4Hz 1샘플 놓침까지)
                                //   0.4였을 때 1초 꺼짐 한가운데 0.2초 잡음이 꺼짐을 0.4+0.4로 쪼개 양쪽 다 "짧은 꺼짐"이 되어 두 점멸이 합쳐졌음 (실물 재생 10/28)
  double min_share = 0.5;       // 켜진 구간에서 가장 많은 색의 비율이 이보다 작으면 unknown
  double gap_s = 2.5;           // 마지막 켜짐 뒤 꺼짐(신호등 안 보임 포함)이 이만큼 이어지면 사이클 끝 [s] (점멸 사이 1초 < 이 값 < 사이클 사이 3초)
  int blinks = 12;              // 한 사이클 점멸 수 (발제: 12회)
  int min_count = 1;            // 색별 횟수 하한 (발제: 각 1~6회)
  int max_count = 6;            // 색별 횟수 상한
  int confirm = 2;              // 같은 답이 이만큼 연달아 나와야 ok (반복 신호)
};

// 신호등 상태 샘플을 시간 순으로 받아 사이클별로 해석 (ROS 없이 — 테스트용으로 따로)
class SignalDecoder {
public:
  explicit SignalDecoder(SignalParams p = {});
  // t [s] · state = red | yellow | green | off | all | ""(신호등 안 보임 = off). 사이클이 끝나면 true + out (ok 여부는 out.ok)
  bool feed(double t, const std::string & state, MissionSignal & out);
  std::string progress() const;  // 지금까지 읽은 것 (진단)

private:
  std::string blink_color() const;
  bool boundary(MissionSignal & out);
  void check(MissionSignal & out);

  SignalParams p_;
  bool in_on_ = false;                        // 켜진 구간 안 (짧은 꺼짐 포함)
  double on_start_ = 0.0;                     // 켜진 구간 첫 켜짐 샘플 시각 [s]
  double last_on_ = 0.0;                      // 켜진 구간 마지막 켜짐 샘플 시각 [s]
  std::map<std::string, int> votes_;          // 켜진 구간의 상태별 샘플 수 (red | yellow | green | all …)
  double off_since_ = -1.0;                   // 꺼짐이 시작된 시각 = 마지막 점멸의 마지막 켜짐 샘플 (음수 = 아직 샘플 없음)
  double off_since_saved_ = -1.0;             // 켜진 구간이 시작되기 전 off_since_ — 잡음이면 되돌림
  bool marked_ = false;                       // 이번 꺼짐 구간이 이미 사이클 끝으로 처리됨
  bool started_ = false;                      // 사이클 끝을 한 번이라도 봤음 — 그 전 조각은 중간부터 본 것이라 버림
  std::vector<std::string> seg_;              // 지금 사이클의 점멸 (red | yellow | green | unknown)
  std::string last_raw_;                      // 직전 정상 사이클의 답 (raw)
  int same_ = 0;                              // 같은 정상 답 연속 횟수
};

class SignalDecoderNode : public rclcpp::Node {
public:
  SignalDecoderNode();

private:
  void on_objects(const perception::msg::TrackedObjectArray & m);

  SignalDecoder dec_;
  std::string state_ = "off";
  MissionSignal last_;                        // 마지막 사이클 결과 (ok든 아니든 — 진단)
  bool have_ = false;
  std::string published_raw_;                 // 마지막으로 발행한 확정 답 — 같은 답은 다시 안 냄
  rclcpp::Publisher<MissionSignal>::SharedPtr pub_;
  rclcpp::Publisher<MissionSignal>::SharedPtr status_pub_;  // 사이클마다 결과
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr sub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace perception
