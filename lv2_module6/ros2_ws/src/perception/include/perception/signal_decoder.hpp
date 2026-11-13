#pragma once

// signal_decoder (🛠 직접 작성, 담당: 전승혜) — 신호등 12회 점멸 (발제 v5 §02 「신호와 바코드 규칙」)을 시간축으로 읽는다
//   입력: ✉️ /tracked_objects — class traffic_light의 color (perception: red | yellow | green | off | all(두 색 이상 동시에 켜짐))
//   출력: ✉️ /mission/signal (perception/MissionSignal, reliable + transient_local) → 통합(판단) — 새 답이 확정될 때만 (ok=true)
//         ✉️ /mission/signal_status (같은 타입, reliable) — 사이클마다 결과 (ok · 오류 이유 count | unknown | range | unconfirmed)
//            → 통합이 "아직 안 옴"과 "읽기 실패"를 가려 대응(발제: 불명 색이면 오류 기록·안전 정지)하거나 로그로 남길 때
//   (인지 쪽이 깜빡임을 세서 미션 토픽으로 낸다 — 바코드 → /mission/qr 와 같은 자리. 정렬·배정·퍽 할당은 받는 쪽에서)
//         ✉️ /diagnostics
//   발제 신호: 한 번 = 한 색 1초 켜짐 + 1초 꺼짐 (같은 색 연속도 꺼짐으로 구분) · 12번이 한 사이클 · 끝나면 3초 꺼짐 후 반복
//              R·Y·G 각 1~6회, 합 12. 소등은 세지 않음. 누락·불명 색은 추정 금지 → 오류로 알림
//   읽는 법:
//     점멸 1번 = 켜진 구간 하나 (red·yellow·green·all 아무거나 켜짐 → min_off_s 넘는 꺼짐까지)
//       - 구간 안에서 모델이 잠깐 다른 색으로 봐도 쪼개지 않고, 구간에서 가장 많이 나온 색으로 정함
//         (몇 프레임 다른 색으로 보여도 한 번이 두 번으로 쪼개지지 않게). 가장 많은 색이 min_share 미만이거나 all이면 unknown (추정 금지)
//       - min_off_s보다 짧은 꺼짐(모델이 한두 프레임 놓침)은 구간 안 — 진짜 점멸 사이 꺼짐은 1초
//       - min_on_s보다 짧은 켜짐 구간은 잡음 — 버리고 꺼짐 시간도 이어서 잼 (3초 꺼짐 중 잡음이 사이클 경계를 지우지 않게)
//     마지막 켜짐 뒤 꺼짐(신호등 안 보임 포함)이 gap_s 넘게 이어지면 사이클 끝 — 점멸 사이 꺼짐(1초)과 사이클 사이 꺼짐(3초)을 가름
//     사이클 끝 앞에서 끊긴 조각(blinks개 미만 — 중간부터 봄 · 놓쳐서 생긴 긴 꺼짐)은 그 사이클의 뒷부분으로 보관(tail_)
//       → 다음 사이클 앞부분 점멸이 들어와 앞부분 + 보관한 뒷부분 = blinks개가 되면 그 순간 검사 (사이클 끝까지 안 기다림)
//       예: 3번째를 놓침 → [1 2] (가짜 끝) · [4..12] (진짜 끝 — 보관 교체) · 다음 [1 2 3] → 1 2 3 + 4..12 = 12개
//       가짜 끝(사이클 중간에 놓쳐 생긴 긴 꺼짐) 뒤 조각은 같은 사이클 나머지라 합이 blinks개가 못 되고, 다음 진짜 끝에서 보관이 바뀜
//       보관 조각 끝 ~ 다음 첫 켜짐이 tail_gap_max_s보다 길면 버림 — 사이클 사이 소등은 3초, 끝(12번째)이나 처음(1번째)을
//       놓치면 5초 이상이라 자리를 모름 (예: 12번째를 놓친 1..11을 2..12로 오인하지 않게) · 신호등이 멈췄다 다시 켜진 경우도
//     검사: blinks개인가 · 모르는 색 없나 · 각 색 min_count~max_count인가 → 맞으면 같은 답 연속 횟수를 세고
//     confirm번 연속 같으면 ok=true (기본 1 — 12개를 한 번 다 보면 확정, 실기 11-12)
//     /mission/signal은 새 답이 확정될 때만 (받을 때마다 mission_manager가 다시 계획함). 사이클마다 결과는 /mission/signal_status · 로그(WARN) · 진단
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
                                //   너무 크면 1초 꺼짐 한가운데 짧은 잡음이 꺼짐을 둘로 쪼개 양쪽 다 "짧은 꺼짐"이 되어 두 점멸이 합쳐짐
  double min_share = 0.5;       // 켜진 구간에서 가장 많은 색의 비율이 이보다 작으면 unknown
  double gap_s = 2.5;           // 마지막 켜짐 뒤 꺼짐(신호등 안 보임 포함)이 이만큼 이어지면 사이클 끝 [s] (점멸 사이 1초 < 이 값 < 사이클 사이 3초)
  int blinks = 12;              // 한 사이클 점멸 수 (발제: 12회)
  int min_count = 1;            // 색별 횟수 하한 (발제: 각 1~6회)
  int max_count = 6;            // 색별 횟수 상한
  double tail_gap_max_s = 4.5;  // 보관한 뒷부분 끝 ~ 다음 사이클 첫 켜짐이 이보다 길면 안 합침 [s] (정상 3초 · 한 칸 놓치면 5초)
  int confirm = 1;              // 같은 답이 이만큼 연달아 나와야 ok (1 = 12개를 한 번 다 보면 확정)
};

// 신호등 상태 샘플을 시간 순으로 받아 사이클별로 해석 (ROS 없이 — 테스트용으로 따로)
// 신호등 상태 하나 고르기 — lights = (마지막 본 시각 [s], 색) 목록. max_age_s 안에 본 것 중 켜진 것(red·yellow·green·all)이
//   있으면 그중 가장 최근 색, 없으면 가장 최근 것의 색, 아무것도 없으면 "" (안 보임 = 꺼짐으로 셈).
//   11/13 젯슨: 신호등이 트랙 둘로 갈려 꺼진 쪽(off)을 따라가다 깜빡임 사이 1초 꺼짐이 3초 소등처럼 보여 사이클이 끊김
std::string light_state(const std::vector<std::pair<double, std::string>> & lights, double now_s, double max_age_s);

class SignalDecoder {
public:
  explicit SignalDecoder(SignalParams p = {});
  // t [s] · state = red | yellow | green | off | all | ""(신호등 안 보임 = off). 사이클이 끝나면 true + out (ok 여부는 out.ok)
  bool feed(double t, const std::string & state, MissionSignal & out);
  std::string progress() const;  // 지금까지 읽은 것 (진단)

private:
  std::string blink_color() const;
  bool boundary(MissionSignal & out);
  void check(MissionSignal & out, const std::vector<std::string> & seq);

  SignalParams p_;
  bool in_on_ = false;                        // 켜진 구간 안 (짧은 꺼짐 포함)
  double on_start_ = 0.0;                     // 켜진 구간 첫 켜짐 샘플 시각 [s]
  double last_on_ = 0.0;                      // 켜진 구간 마지막 켜짐 샘플 시각 [s]
  std::map<std::string, int> votes_;          // 켜진 구간의 상태별 샘플 수 (red | yellow | green | all …)
  double off_since_ = -1.0;                   // 꺼짐이 시작된 시각 = 마지막 점멸의 마지막 켜짐 샘플 (음수 = 아직 샘플 없음)
  double off_since_saved_ = -1.0;             // 켜진 구간이 시작되기 전 off_since_ — 잡음이면 되돌림
  bool marked_ = false;                       // 이번 꺼짐 구간이 이미 사이클 끝으로 처리됨
  std::vector<std::string> seg_;              // 지금 사이클의 점멸 (red | yellow | green | unknown)
  std::vector<std::string> tail_;             // 사이클 끝 앞에서 끊긴 조각 = 그 사이클 뒷부분 (다음 사이클 앞부분과 합쳐 blinks개)
  double tail_end_ = 0.0;                     // 보관 조각 마지막 켜짐 시각 [s]
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
