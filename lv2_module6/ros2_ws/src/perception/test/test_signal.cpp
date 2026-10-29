// signal_decoder 테스트: 신호등 상태를 4Hz(perception 추론 주기)로 흉내 내 점멸 세기 (documents/mission/README.md §3)
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "perception/signal_decoder.hpp"

using namespace perception;

namespace {
// 신호 한 줄: "R"/"Y"/"G" = 1초 켜짐 + 1초 소등 · "A" = 두 색 동시 1초 · "." = 켜짐 중 한 장 놓침 (다음 글자와 같은 점멸)
struct Sim {
  SignalDecoder dec;
  double t = 0.0;
  int changes = 0;
  explicit Sim(SignalParams p = {}) : dec(p) {}
  void hold(const std::string & state, double dur) {
    for (double end = t + dur; t < end - 1e-9; t += 0.25) changes += dec.feed(t, state);  // 4Hz 추론
  }
  void play(const std::string & seq, double end_off = 3.5) {
    for (size_t i = 0; i < seq.size(); ++i) {
      const char c = seq[i];
      if (c == ' ') continue;
      const std::string s = c == 'R' ? "red" : c == 'Y' ? "yellow" : c == 'G' ? "green" : "all";
      if (i + 1 < seq.size() && seq[i + 1] == '.') {  // 켜짐 중간에 0.25초 '안 보임'
        hold(s, 0.5); hold("", 0.25); hold(s, 0.25); ++i;
      } else {
        hold(s, 1.0);
      }
      hold("off", 1.0);
    }
    hold("off", end_off);
  }
};
}  // namespace

TEST(Signal, Q1) {
  Sim s;
  s.play("RGRRGRGGYGYG");  // 발제 공개 예제 Q1
  const auto & r = s.dec.result();
  EXPECT_EQ(r.state, MissionSignal::DONE);
  ASSERT_EQ(r.sequence.size(), 12u);
  EXPECT_EQ(r.sequence, (std::vector<std::string>{"R", "G", "R", "R", "G", "R", "G", "G", "Y", "G", "Y", "G"}));
  EXPECT_EQ(r.count_r, 4);
  EXPECT_EQ(r.count_y, 2);
  EXPECT_EQ(r.count_g, 6);
  EXPECT_EQ(r.onsets.size(), 12u);
  EXPECT_EQ(s.changes, 13);  // 점멸 12번 + 끝 1번
}

TEST(Signal, SameColorSeparatedByOff) {
  Sim s;
  s.play("RRRRRRYYYGGG");  // 같은 색 연속도 소등으로 구별
  EXPECT_EQ(s.dec.result().state, MissionSignal::DONE);
  EXPECT_EQ(s.dec.result().count_r, 6);
}

TEST(Signal, DropoutInsideBlinkIsOneBlink) {
  Sim s;
  s.play("R.GRRGRGGYGYG");  // 첫 점멸 중간에 한 장 놓쳐도 한 번
  EXPECT_EQ(s.dec.result().state, MissionSignal::DONE);
  EXPECT_EQ(s.dec.result().sequence.size(), 12u);
}

TEST(Signal, NotDoneUntilEndOff) {
  Sim s;
  s.play("RGRRGRGGYGYG", 1.5);  // 마지막 소등이 3초가 안 됨
  EXPECT_EQ(s.dec.result().state, MissionSignal::OBSERVING);
  s.hold("off", 2.0);
  EXPECT_EQ(s.dec.result().state, MissionSignal::DONE);
}

TEST(Signal, Errors) {
  Sim a;
  a.play("RGRA");  // 두 색 동시 — 추정 금지
  EXPECT_EQ(a.dec.result().state, MissionSignal::ERROR);
  EXPECT_EQ(a.dec.result().error, "unknown_color");
  Sim b;
  b.play("RGRRGRGGYGYGG", 0.0);  // 13번째
  EXPECT_EQ(b.dec.result().state, MissionSignal::ERROR);
  EXPECT_EQ(b.dec.result().error, "too_many");
  Sim c;
  c.play("RGRRG", 10.0);  // 12회 미만은 계속 세는 중 (끊는 건 plan)
  EXPECT_EQ(c.dec.result().state, MissionSignal::OBSERVING);
}
