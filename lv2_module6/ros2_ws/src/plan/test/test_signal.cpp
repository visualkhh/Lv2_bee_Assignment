// signal_decoder 테스트: 신호등 상태를 4Hz(perception 추론 주기)로 흉내 내 3단계 해석 (documents/mission/README.md §2·§3)
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "plan/signal_decoder.hpp"

using namespace plan;

namespace {
// 신호 한 줄: "S" = 세 색 동시 3초 · "R"/"Y"/"G" = 0.5초 켜짐 + 0.5초 꺼짐 · "_" = 4초 꺼짐 · "!" = 0.1초 빨강 잡음
struct Sim {
  SignalDecoder dec;
  double t = 0.0;
  std::vector<MissionSignal> got;
  explicit Sim(SignalParams p = {}) : dec(p) {}
  void hold(const std::string & state, double dur) {
    for (double end = t + dur; t < end - 1e-9; t += 0.05) {  // 20Hz로 넣되 상태는 0.25초마다만 바뀐 것처럼 (4Hz 추론)
      MissionSignal o;
      if (dec.feed(t, state, o)) got.push_back(o);
    }
  }
  void play(const std::string & seq) {
    for (char c : seq) {
      if (c == 'S') hold("all", 3.0);
      else if (c == '_') hold("off", 4.0);
      else if (c == '!') { hold("red", 0.1); hold("off", 0.4); }
      else if (c != ' ') {
        hold(c == 'R' ? "red" : c == 'Y' ? "yellow" : "green", 0.5);
        hold("off", 0.5);
      }
    }
  }
};
}  // namespace

TEST(Signal, MissionExample) {
  Sim s;
  s.play("S RR S RYGGR S YY S");  // MISSION §3: 빨강 2 · 합 5 · 13번
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_EQ(s.got[0].color, "red");
  EXPECT_EQ(s.got[0].count, 2);
  EXPECT_EQ(s.got[0].sum, 5);
  EXPECT_EQ(s.got[0].dest_tag, 13);
  EXPECT_EQ(s.got[0].raw, "R R | R Y G G R | Y Y");
}

TEST(Signal, StartMidwayInRepeat) {
  Sim s;
  s.play("YGGR S YY S GGG S RRYY S Y S GGG S RRYY S Y S");  // 2단계 중간부터 봄 — 앞 조각은 모양이 안 맞아 버림
  ASSERT_FALSE(s.got.empty());
  EXPECT_EQ(s.got[0].color, "green");
  EXPECT_EQ(s.got[0].count, 3);
  EXPECT_EQ(s.got[0].sum, 4);
  EXPECT_EQ(s.got[0].dest_tag, 12);
}

TEST(Signal, ScoresAndGapAndNoise) {
  SignalParams p;
  p.scores = {{"red", 1}, {"yellow", 2}, {"green", 3}};  // 2단계 점수가 다를 때
  Sim s(p);
  s.play("_ G!G _ RYG _ YYY _");  // 단계 사이가 '꺼짐 4초'여도 · 0.1초 잡음은 안 셈
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_EQ(s.got[0].color, "green");
  EXPECT_EQ(s.got[0].count, 2);
  EXPECT_EQ(s.got[0].sum, 6);
  EXPECT_EQ(s.got[0].dest_tag, 14);
}

TEST(Signal, ConfirmTwice) {
  SignalParams p;
  p.confirm = 2;  // 반복 신호 — 같은 답이 두 번 나와야
  Sim s(p);
  s.play("S RR S RYGGR S YY S");
  EXPECT_TRUE(s.got.empty());
  s.play("RR S RYGGR S YY S");
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_EQ(s.got[0].sum, 5);
}
