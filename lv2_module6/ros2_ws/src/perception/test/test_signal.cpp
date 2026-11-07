// signal_decoder 테스트: 신호등 상태를 10Hz(perception /tracked_objects 주기)로 흉내 내 12회 점멸 사이클 해석 (발제 v5 §02)
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "perception/signal_decoder.hpp"

using namespace perception;

namespace {
const char * kQ1 = "RGRRGRGGYGYG";  // 발제 공개 연습 문제 Q1 — R4 Y2 G6

// 신호 한 줄: "R"/"Y"/"G" = 1초 켜짐 + 1초 꺼짐 · "A" = 두 색 동시(all) 1초 + 1초 꺼짐
//            "_" = 사이클 사이 3초 꺼짐 · "!" = 0.1초 빨강 잡음 + 0.9초 꺼짐 · " " = 무시(읽기 편하게)
struct Sim {
  SignalDecoder dec;
  double t = 0.0;
  double dt = 0.1;  // 샘플 간격 — 10Hz (perception rate_hz) · 젯슨 CPU 추론이 느리면 0.25(4Hz)
  std::vector<MissionSignal> got;
  explicit Sim(SignalParams p = {}) : dec(p) {}
  void hold(const std::string & state, double dur) {
    for (double end = t + dur; t < end - 1e-9; t += dt) {
      MissionSignal o;
      if (dec.feed(t, state, o)) got.push_back(o);
    }
  }
  void play(const std::string & seq) {
    for (char c : seq) {
      if (c == '_') hold("off", 3.0);
      else if (c == '!') { hold("red", 0.1); hold("off", 0.9); }
      else if (c == 'A') { hold("all", 1.0); hold("off", 1.0); }
      else if (c != ' ') {
        hold(c == 'R' ? "red" : c == 'Y' ? "yellow" : "green", 1.0);
        hold("off", 1.0);
      }
    }
  }
};
}  // namespace

TEST(Signal, Q1TwoCycles) {
  Sim s;
  s.play(std::string("_ ") + kQ1 + " _ " + kQ1 + " _");  // 시작 전 꺼짐 → 두 사이클
  ASSERT_EQ(s.got.size(), 2u);
  // 첫 사이클: 정상이지만 한 번만 봄
  EXPECT_FALSE(s.got[0].ok);
  EXPECT_EQ(s.got[0].error, "unconfirmed");
  EXPECT_EQ(s.got[0].cycles, 1);
  // 두 번째 사이클: 같은 답 → 확정
  const auto & m = s.got[1];
  EXPECT_TRUE(m.ok);
  EXPECT_EQ(m.error, "");
  EXPECT_EQ(m.cycles, 2);
  EXPECT_EQ(m.red, 4);
  EXPECT_EQ(m.yellow, 2);
  EXPECT_EQ(m.green, 6);
  ASSERT_EQ(m.sequence.size(), 12u);
  EXPECT_EQ(m.sequence[0], "red");
  EXPECT_EQ(m.sequence[1], "green");
  EXPECT_EQ(m.sequence[8], "yellow");
  EXPECT_EQ(m.raw, "R G R R G R G G Y G Y G");
  // 옛 칸은 비어 있음 (planner 빌드용으로만 남김)
  EXPECT_EQ(m.color, "");
  EXPECT_EQ(m.count, 0);
}

TEST(Signal, SameColorInARowCountsEach) {
  Sim s;
  s.play("_ RRRRRYYYYGGG _ RRRRRYYYYGGG _");  // 같은 색 연속도 사이 1초 꺼짐으로 따로 셈
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].red, 5);
  EXPECT_EQ(s.got[1].yellow, 4);
  EXPECT_EQ(s.got[1].green, 3);
}

TEST(Signal, StartMidwayDropsFragment) {
  Sim s;
  s.play(std::string("GGYGYG _ ") + kQ1 + " _ " + kQ1 + " _");  // 사이클 중간부터 봄 — 첫 조각은 버림
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_EQ(s.got[0].error, "unconfirmed");
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].raw, "R G R R G R G G Y G Y G");
}

TEST(Signal, MissedBlinkIsCountError) {
  Sim s;
  s.play(std::string("_ RGRRGRGGYGY _ ") + kQ1 + " _ " + kQ1 + " _");  // 첫 사이클 하나 놓침(11개) — 추정하지 않음
  ASSERT_EQ(s.got.size(), 3u);
  EXPECT_FALSE(s.got[0].ok);
  EXPECT_EQ(s.got[0].error, "count");
  EXPECT_EQ(s.got[0].sequence.size(), 11u);
  EXPECT_EQ(s.got[0].cycles, 0);
  EXPECT_EQ(s.got[1].error, "unconfirmed");  // 오류 뒤에는 처음부터 다시 셈
  EXPECT_TRUE(s.got[2].ok);
}

TEST(Signal, BothColorsIsUnknown) {
  Sim s;
  s.play("_ RGRRGRGGAGYG _");  // 9번째가 두 색 동시 — 노랑으로 추정하지 않음
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_FALSE(s.got[0].ok);
  EXPECT_EQ(s.got[0].error, "unknown");
  EXPECT_EQ(s.got[0].sequence[8], "unknown");
  EXPECT_EQ(s.got[0].raw, "R G R R G R G G ? G Y G");
}

TEST(Signal, CountOutOfRange) {
  Sim s;
  s.play("_ RRRRRRRYYYGG _");  // 빨강 7회 — 발제 범위(1~6) 밖
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_EQ(s.got[0].error, "range");
  EXPECT_EQ(s.got[0].red, 7);
}

TEST(Signal, NoiseIgnored) {
  Sim s;
  s.play("_ RGR!RGRGGYGYG _ RGRRG!RGGYGYG _");  // 0.1초 잡음 켜짐은 안 셈
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].red, 4);
}

TEST(Signal, DifferentCyclesNotConfirmed) {
  Sim s;
  s.play(std::string("_ ") + kQ1 + " _ RRRRYYGGGGGG _");  // 두 사이클이 다르게 읽힘 — 둘 다 정상이어도 확정 안 함
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_FALSE(s.got[1].ok);
  EXPECT_EQ(s.got[1].error, "unconfirmed");
  EXPECT_EQ(s.got[1].cycles, 1);
}

TEST(Signal, ConfirmOne) {
  SignalParams p;
  p.confirm = 1;  // 한 사이클만으로 확정 (테스트·시연용)
  Sim s(p);
  s.play(std::string("_ ") + kQ1 + " _");
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_TRUE(s.got[0].ok);
}

// ---- 실물 녹화 재생(10/28, tl_bags · bee_yolo11n_v2 모델)에서 나온 경우들
TEST(Signal, ColorFlickerInsideBlinkNotSplit) {
  Sim s;
  // 빨강 1초 중 0.3초를 모델이 노랑으로 봄 → 쪼개지 않고 다수결 빨강 1번
  const auto red_flicker = [&s] { s.hold("red", 0.4); s.hold("yellow", 0.3); s.hold("red", 0.3); s.hold("off", 1.0); };
  for (int c = 0; c < 2; ++c) {
    s.hold("off", 3.0);
    s.play("RGRRG");
    red_flicker();  // 6번째 R
    s.play("GGYGYG");
  }
  s.hold("off", 3.0);
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].raw, "R G R R G R G G Y G Y G");
}

TEST(Signal, DroppedFrameInsideBlinkNotSplit) {
  Sim s;
  // 초록 1초 중 0.2초 신호등을 놓침('' = 안 보임) → min_off_s(0.3초)보다 짧아 한 번으로
  const auto green_drop = [&s] { s.hold("green", 0.4); s.hold("", 0.2); s.hold("green", 0.4); s.hold("off", 1.0); };
  for (int c = 0; c < 2; ++c) {
    s.hold("off", 3.0);
    s.play("RGRRGRG");
    green_drop();  // 8번째 G
    s.play("YGYG");
  }
  s.hold("off", 3.0);
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].green, 6);
}

TEST(Signal, NoiseInsideGapKeepsBoundary) {
  Sim s;
  // 사이클 사이 3초 꺼짐 가운데 0.2초 잡음 켜짐 → 버리고 꺼짐 시간은 이어서 셈 (두 사이클이 합쳐지지 않음)
  s.hold("off", 3.0);
  s.play(kQ1);
  s.hold("off", 1.0);
  s.hold("green", 0.2);
  s.hold("off", 1.8);
  s.play(kQ1);
  s.hold("off", 3.0);
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_EQ(s.got[0].error, "unconfirmed");
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].sequence.size(), 12u);
}

TEST(Signal, MixedColorsIsUnknown) {
  Sim s;
  // 켜진 구간 안 색이 반반 이하로 갈림 (빨 0.4 · 노 0.3 · 초 0.3) → 추정하지 않고 unknown
  s.hold("off", 3.0);
  s.play("RGRRGRGG");
  s.hold("red", 0.4); s.hold("yellow", 0.3); s.hold("green", 0.3); s.hold("off", 1.0);
  s.play("GYG");
  s.hold("off", 3.0);
  ASSERT_EQ(s.got.size(), 1u);
  EXPECT_EQ(s.got[0].error, "unknown");
}

TEST(Signal, Slow4Hz) {
  Sim s;
  s.dt = 0.25;  // 젯슨 CPU 추론이 느릴 때 (4Hz)
  s.play(std::string("_ ") + kQ1 + " _ " + kQ1 + " _");
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_TRUE(s.got[1].ok);
  EXPECT_EQ(s.got[1].raw, "R G R R G R G G Y G Y G");
}

TEST(Signal, NoiseInsideBlinkGapDoesNotMerge) {
  Sim s;
  // 실물 재생(light1 seed 2)에서 나온 경우: 3·4번째 R 사이 1초 꺼짐 한가운데 0.2초 노랑 잡음
  //   → 꺼짐이 0.4 + 0.4초로 쪼개져도 각각 점멸 사이로 끊고, 잡음은 버림 (R 두 번이 하나로 합쳐지면 안 됨)
  for (int c = 0; c < 2; ++c) {
    s.hold("off", 3.0);
    s.play("RG");
    s.hold("red", 1.0); s.hold("off", 0.4); s.hold("yellow", 0.2); s.hold("off", 0.4);  // 3번째 R + 잡음 낀 꺼짐
    s.play("RGRGGYGYG");
  }
  s.hold("off", 3.0);
  ASSERT_EQ(s.got.size(), 2u);
  EXPECT_EQ(s.got[0].raw, "R G R R G R G G Y G Y G");
  EXPECT_TRUE(s.got[1].ok);
}
