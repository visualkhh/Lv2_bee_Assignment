// plan_next C++이 Python(가상 미션 + TorchScript)과 같은 다음 행동을 고르는지 — 기준값: test/plan_next_cases.txt (40개 상태)
#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "plan/plan_next.hpp"

TEST(PlanNext, MatchesPythonPlanner) {
  plan::PlanNext p;
  std::string why;
  ASSERT_TRUE(p.load(std::string(MODEL_DIR) + "/plan/plan_next.policy", why)) << why;
  std::ifstream f(std::string(CASES));
  ASSERT_TRUE(f.good());
  std::string line;
  int n = 0;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    for (auto & ch : line) if (ch == '|') ch = ' ';
    std::istringstream in(line);
    plan::PlanNext::State s;
    for (auto & v : s.status) in >> v;
    for (auto & v : s.tries) in >> v;
    for (int i = 0; i < 12; ++i) in >> s.loc[i] >> s.cell[i];
    int expect = -1;
    in >> s.at >> s.t >> expect;
    EXPECT_EQ(p.next(s), expect) << "case " << n;
    ++n;
  }
  EXPECT_EQ(n, 40);
}

TEST(PlanNext, RepeatedFailureIsMasked) {
  plan::PlanNext p;
  plan::PlanNext::State s;
  s.status.fill(plan::PlanNext::OK);
  s.status[3] = plan::PlanNext::OFF;
  const int a = 3 * plan::PlanNext::N_SKILL + plan::PlanNext::REPICK;
  EXPECT_TRUE(p.mask(s)[a]);
  for (int k = 0; k < 3; ++k) p.record(a, false);
  EXPECT_FALSE(p.mask(s)[a]);
  EXPECT_TRUE(p.mask(s)[plan::PlanNext::RETURN]);
}
