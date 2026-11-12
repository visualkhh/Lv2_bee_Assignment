#pragma once

// plan_next — 학습된 플래너: 지금 상태 → 다음 skill 하나 + 대상 퍽 (가상 미션 learning/plan/mission_sim.py에서 학습)
//   입력·행동·막기 규칙은 mission_sim.MissionSim과 같음 (test/test_plan_next.cpp가 Python 결과와 비교):
//     퍽 i (12개, 시작 박스 i/2): 상태 one-hot 8 · 장소 one-hot 3 · 가운데 칸 1 · min(시도,5)/5 1 → 156
//     + 로봇 자리 one-hot 10 (시작 박스 0~5 · 장소 6~8 · 출발 구역 9) + min(시간/3600, 1) → 167
//   행동: 퍽 i × 8 skill + 복귀(96). 시작 조건이 안 맞는 행동 · 아직 로봇에 없는 skill(밀어 맞추기·놓인 퍽 세우기) · 3번 연달아
//   바뀐 게 없던 행동은 고를 수 없음. 정책 파일: share/ml/model/plan/plan_next.onnx (ml::Policy · 불러올 때 검산 · plan_model로 .policy도 가능)
#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <vector>

#include "ml/policy.hpp"

namespace plan {

class PlanNext {
public:
  enum Status { BOX_UP, BOX_LYING, HAND, BASKET, OK, OFF, PLACED_LYING, LOST, N_STATUS };
  enum Skill { PICK, STAND, LOAD, UNLOAD, PLACE, NUDGE, REPICK, STAND_PLACED, N_SKILL };
  static constexpr int N_PUCK = 12, RETURN = N_PUCK * N_SKILL, N_PLACES = 10, HOME = 9;
  static constexpr double TIME_LIMIT = 3600.0;

  struct State {
    std::array<int, N_PUCK> status{}, tries{}, loc{}, cell{};  // loc 0~2 (L1~L3) · cell 0~8 (4 = 가운데)
    int at = HOME;      // 로봇 자리
    double t = 0.0;     // 미션 시작 뒤 [s]
  };

  bool load(const std::string & path, std::string & why) { return policy_.load(path, why) && policy_.inputs() == 167 && policy_.outputs() == size_t(RETURN + 1); }

  static std::vector<double> obs(const State & s) {
    std::vector<double> o;
    o.reserve(167);
    for (int i = 0; i < N_PUCK; ++i) {
      for (int k = 0; k < N_STATUS; ++k) o.push_back(s.status[i] == k ? 1.0 : 0.0);
      for (int k = 0; k < 3; ++k) o.push_back(s.loc[i] == k ? 1.0 : 0.0);
      o.push_back(s.cell[i] == 4 ? 1.0 : 0.0);
      o.push_back(std::min(s.tries[i], 5) / 5.0);
    }
    for (int k = 0; k < N_PLACES; ++k) o.push_back(s.at == k ? 1.0 : 0.0);
    o.push_back(std::min(s.t / TIME_LIMIT, 1.0));
    return o;
  }

  // 고를 수 있는 행동 (mission_sim.action_masks + 로봇에 없는 skill + 반복 실패)
  std::vector<bool> mask(const State & s) const {
    std::vector<bool> m(RETURN + 1, false);
    const bool hand_empty = std::none_of(s.status.begin(), s.status.end(), [](int v) { return v == HAND; });
    const int in_basket = static_cast<int>(std::count(s.status.begin(), s.status.end(), static_cast<int>(BASKET)));
    for (int i = 0; i < N_PUCK; ++i) {
      const int b = i * N_SKILL, st = s.status[i];
      m[b + PICK] = st == BOX_UP && hand_empty;
      m[b + STAND] = st == BOX_LYING && hand_empty;
      m[b + LOAD] = st == HAND && in_basket < 12;
      m[b + UNLOAD] = st == BASKET && hand_empty;
      m[b + PLACE] = st == HAND;
      m[b + REPICK] = st == OFF && hand_empty;
      // NUDGE · STAND_PLACED: 로봇에 아직 없음
    }
    m[RETURN] = true;
    for (const auto & [a, n] : fails_) if (n >= 3) m[a] = false;
    return m;
  }

  // 다음 행동 (RETURN이면 복귀)
  int next(const State & s) const {
    const auto o = obs(s);
    std::vector<double> logits(RETURN + 1);
    policy_.act_into(o.data(), logits.data());
    const auto m = mask(s);
    int best = RETURN;
    for (int a = 0; a <= RETURN; ++a) if (m[a] && logits[a] > logits[best]) best = a;
    return best;
  }

  // 행동 결과 기록 — 바뀐 게 없으면 실패 횟수 +1 (3번이면 그 행동 막음)
  void record(int action, bool changed) { fails_[action] = changed ? 0 : fails_[action] + 1; }
  void reset() { fails_.clear(); }

  static int puck_of(int action) { return action / N_SKILL; }
  static int skill_of(int action) { return action % N_SKILL; }

private:
  ml::Policy policy_;
  std::map<int, int> fails_;
};

}  // namespace plan
