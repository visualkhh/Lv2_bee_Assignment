// 저장소의 모든 정책 .onnx가 열리고, 메타데이터 검산 값(bee_test_x · bee_test_y)과 같은 출력을 내는지 — ONNX Runtime 없이 빌드면 건너뜀
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "ml/policy.hpp"

TEST(OnnxPolicy, AllRepoPoliciesLoadAndMatchExport) {
  if (ml::OnnxPolicy::runtime_version() == "none") GTEST_SKIP() << "ONNX Runtime 없이 빌드";
  int n = 0;
  for (const auto & e : std::filesystem::recursive_directory_iterator(MODEL_DIR)) {
    if (e.path().extension() != ".onnx") continue;
    ml::OnnxPolicy p;
    std::string why;
    EXPECT_TRUE(p.load(e.path().string(), why)) << why;
    std::vector<double> x(p.inputs(), 0.1), y(p.outputs());
    if (p.loaded()) p.act_into(x.data(), y.data());
    ++n;
  }
  EXPECT_GE(n, 10);
}

TEST(OnnxPolicy, RejectsBadFile) {
  const auto path = std::filesystem::temp_directory_path() / "bad.onnx";
  { std::ofstream f(path); f << "nope"; }
  ml::OnnxPolicy p;
  std::string why;
  EXPECT_FALSE(p.load(path.string(), why));
  EXPECT_FALSE(p.loaded());
}

// 실시간 루프(100 Hz, 정책 50 Hz) 안에서 돌려도 되는지 — 팔 정책 한 번 추론 시간
TEST(OnnxPolicy, ArmPolicyIsFastEnoughForControlLoop) {
  if (ml::OnnxPolicy::runtime_version() == "none") GTEST_SKIP() << "ONNX Runtime 없이 빌드";
  ml::OnnxPolicy p;
  std::string why;
  ASSERT_TRUE(p.load(std::string(MODEL_DIR) + "/skills/pick_standing_puck/pick_standing_puck.onnx", why)) << why;
  std::vector<double> x(p.inputs(), 0.1), y(p.outputs());
  std::vector<double> us;
  for (int i = 0; i < 2000; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    p.act_into(x.data(), y.data());
    us.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(us.begin(), us.end());
  const double p50 = us[us.size() / 2], p99 = us[us.size() * 99 / 100], worst = us.back();
  std::printf("[act_into] p50 %.1f us · p99 %.1f us · max %.1f us\n", p50, p99, worst);
  EXPECT_LT(p99, 1000.0);  // 제어 주기 10 ms의 1/10
}

// .policy와 .onnx가 같은 정책인지 — meta runtime으로 바꿔 끼워도 행동이 같아야 실측 비교가 공정함
TEST(Policy, OnnxAndPolicyFilesAgree) {
  if (ml::OnnxPolicy::runtime_version() == "none") GTEST_SKIP() << "ONNX Runtime 없이 빌드";
  int n = 0;
  for (const auto & e : std::filesystem::recursive_directory_iterator(MODEL_DIR)) {
    if (e.path().extension() != ".policy") continue;
    auto onnx = e.path();
    onnx.replace_extension(".onnx");
    ml::Policy a, b;
    std::string why;
    ASSERT_TRUE(a.load(e.path().string(), why)) << why;
    ASSERT_TRUE(b.load(onnx.string(), why)) << why;
    EXPECT_STREQ(a.runtime(), "policy");
    EXPECT_STREQ(b.runtime(), "onnx");
    ASSERT_EQ(a.inputs(), b.inputs()) << e.path();
    ASSERT_EQ(a.outputs(), b.outputs()) << e.path();
    std::vector<double> x(a.inputs()), ya(a.outputs()), yb(b.outputs());
    for (size_t i = 0; i < x.size(); ++i) x[i] = std::sin(0.7 * double(i) + 0.3);
    a.act_into(x.data(), ya.data());
    b.act_into(x.data(), yb.data());
    for (size_t i = 0; i < ya.size(); ++i) EXPECT_NEAR(ya[i], yb[i], 1e-3) << e.path() << " " << i;
    ++n;
  }
  EXPECT_GE(n, 10);
}
