// 내보낸 정책 파일이 C++ 계산으로 TorchScript와 같은 값을 내는지 (파일 안 'test' 줄로 검산) — 모든 .policy
#include <gtest/gtest.h>

#include <filesystem>

#include "ml/mlp_policy.hpp"

TEST(MlpPolicy, AllExportedPoliciesMatchTorch) {
  const std::filesystem::path root = std::filesystem::path(MODEL_DIR);
  int n = 0;
  for (const auto & e : std::filesystem::recursive_directory_iterator(root)) {
    if (e.path().extension() != ".policy") continue;
    ml::MlpPolicy p;
    std::string why;
    EXPECT_TRUE(p.load(e.path().string(), why)) << why;
    ++n;
  }
  EXPECT_GE(n, 1);
}

TEST(MlpPolicy, RejectsBadFile) {
  const auto path = std::filesystem::temp_directory_path() / "bad.policy";
  { std::ofstream f(path); f << "nope"; }
  ml::MlpPolicy p;
  std::string why;
  EXPECT_FALSE(p.load(path.string(), why));
}
