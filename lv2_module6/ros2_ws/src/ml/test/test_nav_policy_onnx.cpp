// Nav2 주행 정책 ONNX 로드 테스트 — rl_nav_controller가 기대하는 입출력과 맞는지
// (nav2_params.yaml FollowPath 값과 동기화: lookahead=5 · lidar_rays=360 → 입력 2*5+360+5=375, 출력 2 이상)
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <iostream>

#include "ml/onnx_policy.hpp"

namespace fs = std::filesystem;

TEST(NavPolicyOnnx, LoadsWithExpectedDims) {
  if (ml::OnnxPolicy::runtime_version() == "none") GTEST_SKIP() << "onnxruntime_vendor 없음 — ONNX 없이 빌드됨";
  const fs::path path = fs::path(MODEL_DIR) / "rl_nav_controller" / "policy.onnx";
  ASSERT_TRUE(fs::exists(path)) << "모델 없음: " << path;
  ml::OnnxPolicy p;
  EXPECT_NO_THROW(p.load(path.string()));
  ASSERT_TRUE(p.loaded());
  std::cout << "[onnx] device=" << p.device() << " (CUDA EP 있으면 CUDA, 없으면 CPU)"
            << (p.device_note().empty() ? "" : " · CUDA 실패: " + p.device_note()) << std::endl;
  // nav2_params.yaml FollowPath 값(lookahead_=5 · lidar_rays_=360)과 동기화 — 바꾸면 여기 숫자도 함께
  EXPECT_EQ(p.inputs(), 2 * 5 + 360 + 5);
  EXPECT_GE(p.outputs(), 2);
}

TEST(NavPolicyOnnx, RunsZeroObs) {
  if (ml::OnnxPolicy::runtime_version() == "none") GTEST_SKIP() << "onnxruntime_vendor 없음 — ONNX 없이 빌드됨";
  const fs::path path = fs::path(MODEL_DIR) / "rl_nav_controller" / "policy.onnx";
  ml::OnnxPolicy p;
  p.load(path.string());
  ASSERT_TRUE(p.loaded());
  const std::vector<float> out = p.run(std::vector<float>(static_cast<size_t>(p.inputs()), 0.0f));
  ASSERT_EQ(static_cast<int64_t>(out.size()), p.outputs());
  for (float v : out) EXPECT_TRUE(std::isfinite(v));
}
