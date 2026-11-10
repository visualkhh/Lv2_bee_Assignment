#pragma once

// Policy — 학습 정책 하나를 파일 확장자로 골라 실행 (팔 보정 · 도킹 · plan_next가 씀)
//   .onnx   → OnnxPolicy (ONNX Runtime, CPU)      — 기본
//   .policy → MlpPolicy  (직접 짠 C++ MLP)         — 실측에서 ONNX가 나쁜 정책만 되돌릴 때
//   고르는 곳: skill 정책은 models/skills/<skill>/meta.yaml `runtime: onnx|policy` · plan_next는 plan.yaml plan_model 경로
#include <string>

#include "ml/mlp_policy.hpp"
#include "ml/onnx_policy.hpp"

namespace ml {

class Policy {
public:
  bool load(const std::string & path, std::string & why) {
    onnx_ = !(path.size() >= 7 && path.compare(path.size() - 7, 7, ".policy") == 0);
    return onnx_ ? onnx_policy_.load(path, why) : mlp_.load(path, why);
  }
  size_t inputs() const { return onnx_ ? size_t(onnx_policy_.inputs()) : mlp_.obs_dim(); }
  size_t outputs() const { return onnx_ ? size_t(onnx_policy_.outputs()) : mlp_.act_dim(); }
  const char * runtime() const { return onnx_ ? "onnx" : "policy"; }
  // x[inputs()] → y[outputs()] (실시간 루프에서 써도 됨 — 둘 다 불러올 때 잡은 버퍼만 씀)
  void act_into(const double * x, double * y) const { onnx_ ? onnx_policy_.act_into(x, y) : mlp_.act_into(x, y); }

private:
  bool onnx_ = true;
  OnnxPolicy onnx_policy_;
  MlpPolicy mlp_;
};

}  // namespace ml
