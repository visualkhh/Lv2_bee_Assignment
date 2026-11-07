#pragma once

// 학습 정책 (작은 MLP) 계산 — ONNX 런타임 없이 (실시간 컨트롤러에서 할당 없이 돌게, Jetson에 따로 설치할 것 없음)
//   파일: scripts/export_policy.py가 TorchScript(.pt, rsl_rl·sb3)에서 뽑은 텍스트 (.policy)
//     bee_policy 1 <elu|tanh> <obs_dim> <act_dim> <층 수>
//     mean[obs] · scale[obs]          → x' = (x − mean) · scale   (rsl_rl: scale = 1/(std + 0.01))
//     층마다: rows cols · W[rows·cols, 행 우선] · b[rows]   (마지막 층 뒤엔 활성화 없음)
//     test x[obs] y[act]              → 불러올 때 같은 값이 나오는지 확인 (내보낼 때 TorchScript로 계산한 값)
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

namespace ml {

class MlpPolicy {
public:
  bool load(const std::string & path, std::string & why) {
    std::ifstream f(path);
    std::string magic, act;
    int ver = 0;
    size_t n_layers = 0;
    if (!(f >> magic >> ver >> act >> obs_dim_ >> act_dim_ >> n_layers) || magic != "bee_policy" || ver != 1) {
      why = "형식 아님: " + path;
      return false;
    }
    tanh_ = act == "tanh";
    mean_.resize(obs_dim_);
    scale_.resize(obs_dim_);
    for (auto & v : mean_) f >> v;
    for (auto & v : scale_) f >> v;
    layers_.resize(n_layers);
    for (auto & l : layers_) {
      f >> l.rows >> l.cols;
      l.w.resize(l.rows * l.cols);
      l.b.resize(l.rows);
      for (auto & v : l.w) f >> v;
      for (auto & v : l.b) f >> v;
    }
    if (!f || layers_.empty() || layers_.front().cols != obs_dim_ || layers_.back().rows != act_dim_) {
      why = "크기가 안 맞음: " + path;
      return false;
    }
    size_t widest = obs_dim_;
    for (const auto & l : layers_) widest = std::max(widest, l.rows);
    a_.resize(widest);
    b_.resize(widest);
    std::string tag;
    if (f >> tag && tag == "test") {  // 내보낼 때 계산한 값과 같은지
      std::vector<double> x(obs_dim_), y(act_dim_), out(act_dim_);
      for (auto & v : x) f >> v;
      for (auto & v : y) f >> v;
      act_into(x.data(), out.data());
      for (size_t i = 0; i < act_dim_; ++i) {
        if (std::abs(out[i] - y[i]) > 1e-4) {
          why = "검산 실패 (" + std::to_string(out[i]) + " ≠ " + std::to_string(y[i]) + "): " + path;
          return false;
        }
      }
    }
    return true;
  }

  // x[obs_dim] → y[act_dim] (할당 없음 — 실시간 루프에서 써도 됨)
  void act_into(const double * x, double * y) const {
    double * in = a_.data();
    double * out = b_.data();
    for (size_t i = 0; i < obs_dim_; ++i) in[i] = (x[i] - mean_[i]) * scale_[i];
    for (size_t k = 0; k < layers_.size(); ++k) {
      const auto & l = layers_[k];
      const bool last = k + 1 == layers_.size();
      for (size_t r = 0; r < l.rows; ++r) {
        double s = l.b[r];
        const double * w = &l.w[r * l.cols];
        for (size_t c = 0; c < l.cols; ++c) s += w[c] * in[c];
        out[r] = last ? s : tanh_ ? std::tanh(s) : (s > 0.0 ? s : std::expm1(s));  // ELU(α=1)
      }
      if (last) std::copy(out, out + l.rows, y);
      std::swap(in, out);
    }
  }

  size_t obs_dim() const { return obs_dim_; }
  size_t act_dim() const { return act_dim_; }

private:
  struct Layer { size_t rows = 0, cols = 0; std::vector<double> w, b; };
  size_t obs_dim_ = 0, act_dim_ = 0;
  bool tanh_ = false;
  std::vector<double> mean_, scale_;
  std::vector<Layer> layers_;
  mutable std::vector<double> a_, b_;  // 층 사이 버퍼 (불러올 때 한 번 잡음)
};

}  // namespace ml
