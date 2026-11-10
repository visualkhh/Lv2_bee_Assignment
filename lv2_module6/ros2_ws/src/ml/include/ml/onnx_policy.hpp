// OnnxPolicy — 로봇의 모든 학습 정책(팔 보정 · 도킹 · Nav2 주행 · plan_next)을 ONNX Runtime으로 추론하는 한 곳.
//   입력 [1, n_in] float · 출력 [1, n_out] float (스케일·clamp는 쓰는 쪽에서).
//   파일은 가중치를 안에 담은 .onnx 하나 (isaac/policies/export_onnx.py). 메타데이터 bee_test_x · bee_test_y가 있으면
//   불러올 때 같은 출력이 나오는지 검산한다 (내보낼 때 PyTorch로 계산한 값).
//   기본 CPU (작은 MLP — GPU는 복사·실행 준비 비용이 더 큼). gpu=true면 CUDA EP를 먼저 시도.
//   act_into: 불러올 때 잡은 버퍼에 묶은 텐서로 실행 — 호출마다 입출력 버퍼를 새로 잡지 않음 (rl_arm_correction 실시간 루프).
//   onnxruntime_vendor(ros-lyrical-onnxruntime-vendor)가 없으면 ML_HAS_ONNX 미정의 → load()가 실패하고 쓰는 쪽은 바이패스.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
struct Value;
}  // namespace Ort

namespace ml {

class OnnxPolicy {
public:
  OnnxPolicy();
  ~OnnxPolicy();
  OnnxPolicy(OnnxPolicy &&) noexcept;
  OnnxPolicy & operator=(OnnxPolicy &&) noexcept;

  void load(const std::string & path, bool gpu = false);  // 실패하면 std::runtime_error (이전 모델 유지)
  bool load(const std::string & path, std::string & why) noexcept;  // 실패하면 false + why
  void unload();
  bool loaded() const { return session_ != nullptr; }
  int64_t inputs() const { return n_in_; }
  int64_t outputs() const { return n_out_; }
  const std::string & path() const { return path_; }
  const std::string & device() const { return device_; }  // "CUDA" 또는 "CPU" — 실제 세션이 쓰는 EP
  const std::string & device_note() const { return device_note_; }  // CUDA 시도가 실패한 이유 (없으면 "")
  std::string provider() const { return device_ == "CUDA" ? "CUDAExecutionProvider" : "CPUExecutionProvider"; }
  static std::string runtime_version();  // 예: "1.24.3" (ONNX 없이 빌드면 "none")
  std::vector<float> run(const std::vector<float> & obs) const;  // obs.size() == inputs()
  // x[inputs()] → y[outputs()] — 같은 객체를 두 스레드에서 동시에 부르지 말 것 (버퍼 공유)
  void act_into(const double * x, double * y) const;

private:
  std::shared_ptr<Ort::Session> session_;  // 불완전 타입 OK (소멸자 타입소거)
  std::shared_ptr<Ort::Value> in_t_, out_t_;  // in_buf_ · out_buf_에 묶인 텐서
  mutable std::vector<float> in_buf_, out_buf_;
  std::string path_, in_name_, out_name_, device_ = "CPU", device_note_;
  int64_t n_in_ = 0, n_out_ = 0;
};

}  // namespace ml
