// OnnxPolicy — Isaac Lab/rsl_rl 학습 정책(policy.onnx)을 ONNX Runtime으로 추론.
//   입력 [1, n_in] float · 출력 [1, n_out] float (스케일·clamp는 쓰는 쪽에서).
//   onnxruntime_vendor(ros-lyrical-onnxruntime-vendor)가 없으면 ML_HAS_ONNX 미정의 → 이 클래스는 비어 있고
//   쓰는 쪽은 바이패스한다.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
}  // namespace Ort

namespace ml {

class OnnxPolicy {
public:
  OnnxPolicy();
  ~OnnxPolicy();
  OnnxPolicy(OnnxPolicy &&) noexcept;
  OnnxPolicy & operator=(OnnxPolicy &&) noexcept;

  void load(const std::string & path);  // 실패하면 std::runtime_error (이전 모델 유지)
  void unload();
  bool loaded() const { return session_ != nullptr; }
  int64_t inputs() const { return n_in_; }
  int64_t outputs() const { return n_out_; }
  const std::string & path() const { return path_; }
  std::vector<float> run(const std::vector<float> & obs) const;  // obs.size() == inputs()

private:
  std::shared_ptr<Ort::Session> session_;  // 불완전 타입 OK (소멸자 타입소거)
  std::string path_, in_name_, out_name_;
  int64_t n_in_ = 0, n_out_ = 0;
};

}  // namespace ml
