// OnnxPolicy 구현 — onnxruntime_vendor가 있을 때만 컴파일(ML_HAS_ONNX). 없으면 빈 구현(load → 예외).
#include "ml/onnx_policy.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <stdexcept>

#ifdef ML_HAS_ONNX
#include <onnxruntime_cxx_api.h>
#endif

namespace ml {

#ifdef ML_HAS_ONNX

namespace {
Ort::Env & env() {
  static Ort::Env e(ORT_LOGGING_LEVEL_WARNING, "ml_onnx");
  return e;
}

std::vector<float> floats(const std::string & csv) {
  std::vector<float> v;
  std::stringstream in(csv);
  std::string tok;
  while (std::getline(in, tok, ',')) v.push_back(std::stof(tok));
  return v;
}

bool cuda_available() {
  // 기본은 CUDA(가능하면) — 끄려면 환경변수 ML_ONNX_CUDA=0. (작은 MLP는 CPU가 더 빠르지만 GPU 사용이 요구사항)
  const char * off = std::getenv("ML_ONNX_CUDA");
  if (off && std::string(off) == "0") return false;
  try {
    for (const auto & p : Ort::GetAvailableProviders()) {
      if (p == "CUDAExecutionProvider") return true;
    }
  } catch (const Ort::Exception &) {
  }
  return false;
}
}  // namespace

OnnxPolicy::OnnxPolicy() = default;
OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy &&) noexcept = default;
OnnxPolicy & OnnxPolicy::operator=(OnnxPolicy &&) noexcept = default;

std::string OnnxPolicy::runtime_version() { return Ort::GetVersionString(); }

void OnnxPolicy::load(const std::string & path, bool gpu) {
  try {
    std::shared_ptr<Ort::Session> s;
    std::string device = "CPU", note;
    if (gpu && cuda_available()) {
      try {
        env();  // CUDA provider는 로드 시점에 DefaultLogger를 쓴다 — Env를 먼저 만들어 둬야 함
        Ort::SessionOptions opt;
        opt.SetIntraOpNumThreads(1);  // 20Hz 작은 MLP — 다른 노드와 CPU를 덜 다툰다
        OrtCUDAProviderOptions cuda{};
        cuda.device_id = 0;
        opt.AppendExecutionProvider_CUDA(cuda);
        s = std::make_shared<Ort::Session>(env(), path.c_str(), opt);
        device = "CUDA";
      } catch (const std::exception & e) {
        s.reset();  // CUDA EP/드라이버/커널 없음 → CPU로 폴백
        device = "CPU";
        note = e.what();
      }
    }
    if (!s) {
      Ort::SessionOptions opt;
      opt.SetIntraOpNumThreads(1);
      s = std::make_shared<Ort::Session>(env(), path.c_str(), opt);
      device = "CPU";
    }
    if (s->GetInputCount() != 1 || s->GetOutputCount() != 1) {
      throw std::runtime_error("입력·출력이 하나씩이 아님");
    }
    Ort::AllocatorWithDefaultOptions alloc;
    const auto in_shape = s->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto out_shape = s->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (in_shape.size() != 2 || out_shape.size() != 2 || in_shape[1] <= 0 || out_shape[1] <= 0) {
      throw std::runtime_error("모양이 [1, n]이 아님");
    }
    // 새 객체에 다 만든 뒤 검산하고, 통과하면 통째로 교체 (실패하면 이전 모델 유지)
    OnnxPolicy p;
    p.in_name_ = s->GetInputNameAllocated(0, alloc).get();
    p.out_name_ = s->GetOutputNameAllocated(0, alloc).get();
    p.n_in_ = in_shape[1];
    p.n_out_ = out_shape[1];
    p.in_buf_.assign(p.n_in_, 0.0f);
    p.out_buf_.assign(p.n_out_, 0.0f);
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const int64_t in_dims[2] = {1, p.n_in_}, out_dims[2] = {1, p.n_out_};
    p.in_t_ = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(mem, p.in_buf_.data(), p.in_buf_.size(), in_dims, 2));
    p.out_t_ = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<float>(mem, p.out_buf_.data(), p.out_buf_.size(), out_dims, 2));
    p.session_ = std::move(s);
    p.path_ = path;
    p.device_ = device;
    p.device_note_ = note;
    const auto meta = p.session_->GetModelMetadata();
    auto tx = meta.LookupCustomMetadataMapAllocated("bee_test_x", alloc);
    auto ty = meta.LookupCustomMetadataMapAllocated("bee_test_y", alloc);
    if (tx && ty) {  // 내보낼 때 계산한 값과 같은지
      const auto x = floats(tx.get()), y = floats(ty.get());
      if (int64_t(x.size()) != p.n_in_ || int64_t(y.size()) != p.n_out_) throw std::runtime_error("검산 값 크기가 안 맞음");
      std::vector<double> xd(x.begin(), x.end()), out(p.n_out_);
      p.act_into(xd.data(), out.data());
      for (int64_t i = 0; i < p.n_out_; ++i) {
        if (std::abs(out[i] - y[i]) > 1e-3) {
          throw std::runtime_error("검산 실패 (" + std::to_string(out[i]) + " ≠ " + std::to_string(y[i]) + ")");
        }
      }
    }
    *this = std::move(p);
  } catch (const Ort::Exception & e) {
    throw std::runtime_error(std::string("ONNX 열기 실패: ") + path + " (" + e.what() + ")");
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(std::string("ONNX 형식: ") + path + " — " + e.what());
  }
}

bool OnnxPolicy::load(const std::string & path, std::string & why) noexcept {
  try {
    load(path);
    return true;
  } catch (const std::exception & e) {
    why = e.what();
    return false;
  }
}

void OnnxPolicy::act_into(const double * x, double * y) const {
  if (!session_) throw std::runtime_error("정책 없음");
  for (int64_t i = 0; i < n_in_; ++i) in_buf_[i] = static_cast<float>(x[i]);
  const char * in_names[] = {in_name_.c_str()};
  const char * out_names[] = {out_name_.c_str()};
  session_->Run(Ort::RunOptions{nullptr}, in_names, in_t_.get(), 1, out_names, out_t_.get(), 1);
  for (int64_t i = 0; i < n_out_; ++i) y[i] = out_buf_[i];
}

void OnnxPolicy::unload() {
  session_.reset();
  in_t_.reset();
  out_t_.reset();
  path_.clear();
  n_in_ = n_out_ = 0;
}

std::vector<float> OnnxPolicy::run(const std::vector<float> & obs) const {
  if (!session_) throw std::runtime_error("정책 없음");
  if (int64_t(obs.size()) != n_in_) throw std::runtime_error("입력 크기 불일치");
  const int64_t shape[2] = {1, n_in_};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto in = Ort::Value::CreateTensor<float>(mem, const_cast<float *>(obs.data()), obs.size(), shape, 2);
  const char * in_names[] = {in_name_.c_str()};
  const char * out_names[] = {out_name_.c_str()};
  auto out = session_->Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);
  const float * p = out[0].GetTensorData<float>();
  return std::vector<float>(p, p + n_out_);
}

#else  // ML_HAS_ONNX 없음 — onnxruntime_vendor 미설치

OnnxPolicy::OnnxPolicy() = default;
OnnxPolicy::~OnnxPolicy() = default;
OnnxPolicy::OnnxPolicy(OnnxPolicy &&) noexcept = default;
OnnxPolicy & OnnxPolicy::operator=(OnnxPolicy &&) noexcept = default;

std::string OnnxPolicy::runtime_version() { return "none"; }

void OnnxPolicy::load(const std::string &, bool) {
  throw std::runtime_error("onnxruntime_vendor 없이 빌드됨 — ros-lyrical-onnxruntime-vendor 설치 후 다시 빌드");
}
bool OnnxPolicy::load(const std::string & path, std::string & why) noexcept {
  why = "onnxruntime_vendor 없이 빌드됨 (" + path + ")";
  return false;
}
void OnnxPolicy::act_into(const double *, double *) const { throw std::runtime_error("ONNX 지원 없이 빌드됨"); }
void OnnxPolicy::unload() { path_.clear(); n_in_ = n_out_ = 0; }  // session_은 항상 null (stub)
std::vector<float> OnnxPolicy::run(const std::vector<float> &) const {
  throw std::runtime_error("ONNX 지원 없이 빌드됨");
}

#endif

}  // namespace ml
