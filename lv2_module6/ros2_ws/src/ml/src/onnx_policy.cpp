// OnnxPolicy 구현 — onnxruntime_vendor가 있을 때만 컴파일(ML_HAS_ONNX). 없으면 빈 구현(load → 예외).
#include "ml/onnx_policy.hpp"

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

bool cuda_available() {
  // 이 ORT 빌드에 CUDA EP가 들어 있는지 — 없으면(CPU-only vendor) 조용히 CPU로 간다.
  // GetAvailableProviders는 컴파일된 목록이라, sm_87 커널이 없는 빌드도 CUDA를 넣을 수 있다.
  // 실제 GPU에서 커널이 없으면 세션 생성이 실패하므로 그때도 CPU로 되돌린다.
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

void OnnxPolicy::load(const std::string & path) {
  try {
    std::shared_ptr<Ort::Session> s;
    std::string device = "CPU", note;
    if (cuda_available()) {
      try {
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
    in_name_ = s->GetInputNameAllocated(0, alloc).get();
    out_name_ = s->GetOutputNameAllocated(0, alloc).get();
    n_in_ = in_shape[1];
    n_out_ = out_shape[1];
    session_ = std::move(s);  // 다 읽은 뒤에만 교체
    path_ = path;
    device_ = device;
    device_note_ = note;
  } catch (const Ort::Exception & e) {
    throw std::runtime_error(std::string("ONNX 열기 실패: ") + path + " (" + e.what() + ")");
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(std::string("ONNX 형식: ") + path + " — " + e.what());
  }
}

void OnnxPolicy::unload() {
  session_.reset();
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

void OnnxPolicy::load(const std::string &) {
  throw std::runtime_error("onnxruntime_vendor 없이 빌드됨 — ros-lyrical-onnxruntime-vendor 설치 후 다시 빌드");
}
void OnnxPolicy::unload() { path_.clear(); n_in_ = n_out_ = 0; }  // session_은 항상 null (stub)
std::vector<float> OnnxPolicy::run(const std::vector<float> &) const {
  throw std::runtime_error("ONNX 지원 없이 빌드됨");
}

#endif

}  // namespace ml
