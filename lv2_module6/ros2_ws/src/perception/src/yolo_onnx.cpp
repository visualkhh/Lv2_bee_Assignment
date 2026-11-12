// yolo_onnx 구현 — 설명·선언은 include/perception/yolo_onnx.hpp
//   숫자 규칙은 ultralytics 8.x detect(LetterBox · non_max_suppression · scale_boxes)를 그대로 옮김
#include "perception/yolo_onnx.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numeric>
#include <regex>

#include <opencv2/imgproc.hpp>

namespace perception {

namespace {

// 파이썬 round()와 같은 반올림 (0.5는 짝수 쪽 — 기본 반올림 모드 to-nearest-even)
double py_round(double v) { return std::nearbyint(v); }

// 두 네모 IoU (torchvision.ops.nms와 같은 넓이 계산 — +1 없음)
float iou_xyxy(const std::array<float, 4> & a, const std::array<float, 4> & b) {
  const float w = std::min(a[2], b[2]) - std::max(a[0], b[0]);
  const float h = std::min(a[3], b[3]) - std::max(a[1], b[1]);
  if (w <= 0.f || h <= 0.f) return 0.f;
  const float inter = w * h;
  const float ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
  return ua > 0.f ? inter / ua : 0.f;
}

// "{0: 'puck_red_box', 1: 'puck_red_cyl', ...}" → {0: puck_red_box, ...} (ultralytics가 onnx metadata에 넣는 형식)
std::map<int, std::string> parse_names(const std::string & s) {
  std::map<int, std::string> out;
  static const std::regex re(R"((\d+)\s*:\s*['"]([^'"]*)['"])");
  for (auto it = std::sregex_iterator(s.begin(), s.end(), re); it != std::sregex_iterator(); ++it)
    out[std::stoi((*it)[1].str())] = (*it)[2].str();
  return out;
}

}  // namespace

struct YoloOnnx::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "perception_yolo"};  // 세션보다 먼저 만들고 늦게 지워야 함 (멤버 순서)
  std::unique_ptr<Ort::Session> session;
  std::string input_name, output_name, provider{"none"}, error;
  int in_h{0}, in_w{0};  // 모델 입력 크기 [px]
  std::map<int, std::string> names;
};

YoloOnnx::YoloOnnx(const std::string & path, const std::string & device, int cpu_threads, int imgsz)
: impl_(std::make_shared<Impl>()) {
  auto & m = *impl_;
  // GPU를 원하나: "cuda" · "cuda:0" · "0"(GPU 번호, ultralytics device 표기)
  std::string dev = device;
  std::transform(dev.begin(), dev.end(), dev.begin(), [](unsigned char c) { return std::tolower(c); });
  const bool want_cuda = dev.rfind("cuda", 0) == 0 || (!dev.empty() && std::all_of(dev.begin(), dev.end(), ::isdigit));
  int device_id = 0;
  if (const auto c = dev.find(':'); c != std::string::npos) device_id = std::atoi(dev.c_str() + c + 1);
  else if (!dev.empty() && std::isdigit(static_cast<unsigned char>(dev[0]))) device_id = std::atoi(dev.c_str());

  const auto make_options = [&](bool cuda) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(std::max(1, cpu_threads));  // CPU 연산 스레드 (GPU여도 앞뒤 처리에 씀)
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    if (cuda) {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = device_id;
      so.AppendExecutionProvider_CUDA(cuda_options);  // CUDA 빌드가 아니면 여기서 예외
    }
    return so;
  };
  try {
    if (want_cuda) {
      const auto av = Ort::GetAvailableProviders();
      if (std::find(av.begin(), av.end(), "CUDAExecutionProvider") != av.end()) {
        try {
          m.session = std::make_unique<Ort::Session>(m.env, path.c_str(), make_options(true));
          m.provider = "CUDAExecutionProvider";
        } catch (const Ort::Exception & e) {
          m.error = std::string("CUDA 실패 — CPU로: ") + e.what();
        }
      } else {
        m.error = "이 ONNX Runtime에 CUDAExecutionProvider 없음 — CPU로";
      }
    }
    if (!m.session) {
      m.session = std::make_unique<Ort::Session>(m.env, path.c_str(), make_options(false));
      m.provider = "CPUExecutionProvider";
    }
    Ort::AllocatorWithDefaultOptions alloc;
    m.input_name = m.session->GetInputNameAllocated(0, alloc).get();
    m.output_name = m.session->GetOutputNameAllocated(0, alloc).get();
    // 입력 크기: 고정이면 그 값 (1×3×H×W) · 동적(-1)이면 imgsz 정사각형
    const auto shape = m.session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    m.in_h = shape.size() == 4 && shape[2] > 0 ? static_cast<int>(shape[2]) : imgsz;
    m.in_w = shape.size() == 4 && shape[3] > 0 ? static_cast<int>(shape[3]) : imgsz;
    // 클래스 이름: ultralytics가 넣은 metadata "names"
    const auto meta = m.session->GetModelMetadata();
    auto names = meta.LookupCustomMetadataMapAllocated("names", alloc);
    if (names) m.names = parse_names(names.get());
  } catch (const Ort::Exception & e) {
    m.session.reset();
    m.provider = "none";
    m.error = std::string("모델 로드 실패: ") + e.what();
  }
}

bool YoloOnnx::ok() const { return impl_->session != nullptr; }
std::string YoloOnnx::provider() const { return impl_->provider; }
std::string YoloOnnx::error() const { return impl_->error; }
const std::map<int, std::string> & YoloOnnx::names() const { return impl_->names; }

std::vector<YoloBox> YoloOnnx::predict(const cv::Mat & bgr, float conf, bool agnostic_nms, float iou, int max_det) {
  auto & m = *impl_;
  std::vector<YoloBox> out;
  if (!m.session || bgr.empty()) return out;
  const int h0 = bgr.rows, w0 = bgr.cols, H = m.in_h, W = m.in_w;

  // ---- LetterBox (ultralytics: auto=False · 가운데 · 여백 114) — 비율 유지해 줄이고(키우고) 남는 곳을 회색으로
  const double r = std::min(static_cast<double>(H) / h0, static_cast<double>(W) / w0);
  const int nw = static_cast<int>(py_round(w0 * r)), nh = static_cast<int>(py_round(h0 * r));
  const double dw = (W - nw) / 2.0, dh = (H - nh) / 2.0;
  cv::Mat img = bgr;
  if (nw != w0 || nh != h0) cv::resize(bgr, img, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  const int top = static_cast<int>(py_round(dh - 0.1)), bottom = static_cast<int>(py_round(dh + 0.1));
  const int left = static_cast<int>(py_round(dw - 0.1)), right = static_cast<int>(py_round(dw + 0.1));
  cv::Mat lb;
  cv::copyMakeBorder(img, lb, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  if (lb.rows != H || lb.cols != W) cv::resize(lb, lb, cv::Size(W, H));  // 반올림으로 1px 어긋나면 맞춤 (보통 안 일어남)

  // ---- BGR → RGB · 0~1 · NCHW
  std::vector<float> input(static_cast<size_t>(3) * H * W);
  for (int y = 0; y < H; ++y) {
    const auto * row = lb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < W; ++x) {
      const size_t i = static_cast<size_t>(y) * W + x;
      input[i] = row[x][2] / 255.f;                               // R
      input[static_cast<size_t>(H) * W + i] = row[x][1] / 255.f;  // G
      input[static_cast<size_t>(2) * H * W + i] = row[x][0] / 255.f;  // B
    }
  }
  const std::array<int64_t, 4> in_shape{1, 3, H, W};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value in_tensor = Ort::Value::CreateTensor<float>(mem, input.data(), input.size(), in_shape.data(), in_shape.size());
  const char * in_names[] = {m.input_name.c_str()};
  const char * out_names[] = {m.output_name.c_str()};
  auto outputs = m.session->Run(Ort::RunOptions{nullptr}, in_names, &in_tensor, 1, out_names, 1);
  const auto oshape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();  // (1, 4 + 클래스 수, N)
  if (oshape.size() != 3) return out;
  const int nc = static_cast<int>(oshape[1]) - 4, n = static_cast<int>(oshape[2]);
  const float * p = outputs[0].GetTensorData<float>();

  // ---- 후보: 가장 높은 클래스 점수 > conf (ultralytics non_max_suppression, multi_label 끔)
  struct Cand { std::array<float, 4> b; float s; int c; };
  std::vector<Cand> cands;
  for (int i = 0; i < n; ++i) {
    int best = 0;
    float s = -1.f;
    for (int c = 0; c < nc; ++c) {
      const float v = p[static_cast<size_t>(4 + c) * n + i];
      if (v > s) { s = v; best = c; }
    }
    if (!(s > conf)) continue;
    const float cx = p[i], cy = p[n + i], bw = p[2 * n + i], bh = p[3 * n + i];
    cands.push_back({{cx - bw / 2.f, cy - bh / 2.f, cx + bw / 2.f, cy + bh / 2.f}, s, best});
  }
  // 점수 높은 순 (같으면 원래 순서) · 최대 30000개 (ultralytics max_nms)
  std::stable_sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) { return a.s > b.s; });
  if (cands.size() > 30000) cands.resize(30000);

  // ---- NMS: 클래스마다 따로 (클래스 번호 × 7680만큼 옮겨 서로 안 겹치게 — agnostic이면 안 옮김) · IoU > iou면 지움
  const float max_wh = 7680.f;
  std::vector<std::array<float, 4>> shifted(cands.size());
  for (size_t i = 0; i < cands.size(); ++i) {
    const float off = agnostic_nms ? 0.f : cands[i].c * max_wh;
    shifted[i] = {cands[i].b[0] + off, cands[i].b[1] + off, cands[i].b[2] + off, cands[i].b[3] + off};
  }
  std::vector<char> removed(cands.size(), 0);
  std::vector<size_t> keep;
  for (size_t i = 0; i < cands.size() && static_cast<int>(keep.size()) < max_det; ++i) {
    if (removed[i]) continue;
    keep.push_back(i);
    for (size_t j = i + 1; j < cands.size(); ++j)
      if (!removed[j] && iou_xyxy(shifted[i], shifted[j]) > iou) removed[j] = 1;
  }

  // ---- 원본 좌표로 (scale_boxes) · 화면 안으로 자름 (clip_boxes)
  const double gain = std::min(static_cast<double>(H) / h0, static_cast<double>(W) / w0);
  const double pad_x = py_round((W - w0 * gain) / 2.0 - 0.1), pad_y = py_round((H - h0 * gain) / 2.0 - 0.1);
  for (size_t k : keep) {
    const auto & c = cands[k];
    const auto sx = [&](float v) { return std::clamp(static_cast<float>((v - pad_x) / gain), 0.f, static_cast<float>(w0)); };
    const auto sy = [&](float v) { return std::clamp(static_cast<float>((v - pad_y) / gain), 0.f, static_cast<float>(h0)); };
    out.push_back({sx(c.b[0]), sy(c.b[1]), sx(c.b[2]), sy(c.b[3]), c.s, c.c});
  }
  return out;
}

}  // namespace perception
