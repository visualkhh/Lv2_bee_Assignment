#pragma once

// yolo_onnx (🛠 직접 작성, 담당: 전승혜) — YOLO(.onnx) 추론: 카메라 사진 한 장 → 물체 네모들
//   파이썬 노드는 ultralytics(YOLO(path).predict)로 돌렸음 → C++에는 그 라이브러리가 없어 ONNX Runtime으로 직접 돌림.
//   전처리·후처리는 ultralytics detect와 같게 맞춤 (같은 사진이면 같은 네모):
//     LetterBox(가운데 · 회색 114 여백) → BGR→RGB · /255 · NCHW float
//     출력 (1, 4+클래스 수, N) = cx·cy·w·h + 클래스별 점수 → 가장 높은 클래스 · 점수 > conf
//     클래스별 NMS(IoU 0.7, 최대 300개, 점수 높은 순) → 원본 화면 좌표로 되돌리고 화면 안으로 자름
//   장치: device가 cuda(또는 숫자 GPU 번호)면 ONNX Runtime CUDA를 먼저 시도 (팀원 yolo_detector_cpp와 같은 방식),
//         안 되면 CPU로 (provider()가 실제로 쓰는 것을 알려 줌 — 진단에 표시)
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception {

// 네모 하나 — 원본 영상 좌표 [px] (x0, y0, x1, y1) · 점수 · 클래스 번호
struct YoloBox {
  float x0, y0, x1, y1;
  float conf;
  int cls;
};

class YoloOnnx {
public:
  // path: ultralytics로 내보낸 .onnx (예: bee_yolo11n_v5.onnx — 입력 1×3×384×640 고정)
  // device: "cuda" · "0"(GPU 번호) → GPU 시도 · 그 밖("cpu")은 CPU
  // cpu_threads: CPU로 돌 때 연산 스레드 수 (코어를 다 쓰면 제어·Nav2가 밀림) · imgsz: 입력 크기가 고정이 아닐 때 정사각형 한 변
  YoloOnnx(const std::string & path, const std::string & device, int cpu_threads, int imgsz);
  bool ok() const;               // 모델을 읽었나
  std::string provider() const;  // 실제로 쓰는 장치 (CUDAExecutionProvider / CPUExecutionProvider)
  std::string error() const;     // 못 읽었으면 이유 · GPU를 못 써서 CPU로 바꿨으면 그 이유
  const std::map<int, std::string> & names() const;  // 클래스 번호 → 이름 (모델 안 metadata "names")
  // bgr: 컬러 영상 · conf: 점수 하한 · agnostic_nms: 클래스가 달라도 겹치면 하나만
  std::vector<YoloBox> predict(const cv::Mat & bgr, float conf, bool agnostic_nms, float iou = 0.7f, int max_det = 300);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace perception
