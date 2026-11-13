// YOLO 메모리·속도 벤치 (colcon 빌드에 안 들어감 — 젯슨에서 g++로 따로) — GPU 메모리 설정(cudnn_conv_algo_search ·
//   gpu_arena_same_as_requested)을 바꿨을 때 RSS · MemAvailable · 추론 시간 · 검출이 같은지 본다.
//   bringup 끄고 MemAvailable > 2GB일 때만 (젯슨 GPU 메모리 = RAM — 옆에 bringup이 있으면 재부팅됨, 11/14)
//
//   P=lv2_module6/ros2_ws/src/perception
//   O="-I$P/include -I/opt/onnxruntime/include -I/usr/include/opencv4 -L/usr/lib -L/opt/onnxruntime/lib -Wl,-rpath,/opt/onnxruntime/lib"
//   g++ -O2 -std=c++17 $O $P/test/yolo_mem_bench.cpp $P/src/yolo_onnx.cpp -o /tmp/yolo_mem_bench -lonnxruntime -lopencv_core -lopencv_imgproc -lopencv_imgcodecs
//   M=lv2_module6/ros2_ws/install/perception/share/perception/model/bee_yolo11n_v6.onnx
//   for c in "exhaustive 0" "heuristic 0" "exhaustive 1" "heuristic 1"; do
//     timeout -s KILL 180 /tmp/yolo_mem_bench $M $c 200 /tmp/yolo_out_${c// /_}.txt $P/test/data/*.jpg $P/test/data/*.png; done
//   md5sum /tmp/yolo_out_*.txt   # 검출이 같으면 해시가 같음
//
//   인자: <model.onnx> <exhaustive|heuristic|default|cpu> <arena_same 0|1> <N> <검출 출력.txt> <사진...>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "perception/yolo_onnx.hpp"

static long kb(const char * f, const char * key) {
  std::ifstream in(f);
  std::string k;
  long v;
  while (in >> k)
    if (k == key && in >> v) return v;
  return -1;
}
static void mem(const char * tag) {
  std::printf("%-5s RSS %5ld MB  HWM %5ld MB  MemAvailable %5ld MB\n", tag, kb("/proc/self/status", "VmRSS:") / 1024,
              kb("/proc/self/status", "VmHWM:") / 1024, kb("/proc/meminfo", "MemAvailable:") / 1024);
}

int main(int argc, char ** argv) {
  if (argc < 7) {
    std::fprintf(stderr, "usage: %s model algo arena_same N out.txt img...\n", argv[0]);
    return 2;
  }
  const std::string algo = argv[2];
  const int n = std::max(1, std::atoi(argv[4]));
  std::vector<cv::Mat> imgs;
  for (int i = 6; i < argc; ++i) {
    cv::Mat m = cv::imread(argv[i], cv::IMREAD_COLOR), f;
    if (m.empty()) continue;
    cv::flip(m, f, 1);
    imgs.push_back(m);
    imgs.push_back(f);
  }
  if (imgs.empty()) return 2;
  mem("start");
  perception::YoloOnnx y(argv[1], algo == "cpu" ? "cpu" : "cuda", 2, 640, algo, std::atoi(argv[3]) != 0);
  std::printf("provider %s %s\n", y.provider().c_str(), y.error().c_str());
  mem("load");
  std::ofstream out(argv[5]);
  for (size_t i = 0; i < imgs.size(); ++i)
    for (const auto & b : y.predict(imgs[i], 0.25f, false))
      out << i << ' ' << b.cls << ' ' << b.conf << ' ' << b.x0 << ' ' << b.y0 << ' ' << b.x1 << ' ' << b.y1 << '\n';
  mem("warm");
  std::vector<double> ms;
  for (int k = 0; k < n; ++k) {
    const auto t0 = std::chrono::steady_clock::now();
    y.predict(imgs[k % imgs.size()], 0.4f, false);
    ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  }
  std::sort(ms.begin(), ms.end());
  mem("end");
  std::printf("latency median %.1f ms  p90 %.1f ms  (N=%d)\n", ms[ms.size() / 2], ms[ms.size() * 9 / 10], n);
}
