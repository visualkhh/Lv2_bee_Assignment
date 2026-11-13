#pragma once
// 바코드 보조 읽기 — 기본 zxing 읽기가 실패했을 때만 (perception_node Barcode::read)
//   실기 11-12: 벽 라벨이 막대 바로 옆에서 잘려 흰 여백(quiet zone)이 막대 1~2개 폭뿐이고 가장자리에 여분 막대가 붙어,
//   zxing·OpenCV가 바코드가 커도(195 px) 0건 → 표준 1D 바코드는 양 끝에 막대 단위 10배 이상 흰 여백이 있어야 시작을 찾음.
//   방법: 흰 라벨(밝고 채도 낮음) 찾기 → 라벨 안 원본 줄을 2 px마다 → (그대로 · 맨 앞 검은 막대 뺌 · 맨 뒤 검은 막대 뺌)에
//   흰 여백을 붙여 1D 그림을 만들고 2·3배 확대해 다시 읽음. 회전 보정은 안 함 (원근으로 비스듬한 막대가 더 기울어짐).
//   오늘 저장 화면 12장: 기본 0 → 보조 4 (손가락에 가린 것 · 미션 중 다가간 것 포함) · 바코드 없는 화면 8장 오검출 0
#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace perception {

// 흰 라벨 후보 (넓은 것부터 최대 4개) — 벽이 밝은 회청색이라 밝기만으로는 안 갈려 채도도 봄
inline std::vector<cv::Rect> barcode_labels(const cv::Mat & bgr) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  std::vector<uchar> v(ch[2].begin<uchar>(), ch[2].end<uchar>());
  std::nth_element(v.begin(), v.begin() + v.size() * 97 / 100, v.end());
  const int v_thr = std::max(150, static_cast<int>(v[v.size() * 97 / 100]) - 40);
  cv::Mat mask = (ch[1] < 60) & (ch[2] > v_thr);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {9, 3}));
  std::vector<std::vector<cv::Point>> cs;
  cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Rect> out;
  for (const auto & c : cs) {
    const auto r = cv::boundingRect(c);
    const double ar = static_cast<double>(r.width) / std::max(r.height, 1);
    if (r.area() >= 600 && r.width >= 40 && ar > 1.3 && ar < 12) out.push_back(r);
  }
  std::sort(out.begin(), out.end(), [](const cv::Rect & a, const cv::Rect & b) { return a.area() > b.area(); });
  if (out.size() > 4) out.resize(4);
  return out;
}

// 한 줄(밝기)에서 막대 구간 후보 [시작, 끝) — 그대로 · 맨 앞 검은 막대 뺌 · 맨 뒤 검은 막대 뺌. 막대가 적으면 없음 (카펫·글자 줄)
inline std::vector<std::pair<int, int>> barcode_cuts(const std::vector<uchar> & row) {
  if (row.size() < 20) return {};
  std::vector<uchar> s(row);
  std::nth_element(s.begin(), s.begin() + s.size() / 10, s.end());
  const int lo = s[s.size() / 10];
  std::nth_element(s.begin(), s.begin() + s.size() * 9 / 10, s.end());
  const int hi = s[s.size() * 9 / 10];
  if (hi - lo < 30) return {};
  const int mid = (lo + hi) / 2;
  std::vector<int> bstart, bend;  // 검은 막대 [시작, 끝)
  int edges = 0;
  for (size_t i = 0; i < row.size(); ++i) {
    const bool d = row[i] < mid, prev = i > 0 && row[i - 1] < mid;
    if (i > 0 && d != prev) ++edges;
    if (d && !prev) bstart.push_back(static_cast<int>(i));
    if (!d && prev) bend.push_back(static_cast<int>(i));
  }
  if (bend.size() < bstart.size()) bend.push_back(static_cast<int>(row.size()));
  if (bstart.size() < 12 || edges < 24) return {};  // Code128은 짧아도 경계 30개쯤
  const int a = bstart.front(), b = bend.back();
  return {{a, b}, {bstart[1], b}, {a, bend[bend.size() - 2]}};
}

// decode(회색 그림, 이진화 종류 0 = 고정 임계 · 1 = 지역 평균) → 내용 ("" = 못 읽음). where에 읽은 라벨 사각형
inline std::string barcode_fallback(const cv::Mat & bgr, const std::function<std::string(const cv::Mat &, int)> & decode,
                                    double budget_s = 0.15, cv::Rect * where = nullptr) {
  const auto t0 = std::chrono::steady_clock::now();
  cv::Mat g;
  cv::cvtColor(bgr, g, cv::COLOR_BGR2GRAY);
  for (const auto & r : barcode_labels(bgr)) {
    for (int y = r.y + 2; y < r.y + r.height - 2; y += 2) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > budget_s) return "";
      const uchar * p = g.ptr<uchar>(y) + r.x;
      const std::vector<uchar> row(p, p + r.width);
      for (const auto & [s, e] : barcode_cuts(row)) {
        const int s0 = std::max(0, s - 1), e0 = std::min(static_cast<int>(row.size()), e + 1);
        const int q = std::max(20, (e0 - s0) / 6);  // 흰 여백
        cv::Mat line(1, e0 - s0 + 2 * q, CV_8UC1, cv::Scalar(255));
        std::copy(row.begin() + s0, row.begin() + e0, line.ptr<uchar>(0) + q);
        cv::Mat img;
        cv::repeat(line, 24, 1, img);
        for (const double k : {2.0, 3.0}) {
          cv::Mat up;
          cv::resize(img, up, cv::Size(), k, k, cv::INTER_CUBIC);
          for (const int bz : {0, 1}) {
            const auto t = decode(up, bz);
            if (!t.empty()) {
              if (where) *where = r;
              return t;
            }
          }
        }
      }
    }
  }
  return "";
}

}  // namespace perception
