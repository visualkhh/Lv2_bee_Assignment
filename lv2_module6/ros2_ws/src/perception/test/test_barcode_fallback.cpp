// 바코드 보조 읽기 (barcode_fallback.hpp) — 실기 11-12 손 카메라 화면(라벨 주변만 자름)으로
//   여백 좁은 라벨 · 미션 중 다가간 화면은 기본 zxing이 못 읽고 보조는 읽음 · 바닥(카펫)은 아무것도 안 읽음
#include <gtest/gtest.h>

#include <cstdlib>
#include <string>

#include <ZXing/ReadBarcode.h>
#include <opencv2/imgcodecs.hpp>

#include "perception/barcode_fallback.hpp"

namespace {
std::string data(const std::string & name) {
  const char * d = std::getenv("PERCEPTION_TEST_DATA");
  return std::string(d ? d : "test/data") + "/" + name;
}
std::string zx(const cv::Mat & g, int bz) {
  ZXing::ImageView iv(g.data, g.cols, g.rows, ZXing::ImageFormat::Lum, static_cast<int>(g.step));
  const auto o = ZXing::ReaderOptions().setFormats(ZXing::BarcodeFormat::LinearCodes)
    .setBinarizer(bz == 0 ? ZXing::Binarizer::FixedThreshold : ZXing::Binarizer::LocalAverage);
  for (const auto & r : ZXing::ReadBarcodes(iv, o)) if (!r.text().empty()) return r.text();
  return "";
}
std::string plain(const cv::Mat & bgr) {  // 노드의 기본 읽기와 같음
  ZXing::ImageView iv(bgr.data, bgr.cols, bgr.rows, ZXing::ImageFormat::BGR, static_cast<int>(bgr.step));
  for (const auto & r : ZXing::ReadBarcodes(iv, ZXing::ReaderOptions().setFormats(ZXing::BarcodeFormat::LinearCodes)))
    if (!r.text().empty()) return r.text();
  return "";
}
}  // namespace

TEST(BarcodeFallback, ReadsTightLabels) {
  for (const auto & [file, text] : {std::pair<std::string, std::string>{"barcode_tight_p2-03.jpg", "P2-03"},
                                     {"barcode_mission_p2-01.jpg", "P2-01"}}) {
    const cv::Mat img = cv::imread(data(file));
    ASSERT_FALSE(img.empty()) << file;
    EXPECT_EQ(plain(img), "") << file << " — 기본 읽기로 읽히면 이 시험이 보조를 시험하지 못함";
    cv::Rect r;
    EXPECT_EQ(perception::barcode_fallback(img, zx, 1.0, &r), text) << file;
    EXPECT_GT(r.width, 40) << file;
  }
}

TEST(BarcodeFallback, NothingOnFloor) {
  const cv::Mat img = cv::imread(data("barcode_none_floor.jpg"));
  ASSERT_FALSE(img.empty());
  EXPECT_EQ(perception::barcode_fallback(img, zx, 1.0), "");
}

TEST(BarcodeFallback, CutsSkipFewBars) {
  std::vector<uchar> row(200, 255);
  for (int i = 20; i < 40; ++i) row[i] = 0;  // 막대 하나 — 바코드 아님
  EXPECT_TRUE(perception::barcode_cuts(row).empty());
}
