// perception_core 테스트 — 파이썬 test/test_node.py의 같은 이름 시험을 C++판 함수로 (숫자·기대값 그대로)
#include <gtest/gtest.h>
#include <map>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>  // OpenCV 4.7+ (젯슨 4.8)
#else
#include <opencv2/aruco.hpp>  // OpenCV 4.6 contrib (PC 도커)
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/perception_core.hpp"

using namespace perception;  // NOLINT
namespace {
constexpr double PI = 3.14159265358979323846;
double rad(double d) { return d * PI / 180.0; }

cv::Mat full(int h, int w, cv::Vec3b c) { return cv::Mat(h, w, CV_8UC3, cv::Scalar(c[0], c[1], c[2])); }
cv::Mat gain(const cv::Mat & m, double g) {
  cv::Mat f, o;
  m.convertTo(f, CV_64F, g);
  f.convertTo(o, CV_8U);  // 0~255로 자름 · 반올림은 OpenCV (파이썬은 astype 내림 — 이 시험 색에서는 같은 답)
  return o;
}
cv::Mat roi_hsv(int h, int s = 200, int v = 150) {
  cv::Mat hsv(20, 20, CV_8UC3, cv::Scalar(h, s, v)), bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}
cv::Mat marker(int id, int size) {
  cv::Mat m;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, size, m);
#else
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), id, size, m);
#endif
  cv::Mat bgr;
  cv::cvtColor(m, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}
std::string data_dir() {
  const char * d = std::getenv("PERCEPTION_TEST_DATA");
  return d ? d : "test/data";
}
// 파이썬 _cam_rot: yaw만큼 돌고 pitch만큼 아래를 보는 카메라의 카메라 → odom 회전
cv::Matx33d cam_rot(double yaw_deg, double pitch_deg) {
  const double a = rad(yaw_deg), p = rad(pitch_deg);
  const cv::Vec3d fwd(std::cos(p) * std::cos(a), std::cos(p) * std::sin(a), -std::sin(p));
  const cv::Vec3d right(std::sin(a), -std::cos(a), 0.0);
  const cv::Vec3d down = fwd.cross(right);
  return {right[0], down[0], fwd[0], right[1], down[1], fwd[1], right[2], down[2], fwd[2]};
}
struct BoxSpec { double x, y, side, h, yaw; };
struct Cyl { double x, y, r, h; };
// 파이썬 _box_depth: 위에서 비스듬히(50°) 내려다보는 카메라로 찍은 가짜 depth (바닥 z=0)
struct Scene { cv::Mat1f d; cv::Matx33d rot; cv::Vec3d trans; double fx, fy, cx, cy; };
Scene box_depth(const std::vector<BoxSpec> & boxes, const std::vector<Cyl> & extra = {}, double cam_h = 0.8) {
  Scene s{cv::Mat1f(240, 320), cam_rot(0.0, 50.0), {0, 0, cam_h}, 300, 300, 160, 120};
  for (int v = 0; v < 240; ++v)
    for (int u = 0; u < 320; ++u) {
      const cv::Vec3d ray = s.rot * cv::Vec3d((u - 160) / 300.0, (v - 120) / 300.0, 1.0);
      const double rz = std::min(ray[2], -1e-9);
      double depth = ray[2] < 0 ? -cam_h / rz : 0.0;
      for (const auto & b : boxes) {
        const double t = (b.h - cam_h) / rz;
        const double px = t * ray[0] - b.x, py = t * ray[1] - b.y;
        const double lx = px * std::cos(-b.yaw) - py * std::sin(-b.yaw), ly = px * std::sin(-b.yaw) + py * std::cos(-b.yaw);
        if (ray[2] < 0 && std::abs(lx) < b.side / 2 && std::abs(ly) < b.side / 2 && t < depth) depth = t;
      }
      for (const auto & o : extra)
        for (int k = 0; k < 40; ++k) {
          const double z = o.h - k * o.h / 39.0;
          const double t = (z - cam_h) / rz;
          const double px = t * ray[0] - o.x, py = t * ray[1] - o.y;
          if (ray[2] < 0 && px * px + py * py < o.r * o.r && t < depth) depth = t;
        }
      s.d(v, u) = static_cast<float>(depth);
    }
  return s;
}
std::vector<BoxTop> tops_of(const Scene & s, const std::vector<cv::Vec3d> & mask = {}) {
  return find_box_tops(s.d, s.rot, s.trans, s.fx, s.fy, s.cx, s.cy, 0.0, {0.05, 0.09}, 0.17, 1.0, 0.02, 2, 3.0, std::nullopt,
                       std::nullopt, mask);
}
}  // namespace

TEST(Core, ClassifyColor) {  // 퍽 색 · 신호등 상태 — 조명 0.55~1.45배
  const std::vector<std::pair<std::string, cv::Vec3b>> puck = {
    {"red", {40, 40, 220}}, {"green", {60, 180, 60}}, {"blue", {200, 90, 30}}, {"yellow", {40, 220, 230}},
    {"orange", {30, 140, 250}}, {"purple", {170, 60, 140}}, {"white", {235, 235, 235}}, {"black", {40, 40, 40}}};
  const std::vector<std::pair<std::string, cv::Vec3b>> lamp = {
    {"red", {40, 40, 220}}, {"yellow", {40, 220, 230}}, {"green", {60, 180, 60}}, {"off", {12, 54, 57}}};
  for (double g : {0.55, 1.0, 1.45}) {
    for (const auto & [name, bgr] : puck) {
      cv::Mat roi = full(20, 20, bgr);
      roi.rowRange(0, 3).setTo(cv::Scalar(60, 140, 190));  // 위쪽 바닥 — 가운데만 보므로 무시
      EXPECT_EQ(classify_color("puck", gain(roi, g)), name) << name << " " << g;
    }
    for (const auto & [name, bgr] : lamp) {
      cv::Mat roi = full(30, 14, {40, 40, 40});
      roi(cv::Rect(4, 4, 6, 6)).setTo(cv::Scalar(bgr[0], bgr[1], bgr[2]));
      EXPECT_EQ(classify_color("traffic_light", gain(roi, g)), name) << name << " " << g;
    }
  }
  EXPECT_EQ(classify_color("person", cv::Mat::zeros(5, 5, CV_8UC3)), "");
  cv::Mat tl = full(40, 14, {40, 40, 40});  // 세 색 동시 → all
  tl(cv::Rect(4, 2, 6, 8)).setTo(cv::Scalar(40, 40, 220));
  tl(cv::Rect(4, 16, 6, 8)).setTo(cv::Scalar(40, 220, 230));
  tl(cv::Rect(4, 30, 6, 8)).setTo(cv::Scalar(60, 180, 60));
  EXPECT_EQ(classify_color("traffic_light", tl), "all");
}

TEST(Core, PuckColorRealGreen) {
  const std::vector<std::string> rg = {"red", "green"};
  EXPECT_EQ(puck_color(full(20, 20, {71, 128, 19}), rg, 10).first, "green");
  for (int h : {64, 71, 77, 85, 90}) EXPECT_EQ(puck_color(roi_hsv(h), rg, 10).first, "green") << h;
  EXPECT_EQ(puck_color(roi_hsv(2), rg, 10).first, "red");
  EXPECT_EQ(puck_color(roi_hsv(13), rg, 10).first, "other");
  EXPECT_EQ(puck_color(roi_hsv(110), rg, 10).first, "other");
}

TEST(Core, SettlePuckColor) {
  const std::vector<std::string> pc = {"red", "green"};
  auto settle = [&](const cv::Mat & roi, const std::string & model) {
    return settle_puck_color(roi, puck_color(roi, pc, 10).first, model, pc);
  };
  EXPECT_EQ(settle(roi_hsv(77), "green").first.value_or("-"), "green");
  EXPECT_EQ(settle(roi_hsv(77), "red").first.value_or("-"), "green");
  EXPECT_FALSE(settle(roi_hsv(13), "red").first);
  EXPECT_FALSE(settle(roi_hsv(110), "green").first);
  EXPECT_EQ(settle(roi_hsv(0, 40, 40), "red").first.value_or("-"), "red");
  EXPECT_FALSE(settle(roi_hsv(0, 40, 40), "").first);
  EXPECT_EQ(settle_puck_color(roi_hsv(77), "green", "red", pc, false).first.value_or("-"), "green");
}

TEST(Core, ClassNames) {
  EXPECT_EQ(split_class("puck_red"), std::make_pair(std::string("puck"), std::string("red")));
  EXPECT_EQ(split_class("puck_lying").second, "");
  EXPECT_EQ(split_class("traffic_light_off"), std::make_pair(std::string("traffic_light"), std::string("off")));
  EXPECT_EQ(split_class("traffic_light").second, "");
  EXPECT_EQ(split_class("person").first, "person");
  EXPECT_EQ(puck_shape("puck_red_box"), "box");
  EXPECT_EQ(puck_shape("puck_green_cyl"), "cyl");
  EXPECT_EQ(puck_shape("puck"), "");
  EXPECT_EQ(model_posture("puck_red_cyl_lying"), "lying");
  EXPECT_EQ(model_posture("puck_red_box"), "standing");
  EXPECT_EQ(tag_shape("side", "standing"), "box");
  EXPECT_EQ(tag_shape("top", "lying"), "box");
  EXPECT_EQ(tag_shape("top", "standing"), "");
}

TEST(Core, ResolveModelPath) {
  namespace fs = std::filesystem;
  const auto d = fs::temp_directory_path() / "perception_core_test_models";
  fs::create_directories(d);
  std::ofstream(d / "model.pt").close();
  std::ofstream(d / "bee_yolo11n_v1.pt").close();
  EXPECT_EQ(resolve_model_path("", d.string()), (d / "model.pt").string());
  EXPECT_EQ(resolve_model_path("bee_yolo11n_v1.pt", d.string()), (d / "bee_yolo11n_v1.pt").string());
  EXPECT_EQ(resolve_model_path("/x/y/my.pt", d.string()), "/x/y/my.pt");
  EXPECT_EQ(resolve_model_path("none.pt", d.string()), "none.pt");
  fs::remove(d / "model.pt");
  EXPECT_EQ(resolve_model_path("", d.string()), "");
  fs::remove_all(d);
}

TEST(Core, TagsAndMarkers) {
  cv::Mat img = full(240, 320, {255, 255, 255});
  marker(2, 60).copyTo(img(cv::Rect(40, 40, 60, 60)));
  marker(13, 80).copyTo(img(cv::Rect(180, 40, 80, 80)));
  TagDetector det;
  auto tags = detect_tags(&det, img);
  std::vector<int> ids;
  for (const auto & t : tags) ids.push_back(t.id);
  std::sort(ids.begin(), ids.end());
  EXPECT_EQ(ids, (std::vector<int>{2, 13}));
  std::set<int> puck_ids;
  for (int i = 0; i < 12; ++i) puck_ids.insert(i);
  const auto a = tag_in_box(tags, {30, 30, 110, 110}, puck_ids);
  ASSERT_TRUE(a);
  EXPECT_EQ(a->first, 2);
  EXPECT_FALSE(tag_in_box(tags, {170, 30, 270, 130}, puck_ids));  // 장소 태그는 퍽 번호 아님
}

TEST(Core, TagFilesAndMinPerimeter) {  // 반사 태그(감마) · 크게 찍힌 태그(반 크기) · 노드 기본 최소 둘레도 같은 태그
  TagDetector det;
  const cv::Mat glare = cv::imread(data_dir() + "/glare_tag0.png");
  const cv::Mat big = cv::imread(data_dir() + "/big_tag7.png");
  ASSERT_FALSE(glare.empty());
  ASSERT_FALSE(big.empty());
  for (std::optional<double> px : {std::optional<double>(), std::optional<double>(16.0)}) {
    auto g = detect_tags(&det, glare, 2.5, false, px, px ? std::optional<double>(24.0) : std::nullopt);
    ASSERT_EQ(g.size(), 1u);
    EXPECT_EQ(g[0].id, 0);
    auto b = detect_tags(&det, big, 0.0, true, px, px ? std::optional<double>(24.0) : std::nullopt);
    ASSERT_EQ(b.size(), 1u);
    EXPECT_EQ(b[0].id, 7);
    EXPECT_GT(b[0].box.x1 - b[0].box.x0, 60);
    EXPECT_LT(b[0].box.x1 - b[0].box.x0, 120);
  }
  std::vector<Tag> tags = {{2, {1290.0, 200.0}, {}, {}}};  // 겹친 퍽: 가운데에 더 가까운 앞 퍽 것
  const Box front{1243, 160, 1316, 285}, back{1283, 97, 1350, 198};
  std::set<int> ids;
  for (int i = 0; i < 12; ++i) ids.insert(i);
  EXPECT_EQ(tag_in_box(tags, front, ids, {front, back})->first, 2);
  EXPECT_FALSE(tag_in_box(tags, back, ids, {front, back}));
  EXPECT_EQ(tag_in_box(tags, back, ids)->first, 2);
}

TEST(Core, PuckYaw) {
  EXPECT_NEAR(wrap_yaw(rad(100), PI / 2), rad(10), 1e-9);
  EXPECT_NEAR(wrap_yaw(rad(-170), PI), rad(10), 1e-9);
  const cv::Matx33d rot(0, 0, 1, -1, 0, 0, 0, -1, 0);  // 카메라가 +x를 수평으로 봄
  cv::Mat img = full(240, 320, {255, 255, 255});
  marker(5, 60).copyTo(img(cv::Rect(130, 90, 60, 60)));
  TagDetector det;
  auto tags = detect_tags(&det, img);
  ASSERT_EQ(tags.size(), 1u);
  EXPECT_EQ(tags[0].id, 5);
  auto [yaw, face] = tag_yaw(tags[0].corners, 0.016, 300, 300, 160, 120, rot);
  EXPECT_EQ(face, "side");
  ASSERT_TRUE(yaw);
  EXPECT_LT(std::abs(wrap_yaw(*yaw, PI / 2)), rad(5));
  std::mt19937 g(0);
  std::uniform_real_distribution<double> ut(-0.03, 0.03), us(-0.015, 0.015);
  std::normal_distribution<double> nz(0, 0.003);
  std::vector<cv::Vec2d> pts, sq;
  for (int i = 0; i < 300; ++i) {
    const double t = ut(g);
    pts.emplace_back(t * std::cos(rad(30)) + nz(g), t * std::sin(rad(30)) + nz(g));
    sq.emplace_back(us(g), us(g));
  }
  const auto a = axis_yaw(pts, 30, 1.8);
  ASSERT_TRUE(a);
  EXPECT_LT(std::abs(wrap_yaw(*a - rad(30), PI)), rad(5));
  EXPECT_FALSE(axis_yaw(sq, 30, 1.8));
  EXPECT_FALSE(axis_yaw(std::vector<cv::Vec2d>(pts.begin(), pts.begin() + 10), 30, 1.8));
}

TEST(Core, LocationYaw) {
  const cv::Matx33d rot(0, 0, 1, -1, 0, 0, 0, -1, 0);
  cv::Mat img = full(240, 320, {255, 255, 255});
  marker(13, 100).copyTo(img(cv::Rect(110, 60, 100, 100)));
  TagDetector det;
  auto tags = detect_tags(&det, img);
  ASSERT_EQ(tags.size(), 1u);
  auto [yaw, face] = tag_yaw(tags[0].corners, 0.048, 300, 300, 160, 120, rot);
  EXPECT_EQ(face, "side");
  ASSERT_TRUE(yaw);
  EXPECT_LT(std::abs(std::abs(*yaw) - PI), rad(5));
}

TEST(Core, BoxTops) {
  auto s = box_depth({{1.0, 0.1, 0.205, 0.07, rad(30)}});
  auto tops = tops_of(s);
  ASSERT_EQ(tops.size(), 1u);
  const auto & t = tops[0];
  EXPECT_LT(cv::norm(t.c - cv::Vec2d(1.0, 0.1)), 0.03);
  EXPECT_LT(std::abs(t.a - 0.205), 0.04);
  EXPECT_LT(std::abs(t.b - 0.205), 0.04);
  ASSERT_TRUE(t.yaw);
  EXPECT_LT(std::abs(wrap_yaw(*t.yaw - rad(30), PI / 2)), rad(6));
  EXPECT_LT(std::abs(t.top_z - 0.07), 0.01);
  EXPECT_TRUE(tag_on_box({1.0 - 0.11, 0.1}, t, 0.08));
  EXPECT_FALSE(tag_on_box({1.5, 0.1}, t, 0.08));
  std::vector<BoxSpec> big;  // 4개 붙인 판 → 하나
  for (double dx : {-0.1025, 0.1025})
    for (double dy : {-0.1025, 0.1025}) big.push_back({1.1 + dx, dy, 0.205, 0.07, 0.0});
  tops = tops_of(box_depth(big));
  ASSERT_EQ(tops.size(), 1u);
  EXPECT_LT(std::abs(tops[0].a - 0.41), 0.05);
  EXPECT_LT(std::abs(tops[0].b - 0.41), 0.05);
  EXPECT_TRUE(tops_of(box_depth({{1.0, 0.0, 0.10, 0.07, 0.0}}, {{1.2, -0.3, 0.10, 0.30}})).empty());  // 원통 · 작은 물체
}

TEST(Core, PostureFromHeight) {
  struct C { double top, base; const char * want; };
  for (const auto & c : {C{0.06, 0.0, "standing"}, C{0.03, 0.0, "lying"}, C{0.13, 0.07, "standing"}, C{0.10, 0.07, "lying"}}) {
    std::vector<BoxSpec> boxes;
    if (c.base > 0) boxes.push_back({1.0, 0.0, 0.205, c.base, 0.0});
    boxes.push_back({1.0, 0.0, 0.04, c.top, 0.0});
    const auto s = box_depth(boxes);
    EXPECT_FALSE(view_from_side(s.rot));
    int x0 = 1e9, y0 = 1e9, x1 = -1, y1 = -1;
    for (int v = 0; v < s.d.rows; ++v)
      for (int u = 0; u < s.d.cols; ++u) {
        const float z = s.d(v, u);
        const cv::Vec3d p = s.rot * cv::Vec3d((u - s.cx) * z / s.fx, (v - s.cy) * z / s.fy, z) + s.trans;
        if (p[2] > c.top - 0.005) { x0 = std::min(x0, u); y0 = std::min(y0, v); x1 = std::max(x1, u); y1 = std::max(y1, v); }
      }
    const Box box{x0, y0, x1, y1};
    const double zm = box_depth_m(s.d, box, 0.5);
    EXPECT_EQ(posture_from_height(s.d, box, zm, s.rot, s.trans, s.fx, s.fy, s.cx, s.cy, 0.0).first, c.want) << c.top << " " << c.base;
  }
}

TEST(Core, BoxCutAndArmMask) {
  auto s = box_depth({{1.0, 0.0, 0.205, 0.07, rad(20)}});
  auto tops = tops_of(s);
  ASSERT_EQ(tops.size(), 1u);
  EXPECT_FALSE(box_cut(tops[0], s.rot, s.trans, s.fx, s.fy, s.cx, s.cy, 320, 240, 8));
  s = box_depth({{1.0, 0.55, 0.205, 0.07, rad(20)}});
  tops = tops_of(s);
  ASSERT_EQ(tops.size(), 1u);
  EXPECT_TRUE(box_cut(tops[0], s.rot, s.trans, s.fx, s.fy, s.cx, s.cy, 320, 240, 8));
  const BoxSpec grip{1.0, 0.1025 + 0.03, 0.06, 0.07, 0.0};
  s = box_depth({{1.0, 0.0, 0.205, 0.07, 0.0}, grip});
  tops = tops_of(s);
  ASSERT_EQ(tops.size(), 1u);
  EXPECT_GT(tops[0].a, 0.28);  // 그리퍼까지 합쳐 커짐
  const cv::Vec3d q = s.rot.t() * (cv::Vec3d(grip.x, grip.y, 0.07) - s.trans);
  tops = tops_of(s, {cv::Vec3d(s.fx * q[0] / q[2] + s.cx, s.fy * q[1] / q[2] + s.cy, s.fx * 0.04 / q[2])});
  ASSERT_EQ(tops.size(), 1u);
  EXPECT_LT(tops[0].a, 0.25);  // 팔을 빼면 원래 크기
}

TEST(Core, MergeDuplicatePucks) {
  auto d = [](Box b, double conf, std::optional<int> tag = std::nullopt, const std::string & color = "red") {
    Det x;
    x.class_name = "puck"; x.box = b; x.conf = conf; x.color = color; x.tag = tag;
    return x;
  };
  auto out = merge_overlap_dets({d({100, 100, 140, 180}, 0.6, 4), d({102, 98, 142, 175}, 0.9), d({300, 100, 340, 180}, 0.8)}, 0.5);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].conf, 0.9);
  EXPECT_EQ(out[0].tag.value_or(-1), 4);
  EXPECT_EQ(merge_overlap_dets({d({100, 100, 140, 180}, 0.6, 4), d({102, 98, 142, 175}, 0.9, 5)}, 0.5).size(), 2u);
  EXPECT_EQ(merge_overlap_dets({d({100, 100, 140, 180}, 0.6), d({102, 98, 142, 175}, 0.9, std::nullopt, "green")}, 0.5).size(), 2u);
  auto tr = [](cv::Vec3d p, double t, std::optional<int> tag = std::nullopt) {
    Track x;
    x.class_name = "puck"; x.p = p; x.P = cv::Matx33d::eye(); x.t = t; x.color = "red"; x.tag = tag;
    return x;
  };
  std::map<int, Track> tracks = {{1, tr({0.257, 0.026, 0.13}, 1.0)}, {2, tr({0.258, 0.034, 0.13}, 2.0, 4)}, {3, tr({0.172, -0.03, 0.13}, 2.0)}};
  EXPECT_EQ(merge_tracks(tracks, 0.03), (std::map<int, int>{{2, 1}}));
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks.at(1).tag.value_or(-1), 4);
  EXPECT_EQ(tracks.at(1).p[1], 0.034);
  tracks = {{1, tr({0.25, 0.03, 0.13}, 1.0, 4)}, {2, tr({0.26, 0.03, 0.13}, 2.0, 10)}};
  EXPECT_TRUE(merge_tracks(tracks, 0.03).empty());
  EXPECT_EQ(tracks.size(), 2u);
}

TEST(Core, PuckCenterFromSurface) {
  const cv::Vec3d cam(0.09, 0.03, 0.31), surf(0.25, 0.03, 0.12);
  const auto c = puck_center(surf, cam, 0.015, 45.0);
  EXPECT_NEAR(c[1], surf[1], 1e-9);
  EXPECT_NEAR(c[2], surf[2], 1e-9);
  EXPECT_GT(c[0] - surf[0], 0.010);
  EXPECT_LT(c[0] - surf[0], 0.015);
  EXPECT_NEAR(puck_center({0.40, 0.03, 0.12}, cam, 0.015, 45.0)[0], 0.415, 1e-9);
  EXPECT_LT(cv::norm(puck_center({0.09, 0.03, 0.12}, cam, 0.015, 45.0) - cv::Vec3d(0.09, 0.03, 0.12)), 1e-9);
  EXPECT_LT(cv::norm(puck_center(surf, cam, 0.0, 45.0) - surf), 1e-12);
}

TEST(Core, TagCenterTopAndSide) {
  const double fx = 600, fy = 600, cx = 320, cy = 240;
  auto view = [](cv::Vec3d cam, cv::Vec3d fwd, cv::Vec3d up) {
    const cv::Vec3d z = fwd / cv::norm(fwd);
    cv::Vec3d x = z.cross(up);
    x /= cv::norm(x);
    const cv::Vec3d y = z.cross(x);
    return std::make_pair(cv::Matx33d(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]), cam);
  };
  auto corners_of = [&](const cv::Matx33d & rot, const cv::Vec3d & trans, cv::Vec3d centre, cv::Vec3d ax, cv::Vec3d ay) {
    const double hs = 0.008;
    const cv::Vec3d pts[4] = {centre - hs * ax + hs * ay, centre + hs * ax + hs * ay, centre + hs * ax - hs * ay, centre - hs * ax - hs * ay};
    std::array<cv::Point2d, 4> out;
    for (int i = 0; i < 4; ++i) {
      const cv::Vec3d c = rot.t() * (pts[i] - trans);
      out[i] = {fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy};
    }
    return std::make_pair(out, (rot.t() * (centre - trans))[2]);
  };
  auto [rot, trans] = view({0.10, 0.02, 0.32}, {0.15, 0.0, -0.19}, {0, 0, 1});
  const cv::Vec3d top(0.25, 0.02, 0.13);
  auto [cs, z] = corners_of(rot, trans, top, {0, -1, 0}, {1, 0, 0});
  auto c = tag_center(cs, cv::Mat1f(480, 640, static_cast<float>(z)), 0.016, 0.015, fx, fy, cx, cy, rot, trans);
  ASSERT_TRUE(c);
  EXPECT_LT(cv::norm(*c - top), 0.002);
  std::tie(rot, trans) = view({0.05, 0.02, 0.16}, {0.18, 0.0, -0.06}, {0, 0, 1});
  std::tie(cs, z) = corners_of(rot, trans, {0.235, 0.02, 0.10}, {0, -1, 0}, {0, 0, 1});
  c = tag_center(cs, cv::Mat1f(480, 640, static_cast<float>(z)), 0.016, 0.015, fx, fy, cx, cy, rot, trans);
  ASSERT_TRUE(c);
  EXPECT_LT(std::hypot((*c)[0] - 0.25, (*c)[1] - 0.02), 0.002);
  c = tag_center(cs, cv::Mat1f::zeros(480, 640), 0.016, 0.015, fx, fy, cx, cy, rot, trans);  // depth 없음 → solvePnP
  ASSERT_TRUE(c);
  EXPECT_LT(std::hypot((*c)[0] - 0.25, (*c)[1] - 0.02), 0.002);
  std::tie(rot, trans) = view({0.25, 0.02, 0.21}, {0, 0, -1}, {1, 0, 0});
  cs = corners_of(rot, trans, top, {0, -1, 0}, {1, 0, 0}).first;
  c = tag_center(cs, cv::Mat1f::zeros(480, 640), 0.016, 0.015, fx, fy, cx, cy, rot, trans);
  ASSERT_TRUE(c);
  EXPECT_LT(cv::norm(*c - top), 0.001);
}

TEST(Core, ArmNearDropsFingerDepth) {
  cv::Mat1f d(90, 90, 0.25f);
  d(cv::Rect(30, 30, 15, 30)).setTo(0.11f);
  cv::Mat1b arm = cv::Mat1b::zeros(90, 90);
  arm.colRange(0, 50).setTo(1);
  EXPECT_NEAR(box_depth_m(d, {0, 0, 90, 90}, 0.3, arm), 0.25, 1e-6);
  EXPECT_EQ(cv::countNonZero(arm_near(d(cv::Rect(30, 30, 30, 30)), cv::Mat1b())), 0);
  EXPECT_EQ(cv::countNonZero(arm_near(cv::Mat1f(30, 30, 0.11f), cv::Mat1b(30, 30, uchar(1)))), 0);
}

TEST(Core, JudgePuck) {  // test_puck_judge_pipeline의 크기·색 판정 부분 (1 m 앞 fx 606: 18 px ≈ 3 cm)
  JudgeParams jp;
  auto cm = [](int px) { return px * 1.0 / 606.0 * 100; };
  EXPECT_EQ(std::get<2>(judge_puck(jp, cm(18), cm(36), "red")), "standing");
  EXPECT_EQ(std::get<2>(judge_puck(jp, cm(36), cm(18), "red")), "lying");
  EXPECT_FALSE(std::get<0>(judge_puck(jp, cm(18), cm(36), "other")));
  EXPECT_FALSE(std::get<0>(judge_puck(jp, cm(6), cm(36), "red")));     // 빨대
  EXPECT_FALSE(std::get<0>(judge_puck(jp, cm(100), cm(200), "red")));  // 큰 판
  EXPECT_FALSE(std::get<0>(judge_puck(jp, -1, -1, "red")));            // 깊이 없음
  EXPECT_TRUE(std::get<0>(judge_puck(jp, -1, -1, "red", false, "", true)));  // 너무 가까움 → 퍽은 맞음
}

TEST(Core, NumpyStats) {
  EXPECT_DOUBLE_EQ(np_median({3, 1, 2}), 2.0);
  EXPECT_DOUBLE_EQ(np_median({4, 1, 3, 2}), 2.5);
  EXPECT_DOUBLE_EQ(np_percentile({1, 2, 3, 4, 5}, 90), 4.6);
}

// 11/12 CPU 절약: AprilTag를 퍽·상자 네모 안에서만 찾아도 같은 태그 · 같은 꼭짓점 (화면 좌표로 되돌림) · 네모 합치기
TEST(Core, TagsInRois) {
  cv::Mat img = full(360, 640, {255, 255, 255});
  marker(2, 40).copyTo(img(cv::Rect(60, 50, 40, 40)));
  marker(13, 70).copyTo(img(cv::Rect(400, 200, 70, 70)));
  marker(5, 40).copyTo(img(cv::Rect(560, 20, 40, 40)));  // 네모 밖 — 찾지 않음
  TagDetector det;
  const auto all = detect_tags(&det, img);
  const auto rois = merge_rois({{60, 50, 100, 90}, {400, 200, 470, 270}, {405, 205, 460, 260}}, 0.3, 12, 640, 360);
  ASSERT_EQ(rois.size(), 2u);  // 겹친 두 네모는 하나로
  const auto in = detect_tags_in(&det, img, rois);
  std::map<int, Tag> a, b;
  for (const auto & t : all) a[t.id] = t;
  for (const auto & t : in) b[t.id] = t;
  ASSERT_EQ(b.size(), 2u);
  EXPECT_FALSE(b.count(5));
  for (int id : {2, 13}) {
    ASSERT_TRUE(a.count(id) && b.count(id)) << id;
    for (int j = 0; j < 4; ++j) EXPECT_LT(cv::norm(a[id].corners[j] - b[id].corners[j]), 0.5) << id;
    EXPECT_EQ(a[id].box, b[id].box);
  }
  // 네모가 화면 대부분이면 그냥 전체 (같은 결과)
  EXPECT_EQ(detect_tags_in(&det, img, {{0, 0, 640, 360}}).size(), all.size());
}

// 11/12 도킹: 로봇 쪽 면까지 거리 · 모서리 · 면 (로봇 앞 0.30 m에 20.5 cm 상자, 10° 돌아감)
TEST(Core, BoxFaceDistCornersFaces) {
  BoxTop bt;
  const double yaw = 10.0 * PI / 180.0, half = 0.1025;
  bt.c = {0.069 + 0.30 + half / std::cos(yaw), 0.0};  // 로봇 앞면(0.069)에서 0.30 앞에 가까운 면 (가운데 줄 기준)
  bt.ax = bt.ay = 2 * half;
  bt.yaw = yaw;
  for (double u = -half; u <= half; u += 0.005)  // 윗면 점 (5 mm 간격)
    for (double v = -half; v <= half; v += 0.005)
      bt.pts.emplace_back(bt.c[0] + u * std::cos(yaw) - v * std::sin(yaw), bt.c[1] + u * std::sin(yaw) + v * std::cos(yaw));
  const auto d = box_face_dist(bt, {0.0, 0.0}, 0.0, 0.069, 0.08, 0.0);
  ASSERT_TRUE(d);
  // 띠(|y| < 8 cm) 안 가장 가까운 점 = 돌아간 면이 y = ±8 cm에서 앞으로 나온 곳 → 0.30 − 0.08·tan10° 근처
  EXPECT_NEAR(*d, 0.30 - 0.08 * std::tan(yaw), 0.006);
  const auto cs = box_corners(bt.c, yaw, bt.ax, bt.ay);
  for (const auto & q : cs) EXPECT_NEAR(cv::norm(q - bt.c), half * std::sqrt(2.0), 1e-9);
  const auto fs = box_faces(bt.c, yaw, bt.ax, bt.ay);
  EXPECT_NEAR(fs[2][2], wrap_yaw(yaw + PI, 2 * PI), 1e-9);  // 로봇 쪽 면 = 바깥 방향이 로봇을 향함 (yaw + 180°)
  EXPECT_NEAR(fs[2][0], bt.c[0] - half * std::cos(yaw), 1e-9);
  // 잘려서 점이 적으면 없음
  BoxTop few;
  few.pts = {{0.4, 0.0}, {0.41, 0.0}};
  EXPECT_FALSE(box_face_dist(few, {0.0, 0.0}, 0.0, 0.069));
}
