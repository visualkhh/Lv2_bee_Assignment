// perception_core 구현 — 설명·선언은 include/perception/perception_core.hpp
//   파이썬 scripts/perception_node.py의 같은 이름 함수와 규칙·숫자가 같다 (numpy 동작: 중앙값·백분위·argmin 동점은 앞쪽)
#include "perception/perception_core.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <numeric>

#include <opencv2/core/version.hpp>
// AprilTag(aruco): OpenCV 4.7부터는 본체(objdetect) 안 ArucoDetector · 4.6 이하는 contrib aruco 모듈
//   (젯슨 JetPack OpenCV 4.8 = 본체 · PC 도커 Ubuntu 24.04 apt OpenCV 4.6 = contrib)
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define PERCEPTION_ARUCO_DETECTOR 1
#else
#include <opencv2/aruco.hpp>
#endif
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace perception {

namespace {

constexpr double PI = 3.14159265358979323846;
double deg2rad(double d) { return d * PI / 180.0; }

// 색 이름별 기준 H (OpenCV H 0~180) — 빨강 0·180 · 주황 17 · 노랑 30 · 초록 70·84 · 파랑 110 · 보라 142 (파이썬 HUES와 같은 순서)
const std::vector<std::pair<std::string, std::vector<int>>> & hues() {
  static const std::vector<std::pair<std::string, std::vector<int>>> h = {
    {"red", {0, 180}}, {"orange", {17}}, {"yellow", {30}}, {"green", {70, 84}}, {"blue", {110}}, {"purple", {142}}};
  return h;
}
const std::vector<std::string> LAMP_COLORS = {"red", "yellow", "green"};
constexpr int ACHROMATIC_S = 60;              // 채도가 이보다 낮은 화소가 대부분이면 무채색 퍽
constexpr double WHITE_V = 110, BLACK_V = 80; // 무채색일 때 밝기 중앙값 경계

const std::vector<int> & hue_refs(const std::string & name) {
  for (const auto & [n, v] : hues())
    if (n == name) return v;
  static const std::vector<int> none;
  return none;
}

std::vector<std::string> hue_names() {
  std::vector<std::string> out;
  for (const auto & [n, v] : hues()) out.push_back(n);
  return out;
}

// H 값들 → {색: 표 수} (처음 표를 받은 순서 유지) — 최대 약 400개를 골라 화소마다 가장 가까운 색에 한 표 (_hue_votes)
std::vector<std::pair<std::string, int>> hue_votes(const std::vector<int> & sel, const std::vector<std::string> & names) {
  const size_t step = std::max<size_t>(1, sel.size() / 400);
  std::vector<std::pair<std::string, int>> votes;
  for (size_t i = 0; i < sel.size(); i += step) {
    size_t best = 0;
    int bd = std::numeric_limits<int>::max();
    for (size_t k = 0; k < names.size(); ++k) {
      const int d = hue_dist(sel[i], names[k]);
      if (d < bd) { bd = d; best = k; }  // 동점이면 앞쪽 색 (np.argmin)
    }
    auto it = std::find_if(votes.begin(), votes.end(), [&](const auto & v) { return v.first == names[best]; });
    if (it == votes.end()) votes.emplace_back(names[best], 1);
    else ++it->second;
  }
  return votes;
}

// dict의 max(votes, key=votes.get) — 같으면 먼저 들어간 것
std::string vote_max(const std::vector<std::pair<std::string, int>> & votes) {
  std::string best;
  int bn = -1;
  for (const auto & [n, c] : votes)
    if (c > bn) { bn = c; best = n; }
  return best;
}

// 가장자리 20%씩 뺀 안쪽 (파이썬: 남는 게 없으면 원래 영역)
cv::Mat inner(const cv::Mat & roi, bool keep_if_empty) {
  const int h0 = roi.rows, w0 = roi.cols;
  const int mh = static_cast<int>(h0 * 0.2), mw = static_cast<int>(w0 * 0.2);
  if (h0 - 2 * mh > 0 && w0 - 2 * mw > 0) return roi(cv::Range(mh, h0 - mh), cv::Range(mw, w0 - mw));
  return keep_if_empty ? roi : cv::Mat();
}

cv::Matx33d rodrigues(const cv::Vec3d & rvec) {
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  return r;
}

cv::Matx33d cam_k(double fx, double fy, double cx, double cy) { return {fx, 0, cx, 0, fy, cy, 0, 0, 1}; }

std::vector<cv::Point3d> tag_obj(double size_m) {
  const double h = size_m / 2.0;  // aruco 꼭짓점 순서와 같은 태그 좌표
  return {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
}

}  // namespace

// ---------- 상수
cv::Vec3d size_fallback(const std::string & cls) {
  static const std::map<std::string, cv::Vec3d> m = {
    {"puck", {0.03, 0.03, 0.06}}, {"puck_lying", {0.06, 0.03, 0.03}}, {"goal_zone", {0.30, 0.30, 0.01}},
    {"station_zone", {0.45, 0.45, 0.01}}, {"qr_board", {0.01, 0.30, 0.30}}, {"obstacle", {0.20, 0.20, 0.30}},
    {"person", {0.40, 0.40, 1.70}}, {"traffic_light", {0.14, 0.14, 0.45}}, {"location_box", {0.205, 0.205, 0.07}},
    {"start_box", {0.205, 0.205, 0.07}}, {"barcode", {0.10, 0.01, 0.05}}};
  const auto it = m.find(cls);
  return it == m.end() ? cv::Vec3d(0.1, 0.1, 0.1) : it->second;
}
double yaw_var_known() { return std::pow(deg2rad(10.0), 2); }
bool is_puck_class(const std::string & cls) { return cls == "puck" || cls == "puck_lying"; }
bool no_color(const std::string & c) { return c.empty() || c == "unknown"; }

// ---------- 통계 (numpy와 같게)
double np_median(std::vector<double> v) {
  if (v.empty()) return std::nan("");
  const size_t n = v.size(), m = n / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  if (n % 2) return v[m];
  const double hi = v[m];
  return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + m));
}

double np_percentile(std::vector<double> v, double q) {
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  const double pos = q / 100.0 * (v.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(pos));
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (pos - lo) * (v[hi] - v[lo]);
}

// ---------- 종류 · 같은 물체
std::string kind(const std::string & cls) { return is_puck_class(cls) ? "puck" : cls; }

bool same_object(const std::string & tr_class, const std::optional<int> & tr_tag, const std::string & tr_content,
                 const std::string & tr_color, const std::string & d_class, const std::optional<int> & d_tag,
                 const std::string & d_content, const std::string & d_color) {
  if (tr_class != kind(d_class)) return false;
  if (tr_tag && d_tag && *tr_tag != *d_tag) return false;           // 태그 번호가 둘 다 있으면 같아야
  if (!tr_content.empty() && !d_content.empty() && tr_content != d_content) return false;  // 바코드 내용
  if (tr_class != "puck") return true;
  return no_color(tr_color) || no_color(d_color) || tr_color == d_color;  // 퍽은 색도 (모르는 쪽이 있으면 안 가름)
}

bool same_object(const Track & tr, const Det & d) {
  return same_object(tr.class_name, tr.tag, tr.content, tr.color, d.class_name, d.tag, d.content, d.color);
}

cv::Vec3d puck_center(const cv::Vec3d & p, const cv::Vec3d & cam, double offset, double full_deg) {
  if (offset <= 0) return p;
  const cv::Vec3d d = p - cam;
  const double h = std::hypot(d[0], d[1]);
  if (h < 1e-6) return p;
  const double cos_down = h / cv::norm(d);  // 내려다본 각의 cos (수평으로 보면 1)
  const double k = std::min(1.0, cos_down / std::cos(deg2rad(full_deg)));
  cv::Vec3d out = p;
  out[0] += d[0] / h * offset * k;
  out[1] += d[1] / h * offset * k;
  return out;
}

double box_iou(const Box & a, const Box & b) {
  const double w = std::min(a.x1, b.x1) - std::max(a.x0, b.x0);
  const double h = std::min(a.y1, b.y1) - std::max(a.y0, b.y0);
  if (w <= 0 || h <= 0) return 0.0;
  const double inter = w * h;
  return inter / ((a.x1 - a.x0) * double(a.y1 - a.y0) + (b.x1 - b.x0) * double(b.y1 - b.y0) - inter);
}

std::vector<Det> merge_overlap_dets(const std::vector<Det> & dets, double iou_min) {
  if (iou_min <= 0) return dets;
  std::vector<Det> sorted = dets;
  std::stable_sort(sorted.begin(), sorted.end(), [](const Det & a, const Det & b) { return a.conf > b.conf; });
  std::vector<Det> out;
  for (const auto & d : sorted) {
    Det * k = nullptr;
    for (auto & o : out) {
      if (d.box && o.box && kind(o.class_name) == "puck" && kind(d.class_name) == "puck" &&
          same_object(kind(o.class_name), o.tag, o.content, o.color, d.class_name, d.tag, d.content, d.color) &&
          box_iou(*o.box, *d.box) >= iou_min) { k = &o; break; }
    }
    if (!k) { out.push_back(d); continue; }
    if (!k->tag && d.tag) k->tag = d.tag;  // 남긴 쪽이 모르는 태그·모양·자세는 버린 쪽 것으로
    if (k->shape.empty() && !d.shape.empty()) k->shape = d.shape;
    if (k->posture.empty() && !d.posture.empty()) k->posture = d.posture;
  }
  return out;
}

std::map<int, int> merge_tracks(std::map<int, Track> & tracks, double dist_m) {
  std::map<int, int> gone;
  if (dist_m <= 0) return gone;
  std::vector<int> ids;
  for (const auto & [t, tr] : tracks)
    if (tr.class_name == "puck") ids.push_back(t);  // map이라 이미 오름차순
  for (size_t i = 0; i < ids.size(); ++i) {
    const int a = ids[i];
    if (gone.count(a)) continue;
    for (size_t j = i + 1; j < ids.size(); ++j) {
      const int b = ids[j];
      if (gone.count(b)) continue;
      Track & ta = tracks.at(a);
      Track & tb = tracks.at(b);
      if (cv::norm(ta.p - tb.p) >= dist_m ||
          !same_object(ta.class_name, ta.tag, ta.content, ta.color, tb.class_name, tb.tag, tb.content, tb.color))
        continue;
      if (tb.t > ta.t) { ta.p = tb.p; ta.P = tb.P; ta.t = tb.t; }  // 위치는 더 최근 쪽
      if (!ta.tag && tb.tag) ta.tag = tb.tag;                      // 태그·모양·자세·방향은 아는 쪽
      if (ta.shape.empty() && !tb.shape.empty()) ta.shape = tb.shape;
      if (ta.posture.empty() && !tb.posture.empty()) ta.posture = tb.posture;
      if (!ta.yaw && tb.yaw) ta.yaw = tb.yaw;
      if (ta.yaw_period == 0.0 && tb.yaw_period != 0.0) ta.yaw_period = tb.yaw_period;
      if (no_color(ta.color)) ta.color = tb.color;
      for (const auto & [c, s] : tb.seen) ta.seen.emplace(c, s);  // {**tb.seen, **ta.seen} — ta 값이 우선
      tracks.erase(b);
      gone[b] = a;
    }
  }
  return gone;
}

// ---------- 자기 몸
double seg_dist(const cv::Vec3d & p, const cv::Vec3d & a, const cv::Vec3d & b) {
  const cv::Vec3d ab = b - a;
  const double t = std::clamp((p - a).dot(ab) / std::max(ab.dot(ab), 1e-12), 0.0, 1.0);
  return cv::norm(p - (a + t * ab));
}

bool is_self(const std::string & cls, const cv::Vec3d & p, const Body & body, double body_radius, double body_height,
             double self_radius, const std::vector<double> & body_box) {
  const cv::Vec3d & base = body.base;
  // 바닥(5cm 아래)은 차체로 보지 않음 — 범퍼 바로 앞 퍽이 차체 반경 안에 걸려 지워지지 않게
  if (base[2] + BODY_MIN_Z < p[2] && p[2] < base[2] + body_height) {
    if (body_box.size() == 3) {
      const cv::Vec3d q = body.rot.t() * (p - base);  // base_frame 좌표
      if (body_box[0] < q[0] && q[0] < body_box[1] && std::abs(q[1]) < body_box[2]) return true;
    } else if (std::hypot(p[0] - base[0], p[1] - base[1]) < body_radius) {
      return true;
    }
  }
  if (body.chain.size() < 2) return false;
  size_t nseg = body.chain.size() - 1;
  if (is_puck_class(cls)) nseg = nseg > 0 ? nseg - 1 : 0;  // 퍽은 마지막 선분(손가락 사이)을 빼고 봄
  for (size_t i = 0; i < nseg; ++i)
    if (seg_dist(p, body.chain[i], body.chain[i + 1]) < self_radius) return true;
  return false;
}

std::vector<cv::Vec3d> arm_pixels(const std::vector<cv::Vec3d> & chain, const cv::Matx33d & rot, const cv::Vec3d & trans,
                                  double fx, double fy, double cx, double cy, double radius, int per_seg) {
  std::vector<cv::Vec3d> out;
  if (chain.size() < 2) return out;
  for (size_t i = 0; i + 1 < chain.size(); ++i) {
    for (int k = 0; k < per_seg; ++k) {
      const double t = per_seg > 1 ? double(k) / (per_seg - 1) : 0.0;
      const cv::Vec3d p = chain[i] + (chain[i + 1] - chain[i]) * t;
      const cv::Vec3d c = rot.t() * (p - trans);
      if (c[2] <= 0.05) continue;  // 카메라 뒤 점은 뺌
      out.emplace_back(fx * c[0] / c[2] + cx, fy * c[1] / c[2] + cy, fx * radius / c[2]);
    }
  }
  return out;
}

bool box_has_arm(const Box & box, const std::vector<cv::Vec3d> & arm_px, int min_hits) {
  if (arm_px.empty()) return false;
  int n = 0;
  for (const auto & a : arm_px)
    if (a[0] + a[2] >= box.x0 && a[0] - a[2] <= box.x1 && a[1] + a[2] >= box.y0 && a[1] - a[2] <= box.y1) ++n;
  return n >= min_hits;
}

// ---------- 추적 수학
cv::Matx33d meas_cov_cam(double z, double fx, double fy, double pixel_sigma, double depth_sigma0, double depth_sigma_k) {
  const double sz = depth_sigma0 + depth_sigma_k * z * z;
  return cv::Matx33d::diag({std::pow(pixel_sigma * z / fx, 2), std::pow(pixel_sigma * z / fy, 2), sz * sz});
}

std::tuple<cv::Vec3d, cv::Matx33d, cv::Matx33d> kf_update(const cv::Vec3d & p, const cv::Matx33d & P, const cv::Vec3d & z,
                                                          const cv::Matx33d & R) {
  const cv::Matx33d K = P * (P + R).inv();
  const cv::Vec3d pn = p + K * (z - p);
  cv::Matx33d Pn = (cv::Matx33d::eye() - K) * P;
  Pn = 0.5 * (Pn + Pn.t());
  return {pn, Pn, K};
}

// ---------- 색
bool is_hue_name(const std::string & name) { return !hue_refs(name).empty(); }

int hue_dist(int h, const std::string & name) {
  int best = std::numeric_limits<int>::max();
  for (int x : hue_refs(name)) best = std::min(best, std::min(std::abs(h - x), 180 - std::abs(h - x)));
  return best;
}

std::string classify_color(const std::string & cls, const cv::Mat & roi_bgr) {
  if (roi_bgr.empty() || !(cls == "puck" || cls == "puck_lying" || cls == "traffic_light")) return "";
  cv::Mat roi = cls != "traffic_light" ? inner(roi_bgr, true) : roi_bgr;
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  std::vector<int> sel;
  if (cls == "traffic_light") {
    for (int y = 0; y < hsv.rows; ++y)
      for (int x = 0; x < hsv.cols; ++x) {
        const auto & v = hsv.at<cv::Vec3b>(y, x);
        if (v[1] > 100 && v[2] > 90) sel.push_back(v[0]);  // 켜진 램프 = 채도·밝기 높음
      }
    if (sel.size() < 4) return "off";
    const auto votes = hue_votes(sel, LAMP_COLORS);
    int total = 0;
    for (const auto & [n, c] : votes) total += c;
    int many = 0;
    for (const auto & [n, c] : votes) many += c >= 0.15 * total;
    if (many >= 2) return "all";  // 두 색 이상 동시에 켜짐
    return vote_max(votes);
  }
  int achrom = 0, n = 0;
  std::vector<double> vals;
  for (int y = 0; y < hsv.rows; ++y)
    for (int x = 0; x < hsv.cols; ++x) {
      const auto & v = hsv.at<cv::Vec3b>(y, x);
      ++n;
      achrom += v[1] < ACHROMATIC_S;
      vals.push_back(v[2]);
      if (v[1] > 80 && v[2] > 50) sel.push_back(v[0]);
    }
  if (n && double(achrom) / n > 0.6) {
    const double med = np_median(vals);
    return med >= WHITE_V ? "white" : med <= BLACK_V ? "black" : "unknown";
  }
  if (sel.size() < 8) return "unknown";
  return vote_max(hue_votes(sel, hue_names()));
}

std::pair<std::string, int> puck_color(const cv::Mat & roi_bgr, const std::vector<std::string> & colors, int hue_tol) {
  cv::Mat roi = roi_bgr.empty() ? cv::Mat() : inner(roi_bgr, false);
  std::vector<std::string> names;
  for (const auto & c : colors)
    if (is_hue_name(c)) names.push_back(c);
  if (roi.empty() || names.empty()) return {"unknown", -1};
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
  std::vector<int> h;
  for (int y = 0; y < hsv.rows; ++y)
    for (int x = 0; x < hsv.cols; ++x) {
      const auto & v = hsv.at<cv::Vec3b>(y, x);
      if (v[1] > 80 && v[2] > 50) h.push_back(v[0]);  // 바닥·그림자·흰 반사 뺌
    }
  if (h.size() < 8) return {"unknown", -1};
  std::vector<int> votes(names.size() + 1, 0);
  for (int hv : h) {
    size_t best = 0;
    int bd = std::numeric_limits<int>::max();
    for (size_t k = 0; k < names.size(); ++k) {
      const int d = hue_dist(hv, names[k]);
      if (d < bd) { bd = d; best = k; }
    }
    ++votes[bd <= hue_tol ? best : names.size()];  // 기준에서 hue_tol 넘게 멀면 'other'
  }
  const size_t pick = std::max_element(votes.begin(), votes.end()) - votes.begin();
  std::vector<double> hd(h.begin(), h.end());
  return {pick < names.size() ? names[pick] : "other", static_cast<int>(np_median(hd))};
}

std::pair<std::optional<std::string>, std::string> settle_puck_color(const cv::Mat & roi_bgr, const std::string & color,
                                                                     const std::string & model_color,
                                                                     const std::vector<std::string> & puck_colors,
                                                                     bool use_model_color) {
  auto in = [&](const std::string & c) { return std::find(puck_colors.begin(), puck_colors.end(), c) != puck_colors.end(); };
  const std::string mc = use_model_color && in(model_color) ? model_color : "";
  if (in(color)) return {color, ""};
  if (color == "other") {
    const auto near = puck_color(roi_bgr, hue_names(), 90).first;  // 모든 색 중 가장 가까운 색으로 다시 투표
    if (!in(near)) return {std::nullopt, "다른 색(" + near + ")"};
    return {mc.empty() ? near : mc, ""};
  }
  if (!mc.empty()) return {mc, ""};
  return {std::nullopt, "색 없음"};
}

// ---------- 클래스 이름
std::string resolve_model_path(const std::string & model_path, const std::string & model_dir) {
  namespace fs = std::filesystem;
  if (model_path.empty()) {
    if (model_dir.empty()) return "";
    const auto cand = fs::path(model_dir) / "model.pt";
    return fs::exists(cand) ? cand.string() : "";
  }
  if (model_path.find('/') == std::string::npos && !model_dir.empty()) {
    const auto cand = fs::path(model_dir) / model_path;
    if (fs::exists(cand)) return cand.string();
  }
  return model_path;
}

static std::vector<std::string> split_us(const std::string & s) {
  std::vector<std::string> out;
  size_t a = 0;
  while (true) {
    const size_t b = s.find('_', a);
    out.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
    if (b == std::string::npos) break;
    a = b + 1;
  }
  return out;
}

std::pair<std::string, std::string> split_class(const std::string & name) {
  if (name.rfind("puck", 0) == 0) {
    const auto parts = split_us(name);
    for (size_t i = 1; i < parts.size(); ++i)
      if (is_hue_name(parts[i])) return {"puck", parts[i]};
    return {"puck", ""};
  }
  const std::string pre = "traffic_light_";
  if (name.rfind(pre, 0) == 0) {
    const std::string st = name.substr(pre.size());
    const bool ok = st == "red" || st == "yellow" || st == "green" || st == "off";
    return {"traffic_light", ok ? st : ""};
  }
  return {name, ""};
}

std::string puck_shape(const std::string & name) {
  if (name.rfind("puck", 0) != 0) return "";
  const auto parts = split_us(name);
  for (size_t i = 1; i < parts.size(); ++i)
    if (parts[i] == "box" || parts[i] == "cyl") return parts[i];
  return "";
}

std::string model_posture(const std::string & name) {
  if (puck_shape(name).empty()) return "";
  const std::string suf = "_lying";
  return name.size() >= suf.size() && name.compare(name.size() - suf.size(), suf.size(), suf) == 0 ? "lying" : "standing";
}

std::string tag_shape(const std::string & face, const std::string & posture) {
  if ((posture == "standing" && face == "side") || (posture == "lying" && face == "top")) return "box";
  return "";
}

// ---------- 깊이 · 자세
cv::Mat1b arm_near(const cv::Mat1f & patch, const cv::Mat1b & arm, double margin) {
  cv::Mat1b out = cv::Mat1b::zeros(patch.size());
  if (arm.empty() || cv::countNonZero(arm) == 0) return out;
  std::vector<double> ok;
  for (int y = 0; y < patch.rows; ++y)
    for (int x = 0; x < patch.cols; ++x) {
      const float v = patch(y, x);
      if (std::isfinite(v) && v > 0.05f) ok.push_back(v);
    }
  if (ok.size() < 8) return out;
  const double thr = np_percentile(ok, 80) - margin;  // 물체 depth 기준보다 margin 넘게 가까운 팔 화소만
  for (int y = 0; y < patch.rows; ++y)
    for (int x = 0; x < patch.cols; ++x) {
      const float v = patch(y, x);
      out(y, x) = (arm(y, x) && std::isfinite(v) && v > 0.05f && v < thr) ? 1 : 0;
    }
  return out;
}

cv::Mat1b arm_mask_image(const std::vector<cv::Vec3d> & arm_px, cv::Size shape) {
  if (arm_px.empty()) return cv::Mat1b();
  cv::Mat1b m = cv::Mat1b::zeros(shape);
  for (const auto & a : arm_px)
    if (-100 < a[0] && a[0] < shape.width + 100 && -100 < a[1] && a[1] < shape.height + 100)
      cv::circle(m, cv::Point(static_cast<int>(a[0]), static_cast<int>(a[1])), static_cast<int>(a[2] * 1.5 + 6), 1, -1);
  return cv::countNonZero(m) ? m : cv::Mat1b();
}

// 파이썬 슬라이스처럼 [a, b)를 0~n 안으로
static cv::Range py_range(int a, int b, int n) {
  a = std::clamp(a < 0 ? a + n : a, 0, n);
  b = std::clamp(b < 0 ? b + n : b, 0, n);
  return cv::Range(a, std::max(a, b));
}

double box_depth_m(const cv::Mat1f & depth_m, const Box & box, double min_valid, const cv::Mat1b & arm) {
  const int bw = box.x1 - box.x0, bh = box.y1 - box.y0;
  const cv::Range ry = py_range(box.y0 + bh / 3, box.y1 - bh / 3 + 1, depth_m.rows);
  const cv::Range rx = py_range(box.x0 + bw / 3, box.x1 - bw / 3 + 1, depth_m.cols);
  const cv::Mat1f patch = depth_m(ry, rx);
  const cv::Mat1b near = arm_near(patch, arm.empty() ? cv::Mat1b() : cv::Mat1b(arm(ry, rx)));  // 손가락 depth는 뺌
  std::vector<double> valid;
  for (int y = 0; y < patch.rows; ++y)
    for (int x = 0; x < patch.cols; ++x) {
      const float v = patch(y, x);
      if (std::isfinite(v) && v > 0.05f && !near(y, x)) valid.push_back(v);
    }
  if (valid.empty() || valid.size() < min_valid * patch.total()) return 0.0;
  return np_median(valid);
}

std::pair<std::string, std::optional<double>> posture_from_height(
  const cv::Mat1f & depth_m, const Box & box, double zm, const cv::Matx33d & rot, const cv::Vec3d & trans,
  double fx, double fy, double cx, double cy, double floor_z, std::pair<double, double> supports, double tol, int min_pts,
  double margin) {
  if (zm <= 0) return {"", std::nullopt};
  const cv::Range ry = py_range(box.y0, box.y1 + 1, depth_m.rows), rx = py_range(box.x0, box.x1 + 1, depth_m.cols);
  std::vector<double> zs;
  for (int y = ry.start; y < ry.end; ++y)
    for (int x = rx.start; x < rx.end; ++x) {
      const float z = depth_m(y, x);
      if (!std::isfinite(z) || std::abs(z - zm) >= 0.04 || z <= 0.05) continue;  // 퍽 자체 (뒤 바닥·벽 빼기)
      const cv::Vec3d p = rot * cv::Vec3d((x - cx) * z / fx, (y - cy) * z / fy, z) + trans;
      zs.push_back(p[2]);
    }
  if (static_cast<int>(zs.size()) < min_pts) return {"", std::nullopt};
  const double top = np_percentile(zs, 90) - floor_z;
  const std::pair<std::string, double> tops[] = {{"standing", 0.06}, {"lying", 0.03}};
  std::map<std::string, double> errs;
  for (double sup : {supports.first, supports.second})
    for (const auto & [pose, hgt] : tops) {
      const double e = std::abs(top - (sup + hgt));
      errs[pose] = errs.count(pose) ? std::min(errs[pose], e) : e;
    }
  // 파이썬 min/max(errs, key) — 같으면 dict 순서 앞(standing)
  const std::string best = errs["lying"] < errs["standing"] ? "lying" : "standing";
  const std::string other = errs["lying"] > errs["standing"] ? "lying" : "standing";
  if (errs[best] > tol || errs[other] - errs[best] < margin) return {"", top};
  return {best, top};
}

bool view_from_side(const cv::Matx33d & rot, double max_down_deg) { return -rot(2, 2) < std::sin(deg2rad(max_down_deg)); }

// ---------- AprilTag
#ifdef PERCEPTION_ARUCO_DETECTOR
struct TagDetector::Impl {  // OpenCV 4.7+ — 검출기 객체에 파라미터를 넣고, 바뀔 때만 다시 넣음
  cv::aruco::DetectorParameters params;
  cv::aruco::ArucoDetector det;
  Impl() : params(make_params()), det(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), params) {}
  static cv::aruco::DetectorParameters make_params() {
    cv::aruco::DetectorParameters p;
    p.minMarkerPerimeterRate = 0.01;
    return p;
  }
};

TagDetector::TagDetector() : impl_(std::make_shared<Impl>()) { ok_ = true; }

void TagDetector::detect(const cv::Mat & gray, std::optional<double> min_px, std::vector<std::vector<cv::Point2f>> & corners,
                         std::vector<int> & ids) {
  const double rate = min_px ? *min_px / std::max(gray.rows, gray.cols) : 0.01;
  if (impl_->params.minMarkerPerimeterRate != rate) {
    impl_->params.minMarkerPerimeterRate = rate;
    impl_->det.setDetectorParameters(impl_->params);
  }
  impl_->det.detectMarkers(gray, corners, ids);
}
#else
struct TagDetector::Impl {  // OpenCV 4.6 이하 — contrib aruco 함수
  cv::Ptr<cv::aruco::Dictionary> dict;
  cv::Ptr<cv::aruco::DetectorParameters> params;
};

TagDetector::TagDetector() : impl_(std::make_shared<Impl>()) {
  impl_->dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
  impl_->params = cv::aruco::DetectorParameters::create();
  impl_->params->minMarkerPerimeterRate = 0.01;
  ok_ = !impl_->dict.empty();
}

void TagDetector::detect(const cv::Mat & gray, std::optional<double> min_px, std::vector<std::vector<cv::Point2f>> & corners,
                         std::vector<int> & ids) {
  impl_->params->minMarkerPerimeterRate = min_px ? *min_px / std::max(gray.rows, gray.cols) : 0.01;
  cv::aruco::detectMarkers(gray, impl_->dict, corners, ids, impl_->params);
}
#endif

std::vector<Tag> detect_tags(TagDetector * det, const cv::Mat & bgr, double gamma, bool half, std::optional<double> min_px,
                             std::optional<double> half_min_px) {
  std::vector<Tag> out;
  if (!det || !det->ok()) return out;
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  struct Pass { cv::Mat img; double scale; std::optional<double> px; };
  std::vector<Pass> passes{{gray, 1.0, min_px}};
  if (gamma > 0) {  // 어둡게 편 영상 — 반사로 뜬 회색 칸을 검정 쪽으로
    cv::Mat lut(1, 256, CV_8U);
    for (int v = 0; v < 256; ++v) lut.at<uchar>(v) = static_cast<uchar>(255.0 * std::pow(v / 255.0, gamma));
    cv::Mat g;
    cv::LUT(gray, lut, g);
    passes.push_back({g, 1.0, min_px});
  }
  if (half) {  // 반 크기 — 크게 찍힌 태그
    cv::Mat hgray;
    cv::resize(gray, hgray, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    passes.push_back({hgray, 2.0, half_min_px});
  }
  std::vector<std::pair<int, std::array<cv::Point2d, 4>>> found;
  for (const auto & ps : passes) {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    det->detect(ps.img, ps.px, corners, ids);
    for (size_t k = 0; k < ids.size(); ++k) {
      std::array<cv::Point2d, 4> pts;
      cv::Point2d mean(0, 0);
      for (int j = 0; j < 4; ++j) {
        pts[j] = cv::Point2d(corners[k][j].x, corners[k][j].y) * ps.scale;
        mean += pts[j] * 0.25;
      }
      bool dup = false;  // 두 영상 모두 찾은 태그는 한 번만 (같은 번호, 중심 8 px 안)
      for (const auto & [j, p] : found) {
        cv::Point2d m2(0, 0);
        for (const auto & q : p) m2 += q * 0.25;
        if (j == ids[k] && cv::norm(m2 - mean) < 8.0) { dup = true; break; }
      }
      if (!dup) found.emplace_back(ids[k], pts);
    }
  }
  for (const auto & [id, pts] : found) {
    Tag t;
    t.id = id;
    t.corners = pts;
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const auto & p : pts) {
      t.c += p * 0.25;
      x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
    }
    t.box = {static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(x1) + 1, static_cast<int>(y1) + 1};
    out.push_back(t);
  }
  return out;
}

std::vector<Box> merge_rois(std::vector<Box> rois, double margin, int pad_px, int w, int h) {
  // 네모마다 margin(변 길이 비율) + pad_px만큼 넓히고 화면 안으로 · 겹치거나 닿는 네모는 하나로 (같은 곳을 두 번 안 찾게)
  for (auto & r : rois) {
    const int mx = static_cast<int>((r.x1 - r.x0) * margin) + pad_px, my = static_cast<int>((r.y1 - r.y0) * margin) + pad_px;
    r = {std::max(r.x0 - mx, 0), std::max(r.y0 - my, 0), std::min(r.x1 + mx, w), std::min(r.y1 + my, h)};
  }
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t i = 0; i < rois.size() && !changed; ++i)
      for (size_t j = i + 1; j < rois.size(); ++j) {
        const Box & a = rois[i], & b = rois[j];
        if (a.x0 <= b.x1 && b.x0 <= a.x1 && a.y0 <= b.y1 && b.y0 <= a.y1) {
          rois[i] = {std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1)};
          rois.erase(rois.begin() + static_cast<long>(j));
          changed = true;
          break;
        }
      }
  }
  return rois;
}

std::vector<Tag> detect_tags_in(TagDetector * det, const cv::Mat & bgr, const std::vector<Box> & rois, double gamma, bool half,
                                std::optional<double> min_px, std::optional<double> half_min_px) {
  // 네모(merge_rois 결과)마다 잘라서 detect_tags → 원래 화면 좌표로 옮김. 네모가 화면 전체 넓이의 60 %를 넘으면 그냥 전체
  double area = 0.0;
  for (const auto & r : rois) area += static_cast<double>(r.x1 - r.x0) * (r.y1 - r.y0);
  if (area > 0.6 * bgr.cols * bgr.rows) return detect_tags(det, bgr, gamma, half, min_px, half_min_px);
  std::vector<Tag> out;
  for (const auto & r : rois) {
    if (r.x1 - r.x0 < 8 || r.y1 - r.y0 < 8) continue;
    const cv::Mat crop = bgr(cv::Range(r.y0, r.y1), cv::Range(r.x0, r.x1));
    for (auto t : detect_tags(det, crop, gamma, half, min_px, half_min_px)) {
      const cv::Point2d o(r.x0, r.y0);
      t.c += o;
      for (auto & q : t.corners) q += o;
      t.box = {t.box.x0 + r.x0, t.box.y0 + r.y0, t.box.x1 + r.x0, t.box.y1 + r.y0};
      out.push_back(t);
    }
  }
  return out;
}

Box clip_box(double x0, double y0, double x1, double y1, int w, int h) {
  return {std::max(static_cast<int>(x0), 0), std::max(static_cast<int>(y0), 0), std::min(static_cast<int>(x1), w - 1),
          std::min(static_cast<int>(y1), h - 1)};
}

std::optional<std::pair<int, std::array<cv::Point2d, 4>>> tag_in_box(const std::vector<Tag> & tags, const Box & box,
                                                                    const std::set<int> & ids,
                                                                    const std::vector<Box> & others) {
  auto inside = [](const Box & b, double u, double v) {
    const double mx = (b.x1 - b.x0) * 0.1, my = (b.y1 - b.y0) * 0.1;
    return b.x0 - mx <= u && u <= b.x1 + mx && b.y0 - my <= v && v <= b.y1 + my;
  };
  auto off = [](const Box & b, double u, double v) {  // 박스 가운데에서 떨어진 정도 (박스 폭·높이 단위)
    return std::hypot((u - (b.x0 + b.x1) / 2.0) / std::max(b.x1 - b.x0, 1), (v - (b.y0 + b.y1) / 2.0) / std::max(b.y1 - b.y0, 1));
  };
  for (const auto & t : tags) {
    if (!ids.count(t.id) || !inside(box, t.c.x, t.c.y)) continue;
    bool closer = false;  // 겹친 퍽에서 다른 박스 가운데에 더 가까우면 그쪽 것
    for (const auto & o : others)
      if (!(o == box) && inside(o, t.c.x, t.c.y) && off(o, t.c.x, t.c.y) < off(box, t.c.x, t.c.y)) { closer = true; break; }
    if (!closer) return std::make_pair(t.id, t.corners);
  }
  return std::nullopt;
}

// ---------- 방향
double wrap_yaw(double yaw, double period) {
  double r = std::fmod(yaw + period / 2.0, period);  // 파이썬 %는 결과가 항상 0 이상
  if (r < 0) r += period;
  return r - period / 2.0;
}

std::optional<std::pair<cv::Vec3d, cv::Vec3d>> tag_pnp(const std::array<cv::Point2d, 4> & corners, double size_m, double fx,
                                                       double fy, double cx, double cy) {
  std::vector<cv::Point2d> img(corners.begin(), corners.end());
  cv::Vec3d rvec, tvec;
  try {
    if (!cv::solvePnP(tag_obj(size_m), img, cv::Mat(cam_k(fx, fy, cx, cy)), cv::noArray(), rvec, tvec, false,
                      cv::SOLVEPNP_IPPE_SQUARE))
      return std::nullopt;
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
  return std::make_pair(rvec, tvec);
}

std::pair<std::optional<double>, std::string> tag_yaw(const std::array<cv::Point2d, 4> & corners, double size_m, double fx,
                                                      double fy, double cx, double cy, const cv::Matx33d & rot) {
  const auto pnp = tag_pnp(corners, size_m, fx, fy, cx, cy);
  if (!pnp) return {std::nullopt, ""};
  const cv::Matx33d r_tag = rot * rodrigues(pnp->first);  // 태그 축 → 추적 좌표
  const cv::Vec3d n(r_tag(0, 2), r_tag(1, 2), r_tag(2, 2)), xa(r_tag(0, 0), r_tag(1, 0), r_tag(2, 0));
  if (std::abs(n[2]) < 0.5) return {std::atan2(n[1], n[0]), "side"};    // 법선이 수평에서 30° 안 → 옆면
  if (std::abs(n[2]) > 0.87) return {std::atan2(xa[1], xa[0]), "top"};  // 수직에서 30° 안 → 윗면
  return {std::nullopt, ""};
}

std::optional<cv::Vec3d> tag_center(const std::array<cv::Point2d, 4> & corners, const cv::Mat1f & depth_m, double size_m,
                                    double half_w, double fx, double fy, double cx, double cy, const cv::Matx33d & rot,
                                    const cv::Vec3d & trans, const cv::Mat1b & arm) {
  const auto pnp = tag_pnp(corners, size_m, fx, fy, cx, cy);
  if (!pnp) return std::nullopt;
  double u = 0, v = 0, x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
  for (const auto & p : corners) {
    u += p.x / 4; v += p.y / 4;
    x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y);
  }
  const double qx = (x1 - x0) / 4, qy = (y1 - y0) / 4;
  const int h = depth_m.rows, w = depth_m.cols;
  const cv::Range ry(std::max(static_cast<int>(y0 + qy), 0), std::max(std::max(static_cast<int>(y0 + qy), 0),
                                                                      std::min(static_cast<int>(std::ceil(y1 - qy)), h)));
  const cv::Range rx(std::max(static_cast<int>(x0 + qx), 0), std::max(std::max(static_cast<int>(x0 + qx), 0),
                                                                      std::min(static_cast<int>(std::ceil(x1 - qx)), w)));
  const cv::Mat1f patch = depth_m(ry, rx);
  const cv::Mat1b near = arm_near(patch, arm.empty() ? cv::Mat1b() : cv::Mat1b(arm(ry, rx)));
  std::vector<double> valid;
  for (int y = 0; y < patch.rows; ++y)
    for (int x = 0; x < patch.cols; ++x) {
      const float z = patch(y, x);
      if (std::isfinite(z) && z > 0.05f && !near(y, x)) valid.push_back(z);
    }
  cv::Vec3d p;
  if (valid.size() >= 3) {
    const double z = np_median(valid);
    p = rot * cv::Vec3d((u - cx) * z / fx, (v - cy) * z / fy, z) + trans;
  } else {
    p = rot * pnp->second + trans;  // depth 없음(너무 가까움) → solvePnP 위치
  }
  const cv::Matx33d r = rot * rodrigues(pnp->first);
  const cv::Vec3d n(r(0, 2), r(1, 2), r(2, 2));  // 태그 법선 (추적 좌표)
  if (std::abs(n[2]) > 0.87) return p;            // 윗면
  if (std::abs(n[2]) < 0.5) {                     // 옆면 — 법선 반대로 half_w 들어감
    cv::Vec3d nh(n[0], n[1], 0.0);
    nh *= 1.0 / std::max(std::hypot(n[0], n[1]), 1e-9);
    if (nh.dot(trans - p) < 0) nh = -nh;  // 법선이 카메라 반대쪽이면 뒤집음
    return p - nh * half_w;
  }
  return std::nullopt;
}

std::optional<double> axis_yaw(const std::vector<cv::Vec2d> & pts, int min_points, double min_elong) {
  if (static_cast<int>(pts.size()) < min_points) return std::nullopt;
  cv::Vec2d m(0, 0);
  for (const auto & p : pts) m += p;
  m *= 1.0 / pts.size();
  double sxx = 0, sxy = 0, syy = 0;  // np.cov (n−1로 나눔)
  for (const auto & p : pts) {
    const cv::Vec2d d = p - m;
    sxx += d[0] * d[0]; sxy += d[0] * d[1]; syy += d[1] * d[1];
  }
  const double n1 = pts.size() - 1.0;
  cv::Matx22d c(sxx / n1, sxy / n1, sxy / n1, syy / n1);
  cv::Mat w, v;
  cv::eigen(cv::Mat(c), w, v);  // 고윳값 내림차순 · 행 = 고유벡터
  const double w_big = w.at<double>(0), w_small = w.at<double>(1);
  if (w_small <= 0 || std::sqrt(w_big / w_small) < min_elong) return std::nullopt;
  return std::atan2(v.at<double>(0, 1), v.at<double>(0, 0));
}

// ---------- 상자
std::vector<BoxTop> find_box_tops(const cv::Mat1f & depth_m, const cv::Matx33d & rot, const cv::Vec3d & trans, double fx,
                                  double fy, double cx, double cy, double floor_z, std::pair<double, double> h_range,
                                  double min_side, double max_side, double cell, int step, double max_range,
                                  const std::optional<std::pair<cv::Vec2d, double>> & avoid,
                                  const std::optional<std::vector<std::array<double, 4>>> & rois,
                                  const std::vector<cv::Vec3d> & mask_px) {
  std::vector<BoxTop> out;
  if (rois && rois->empty()) return out;
  std::vector<cv::Vec3d> top, tall;
  for (int v = 0; v < depth_m.rows; v += step)
    for (int u = 0; u < depth_m.cols; u += step) {
      const float z = depth_m(v, u);
      if (!(z > 0.15f && z < max_range)) continue;
      if (rois) {
        bool in = false;
        for (const auto & r : *rois)
          if (u >= r[0] && u <= r[2] && v >= r[1] && v <= r[3]) { in = true; break; }
        if (!in) continue;
      }
      bool arm = false;  // 자기 팔 둘레는 뺌
      for (const auto & a : mask_px)
        if ((u - a[0]) * (u - a[0]) + (v - a[1]) * (v - a[1]) <= std::pow(1.5 * a[2] + 6.0, 2)) { arm = true; break; }
      if (arm) continue;
      const cv::Vec3d p = rot * cv::Vec3d((u - cx) * z / fx, (v - cy) * z / fy, z) + trans;
      if (avoid && std::hypot(p[0] - avoid->first[0], p[1] - avoid->first[1]) <= avoid->second) continue;
      const double hz = p[2] - floor_z;
      if (hz > h_range.first && hz < h_range.second) top.push_back(p);
      if (hz > 0.15 && hz < 0.6) tall.push_back(p);
    }
  if (top.size() < 10) return out;
  double ox = 1e9, oy = 1e9;
  for (const auto & p : top) { ox = std::min(ox, p[0]); oy = std::min(oy, p[1]); }
  ox -= cell * 2; oy -= cell * 2;
  int maxi = 0, maxj = 0;
  std::vector<cv::Point> ij;
  for (const auto & p : top) {
    const cv::Point q(static_cast<int>((p[0] - ox) / cell), static_cast<int>((p[1] - oy) / cell));
    ij.push_back(q);
    maxi = std::max(maxi, q.x); maxj = std::max(maxj, q.y);
  }
  cv::Mat1b grid = cv::Mat1b::zeros(maxj + 3, maxi + 3);
  for (const auto & q : ij) grid(q.y, q.x) = 255;
  cv::morphologyEx(grid, grid, cv::MORPH_CLOSE, cv::Mat1b::ones(3, 3));  // 퍽 그림자 구멍을 메움
  cv::Mat lab;
  const int n = cv::connectedComponents(grid, lab);
  for (int k = 1; k < n; ++k) {
    std::vector<cv::Point2f> cells;
    for (int y = 0; y < lab.rows; ++y)
      for (int x = 0; x < lab.cols; ++x)
        if (lab.at<int>(y, x) == k)
          cells.emplace_back(static_cast<float>(x * cell + static_cast<float>(ox)), static_cast<float>(y * cell + static_cast<float>(oy)));
    if (cells.size() < 4) continue;
    const cv::RotatedRect rr = cv::minAreaRect(cells);
    const double mx = rr.center.x, my = rr.center.y;
    const double a = rr.size.width + cell, b = rr.size.height + cell;  // 칸 하나만큼 넓힘
    if (std::min(a, b) < min_side || std::max(a, b) > max_side) continue;
    const double yaw = wrap_yaw(deg2rad(rr.angle), PI / 2);
    if (!tall.empty()) {  // 사각형 안 높은 점이 많으면 상자 아님 (원통 방해물·다리)
      const double c = std::cos(-yaw), sn = std::sin(-yaw);
      int inside = 0;
      for (const auto & p : tall) {
        const double dx = p[0] - mx, dy = p[1] - my;
        const double lx = dx * c - dy * sn, ly = dx * sn + dy * c;
        inside += std::abs(lx) < a / 2 && std::abs(ly) < b / 2;
      }
      if (inside > 0.3 * cells.size()) continue;
    }
    std::vector<double> zs;
    for (const auto & p : top)
      if (std::abs(p[0] - mx) < a / 2 && std::abs(p[1] - my) < b / 2) zs.push_back(p[2]);
    BoxTop bt;
    bt.c = {mx, my};
    bt.a = std::max(a, b);
    bt.b = std::min(a, b);
    bt.yaw = yaw;
    bt.top_z = np_median(zs);
    // yaw는 minAreaRect 각도를 ±45°로 접은 것 — 90°의 홀수배만큼 접었으면 width·height가 yaw 기준으로 바뀜
    const double turns = std::round((deg2rad(rr.angle) - yaw) / (PI / 2));
    const bool swap = static_cast<long>(std::abs(turns)) % 2 == 1;
    bt.ax = swap ? b : a;
    bt.ay = swap ? a : b;
    {  // 이 덩어리(칸 번호 k)의 깊이 점 — 로봇 쪽 면까지 거리용
      for (size_t i = 0; i < top.size(); ++i)
        if (lab.at<int>(ij[i].y, ij[i].x) == k) bt.pts.emplace_back(top[i][0], top[i][1]);
    }
    out.push_back(bt);
  }
  return out;
}

std::optional<double> box_face_dist(const BoxTop & box, const cv::Vec2d & robot_xy, double robot_yaw, double front_x,
                                    double half_band, double q, int min_pts) {
  const double c = std::cos(-robot_yaw), sn = std::sin(-robot_yaw);
  std::vector<double> xs;
  for (const auto & p : box.pts) {
    const double dx = p[0] - robot_xy[0], dy = p[1] - robot_xy[1];
    const double fx = dx * c - dy * sn, fy = dx * sn + dy * c;  // 로봇 기준 (x 앞 · y 왼쪽)
    if (fx > 0.0 && std::abs(fy) < half_band) xs.push_back(fx);
  }
  if (static_cast<int>(xs.size()) < min_pts) return std::nullopt;
  std::sort(xs.begin(), xs.end());
  const double near = xs[static_cast<size_t>(q * static_cast<double>(xs.size() - 1))];  // 맨 앞 몇 점은 깊이 잡음일 수 있어 분위
  return near - front_x;
}

std::array<cv::Vec2d, 4> box_corners(const cv::Vec2d & c, double yaw, double ax, double ay) {
  const double co = std::cos(yaw), sn = std::sin(yaw);
  const double ds[4][2] = {{ax, ay}, {-ax, ay}, {-ax, -ay}, {ax, -ay}};
  std::array<cv::Vec2d, 4> out;
  for (int i = 0; i < 4; ++i)
    out[i] = {c[0] + (ds[i][0] * co - ds[i][1] * sn) / 2, c[1] + (ds[i][0] * sn + ds[i][1] * co) / 2};
  return out;
}

std::array<cv::Vec3d, 4> box_faces(const cv::Vec2d & c, double yaw, double ax, double ay) {
  std::array<cv::Vec3d, 4> out;
  const double half[4] = {ax / 2, ay / 2, ax / 2, ay / 2};
  for (int i = 0; i < 4; ++i) {
    const double n = wrap_yaw(yaw + i * PI / 2, 2 * PI);  // 바깥 방향: yaw, yaw+90°, yaw+180°, yaw+270°
    out[i] = {c[0] + half[i] * std::cos(n), c[1] + half[i] * std::sin(n), n};
  }
  return out;
}

bool tag_on_box(const cv::Vec2d & tag_xy, const BoxTop & box, double margin) {
  const cv::Vec2d d = tag_xy - box.c;
  if (!box.yaw) return std::hypot(d[0], d[1]) < box.a / 2 * std::sqrt(2.0) + margin;  // 잘린 상자 — 대각선 반 + margin
  const double co = std::cos(-*box.yaw), sn = std::sin(-*box.yaw);
  const double lx = d[0] * co - d[1] * sn, ly = d[0] * sn + d[1] * co;
  return std::abs(lx) < box.a / 2 + margin && std::abs(ly) < box.a / 2 + margin;  // 긴 변 기준 (넉넉하게)
}

bool box_cut(const BoxTop & box, const cv::Matx33d & rot, const cv::Vec3d & trans, double fx, double fy, double cx,
             double cy, int w, int h, double edge_px) {
  const double yaw = box.yaw.value_or(0.0);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double la = box.a, lb = box.b;
  const double ds[4][2] = {{la, lb}, {-la, lb}, {-la, -lb}, {la, -lb}};
  for (const auto & d : ds) {
    const cv::Vec3d p(box.c[0] + (d[0] * c - d[1] * s) / 2, box.c[1] + (d[0] * s + d[1] * c) / 2, box.top_z);
    const cv::Vec3d q = rot.t() * (p - trans);  // 추적 좌표 → 카메라
    if (q[2] < 0.05) return true;
    const double u = fx * q[0] / q[2] + cx, v = fy * q[1] / q[2] + cy;
    if (!(edge_px <= u && u <= w - 1 - edge_px && edge_px <= v && v <= h - 1 - edge_px)) return true;
  }
  return false;
}

// ---------- 퍽 판정
std::tuple<bool, std::string, std::string> judge_puck(const JudgeParams & jp, double wcm, double hcm, const std::string & color,
                                                      bool clipped, const std::string & model_color, bool near) {
  if (color == "unknown") return {false, "색 없음", ""};
  if (std::find(jp.puck_colors.begin(), jp.puck_colors.end(), color) == jp.puck_colors.end()) return {false, "색", ""};
  if (jp.check_model_color && !model_color.empty() && model_color != color) return {false, "색 불일치", ""};
  if (wcm <= 0 || hcm <= 0) {
    if (clipped || near) return {true, "", ""};  // D435 최소 거리보다 가까움 — 퍽은 맞음, 거리·자세 모름
    return {false, "depth 없음", ""};
  }
  const double long_cm = std::max(wcm, hcm), short_cm = std::min(wcm, hcm);
  if (clipped) {
    if (long_cm > jp.long_max) return {false, "거리 이상", ""};
    return {true, "", ""};
  }
  if (!(jp.long_min <= long_cm && long_cm <= jp.long_max) || short_cm < jp.short_min) return {false, "크기", ""};
  const double ratio = hcm / wcm;
  if (long_cm < jp.square_long && jp.square_ratio.first <= ratio && ratio <= jp.square_ratio.second) return {true, "", ""};
  return {true, "", ratio >= jp.stand_ratio ? "standing" : "lying"};
}

}  // namespace perception
