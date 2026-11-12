#pragma once

// perception_core (🛠 직접 작성, 담당: 전승혜) — 인지 노드(perception_node)의 계산 함수 (ROS 없음 · 테스트에서 바로 부름)
//   파이썬 노드 scripts/perception_node.py의 같은 이름 함수를 C++로 옮긴 것 — 규칙·숫자는 그대로.
//   좌표: "추적 좌표" = track_frame(odom) · 카메라 광학 좌표(x 오른쪽 · y 아래 · z 앞) · 화면 [px]
//   rot·trans = 카메라 광학 → 추적 좌표 (p_track = rot · p_cam + trans)
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace perception {

// 크기 추정 실패 시 클래스 기본값 (sx, sy, sz) [m] · 없으면 0.1×0.1×0.1
cv::Vec3d size_fallback(const std::string & cls);
constexpr double TRACK_TIMEOUT_S = 2.0;  // 이만큼 어느 카메라에도 안 보이면 트랙 삭제 [s] (모든 물체)
constexpr double ROT_VAR_UNKNOWN = 1e3;  // 방향을 모름 → pose.covariance 회전 대각
constexpr double BODY_MIN_Z = 0.05;      // 차체 판정은 이 높이 위만 [m]
double yaw_var_known();                  // yaw를 구했을 때 yaw 분산 [rad²] (σ 10°)
bool is_puck_class(const std::string & cls);  // puck · puck_lying
bool no_color(const std::string & c);         // 색을 못 정한 값 ('' · unknown)

// 화면 박스 (x0, y0, x1, y1) [px]
struct Box {
  int x0{0}, y0{0}, x1{0}, y1{0};
  bool operator==(const Box & o) const { return x0 == o.x0 && y0 == o.y0 && x1 == o.x1 && y1 == o.y1; }
};

// 검출 하나 (한 카메라 한 프레임) — 파이썬 dets의 dict
struct Det {
  std::string class_name;
  cv::Vec3d z;          // 위치 (추적 좌표) [m]
  cv::Matx33d R;        // 관측 공분산 (추적 좌표) [m²]
  cv::Vec3d size;       // [m]
  double conf{1.0};
  cv::Point px;         // 화면 위치 [px]
  std::string color, posture, shape, content;
  std::optional<int> tag;
  std::optional<double> yaw;
  double yaw_period{0.0};
  std::optional<Box> box;  // 영상 박스 (상자·장소 태그는 없음)
  // 상자: 잘림(화면 가장자리) · 로봇 쪽 면 — 잰 순간의 면 위 점(추적 좌표)과 로봇 앞 방향(단위 벡터). 발행할 때 지금 로봇 앞면까지 거리로 바꿈
  bool cut{false};
  std::optional<cv::Vec2d> face_pt, face_dir;
};

// 트랙 하나 — 파이썬 self.tracks[id]
struct Track {
  std::string class_name;  // 퍽 계열은 'puck' (자세는 posture)
  std::string posture, color, shape, content, camera, yaw_posture;
  cv::Vec3d p;
  cv::Matx33d P;
  double t{0.0};           // 마지막 갱신 영상 시각 [s]
  cv::Vec3d size;
  double conf{0.5};
  int32_t last_seen_sec{0};
  uint32_t last_seen_nsec{0};
  std::map<std::string, double> seen;  // 카메라 → 마지막 본 시각 [s]
  cv::Point px;
  std::optional<int> tag;
  std::optional<double> yaw;
  double yaw_period{0.0};
  std::optional<double> distance;  // 발행 순간 로봇까지 바닥 위 거리 [m]
  bool cut{false};                     // 상자: 마지막 관측이 화면에 잘림 → 가운데·크기·모서리를 믿지 말 것
  std::optional<cv::Vec2d> face_pt, face_dir;  // 상자: 로봇 쪽 면 (Det와 같음, 추적 좌표 — 발행 때 map으로)
  std::optional<double> face_dist;     // 발행 순간 로봇 앞면 → 상자 로봇 쪽 면 [m]
};

// ---------- 종류 · 같은 물체
std::string kind(const std::string & cls);
// 트랙(또는 이미 kind로 바꾼 쪽)과 검출이 같은 물체일 수 있나 — 종류 · 태그 · 바코드 내용 · (퍽은) 색
bool same_object(const std::string & tr_class, const std::optional<int> & tr_tag, const std::string & tr_content,
                 const std::string & tr_color, const std::string & d_class, const std::optional<int> & d_tag,
                 const std::string & d_content, const std::string & d_color);
bool same_object(const Track & tr, const Det & d);

cv::Vec3d puck_center(const cv::Vec3d & p, const cv::Vec3d & cam, double offset, double full_deg);
double box_iou(const Box & a, const Box & b);
std::vector<Det> merge_overlap_dets(const std::vector<Det> & dets, double iou_min);
std::map<int, int> merge_tracks(std::map<int, Track> & tracks, double dist_m);

// ---------- 자기 몸
struct Body {
  cv::Vec3d base;                // 차체 원점 (추적 좌표)
  std::vector<cv::Vec3d> chain;  // 팔 링크 원점들 (순서대로)
  cv::Matx33d rot;               // 차체 회전
};
double seg_dist(const cv::Vec3d & p, const cv::Vec3d & a, const cv::Vec3d & b);
bool is_self(const std::string & cls, const cv::Vec3d & p, const Body & body, double body_radius, double body_height,
             double self_radius, const std::vector<double> & body_box);
// 팔 체인을 화면에 투영 → [(u, v, r)] (r = 팔 두께의 픽셀 반경)
std::vector<cv::Vec3d> arm_pixels(const std::vector<cv::Vec3d> & chain, const cv::Matx33d & rot, const cv::Vec3d & trans,
                                  double fx, double fy, double cx, double cy, double radius = 0.04, int per_seg = 10);
bool box_has_arm(const Box & box, const std::vector<cv::Vec3d> & arm_px, int min_hits);

// ---------- 추적 수학
cv::Matx33d meas_cov_cam(double z, double fx, double fy, double pixel_sigma, double depth_sigma0, double depth_sigma_k);
// 칼만 갱신 → (새 p, 새 P, K)
std::tuple<cv::Vec3d, cv::Matx33d, cv::Matx33d> kf_update(const cv::Vec3d & p, const cv::Matx33d & P, const cv::Vec3d & z,
                                                          const cv::Matx33d & R);

// ---------- 색 (OpenCV H 0~180)
int hue_dist(int h, const std::string & name);
std::string classify_color(const std::string & cls, const cv::Mat & roi_bgr);
std::pair<std::string, int> puck_color(const cv::Mat & roi_bgr, const std::vector<std::string> & colors, int hue_tol);
// (색 또는 없음, 버린 이유)
std::pair<std::optional<std::string>, std::string> settle_puck_color(const cv::Mat & roi_bgr, const std::string & color,
                                                                     const std::string & model_color,
                                                                     const std::vector<std::string> & puck_colors,
                                                                     bool use_model_color = true);
bool is_hue_name(const std::string & name);

// ---------- 클래스 이름
std::string resolve_model_path(const std::string & model_path, const std::string & model_dir);
std::pair<std::string, std::string> split_class(const std::string & name);
std::string puck_shape(const std::string & name);
std::string model_posture(const std::string & name);
std::string tag_shape(const std::string & face, const std::string & posture);

// ---------- 깊이 · 자세
cv::Mat1b arm_near(const cv::Mat1f & patch, const cv::Mat1b & arm, double margin = 0.05);
cv::Mat1b arm_mask_image(const std::vector<cv::Vec3d> & arm_px, cv::Size shape);
double box_depth_m(const cv::Mat1f & depth_m, const Box & box, double min_valid, const cv::Mat1b & arm = cv::Mat1b());
// (자세 standing/lying/'', 받침 바닥 위 윗면 높이 또는 없음)
std::pair<std::string, std::optional<double>> posture_from_height(
  const cv::Mat1f & depth_m, const Box & box, double zm, const cv::Matx33d & rot, const cv::Vec3d & trans,
  double fx, double fy, double cx, double cy, double floor_z, std::pair<double, double> supports = {0.0, 0.07},
  double tol = 0.02, int min_pts = 15, double margin = 0.01);
bool view_from_side(const cv::Matx33d & rot, double max_down_deg = 35.0);

// ---------- AprilTag (OpenCV aruco tag36h11)
struct Tag {
  int id{0};
  cv::Point2d c;                    // 중심 [px]
  Box box;                          // (x0, y0, x1 + 1, y1 + 1)
  std::array<cv::Point2d, 4> corners;  // aruco 순서 (태그 기준 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래)
};
class TagDetector {
public:
  TagDetector();
  bool ok() const { return ok_; }
  // min_px: 태그 후보 최소 둘레 [px] (minMarkerPerimeterRate = min_px / 긴 변) · 없으면 예전 값 0.01
  void detect(const cv::Mat & gray, std::optional<double> min_px, std::vector<std::vector<cv::Point2f>> & corners,
              std::vector<int> & ids);

private:
  bool ok_{false};
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
std::vector<Tag> detect_tags(TagDetector * det, const cv::Mat & bgr, double gamma = 0.0, bool half = false,
                             std::optional<double> min_px = std::nullopt, std::optional<double> half_min_px = std::nullopt);
// AprilTag를 화면 일부(퍽·상자 YOLO 네모)에서만 찾기 — CPU 절약 (젯슨 11/12: 영상 전체 3번 검사에 60~78 ms)
//   merge_rois: 네모를 margin(변 비율) + pad_px만큼 넓히고 겹치는 것을 합침 · detect_tags_in: 네모마다 detect_tags, 화면 좌표로
std::vector<Box> merge_rois(std::vector<Box> rois, double margin, int pad_px, int w, int h);
std::vector<Tag> detect_tags_in(TagDetector * det, const cv::Mat & bgr, const std::vector<Box> & rois, double gamma = 0.0,
                                bool half = false, std::optional<double> min_px = std::nullopt,
                                std::optional<double> half_min_px = std::nullopt);
Box clip_box(double x0, double y0, double x1, double y1, int w, int h);
// 퍽 박스 안 퍽 태그 → (번호, 꼭짓점) · 없으면 없음
std::optional<std::pair<int, std::array<cv::Point2d, 4>>> tag_in_box(const std::vector<Tag> & tags, const Box & box,
                                                                    const std::set<int> & ids,
                                                                    const std::vector<Box> & others = {});

// ---------- 방향
double wrap_yaw(double yaw, double period);
std::pair<std::optional<double>, std::string> tag_yaw(const std::array<cv::Point2d, 4> & corners, double size_m, double fx,
                                                      double fy, double cx, double cy, const cv::Matx33d & rot);
std::optional<std::pair<cv::Vec3d, cv::Vec3d>> tag_pnp(const std::array<cv::Point2d, 4> & corners, double size_m, double fx,
                                                       double fy, double cx, double cy);
std::optional<cv::Vec3d> tag_center(const std::array<cv::Point2d, 4> & corners, const cv::Mat1f & depth_m, double size_m,
                                    double half_w, double fx, double fy, double cx, double cy, const cv::Matx33d & rot,
                                    const cv::Vec3d & trans, const cv::Mat1b & arm = cv::Mat1b());
std::optional<double> axis_yaw(const std::vector<cv::Vec2d> & pts, int min_points, double min_elong);

// ---------- 상자 (depth 윗면)
struct BoxTop {
  cv::Vec2d c;               // 윗면 가운데 xy (추적 좌표)
  double a{0.0}, b{0.0};     // 긴 변, 짧은 변 [m]
  std::optional<double> yaw; // 주기 90° · 잘렸으면 없음
  double top_z{0.0};
  double ax{0.0}, ay{0.0};   // yaw 방향 변 · 그에 수직인 변 [m] (붙인 상자처럼 직사각형일 때 모서리·면을 바르게)
  std::vector<cv::Vec2d> pts;  // 이 윗면에 든 깊이 점 xy (추적 좌표) — 잘려도 로봇 쪽 면까지 거리를 잴 때 (box_face_dist)
};
std::vector<BoxTop> find_box_tops(const cv::Mat1f & depth_m, const cv::Matx33d & rot, const cv::Vec3d & trans, double fx,
                                  double fy, double cx, double cy, double floor_z, std::pair<double, double> h_range,
                                  double min_side, double max_side, double cell, int step = 4, double max_range = 3.0,
                                  const std::optional<std::pair<cv::Vec2d, double>> & avoid = std::nullopt,
                                  const std::optional<std::vector<std::array<double, 4>>> & rois = std::nullopt,
                                  const std::vector<cv::Vec3d> & mask_px = {});
bool tag_on_box(const cv::Vec2d & tag_xy, const BoxTop & box, double margin);
// 로봇 앞면 → 상자의 로봇 쪽 면 거리 [m] — 윗면 깊이 점 중 로봇 앞 띠(|옆| < half_band) 안에서 가장 가까운 쪽(하위 q 분위)
//   잘려서 가운데·크기를 못 믿어도, 로봇 쪽 모서리가 보이는 동안은 잼. robot_xy·robot_yaw = base_footprint (추적 좌표)
//   front_x = base_footprint → 로봇 앞면 [m] · 띠 안 점이 min_pts보다 적으면 없음
std::optional<double> box_face_dist(const BoxTop & box, const cv::Vec2d & robot_xy, double robot_yaw, double front_x,
                                    double half_band = 0.08, double q = 0.05, int min_pts = 8);
// 상자 네 모서리 · 네 면 (가운데 xy, 바깥 방향 yaw) — 가운데 c, yaw 방향 변 ax · 수직 변 ay
std::array<cv::Vec2d, 4> box_corners(const cv::Vec2d & c, double yaw, double ax, double ay);
std::array<cv::Vec3d, 4> box_faces(const cv::Vec2d & c, double yaw, double ax, double ay);
bool box_cut(const BoxTop & box, const cv::Matx33d & rot, const cv::Vec3d & trans, double fx, double fy, double cx,
             double cy, int w, int h, double edge_px);

// ---------- 퍽 판정 (크기·색·잘림)
struct JudgeParams {
  std::vector<std::string> puck_colors{"red", "green"};
  bool check_model_color{true};
  double long_min{2.5}, long_max{10.0}, short_min{2.2}, square_long{4.5};
  std::pair<double, double> square_ratio{0.8, 1.25};
  double stand_ratio{0.8};
};
// (퍽인가, 아니면 이유, 자세)
std::tuple<bool, std::string, std::string> judge_puck(const JudgeParams & jp, double wcm, double hcm, const std::string & color,
                                                      bool clipped = false, const std::string & model_color = "",
                                                      bool near = false);

// ---------- numpy와 같은 통계 (테스트에서도 씀)
double np_median(std::vector<double> v);
double np_percentile(std::vector<double> v, double q);  // numpy 기본(linear)

}  // namespace perception
