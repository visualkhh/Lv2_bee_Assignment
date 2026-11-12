#pragma once

// grasp_scorer (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/11-learn.md
//   입력: /tracked_objects (대상 퍽 위치·크기·방향), /gripper_cam/depth/color/points (근접 점구름, pick 단계에서만)
//   출력: 📞 /score_grasps 서버 (ml/srv/ScoreGrasps) ← coordinator가 pick 직전 1회 호출
// 지금은 규칙 (rule_grasps): 위에서 수직으로 잡고, 수직축 둘레 손 방향 ψ만 고른다 (Isaac Lab pick_env · demo_cycle pick()와 같은 규칙).
//   서 있는 퍽: 손가락 닫힘축을 옆 퍽과 잇는 선에 직각 (want). 사각기둥이고 방향을 알면 want에서 ±45° 안의 면에 맞춤.
//     옆 퍽이 없으면 직교하는 두 방향 — 사각기둥은 면 방향 두 개, 원기둥·방향 모름은 map 0°·90°.
//   누운 퍽: success=false ("lying puck: use stand") — coordinator stand가 먼 끝 위쪽 절반을 살짝 잡아 세운 뒤 pick
//   퍽 방향 = perception pose.orientation(z축 회전) · details yaw_period가 있을 때만 앎 (shape: box · cyl)
//   도달 가능성은 검사하지 않는다 — coordinator가 플랜 실패 시 다음 후보로 넘어간다.
// TODO(송정혁): 학습 모델이 생기면 enabled일 때 모델 점수 (enabled:=false = 이 규칙)
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "ml/srv/score_grasps.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ml {

using ml::srv::ScoreGrasps;
using perception::msg::TrackedObject;
using perception::msg::TrackedObjectArray;
using sensor_msgs::msg::PointCloud2;

constexpr double NEIGHBOR_M = 0.15;  // 이 거리 안의 다른 퍽 = 손가락이 칠 수 있는 옆 퍽 (시작 박스 간격 80 mm)

struct GraspCandidate {
  double yaw;   // 손 방향 ψ [rad, 물체 좌표계] — 손 = Rz(ψ)·Rx(π), 닫힘축(end_effector_link y) 방향 = ψ − 90°
  float score;  // 0~1, 높을수록 먼저
};

// 퍽 방향 (perception) — yaw_period가 details에 있을 때만 앎. period [rad]: 사각기둥 π/2 · 누운 퍽 π
bool puck_yaw(const TrackedObject & o, double & yaw, double & period);
bool is_lying(const TrackedObject & o);
std::string detail(const TrackedObject & o, const std::string & key);  // 없으면 ""
// 규칙 후보 (점수 높은 순) — 비면 못 잡음, note에 이유. objects = 같은 장면 전체 (옆 퍽 찾기)
std::vector<GraspCandidate> rule_grasps(const TrackedObject & target, const std::vector<TrackedObject> & objects,
                                        std::string & note);
geometry_msgs::msg::Quaternion top_down(double psi);  // 손가락이 바닥을 보는 방향, 수직축 yaw ψ

class GraspScorer : public rclcpp::Node {
public:
  GraspScorer();

private:
  void score(const ScoreGrasps::Request & req, ScoreGrasps::Response & res);

  TrackedObjectArray::ConstSharedPtr objects_;
  PointCloud2::ConstSharedPtr cloud_;
  size_t requests_ = 0;
  std::string last_result_ = "-";
  diagnostic_updater::Updater diag_{this};
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<ScoreGrasps>::SharedPtr srv_;
};

}  // namespace ml
