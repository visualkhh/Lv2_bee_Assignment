#pragma once

// grasp_scorer (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/11-learn.md
//   입력: /tracked_objects (대상 퍽 위치·크기), /gripper_cam/depth/color/points (근접 점구름, pick 단계에서만)
//   출력: 📞 /score_grasps 서버 (ml/srv/ScoreGrasps) ← coordinator가 pick 직전 1회 호출
// 지금은 규칙: 서 있는 퍽 = 위에서 수직, 손가락 닫힘축을 옆 퍽과 잇는 선에 직각 (Isaac Lab pick_env와 같은 규칙).
//   옆 퍽이 없으면 직교하는 두 yaw 후보. 누운 퍽은 축 방향을 몰라 success=false.
//   도달 가능성은 검사하지 않는다 — coordinator가 플랜 실패 시 다음 후보로 넘어간다.
// TODO(송정혁): 학습 모델이 생기면 enabled일 때 모델 점수 (enabled:=false = 이 규칙)
#include <algorithm>
#include <cmath>
#include <map>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "ml/srv/score_grasps.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ml {

using ml::srv::ScoreGrasps;
using perception::msg::TrackedObjectArray;
using sensor_msgs::msg::PointCloud2;

class GraspScorer : public rclcpp::Node {
public:
  GraspScorer();

private:
  void score(const ScoreGrasps::Request & req, ScoreGrasps::Response & res);
  static bool is_lying(const perception::msg::TrackedObject & o);
  static geometry_msgs::msg::Quaternion top_down(double psi);  // 손가락이 바닥을 보는 방향, 수직축 yaw ψ

  static constexpr double NEIGHBOR_M = 0.15;  // 이 거리 안의 다른 퍽 = 손가락이 칠 수 있는 옆 퍽 (시작 박스 간격 80 mm)

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
