#pragma once

// grasp_scorer (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/11-learn.md
//   입력: /tracked_objects (대상 퍽 위치·크기), /gripper_cam/depth/color/points (근접 점구름, pick 단계에서만)
//   출력: 📞 /score_grasps 서버 (ml/srv/ScoreGrasps) ← coordinator가 pick 직전 1회 호출
// 지금은 골격: 대상 물체 위치 기준 고정 후보 3개를 점수순으로 돌려준다 (시나리오 1의 B 0.87, C 0.62, A 0.41).
// enabled:=false (또는 모델 없음) → 규칙 기반 점수 (학습 노드는 끌 수 있어야 한다 — 테스트베드·비교 기준).
// TODO(송정혁): 후보 생성 → IK 가능한 것만 → ONNX 분류 모델로 성공 확률 (CPU 추론으로도 돌게)
#include <algorithm>
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
