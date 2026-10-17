#pragma once

// grasp_scorer 대역 (faker, ml:=fake 또는 grasp_scorer:=fake) — 도커·CI에는 GPU·학습 모델이 없어 같은 노드 이름·토픽으로 대신 돈다.
//   📞 /score_grasps (ml/srv/ScoreGrasps) — 대상 물체 위치에 고정 후보 3개 (규칙 점수). 진짜: ml/grasp_scorer
// /diagnostics에 "대역(fake)"으로 상태를 낸다.
#include <algorithm>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "ml/srv/score_grasps.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace faker {

class FakeGraspScorer : public rclcpp::Node {
public:
  FakeGraspScorer();

private:
  void score(const ml::srv::ScoreGrasps::Request & req, ml::srv::ScoreGrasps::Response & res);

  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;
  size_t requests_ = 0;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Service<ml::srv::ScoreGrasps>::SharedPtr srv_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace faker
