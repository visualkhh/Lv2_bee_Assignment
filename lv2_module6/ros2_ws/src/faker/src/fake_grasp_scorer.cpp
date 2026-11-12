// fake_grasp_scorer 구현 — 설명·선언은 include/faker/fake_grasp_scorer.hpp, main은 fake_grasp_scorer_main.cpp
#include "faker/fake_grasp_scorer.hpp"

namespace faker {

using diagnostic_msgs::msg::DiagnosticStatus;

FakeGraspScorer::FakeGraspScorer()
: rclcpp::Node("grasp_scorer") {
  objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { objects_ = m; });
  srv_ = create_service<ml::srv::ScoreGrasps>("/score_grasps",
    [this](const ml::srv::ScoreGrasps::Request::SharedPtr req, ml::srv::ScoreGrasps::Response::SharedPtr res) {
      score(*req, *res);
    });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(DiagnosticStatus::OK, "대역(fake) — 고정 후보");
    s.add("요청 수", requests_);
  });
}

void FakeGraspScorer::score(const ml::srv::ScoreGrasps::Request & req, ml::srv::ScoreGrasps::Response & res) {
  ++requests_;
  const perception::msg::TrackedObject * target = nullptr;
  if (objects_) {
    for (const auto & o : objects_->objects) if (o.id == req.object_id) target = &o;
  }
  if (!target) {
    res.success = false;
    res.message = "object " + std::to_string(req.object_id) + " not in /tracked_objects";
    return;
  }
  const float scores[] = {0.87f, 0.62f, 0.41f};
  const size_t n = std::min<size_t>(3, req.max_candidates ? req.max_candidates : 3);
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose = target->pose.pose;
    res.grasps.push_back(p);
    res.scores.push_back(scores[i]);
  }
  res.success = true;
  res.message = "fake";
}

}  // namespace faker
