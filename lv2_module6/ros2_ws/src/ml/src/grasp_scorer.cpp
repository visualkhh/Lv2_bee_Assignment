// grasp_scorer 구현 — 노드 설명·입출력·멤버 선언은 include/ml/grasp_scorer.hpp, main은 grasp_scorer_main.cpp
#include "ml/grasp_scorer.hpp"

namespace ml {

GraspScorer::GraspScorer()
: rclcpp::Node("grasp_scorer") {
  declare_parameter<bool>("enabled", true);
  objects_sub_ = create_subscription<TrackedObjectArray>(
    "/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr msg) { objects_ = msg; });
  // 파지 점수는 최신 한 장면만 필요 — 큐에 밀린 점구름을 쓰지 않는다.
  cloud_sub_ = create_subscription<PointCloud2>(
    "/gripper_cam/depth/color/points", rclcpp::SensorDataQoS().keep_last(1),
    [this](PointCloud2::ConstSharedPtr msg) { cloud_ = msg; });
  srv_ = create_service<ScoreGrasps>("/score_grasps",
    [this](const ScoreGrasps::Request::SharedPtr req, ScoreGrasps::Response::SharedPtr res) { score(*req, *res); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    const bool enabled = get_parameter("enabled").as_bool();
    if (!objects_) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "/tracked_objects 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, enabled ? "모델" : "규칙 (enabled=false)");
    s.add("enabled", enabled);
    s.add("요청 수", requests_);
    s.add("마지막 결과", last_result_);
    s.add("gripper_cam 점구름", cloud_ ? "있음" : "없음");
  });
}

void GraspScorer::score(const ScoreGrasps::Request & req, ScoreGrasps::Response & res) {
  ++requests_;
  last_result_ = "object " + std::to_string(req.object_id) + " 대상 없음";
  const perception::msg::TrackedObject * target = nullptr;
  if (objects_) {
    for (const auto & o : objects_->objects) if (o.id == req.object_id) target = &o;
  }
  if (!target) {
    res.success = false;
    res.message = "object " + std::to_string(req.object_id) + " not in /tracked_objects";
    return;
  }
  if (!cloud_) RCLCPP_WARN(get_logger(), "gripper_cam 점구름 없음 — 위치만으로 채점");
  const std::vector<std::pair<const char *, float>> cands = {{"side", 0.87f}, {"oblique", 0.62f}, {"top", 0.41f}};
  const size_t n = std::min<size_t>(cands.size(), req.max_candidates ? req.max_candidates : cands.size());
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose = target->pose.pose;  // TODO: 후보별 접근 자세
    res.grasps.push_back(p);
    res.scores.push_back(cands[i].second);
  }
  res.success = true;
  last_result_ = "object " + std::to_string(req.object_id) + " → 후보 " + std::to_string(n);
  res.message = get_parameter("enabled").as_bool() ? "model (TODO)" : "rule";
  RCLCPP_INFO(get_logger(), "/score_grasps object=%u → %zu candidates", req.object_id, n);
}

}  // namespace ml
