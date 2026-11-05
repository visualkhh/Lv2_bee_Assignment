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
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "규칙");
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
  if (is_lying(*target)) {
    // 먼 끝 잡기(isaac/lab/right_env.py)는 퍽 축 방향이 있어야 하는데 perception은 방향을 추정하지 않는다 (orientation = 항등)
    res.success = false;
    res.message = "lying puck: axis yaw not estimated";
    last_result_ = "object " + std::to_string(req.object_id) + " 누움 — 축 방향 없음";
    return;
  }
  // 서 있는 퍽: 위에서 수직으로, 손가락 닫힘축을 옆 퍽과 잇는 선에 직각 (isaac/lab/pick_env.py finger rule perp).
  // 옆 퍽이 없으면 방향을 정할 근거가 없다 → 직교하는 두 방향을 차례로 (직육면체 퍽은 그중 하나가 면에 맞음)
  const auto & tp = target->pose.pose.position;
  const perception::msg::TrackedObject * nb = nullptr;
  double best = NEIGHBOR_M;
  for (const auto & o : objects_->objects) {
    if (o.id == target->id || o.class_name.rfind("puck", 0) != 0) continue;
    const double d = std::hypot(o.pose.pose.position.x - tp.x, o.pose.pose.position.y - tp.y);
    if (d < best) { best = d; nb = &o; }
  }
  const double line = nb ? std::atan2(nb->pose.pose.position.y - tp.y, nb->pose.pose.position.x - tp.x) : 0.0;
  // 닫힘축 = end_effector_link y. 손 방향 Rz(ψ)·Rx(π)에서 닫힘축은 ψ − 90° → 선에 직각이려면 ψ = line + π
  const std::vector<std::pair<double, float>> cands = nb
    ? std::vector<std::pair<double, float>>{{line + M_PI, 0.9f}}
    : std::vector<std::pair<double, float>>{{0.0, 0.5f}, {M_PI / 2, 0.4f}};
  const size_t n = std::min<size_t>(cands.size(), req.max_candidates ? req.max_candidates : cands.size());
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = objects_->header;
    p.pose = target->pose.pose;
    p.pose.orientation = top_down(cands[i].first);
    res.grasps.push_back(p);
    res.scores.push_back(cands[i].second);
  }
  res.success = true;
  last_result_ = "object " + std::to_string(req.object_id) + " → 후보 " + std::to_string(n) + (nb ? " (옆 퍽 기준)" : " (옆 퍽 없음)");
  res.message = "rule";  // TODO(송정혁): 학습 모델이 생기면 enabled일 때 모델 점수
  RCLCPP_INFO(get_logger(), "/score_grasps object=%u → %zu candidates%s", req.object_id, n, nb ? " (neighbour)" : "");
}

bool GraspScorer::is_lying(const perception::msg::TrackedObject & o) {
  if (o.class_name == "puck_lying") return true;
  for (const auto & kv : o.details) if (kv.key == "posture") return kv.value == "lying";
  return false;
}

geometry_msgs::msg::Quaternion GraspScorer::top_down(double psi) {
  // Rz(ψ)·Rx(π) — ψ = 0이면 coordinator move_arm의 (1, 0, 0, 0)과 같다
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(psi / 2);
  q.y = std::sin(psi / 2);
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

}  // namespace ml
