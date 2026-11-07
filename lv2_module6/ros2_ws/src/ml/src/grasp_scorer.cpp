// grasp_scorer 구현 — 노드 설명·입출력·멤버 선언은 include/ml/grasp_scorer.hpp, main은 grasp_scorer_main.cpp
#include "ml/grasp_scorer.hpp"

namespace ml {

namespace {
// (−π, π]
double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }
// a를 기준 ref에서 [−period/2, period/2) 안으로 — 대칭 모양은 period만큼 돌려도 같은 자세 (demo_cycle pick()의 remainder)
double nearest(double a, double ref, double period) {
  double d = std::fmod(a - ref + period / 2, period);
  if (d < 0) d += period;
  return ref + d - period / 2;
}
}  // namespace

bool puck_yaw(const TrackedObject & o, double & yaw, double & period) {
  const std::string p = detail(o, "yaw_period");  // perception은 방향을 알 때만 이 칸을 넣는다 (모르면 단위 쿼터니언)
  if (p.empty()) return false;
  period = std::stod(p) * M_PI / 180.0;
  if (period <= 0) return false;
  const auto & q = o.pose.pose.orientation;  // z축 회전만 — (0, 0, sin(yaw/2), cos(yaw/2))
  yaw = 2.0 * std::atan2(q.z, q.w);
  return true;
}

bool is_lying(const TrackedObject & o) {
  if (o.class_name == "puck_lying") return true;
  return detail(o, "posture") == "lying";
}

std::string detail(const TrackedObject & o, const std::string & key) {
  for (const auto & kv : o.details) if (kv.key == key) return kv.value;
  return "";
}

std::vector<GraspCandidate> rule_grasps(const TrackedObject & target, const std::vector<TrackedObject> & objects,
                                        std::string & note) {
  // 옆 퍽 — 손가락이 칠 수 있는 가장 가까운 다른 퍽 (NEIGHBOR_M 안)
  const auto & tp = target.pose.pose.position;
  const TrackedObject * nb = nullptr;
  double best = NEIGHBOR_M;
  for (const auto & o : objects) {
    if (o.id == target.id || o.class_name.rfind("puck", 0) != 0) continue;
    const double d = std::hypot(o.pose.pose.position.x - tp.x, o.pose.pose.position.y - tp.y);
    if (d < best) { best = d; nb = &o; }
  }
  const double line = nb ? std::atan2(nb->pose.pose.position.y - tp.y, nb->pose.pose.position.x - tp.x) : 0.0;
  // 닫힘축 = ψ − 90° → 옆 퍽 선에 직각이려면 ψ = line + π (손가락이 옆 퍽 쪽으로 벌어지지 않음)
  const double want = wrap_pi(line + M_PI);
  double yaw = 0.0, period = 0.0;
  const bool yaw_known = puck_yaw(target, yaw, period);

  if (is_lying(target)) {
    // 누운 퍽은 위에서 집지 않는다 (손가락이 상자에 닿음) — coordinator stand가 먼 끝을 살짝 잡아 세운 뒤 pick
    note = "lying puck: use stand";
    return {};
  }

  const bool box = detail(target, "shape") == "box";
  if (box && yaw_known) {
    // 사각기둥 면 법선 = yaw + k·90° → 닫힘축(ψ − 90°)이 면 법선과 같으려면 ψ ≡ yaw (mod 90°)
    if (nb) {
      note = "box face (neighbour)";
      return {{wrap_pi(nearest(yaw, want, M_PI / 2)), 0.9f}};
    }
    const double psi = nearest(yaw, 0.0, M_PI / 2);
    note = "box face";
    return {{wrap_pi(psi), 0.6f}, {wrap_pi(psi + M_PI / 2), 0.5f}};
  }
  // 원기둥 · 모양이나 방향을 모르는 퍽 — 방향을 정할 근거는 옆 퍽뿐
  note = box ? "box yaw unknown" : "rule";
  if (nb) return {{want, box ? 0.7f : 0.9f}};
  return {{0.0, 0.5f}, {M_PI / 2, 0.4f}};
}

geometry_msgs::msg::Quaternion top_down(double psi) {
  // Rz(ψ)·Rx(π) — ψ = 0이면 coordinator move_arm의 (1, 0, 0, 0)과 같다
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(psi / 2);
  q.y = std::sin(psi / 2);
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

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
  const std::string id = std::to_string(req.object_id);
  const TrackedObject * target = nullptr;
  if (objects_) {
    for (const auto & o : objects_->objects) if (o.id == req.object_id) target = &o;
  }
  if (!target) {
    res.success = false;
    res.message = "object " + id + " not in /tracked_objects";
    last_result_ = "object " + id + " 대상 없음";
    return;
  }
  std::string note;
  const auto cands = rule_grasps(*target, objects_->objects, note);
  if (cands.empty()) {
    res.success = false;
    res.message = note;
    last_result_ = "object " + id + " 못 잡음 (" + note + ")";
    return;
  }
  const size_t n = std::min<size_t>(cands.size(), req.max_candidates ? req.max_candidates : cands.size());
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = objects_->header;
    p.pose = target->pose.pose;
    p.pose.orientation = top_down(cands[i].yaw);
    res.grasps.push_back(p);
    res.scores.push_back(cands[i].score);
  }
  res.success = true;
  res.message = "rule: " + note;  // TODO(송정혁): 학습 모델이 생기면 enabled일 때 모델 점수
  last_result_ = "object " + id + " → 후보 " + std::to_string(n) + " (" + note + ")";
  RCLCPP_INFO(get_logger(), "/score_grasps object=%u → %zu candidates (%s, yaw %.0f deg)", req.object_id, n,
              note.c_str(), cands[0].yaw * 180.0 / M_PI);
}

}  // namespace ml
