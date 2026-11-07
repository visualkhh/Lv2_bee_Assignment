// skill 노드 공통 틀 — 설명은 include/skills/skill_node.hpp
#include "skills/skill_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace skills {

SkillNode::SkillNode(const std::string & name, const rclcpp::NodeOptions & options)
: rclcpp::Node(name, options) {
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  model_dir_ = declare_parameter<std::string>("model_dir", "");  // 비면 share/skills/model
  if (model_dir_.empty()) model_dir_ = ament_index_cpp::get_package_share_directory("skills") + "/model";
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (active_) s.summary(DiagnosticStatus::OK, "실행 중: " + active_action_ + " · " + stage_);
    else s.summary(DiagnosticStatus::OK, "대기");
    s.add("실행 · 실패", std::to_string(runs_) + " · " + std::to_string(fails_));
    s.add("마지막", last_.empty() ? "-" : last_);
    for (const auto & m : models_)
      s.add("정책 " + m.name, m.use ? m.layout + " " + m.meta.at("sim_policy") + " %"
                                    : "안 씀 (" + (m.meta.count("use") ? "use " + m.meta.at("use") : "폴더 없음") + ")");
  });
}

void SkillNode::serve(const std::string & action, std::function<void(const SkillGoal &, Done)> start) {
  servers_.push_back(rclcpp_action::create_server<SkillAction>(
    this, "/skills/" + action,
    [this, action](const rclcpp_action::GoalUUID &, std::shared_ptr<const SkillGoal>) {
      if (active_) {
        RCLCPP_WARN(get_logger(), "%s 거절 — %s 실행 중 (coordinator는 한 번에 하나만 보냄)", action.c_str(), active_action_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<Handle> h) {
      if (h != active_) return rclcpp_action::CancelResponse::REJECT;
      // 취소 상태로 바뀐 뒤에 마무리 (이 콜백 안에서는 canceled()를 못 부름)
      cancel_timer_ = create_wall_timer(std::chrono::milliseconds(1), [this, h]() {
        cancel_timer_->cancel();
        if (h != active_) return;
        stop();
        ++gen_;
        if (wait_timer_) wait_timer_->cancel();
        auto r = std::make_shared<SkillAction::Result>();
        r->reason = "cancelled";
        h->canceled(r);
        last_ = active_action_ + " 취소";
        active_.reset();
        RCLCPP_WARN(get_logger(), "%s 취소됨", active_action_.c_str());
      });
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this, action, start](const std::shared_ptr<Handle> h) {
      active_ = h;
      active_action_ = action;
      stage_ = "시작";
      ++runs_;
      const uint64_t gen = ++gen_;
      start(*h->get_goal(), [this, gen](bool ok, const std::string & reason) { finish(gen, ok, reason); });
    }));
}

void SkillNode::finish(uint64_t gen, bool ok, const std::string & reason) {
  if (gen != gen_ || !active_) return;  // 취소된 뒤 늦게 온 끝
  if (wait_timer_) wait_timer_->cancel();
  auto r = std::make_shared<SkillAction::Result>();
  r->success = ok;
  r->reason = reason;
  if (ok) active_->succeed(r);
  else { active_->abort(r); ++fails_; }
  last_ = active_action_ + (ok ? " 성공" : " 실패 " + reason);
  RCLCPP_INFO(get_logger(), "%s", last_.c_str());
  active_.reset();
}

void SkillNode::stage(const std::string & text) {
  stage_ = text;
  RCLCPP_INFO(get_logger(), "%s: %s", active_action_.c_str(), text.c_str());
  if (!active_) return;
  auto f = std::make_shared<SkillAction::Feedback>();
  f->stage = text;
  active_->publish_feedback(f);
}

void SkillNode::when_ready(std::function<bool()> ready, const std::string & reason, Done done, std::function<void()> go) {
  if (ready()) { go(); return; }
  const auto t0 = now();
  const uint64_t gen = gen_;
  stage("서버 대기 (" + reason + ")");
  wait_timer_ = create_wall_timer(std::chrono::seconds(1), [this, ready, reason, done, go, t0, gen]() {
    if (gen != gen_) { wait_timer_->cancel(); return; }
    if (ready()) { wait_timer_->cancel(); go(); return; }
    if ((now() - t0).seconds() >= server_wait_s_) { wait_timer_->cancel(); done(false, reason); }
  });
}

SkillModel SkillNode::load_model(const std::string & skill) {
  std::string why;
  auto m = load_skill_model(model_dir_, skill, why);
  if (!m.meta.count("sim_policy")) m.meta["sim_policy"] = "?";
  if (m.use) RCLCPP_INFO(get_logger(), "정책 %s: %s (%s · 시뮬 %s %%)", skill.c_str(), m.policy.c_str(), m.layout.c_str(),
                         m.meta["sim_policy"].c_str());
  else RCLCPP_WARN(get_logger(), "정책 %s 안 씀 — %s (기준 동작만)", skill.c_str(), why.c_str());
  models_.push_back(m);
  return m;
}

}  // namespace skills
