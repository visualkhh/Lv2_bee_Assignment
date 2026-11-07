#pragma once

// skill 노드 공통 틀 — 액션 서버 /skills/<이름> (skills/action/Skill) · 한 번에 goal 하나 · 취소 · 서버 대기 · 정책 폴더 · 진단
//   skill마다 이 클래스를 상속해 start(goal, done)만 채움. 노드 하나가 액션 여럿을 낼 수 있음 (stand: stand · stand_pick)
//   컴포넌트(rclcpp_components)라 skills.launch.py가 팔 skill은 컨테이너 하나(arm_skills), 주행 skill은 다른 하나(base_skills)에 올림
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plan/action/skill.hpp"
#include "skills/model.hpp"

namespace skills {

using SkillAction = plan::action::Skill;
using SkillGoal = SkillAction::Goal;
// 끝: ok · 실패 이유 (TaskStatus.reason으로 그대로 감)
using Done = std::function<void(bool, const std::string &)>;

class SkillNode : public rclcpp::Node {
public:
  SkillNode(const std::string & name, const rclcpp::NodeOptions & options);

protected:
  // /skills/<action> 서버 하나 — start는 비동기 (끝에 done 한 번)
  void serve(const std::string & action, std::function<void(const SkillGoal &, Done)> start);
  // 진행 중인 것을 멈춤 (MoveIt·그리퍼·Nav2 goal · 타이머 · 차체 정지). 취소·새 goal 전에 부름
  virtual void stop() {}
  void stage(const std::string & text);   // 피드백 + INFO 로그
  bool busy() const { return active_ != nullptr; }
  // 서버(MoveIt·Nav2 …)가 뜰 때까지 1초마다 다시 — server_wait_s 넘으면 done(false, reason)
  void when_ready(std::function<bool()> ready, const std::string & reason, Done done, std::function<void()> go);
  // model_dir/<skill>/ — 없거나 use: false면 m.use = false (로그에 이유)
  SkillModel load_model(const std::string & skill);
  diagnostic_updater::Updater diag_{this};
  std::vector<SkillModel> models_;        // 진단에 보임

private:
  using Handle = rclcpp_action::ServerGoalHandle<SkillAction>;
  std::vector<rclcpp_action::Server<SkillAction>::SharedPtr> servers_;
  std::shared_ptr<Handle> active_;
  std::string active_action_, stage_, last_;
  uint64_t gen_ = 0, runs_ = 0, fails_ = 0;
  double server_wait_s_ = 30.0;
  std::string model_dir_;
  rclcpp::TimerBase::SharedPtr wait_timer_, cancel_timer_;
  void finish(uint64_t gen, bool ok, const std::string & reason);
};

}  // namespace skills
