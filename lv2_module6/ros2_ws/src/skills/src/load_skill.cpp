// skill_load — /skills/load: 손에 든 퍽을 적재함 칸에 넣기 (goal.slot = 칸 · goal.shape = 퍽 모양 box|cyl — 칸 고르기·장부는 coordinator)
//   sim: carry → 칸 위(q_over, MoveIt) → 닿는 구간 (load_slot 정책: q_over → q_low → 벌림 → q_over) → stow
//   moveit: carry → q_over → q_low → 벌림 → q_lift → stow (예전)
//   정책: model/skills/load_slot/ (slot_env.LoadEnv — 손 카메라 대신 적재함 칸 − 손 안 퍽, hand_relative slot)
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/basket_table.hpp"
#include "skills/skill_node.hpp"

namespace skills {

class LoadSkill : public SkillNode {
public:
  explicit LoadSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_load", options), arm_(*this) {
    table_.declare(*this);
    policy_ = load_model("load_slot");
    serve("load", [this](const SkillGoal & g, Done done) {
      const std::string shape = g.shape.empty() ? "box" : g.shape;  // 모름 — 사각 표 (손 방향을 칸 벽에 맞춘 자세라 원기둥도 들어감)
      const auto over = table_.slot_q(shape, "q_over", g.slot), low = table_.slot_q(shape, "q_low", g.slot);
      const auto carry = table_.carry_q(g.slot), stow = table_.poses["stow"];
      if (over.empty() || low.empty() || carry.empty() || stow.empty()) { done(false, "basket_table_missing"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, shape, over, low, carry, stow, done]() {
        arm_.clear_flags();
        arm_.reset_sag();
        const auto q5 = [](const std::vector<double> & v) { ArmIO::Q5 q{}; std::copy_n(v.begin(), 5, q.begin()); return q; };
        const auto a = q5(over), d = q5(low);
        const int slot = g.slot;
        auto steps = std::make_shared<std::vector<ArmIO::Step>>(arm_.sim_contact()
          ? std::vector<ArmIO::Step>{
            arm_.joints_step(carry), arm_.joints_step(over),
            [this, slot](ArmIO::Cb c) { arm_.hand_target("slot", -1, slot); c(true); },
            arm_.contact_step(policy_, [a, d](ArmIO::Q5 & x, ArmIO::Q5 & y) { x = a; y = d; return true; }, false),
            arm_.joints_step(stow)}
          : std::vector<ArmIO::Step>{
            arm_.joints_step(carry), arm_.joints_step(over), arm_.joints_step(low), arm_.gripper_step(GRIPPER_OPEN_M),
            arm_.joints_step(table_.slot_q(shape, "q_lift", slot)), arm_.joints_step(stow)});
        stage("칸 " + std::to_string(slot) + " (" + shape + " · carry_" + table_.carry[slot] + ")");
        arm_.run_steps(steps, 0, [this, done](bool ok) { done(ok, ok ? "" : arm_.contact() ? "arm_contact" : "load_failed"); });
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  ArmIO arm_;
  BasketTable table_;
  SkillModel policy_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::LoadSkill)
