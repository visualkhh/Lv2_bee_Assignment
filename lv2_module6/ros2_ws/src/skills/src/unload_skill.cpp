// skill_unload — /skills/unload: 적재함 칸의 퍽을 꺼내 손에 (goal.slot · goal.shape · goal.tag — 칸 고르기·장부는 coordinator)
//   sim: carry → 벌림 → 칸 위(q_over) → 닿는 구간 (unload_slot 정책: q_grasp → 쥐기 → 잡기 판정 → 칸 위) → carry
//   moveit: carry → 벌림 → q_over → q_grasp → 닫기 → 잡기 판정 → q_over → carry (예전)
//   빈손이면 reason grasp_empty (coordinator가 그 칸을 '확인 필요'로)
//   정책: model/skills/unload_slot/ (slot_env.UnloadEnv)
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/basket_table.hpp"
#include "skills/skill_node.hpp"

namespace skills {

class UnloadSkill : public SkillNode {
public:
  explicit UnloadSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_unload", options), arm_(*this) {
    table_.declare(*this);
    policy_ = load_model("unload_slot");
    serve("unload", [this](const SkillGoal & g, Done done) {
      const std::string shape = g.shape.empty() ? "box" : g.shape;
      const auto over = table_.slot_q(shape, "q_over", g.slot), grasp = table_.slot_q(shape, "q_grasp", g.slot);
      const auto carry = table_.carry_q(g.slot);
      if (over.empty() || grasp.empty() || carry.empty()) { done(false, "basket_table_missing"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, shape, over, grasp, carry, done]() {
        arm_.clear_flags();
        arm_.reset_sag();
        const auto q5 = [](const std::vector<double> & v) { ArmIO::Q5 q{}; std::copy_n(v.begin(), 5, q.begin()); return q; };
        const auto a = q5(over), d = q5(grasp);
        const int tag = g.tag;
        auto steps = std::make_shared<std::vector<ArmIO::Step>>(arm_.sim_contact()
          ? std::vector<ArmIO::Step>{
            arm_.joints_step(carry), arm_.gripper_step(GRIPPER_OPEN_M), arm_.joints_step(over),
            [this, tag](ArmIO::Cb c) { arm_.hand_target("puck", tag, -1); c(true); },
            arm_.contact_step(policy_, [a, d](ArmIO::Q5 & x, ArmIO::Q5 & y) { x = a; y = d; return true; }, true),
            arm_.joints_step(carry)}
          : std::vector<ArmIO::Step>{
            arm_.joints_step(carry), arm_.gripper_step(GRIPPER_OPEN_M), arm_.joints_step(over), arm_.joints_step(grasp),
            arm_.gripper_step(GRIPPER_CLOSED_M, arm_.grip_current_a()), arm_.held_step(), arm_.joints_step(over),
            arm_.joints_step(carry)});
        stage("칸 " + std::to_string(g.slot) + " (" + shape + " · carry_" + table_.carry[g.slot] + ")");
        arm_.run_steps(steps, 0, [this, done](bool ok) {
          done(ok, ok ? "" : arm_.grasp_empty() ? "grasp_empty" : arm_.contact() ? "arm_contact" : "unload_failed");
        });
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

RCLCPP_COMPONENTS_REGISTER_NODE(skills::UnloadSkill)
