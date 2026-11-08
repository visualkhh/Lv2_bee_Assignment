// skill_unload — /skills/unload: 적재함 칸의 퍽을 꺼내 손에 (goal.slot · goal.shape · goal.tag — 칸 고르기·장부는 coordinator)
//   sim: carry → 벌림 → 칸 위(q_over) → 닿는 구간 (unload_slot 정책: q_grasp → 쥐기 → 잡기 판정 → 칸 위) → carry
//   moveit: carry → 벌림 → q_over → q_grasp → 닫기 → 잡기 판정 → q_over → carry (예전)
//   빈손이면 reason grasp_empty (coordinator가 그 칸을 '확인 필요'로)
//   정책: model/skills/unload_slot/ (slot_env.UnloadEnv)
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/basket_table.hpp"
#include "skills/skill_node.hpp"
#include "skills/stand_plan.hpp"

namespace skills {

// 칸에서 꺼낼 때 손가락 사이 (시뮬 UnloadEnv open_gap_range 45~50 mm 가운데) — 최대 53 mm면 칸 벽에 가까움
constexpr double UNLOAD_OPEN_GAP_M = 0.0475;

class UnloadSkill : public SkillNode {
public:
  explicit UnloadSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_unload", options), arm_(*this) {
    table_.declare(*this);
    // 끝 자세: carry(기본) · over(칸 위 그대로 — 다음 place가 carry를 멈추지 않고 지나 놓을 자리로)
    declare_parameter<std::string>("unload_end_pose", "carry");  // 실행 중 ros2 param set으로 바꿀 수 있음
    policy_ = load_model("unload_slot");
    serve("unload", [this](const SkillGoal & g, Done done) {
      const std::string shape = g.shape.empty() ? "box" : g.shape;
      const auto over = table_.slot_q(shape, "q_over", g.slot), grasp = table_.slot_q(shape, "q_grasp", g.slot);
      const auto carry = table_.carry_q(g.slot);
      if (over.empty() || grasp.empty() || carry.empty()) { done(false, "basket_table_missing"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, shape, over, grasp, carry, done]() {
        arm_.clear_flags();
        // 처짐 적분은 여기서 0으로 안 함 — 닿는 구간 시작(ctx phase 0, 새 run_id)에서 rl_arm_correction이 0으로 되돌림 (시뮬 에피소드와 같음).
        //   여기서도 되돌리면 팔이 처졌다 다시 올라올 때까지(~0.45 s) 한 번 더 멈춤 (11/08 실기 pick → load 사이). 앞 skill의 Δ가 빠지는 것만 기다림
        if (policy_.use) arm_.settle_next();  // 정책이 꺼져 있으면 Δ도 없음 — 바로 (place → unload 무정지)
        const auto q5 = [](const std::vector<double> & v) { ArmIO::Q5 q{}; std::copy_n(v.begin(), 5, q.begin()); return q; };
        const auto a = q5(over), d = q5(grasp);
        auto steps = std::make_shared<std::vector<ArmIO::Step>>(arm_.sim_contact()
          ? std::vector<ArmIO::Step>{
            // 지금 자세 → carry → 칸 위를 멈추지 않고 (load to_over와 같은 베지어). 손은 비어 있으니 벌리기는 이동과 같이
            // (벌림이 끝날 때까지 기다리면 place 뒤 제자리에서 한 번 멈춤 — 11/08 실기). 이동(~1.8 s)이 벌림(~0.5 s)보다 길어 칸 위에선 벌어져 있음
            [this](ArmIO::Cb c) { arm_.set_gripper(stand::gap_joint(UNLOAD_OPEN_GAP_M), [](bool) {}); c(true); },
            via_step(carry, over),
            // 칸 TF (slot_puck): 손 카메라는 못 씀 — perception이 적재함 안 퍽을 자기 몸으로 지움 (시뮬 UnloadEnv slot_tf)
            [this, slot = g.slot](ArmIO::Cb c) { arm_.hand_target("slot_puck", -1, slot); c(true); },
            arm_.contact_step(policy_, [a, d](ArmIO::Q5 & x, ArmIO::Q5 & y) { x = a; y = d; return true; }, true, GRIPPER_OPEN_M, true),
            get_parameter("unload_end_pose").as_string() == "over" ? ArmIO::Step([](ArmIO::Cb c) { c(true); }) : arm_.joints_step(carry)}
          : std::vector<ArmIO::Step>{
            arm_.joints_step(carry), arm_.gripper_step(GRIPPER_OPEN_M), arm_.joints_step(over), arm_.joints_step(grasp),
            arm_.gripper_step(arm_.grip_close_m(), arm_.grip_current_a()), arm_.held_step(), arm_.joints_step(over),
            arm_.joints_step(carry)});
        stage("칸 " + std::to_string(g.slot) + " (" + shape + " · carry_" + table_.carry[g.slot] + ")");
        arm_.run_steps(steps, 0, [this, done, carry](bool ok) {
          const std::string why = ok ? "" : arm_.grasp_empty() ? "grasp_empty" : arm_.contact() ? "arm_contact" : "unload_failed";
          if (ok || why != "grasp_empty") { done(ok, why); return; }
          arm_.joints_step(carry)([done, why](bool) { done(false, why); });  // 빈손: 칸 위에서 carry로 빠져나옴 (바로 stow면 차체 위로 휙)
        });
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  ArmIO::Step via_step(const std::vector<double> & carry, const std::vector<double> & over) {
    ArmIO::Q5 via{}, to{};
    std::copy_n(carry.begin(), 5, via.begin());
    std::copy_n(over.begin(), 5, to.begin());
    return arm_.traj_step([this, via, to](const ArmIO::Q5 & now) { return stand::sample_via(now, via, to, arm_.velocity_scale(), TRAJ_DT); });
  }

  ArmIO arm_;
  BasketTable table_;
  SkillModel policy_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::UnloadSkill)
