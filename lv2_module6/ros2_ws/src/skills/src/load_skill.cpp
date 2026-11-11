// skill_load — /skills/load: 손에 든 퍽을 적재함 칸에 넣기 (goal.slot = 칸 · goal.shape = 퍽 모양 box|cyl — 칸 고르기·장부는 coordinator)
//   sim: carry → 칸 위(q_over, MoveIt) → 닿는 구간 (load_slot 정책: q_over → q_low → 벌림 → q_over) → stow
//   moveit: carry → q_over → q_low → 벌림 → q_lift → stow (예전)
//   끝 자세 load_end_pose: stow(기본) · carry(다음 pick이 stow를 거치지 않음) · over(칸 위 그대로)
//   load_via_carry false: 집은 자리에서 칸 위(q_over)로 바로 (carry 안 거침) · carry_blend(기본): carry를 멈추지 않고 베지어로 지나감
//   정책: model/skills/load_slot/ (slot_env.LoadEnv — 손 카메라 대신 적재함 칸 − 손 안 퍽, hand_relative slot)
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/basket_table.hpp"
#include "skills/skill_node.hpp"

namespace skills {

// 놓을 때 gripper_left_joint — 손가락 간격 50 mm (퍽 면에서 한쪽 10 mm 여유, 손가락 = (50 − 20) / 2 mm).
// 펌웨어 그리퍼 표에서 38~75 mm 구간은 실측하지 않은 보간이라 실제 간격은 몇 mm 어긋날 수 있다.
constexpr double LOAD_OPEN_M = 0.015;

class LoadSkill : public SkillNode {
public:
  explicit LoadSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_load", options), arm_(*this) {
    table_.declare(*this);
    // 넣은 뒤 끝 자세: stow(기본) · carry(칸 쪽 옆 자세 — 다음 pick이 stow를 거치지 않고 바로 퍽으로) · over(칸 위 그대로)
    declare_parameter<std::string>("load_end_pose", "stow");  // 실행 중 ros2 param set으로 바꿀 수 있음
    // 집은 자리에서 carry를 거쳐 칸 위로(기본) · false면 칸 위(q_over)로 바로 — MoveIt이 팔만 피하고 손 안 퍽은 모름 (부딪힘 주의)
    declare_parameter<bool>("load_via_carry", true);
    // carry를 지나되 멈추지 않음 — 지금 자세 → carry → 칸 위를 관절 공간 베지어 하나로 (stand::sample_via). false면 carry에서 멈췄다 감 (MoveIt 두 번)
    declare_parameter<bool>("carry_blend", true);
    // 앞(박스)에서 시작하면 먼저 손을 곧게 이만큼 올린 뒤 carry로 — pick이 퍽 위 5 cm까지만 들어 손 안 퍽 바닥이 옆 퍽(60 mm) 윗면보다
    //   낮은 채 옆으로 쓸고 감 (11/08 실기 pick → load에서 옆 퍽을 침, unload → place와 같은 문제). 0 = 안 올림
    declare_parameter<double>("load_lift_m", 0.03);
    policy_ = load_model("load_slot");
    serve("load", [this](const SkillGoal & g, Done done) {
      const std::string shape = g.shape.empty() ? "box" : g.shape;  // 모름 — 사각 표 (손 방향을 칸 벽에 맞춘 자세라 원기둥도 들어감)
      const auto over = table_.slot_q(shape, "q_over", g.slot), low = table_.slot_q(shape, "q_low", g.slot);
      const auto carry = table_.carry_q(g.slot), stow = table_.poses["stow"];
      if (over.empty() || low.empty() || carry.empty() || stow.empty()) { done(false, "basket_table_missing"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, shape, over, low, carry, stow, done]() {
        arm_.clear_flags();
        // 처짐 적분은 여기서 0으로 안 함 — 닿는 구간 시작(ctx phase 0, 새 run_id)에서 rl_arm_correction이 0으로 되돌림 (시뮬 에피소드와 같음).
        //   여기서도 되돌리면 팔이 처졌다 다시 올라올 때까지(~0.45 s) 한 번 더 멈춤 (11/08 실기 pick → load 사이). 앞 skill의 Δ가 빠지는 것만 기다림
        arm_.settle_next();
        const auto q5 = [](const std::vector<double> & v) { ArmIO::Q5 q{}; std::copy_n(v.begin(), 5, q.begin()); return q; };
        const auto a = q5(over), d = q5(low);
        const int slot = g.slot;
        auto steps = std::make_shared<std::vector<ArmIO::Step>>(arm_.sim_contact()
          ? std::vector<ArmIO::Step>{
            to_over(carry, over),
            [this, slot](ArmIO::Cb c) { arm_.hand_target("slot", -1, slot); c(true); },
            arm_.contact_step(policy_, [a, d](ArmIO::Q5 & x, ArmIO::Q5 & y) { x = a; y = d; return true; }, false, LOAD_OPEN_M),
            end_step(stow, carry)}
          : std::vector<ArmIO::Step>{
            to_over(carry, over), arm_.joints_step(low), arm_.gripper_step(LOAD_OPEN_M),
            arm_.joints_step(table_.slot_q(shape, "q_lift", slot)), end_step(stow, carry)});
        stage("칸 " + std::to_string(slot) + " (" + shape + " · carry_" + table_.carry[slot] + ")");
        arm_.run_steps(steps, 0, [this, done](bool ok) { done(ok, ok ? "" : arm_.contact() ? "arm_contact" : "load_failed"); });
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  // 칸 위(q_over)까지: carry 경유(베지어로 멈추지 않음 · 또는 멈췄다 감) · load_via_carry false면 바로 (MoveIt)
  ArmIO::Step to_over(const std::vector<double> & carry, const std::vector<double> & over) {
    if (!get_parameter("load_via_carry").as_bool()) return arm_.joints_step(over);
    if (!get_parameter("carry_blend").as_bool()) {
      auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{arm_.joints_step(carry), arm_.joints_step(over)});
      return [this, steps](ArmIO::Cb c) { arm_.run_steps(steps, 0, c); };
    }
    ArmIO::Q5 via{}, to{};
    std::copy_n(carry.begin(), 5, via.begin());
    std::copy_n(over.begin(), 5, to.begin());
    const double lift = get_parameter("load_lift_m").as_double();
    return arm_.traj_step([this, via, to, lift](const ArmIO::Q5 & now) {
      const double vs = arm_.velocity_scale();
      stand::V3 axis{};
      const auto p = stand::fk(now, &axis);
      ArmIO::Q5 up{};
      // 이미 적재함 쪽(|joint1| > 1.2 rad)이면 올릴 필요 없음 · 못 올리면(IK) 예전처럼 바로
      if (lift <= 0.0 || std::abs(now[0]) > 1.2) return stand::sample_via(now, via, to, vs, TRAJ_DT);
      if (!stand::ik_down({p[0], p[1], p[2] + lift}, std::atan2(axis[1], axis[0]), now, up)) {
        RCLCPP_WARN(get_logger(), "load: 손 %.0f cm 올리기 IK 실패 — 바로 carry로 (옆 퍽을 칠 수 있음)", 100 * lift);
        return stand::sample_via(now, via, to, vs, TRAJ_DT);
      }
      // 궤적 하나: 곧게 위로 (손목 그대로) → 거기서 carry를 지나 칸 위로. 이음점에서 속도 0이지만 멈춰 기다리지는 않음
      auto out = stand::sample_move(now, up, vs, TRAJ_DT);
      const auto rest = stand::sample_via(up, via, to, vs, TRAJ_DT);
      out.insert(out.end(), rest.begin(), rest.end());
      return out;
    });
  }

  ArmIO::Step end_step(const std::vector<double> & stow, const std::vector<double> & carry) {
    const std::string e = get_parameter("load_end_pose").as_string();
    if (e == "over") return [](ArmIO::Cb c) { c(true); };
    return arm_.joints_step(e == "carry" ? carry : stow);
  }

  ArmIO arm_;
  BasketTable table_;
  SkillModel policy_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::LoadSkill)
