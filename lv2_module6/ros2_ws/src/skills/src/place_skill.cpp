// skill_place — /skills/place: 손에 든 퍽 내려놓기. goal.pose = 놓을 면 (map, z = 면 높이 — 손 안 퍽 바닥이 여기 닿음)
//   target "cell" (주사위 칸 · 좌표 place):
//     contact_mode sim: 칸 위 5 cm까지 MoveIt → 닿는 구간 (place_cell 정책: 내려가기 → 벌림 → 올리기) — 손 안 퍽 바닥이 면에 닿는 자리에서 벌림 (시뮬 place_m 0)
//     moveit: 위 place_approach_m → 퍽 바닥 + place_release_m까지 내려가 벌림 → 다시 위 (예전)
//   target "over" (물체 위 — location_box · 판): 면 + held_bottom + place_release 위로 values[0](다가가는 높이)만큼 위에서 MoveIt → 벌림
//   정책: model/skills/place_cell/ (팀원 pip_env)
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"

namespace skills {

class PlaceSkill : public SkillNode {
public:
  explicit PlaceSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_place", options), arm_(*this) {
    held_bottom_m_ = declare_parameter<double>("held_bottom_m", 0.0606);      // 손끝 → 손 안 퍽 바닥
    release_m_ = declare_parameter<double>("place_release_m", 0.02);          // 벌리는 높이 (퍽 바닥이 면 위 이만큼)
    approach_m_ = declare_parameter<double>("place_approach_m", 0.08);        // moveit 방식: 위에서 내려오는 높이
    policy_ = load_model("place_cell");
    serve("place", [this](const SkillGoal & g, Done done) {
      if (g.target != "cell" && g.target != "over") { done(false, "place_bad_target"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done, [this, g, done]() {
        arm_.clear_flags();
        arm_.reset_sag();
        auto release = g.pose;  // 벌릴 손끝 자리
        if (release.header.frame_id.empty()) release.header.frame_id = "map";
        release.header.stamp = now();
        release.pose.orientation = geometry_msgs::msg::Quaternion();
        release.pose.position.z += held_bottom_m_ + release_m_;
        const auto fin = [this, done](bool ok) { done(ok, ok ? "" : arm_.contact() ? "arm_contact" : "place_failed"); };
        if (g.target == "over") {
          auto above = release;
          above.pose.position.z += g.values.empty() ? 0.0 : g.values[0];
          stage("물체 위");
          arm_.move_arm(above, [this, fin](bool ok) {
            if (!ok) { fin(false); return; }
            arm_.set_gripper(GRIPPER_OPEN_M, fin);
          });
          return;
        }
        place_cell(release, g.pose.pose.position.z, fin);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void place_cell(const geometry_msgs::msg::PoseStamped & release, double top, ArmIO::Cb done) {
    if (arm_.sim_contact()) {
      auto down = release;
      down.pose.position.z -= release_m_;  // 퍽 바닥이 면에 닿는 자리
      auto above = down;
      above.pose.position.z += CONTACT_ABOVE_M;
      arm_.hand_target("cell", -1, -1, down.pose.position.x, down.pose.position.y, top);
      stage("칸 위");
      arm_.move_arm(above, [this, down, done](bool ok) {
        if (!ok) { done(false); return; }
        arm_.contact_step(policy_, [this, down](ArmIO::Q5 & a, ArmIO::Q5 & d) { return arm_.arm_now(a) && arm_.ik_ee(down, a, d); },
                          false)(done);
      });
      return;
    }
    auto above = release;
    above.pose.position.z += approach_m_;
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      [this, above](ArmIO::Cb c) { arm_.move_arm(above, c); },     // 위
      [this, release](ArmIO::Cb c) { arm_.move_arm(release, c); }, // 내려가기
      arm_.gripper_step(GRIPPER_OPEN_M),                           // 벌리기
      [this, above](ArmIO::Cb c) { arm_.move_arm(above, c); }});   // 다시 위 (놓은 퍽을 건드리지 않게)
    arm_.run_steps(steps, 0, done);
  }

  ArmIO arm_;
  SkillModel policy_;
  double held_bottom_m_, release_m_, approach_m_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PlaceSkill)
