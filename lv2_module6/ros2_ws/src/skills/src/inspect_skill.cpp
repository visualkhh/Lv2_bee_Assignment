// skill_inspect — /skills/inspect: goal.pose(map) 점 위 inspect_height_m에 손끝을 두고 inspect_wait_s 기다림 (camera_look_down)
//   퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘. values[0] = 손 방향 yaw (로봇 → 점, gripper_cam이 그 점을 화면 가운데에) — 없으면 자유
//   읽었는지(details tag)는 coordinator · planner가 /tracked_objects로 봄. 학습 정책 없음 (기준선 100 %)
#include <cmath>
#include <limits>

#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"

namespace skills {

class InspectSkill : public SkillNode {
public:
  explicit InspectSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_inspect", options), arm_(*this) {
    height_m_ = declare_parameter<double>("inspect_height_m", 0.15);  // [m] 손끝을 퍽 윗면 위 이만큼 (16mm 태그 ≈ 50px)
    wait_s_ = declare_parameter<double>("inspect_wait_s", 1.0);       // [s] perception 추론 4Hz × 몇 장
    serve("inspect", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return arm_.moveit_ready(); }, "arm_unavailable", done, [this, g, done]() {
        auto at = g.pose;
        if (at.header.frame_id.empty()) at.header.frame_id = "map";
        at.header.stamp = now();
        at.pose.position.z += height_m_;
        at.pose.orientation = geometry_msgs::msg::Quaternion();
        const double yaw = g.values.empty() ? std::numeric_limits<double>::quiet_NaN() : g.values[0];
        stage("위에서 내려다보기");
        arm_.move_arm(at, [this, done](bool ok) {
          if (!ok) { done(false, "inspect_failed"); return; }
          stage("기다림");
          arm_.wait_step(wait_s_)([done](bool) { done(true, ""); });
        }, yaw);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  ArmIO arm_;
  double height_m_, wait_s_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::InspectSkill)
