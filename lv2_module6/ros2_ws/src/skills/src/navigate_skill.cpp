// skill_navigate — /skills/navigate: goal.pose(map)로 Nav2 NavigateToPose (지도 위 먼 이동, 장애물 피함)
//   목표 자세 계산(물체 앞 · 좌표)은 coordinator가 함. 학습 정책 없음 (Nav2 그대로)
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/skill_node.hpp"

namespace skills {

class NavigateSkill : public SkillNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  explicit NavigateSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_navigate", options) {
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    serve("navigate", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return nav_->action_server_is_ready(); }, "nav_unavailable", done, [this, g, done]() {
        NavigateToPose::Goal goal;
        goal.pose = g.pose;
        if (goal.pose.header.frame_id.empty()) goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        stage("Nav2 (" + std::to_string(g.pose.pose.position.x).substr(0, 5) + ", " +
              std::to_string(g.pose.pose.position.y).substr(0, 5) + ")");
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
        opt.result_callback = [done](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
          const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
          done(ok, ok ? "" : "nav_failed");
        };
        nav_->async_send_goal(goal, opt);
      });
    });
  }

protected:
  void stop() override { nav_->async_cancel_all_goals(); }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::NavigateSkill)
