// skill_navigate — /skills/navigate: goal.pose(map)로 Nav2 NavigateToPose (지도 위 먼 이동, 장애물 피함)
//   목표 자세 계산(물체 앞 · 좌표)은 coordinator가 함. 학습 정책 없음 (Nav2 그대로)
//   진척 감시: 남은 경로 거리가 nav_stall_s 동안 nav_stall_gain_m만큼도 안 줄면 취소하고 nav_no_progress (ProgressWatch)
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/progress_watch.hpp"
#include "skills/skill_node.hpp"

#include <memory>

namespace skills {

class NavigateSkill : public SkillNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  explicit NavigateSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_navigate", options) {
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    stall_s_ = declare_parameter<double>("nav_stall_s", 25.0);       // 0이면 감시 안 함
    stall_gain_m_ = declare_parameter<double>("nav_stall_gain_m", 0.10);
    serve("navigate", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return nav_->action_server_is_ready(); }, "nav_unavailable", done, [this, g, done]() {
        NavigateToPose::Goal goal;
        goal.pose = g.pose;
        if (goal.pose.header.frame_id.empty()) goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        stage("Nav2 (" + std::to_string(g.pose.pose.position.x).substr(0, 5) + ", " +
              std::to_string(g.pose.pose.position.y).substr(0, 5) + ")");
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
        auto finished = std::make_shared<bool>(false);  // 감시가 먼저 끝내면 취소 뒤 오는 결과는 무시
        auto watch = std::make_shared<ProgressWatch>(stall_s_, stall_gain_m_);
        if (stall_s_ > 0.0) {
          opt.feedback_callback = [this, finished, watch, done](
              rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
            if (*finished || !watch->update(now().seconds(), fb->distance_remaining)) return;
            *finished = true;
            RCLCPP_WARN(get_logger(), "진척 없음 — 남은 %.2f m가 %.0f s 동안 %.2f m도 안 줄음 (회복 %d회) → 취소",
                        fb->distance_remaining, stall_s_, stall_gain_m_, fb->number_of_recoveries);
            nav_->async_cancel_goal(h);
            done(false, "nav_no_progress");
          };
        }
        opt.result_callback = [finished, done](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
          if (*finished) return;
          *finished = true;
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
  double stall_s_ = 25.0, stall_gain_m_ = 0.10;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::NavigateSkill)
