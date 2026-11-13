// skill_navigate — /skills/navigate: goal.pose(map)로 Nav2 NavigateToPose (지도 위 먼 이동, 장애물 피함)
//   목표 자세 계산(물체 앞 · 좌표)은 coordinator가 함. 학습 정책 없음 (Nav2 그대로)
//   진척 감시: 남은 경로 거리가 nav_stall_s 동안 nav_stall_gain_m만큼도 안 줄면 취소하고 nav_no_progress (ProgressWatch)
//   goal.target "look" (상자 도킹 앞 주행, 11-13): 남은 거리가 nav_look_m 안이면 팔을 pose_look으로 천천히 (approach가 바로 상자를 봄)
//     팔 동작은 도착을 기다리지 않음 — 주행이 끝나도 계속 올라가고, approach가 같은 자세를 다시 보냄
//   goal.target "near" (상자 앞 다가설 자리, 11-13): 멈추거나(진척 없음) Nav2가 실패해도 목표에서 nav_near_m 안이면 성공 —
//     벽 옆 자리에서 옆으로 10 cm 남기고 MPPI가 못 줄여 실패하던 것. 마지막 몇 cm·방향은 approach가 상자를 보며 맞춤
//   target은 공백으로 여러 개 ("near look")
#include <sstream>
#include "skills/arm_io.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/progress_watch.hpp"
#include "skills/skill_node.hpp"

#include <cmath>
#include <memory>
#include <set>
#include <vector>

namespace skills {

class NavigateSkill : public SkillNode {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  explicit NavigateSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_navigate", options), arm_(*this) {
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    stall_s_ = declare_parameter<double>("nav_stall_s", 25.0);       // 0이면 감시 안 함
    stall_gain_m_ = declare_parameter<double>("nav_stall_gain_m", 0.10);
    look_m_ = declare_parameter<double>("nav_look_m", 0.6);
    near_m_ = declare_parameter<double>("nav_near_m", 0.15);
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
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
        std::set<std::string> flags;
        std::istringstream in(g.target);
        for (std::string t; in >> t;) flags.insert(t);
        auto look = std::make_shared<bool>(flags.count("look") > 0);  // true = 아직 팔을 안 올림
        const bool near = flags.count("near") > 0;
        auto left = std::make_shared<double>(1e9);  // 목표까지 직선 거리 (마지막 피드백)
        const auto gp = goal.pose.pose.position;
        // near: 끝날 때 목표 근처면 성공으로 바꿈
        auto finish = [this, near, left, done](bool ok, const std::string & why) {
          if (!ok && near && *left <= near_m_) {
            RCLCPP_WARN(get_logger(), "%s — 목표 %.2f m 앞이라 성공 처리 (near, 나머지는 approach)", why.c_str(), *left);
            ok = true;
          }
          done(ok, ok ? "" : why);
        };
        if (stall_s_ > 0.0 || *look || near) {
          opt.feedback_callback = [this, finished, watch, look, left, gp, finish](
              rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
            if (*finished) return;
            *left = std::hypot(fb->current_pose.pose.position.x - gp.x, fb->current_pose.pose.position.y - gp.y);
            // 첫 피드백은 경로 전이라 0일 수 있음 → 0.01 m 넘을 때만
            if (*look && fb->distance_remaining > 0.01 && fb->distance_remaining < look_m_) {
              *look = false;
              if (arm_.moveit_ready())
                arm_.move_joints(look_q_, [this](bool ok) { if (!ok) RCLCPP_WARN(get_logger(), "주행 중 look 자세 실패"); });
              else RCLCPP_WARN(get_logger(), "MoveIt 없음 — 주행 중 look 자세 건너뜀");
            }
            if (stall_s_ <= 0.0 || !watch->update(now().seconds(), fb->distance_remaining)) return;
            *finished = true;
            RCLCPP_WARN(get_logger(), "진척 없음 — 남은 %.2f m가 %.0f s 동안 %.2f m도 안 줄음 (회복 %d회) → 취소",
                        fb->distance_remaining, stall_s_, stall_gain_m_, fb->number_of_recoveries);
            nav_->async_cancel_goal(h);
            finish(false, "nav_no_progress");
          };
        }
        opt.result_callback = [finished, finish](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
          if (*finished) return;
          *finished = true;
          const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
          finish(ok, ok ? "" : "nav_failed");
        };
        nav_->async_send_goal(goal, opt);
      });
    });
  }

protected:
  void stop() override {
    nav_->async_cancel_all_goals();
    arm_.stop();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  double stall_s_ = 25.0, stall_gain_m_ = 0.10, look_m_ = 0.6, near_m_ = 0.15;
  ArmIO arm_;
  std::vector<double> look_q_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::NavigateSkill)
