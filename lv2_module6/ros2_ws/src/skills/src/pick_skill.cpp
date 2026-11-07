// skill_pick — /skills/pick: 서 있는 퍽 집기 (goal.target = 물체 id, goal.tag = 퍽 태그)
//   octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보를 점수순으로 max_grasp_tries개까지:
//     contact_mode sim: 후보 손 방향 ψ로 푼 관절값으로 퍽 위 5 cm까지 MoveIt (pregrasp — 기준선으로 충분, 정책 없음) → 벌림 → 닿는 구간 (pick_standing_puck 정책: 내려가기 → 쥐기 → 잡기 판정 → 올리기)
//     moveit: 접근 → 벌림 → 파지 위치 → 닫기(grip_current_a) → 잡기 판정 → 들어올리기 (예전)
//   실패 이유: no_grasp · arm_contact (과전류 접촉) · grasp_empty (빈손) · grasp_failed_all
//   정책: model/skills/pick_standing_puck/ (팀원 pick_env)
#include "ml/srv/score_grasps.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "std_srvs/srv/empty.hpp"

namespace skills {

class PickSkill : public SkillNode {
public:
  using ScoreGrasps = ml::srv::ScoreGrasps;
  explicit PickSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_pick", options), arm_(*this) {
    max_tries_ = declare_parameter<int>("max_grasp_tries", 2);  // 파지 후보를 점수순으로 몇 개까지
    policy_ = load_model("pick_standing_puck");
    score_ = create_client<ScoreGrasps>("/score_grasps");
    clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
    apply_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    serve("pick", [this](const SkillGoal & g, Done done) {
      if (g.target.empty() || g.target.find_first_not_of("0123456789") != std::string::npos) { done(false, "pick_no_target"); return; }
      const auto id = std::stoul(g.target);
      when_ready([this]() {
        return clear_octomap_->service_is_ready() && apply_scene_->service_is_ready() && score_->service_is_ready() &&
               arm_.moveit_ready() && arm_.gripper_ready();
      }, "pick_unavailable", done, [this, id, tag = g.tag, done]() { start(static_cast<uint32_t>(id), tag, done); });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void start(uint32_t id, int tag, Done done) {
    arm_.clear_flags();
    arm_.reset_sag();
    stage("파지 후보");
    clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
      [this, id, tag, done](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        auto scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        scene->scene.is_diff = true;  // TODO: 퍽·주변 장애물 CollisionObject 등록
        apply_scene_->async_send_request(scene, [this, id, tag, done](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture) {
          auto req = std::make_shared<ScoreGrasps::Request>();
          req->object_id = id;
          req->max_candidates = 3;
          score_->async_send_request(req, [this, tag, done](rclcpp::Client<ScoreGrasps>::SharedFuture f) {
            const auto res = f.get();
            if (!res->success || res->grasps.empty()) { done(false, "no_grasp"); return; }
            try_grasp(res, 0, tag, done);
          });
        });
      });
  }

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i, int tag, Done done) {
    if (i >= cands->grasps.size() || static_cast<int>(i) >= max_tries_) {
      done(false, arm_.contact() ? "arm_contact" : arm_.grasp_empty() ? "grasp_empty" : "grasp_failed_all");
      return;
    }
    stage("후보 " + std::to_string(i) + " (점수 " + std::to_string(cands->scores[i]).substr(0, 4) + ")");
    grasp(cands->grasps[i], tag, [this, cands, i, tag, done](bool ok) {
      if (ok) done(true, "");
      else try_grasp(cands, i + 1, tag, done);
    });
  }

  void grasp(geometry_msgs::msg::PoseStamped g, int tag, ArmIO::Cb done) {
    g.pose.position.z = stand::standing_grasp_z(g.pose.position.z);  // 시뮬 pick_env와 같은 높이
    const auto open_then = [this, done](bool ok) {
      if (ok) { done(true); return; }
      arm_.set_gripper(GRIPPER_OPEN_M, [done](bool) { done(false); });
    };
    if (arm_.sim_contact()) {  // 퍽 위 5 cm까지 MoveIt (pregrasp) → 닿는 구간 (pick_standing_puck 정책)
      auto pre = g;
      pre.pose.position.z += CONTACT_ABOVE_M;
      const auto after_pre = [this, g, tag, done, open_then](bool ok) {
        if (!ok) { done(false); return; }
        arm_.set_gripper(stand::gap_joint(OPEN_GAP_M), [this, g, tag, done, open_then](bool ok2) {
          if (!ok2) { done(false); return; }
          arm_.hand_target("puck", tag, -1);
          arm_.contact_step(policy_, [this, g](ArmIO::Q5 & above, ArmIO::Q5 & down) {
            return arm_.arm_now(above) && arm_.ik_ee(g, above, down);
          }, true)(open_then);
        });
      };
      double psi;
      ArmIO::Q5 now, q_pre;
      if (!hand_yaw(g.pose.orientation, psi)) { arm_.move_arm(pre, after_pre); return; }  // 손 방향이 없는 후보 — 예전처럼 MoveIt에 맡김
      if (!arm_.arm_now(now) || !arm_.ik_ee_yaw(pre, psi, now, q_pre)) { done(false); return; }
      arm_.move_joints(std::vector<double>(q_pre.begin(), q_pre.end()), after_pre);  // 내려가기는 이 손목 그대로 (ik_ee)
      return;
    }
    constexpr double APPROACH = 0.10;
    auto pre = g;
    pre.pose.position.z += APPROACH;
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      [this, pre](ArmIO::Cb c) { arm_.move_arm(pre, c); },                          // 접근
      arm_.gripper_step(GRIPPER_OPEN_M),                                             // 벌리기
      [this, g](ArmIO::Cb c) { arm_.move_arm(g, c); },                              // 파지 위치
      arm_.gripper_step(GRIPPER_CLOSED_M, arm_.grip_current_a()),                    // 닫기
      arm_.held_step(),                                                              // 잡기 판정
      [this, pre](ArmIO::Cb c) { arm_.move_arm(pre, c); }});                        // 들어올리기
    arm_.run_steps(steps, 0, open_then);
  }

  static constexpr double OPEN_GAP_M = 0.040;  // 시뮬 pick_env open_gap_mm — 80 mm 옆 퍽을 안 치게
  ArmIO arm_;
  SkillModel policy_;
  int max_tries_ = 2;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PickSkill)
