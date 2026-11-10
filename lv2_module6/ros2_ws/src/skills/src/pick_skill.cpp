// skill_pick — /skills/pick: 서 있는 퍽 집기 (goal.target = 물체 id, goal.tag = 퍽 태그 · goal.objects = 지금 안 보이는 퍽이면 coordinator가 기억한 대상·옆 퍽)
//   octomap 지우기 → 장면 등록 → 📞 /score_grasps → 후보를 점수순으로 max_grasp_tries개까지:
//     contact_mode sim: 후보 손 방향 ψ로 푼 관절값으로 퍽 위 5 cm까지 MoveIt (pregrasp — 기준선으로 충분, 정책 없음) → 벌림 → 닿는 구간 (pick_standing_puck 정책: 내려가기 → 쥐기 → 잡기 판정 → 올리기)
//     moveit: 접근 → 벌림 → 파지 위치 → 닫기(grip_current_a) → 잡기 판정 → 들어올리기 (예전)
//   실패 이유: no_grasp · arm_contact (과전류 접촉) · grasp_empty (빈손) · grasp_failed_all
//   정책: models/skills/pick_standing_puck/ (팀원 pick_env)
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
    // 팔이 뒤(적재함 쪽, |joint1| > 1.2 rad — load_end_pose over)에 있으면 그쪽 carry를 멈추지 않고 지나 퍽 위로 (stand::sample_via)
    //   바로 MoveIt으로 가면 손이 차체 위를 낮게 지나감 (11/07 Isaac). carry 값은 basket.yaml (skills.launch가 넘김)
    via_carry_ = declare_parameter<bool>("pick_via_carry", true);
    // 닿는 구간 전 손가락 사이 [m] — 시뮬 pick_env open_gap_mm 40 (80 mm 옆 퍽을 안 치게). 30 mm 퍽이면 한쪽 5 mm 여유뿐이라
    //   실기 인식 오차(~10 mm)면 손가락이 퍽 윗면을 침 (11/08). 실행 중 바꿈: ros2 param set /skill_pick pick_open_gap_m 0.060 (최대 ~0.070)
    declare_parameter<double>("pick_open_gap_m", OPEN_GAP_M);
    // 닿는 구간 시작 높이 — 쥐는 손끝 위 [m] (시뮬 above_m 0.05). 손가락 끝 ↔ 퍽 윗면 = 이 값 − 20 mm.
    //   손을 수직으로 세운 채 0.05면 로봇 중심에서 x ≈ 256 mm(y ±55)까지, 0.04면 264 mm (11/08 IK 표). ros2 param set /skill_pick pick_above_m 0.04
    declare_parameter<double>("pick_above_m", CONTACT_ABOVE_M);
    carry_left_ = declare_parameter<std::vector<double>>("pose_carry_left", std::vector<double>{});
    carry_right_ = declare_parameter<std::vector<double>>("pose_carry_right", std::vector<double>{});
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
      }, "pick_unavailable", done, [this, id, tag = g.tag, objects = g.objects, done]() {
        start(static_cast<uint32_t>(id), tag, objects, done);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  // objects: coordinator가 기억한 대상(첫째)·옆 퍽 — 대상이 지금 /tracked_objects에 없을 때 (비면 grasp_scorer가 지금 인지에서 찾음)
  void start(uint32_t id, int tag, const std::vector<perception::msg::TrackedObject> & objects, Done done) {
    arm_.clear_flags();
    // 처짐 적분은 여기서 0으로 안 함 — 닿는 구간 시작(ctx phase 0, 새 run_id)에서 rl_arm_correction이 0으로 되돌림 (시뮬 에피소드와 같음).
    //   여기서도 되돌리면 팔이 처졌다 다시 올라올 때까지(~0.45 s) 한 번 더 멈춤 (11/08 실기 pick → load 사이). 앞 skill의 Δ가 빠지는 것만 기다림
    arm_.settle_next();
    stage("파지 후보");
    clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
      [this, id, tag, objects, done](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
        auto scene = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        scene->scene.is_diff = true;  // TODO: 퍽·주변 장애물 CollisionObject 등록
        apply_scene_->async_send_request(scene, [this, id, tag, objects, done](rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedFuture) {
          auto req = std::make_shared<ScoreGrasps::Request>();
          req->object_id = id;
          req->objects = objects;
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
    if (arm_.sim_contact()) {  // 퍽 위 pick_above_m(기본 5 cm)까지 MoveIt (pregrasp) → 닿는 구간 (pick_standing_puck 정책)
      auto pre = g;
      pre.pose.position.z += std::clamp(get_parameter("pick_above_m").as_double(), 0.025, 0.08);  // 0.025 아래면 손가락이 퍽 윗면에 닿음
      const auto after_pre = [this, g, tag, done, open_then](bool ok) {
        if (!ok) { done(false); return; }
        const double open_j = std::min(stand::gap_joint(get_parameter("pick_open_gap_m").as_double()), GRIPPER_OPEN_M);
        arm_.set_gripper(open_j, [this, g, tag, done, open_then](bool ok2) {
          if (!ok2) { done(false); return; }
          arm_.hand_target("puck", tag, -1, g.pose.position.x, g.pose.position.y, g.pose.position.z);  // map 손끝 목표 → insp_rel
          arm_.contact_step(policy_, [this, g](ArmIO::Q5 & above, ArmIO::Q5 & down) {
            return arm_.arm_now(above) && arm_.ik_ee(g, above, down);
          }, true)(open_then);
        });
      };
      double psi;
      ArmIO::Q5 now, q_pre;
      if (!hand_yaw(g.pose.orientation, psi)) { arm_.move_arm(pre, after_pre); return; }  // 손 방향이 없는 후보 — 예전처럼 MoveIt에 맡김
      if (!arm_.arm_now(now) || !arm_.ik_ee_yaw(pre, psi, now, q_pre)) {  // 손을 아래로 향한 채 퍽 위 5 cm에 못 감 (멀거나 높음)
        RCLCPP_WARN(get_logger(), "pick: 퍽 위 %.0f cm 자세 IK 실패 — 손 방향 %.0f° (팔이 못 닿음)", 100 * (pre.pose.position.z - g.pose.position.z),
                    psi * 180 / M_PI);
        done(false);
        return;
      }
      const auto & carry = now[0] > 0 ? carry_left_ : carry_right_;
      if (via_carry_ && std::abs(now[0]) > 1.2 && carry.size() == 5) {  // 적재함 쪽에서 — carry를 지나 퍽 위로 (멈추지 않음)
        ArmIO::Q5 via{};
        std::copy_n(carry.begin(), 5, via.begin());
        arm_.traj_step([this, via, q_pre](const ArmIO::Q5 & from) {
          return stand::sample_via(from, via, q_pre, arm_.velocity_scale(), TRAJ_DT);
        })(after_pre);
        return;
      }
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
      arm_.gripper_step(arm_.grip_close_m(), arm_.grip_current_a()),                    // 닫기
      arm_.held_step(),                                                              // 잡기 판정
      [this, pre](ArmIO::Cb c) { arm_.move_arm(pre, c); }});                        // 들어올리기
    arm_.run_steps(steps, 0, open_then);
  }

  static constexpr double OPEN_GAP_M = 0.040;  // 시뮬 pick_env open_gap_mm — 80 mm 옆 퍽을 안 치게
  ArmIO arm_;
  SkillModel policy_;
  int max_tries_ = 2;
  bool via_carry_ = true;
  std::vector<double> carry_left_, carry_right_;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PickSkill)
