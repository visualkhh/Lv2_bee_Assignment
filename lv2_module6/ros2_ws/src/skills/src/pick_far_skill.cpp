// skill_pick_far — /skills/pick_far: 박스 먼 쪽 서 있는 퍽 집기 (goal은 /skills/pick과 같음: target = 물체 id · tag = 퍽 태그 · objects)
//   /skills/pick이 pick_unreachable(손을 수직으로 퍽 위 5 cm에 못 둠)로 끝난 퍽을 손을 45° 기울여 밀고 들어가 집음.
//   시뮬 isaac/rl/bee_isaac/tasks/pick_env.py 먼 퍽 모드(train.py --task pick_far)와 같은 기준 동작 + 학습 보정(pick_standing_far 정책):
//     📞 /score_grasps (손 방향 후보) → 후보마다 far::plan (먼저 다 풀어 보고 안 닿으면 다음 후보)
//     → 뒤로 뺀 자세까지 MoveIt (관절 목표, 적재함 쪽이면 carry 경유) → 손가락 66 mm 벌림
//     → 닿는 구간 (단계·시간 = 시뮬): 0 멈춤 0.3 s · 1 밀고 들어가기 → 세우기 → 손목 돌리기 (경로 하나, 팀 한계 0.3배)
//       · 2 안정 0.3 s · 3 닫기 0.5 s (+ 손가락 멈출 때까지) · 잡기 판정 · 4 들어올리기 · 5 끝
//   정책 입력: 위 자세 = 뒤로 뺀 자세, 아래 자세 = 쥐는 자세 (시뮬 q_above · q_down과 같음), layout pick_far (ml/arm_obs.hpp)
//   실패 이유: no_grasp · pick_far_unreachable (후보 전부 경로를 못 풂) · pick_far_jump · arm_contact · grasp_empty · grasp_failed_all
//   정책: models/skills/pick_standing_far/ (내 pick_far_y.pt, 시뮬 99.2%)
#include "ml/srv/score_grasps.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/far_plan.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace skills {

namespace {
// 시뮬 PickEnvCfg와 같은 시간 (pick_env.py hold_s · settle_s · release_s(닫기)) — arm_io contact_step과 같은 값
constexpr double HOLD_S = 0.3, SETTLE_S = 0.3, CLOSE_S = 0.5, STOP_S = 3.0;
}  // namespace

class PickFarSkill : public SkillNode {
public:
  using ScoreGrasps = ml::srv::ScoreGrasps;
  explicit PickFarSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_pick_far", options), arm_(*this) {
    max_tries_ = declare_parameter<int>("max_grasp_tries", 2);  // 파지 후보를 점수순으로 몇 개까지
    via_carry_ = declare_parameter<bool>("pick_via_carry", true);  // 적재함 쪽에서 오면 carry 경유 (pick과 같음)
    // 닿는 구간 전 손가락 사이 [m] — 시뮬 far_open_gap_mm 66 (기울어 들어가며 사각 퍽 옆 여유 ±12 mm). 최대 ~0.070
    declare_parameter<double>("far_open_gap_m", 0.066);
    // 시뮬 far_* 값 — 바꾸면 정책을 학습한 조건과 달라짐 (실기 조정용으로만)
    fp_.tilt_deg = declare_parameter<double>("far_tilt_deg", fp_.tilt_deg);
    fp_.back_m = declare_parameter<double>("far_back_m", fp_.back_m);
    fp_.lift_in_max_m = declare_parameter<double>("far_lift_in_max_m", fp_.lift_in_max_m);
    // 닿는 구간(밀고 들어가기 ~ 들어올리기) 속도 = 팀 관절 한계 × 이 값 (시뮬 vel_scale). 정책(models/skills/pick_standing_far)을
    //   학습한 속도 범위 0.5~0.7 안에서만 바꿀 것 (0.3은 학습 안 함 → 시뮬 92 %)
    // 퍽 거리로 속도를 고름 (11/13 시뮬, 박스 ±45°): 70 %는 퍽 x ≤ 270 mm에서 99.0 %지만 그보다 먼 박스 끝(팔 거의 다 뻗음)에서
    //   95.7 %로 떨어짐 · 50 %는 어디서든 99.3 % → 가까우면 빠르게, 박스 끝만 천천히
    contact_vs_ = declare_parameter<double>("far_contact_vel_scale", 0.7);            // 퍽 x ≤ far_slow_x_m
    contact_vs_far_ = declare_parameter<double>("far_contact_vel_scale_far", 0.5);    // 퍽 x > far_slow_x_m (박스 끝)
    slow_x_m_ = declare_parameter<double>("far_slow_x_m", 0.27);                      // 퍽 바닥 가운데 x [m, base_footprint]
    carry_left_ = declare_parameter<std::vector<double>>("pose_carry_left", std::vector<double>{});
    carry_right_ = declare_parameter<std::vector<double>>("pose_carry_right", std::vector<double>{});
    policy_ = load_model("pick_standing_far");
    score_ = create_client<ScoreGrasps>("/score_grasps");
    serve("pick_far", [this](const SkillGoal & g, Done done) {
      if (g.target.empty() || g.target.find_first_not_of("0123456789") != std::string::npos) { done(false, "pick_no_target"); return; }
      const auto id = std::stoul(g.target);
      when_ready([this]() { return score_->service_is_ready() && arm_.moveit_ready() && arm_.gripper_ready(); },
                 "pick_unavailable", done, [this, id, tag = g.tag, objects = g.objects, done]() {
        start(static_cast<uint32_t>(id), tag, objects, done);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void start(uint32_t id, int tag, const std::vector<perception::msg::TrackedObject> & objects, Done done) {
    arm_.clear_flags();
    why_.clear();
    arm_.settle_next();  // 앞 skill의 보정 Δ가 빠지는 동안 기다린 뒤 움직임 (pick과 같음)
    stage("파지 후보");
    auto req = std::make_shared<ScoreGrasps::Request>();
    req->object_id = id;
    req->objects = objects;
    req->max_candidates = 3;
    score_->async_send_request(req, [this, tag, done](rclcpp::Client<ScoreGrasps>::SharedFuture f) {
      const auto res = f.get();
      if (!res->success || res->grasps.empty()) { done(false, "no_grasp"); return; }
      by_wrist_turn(*res);
      try_grasp(res, 0, tag, done);
    });
  }

  // 후보를 손목이 덜 도는 순으로 — grasp_scorer는 손 방향을 map 기준(0° · 90°)으로 골라 로봇 방향에 따라 마지막 손목 돌리기가
  //   90° 가까이 됐음 (11/13 실기: 비뚤어진 퍽에서 손목이 크게 돎). 먼 퍽은 손목을 곧게 두고 들어가 세운 뒤 마지막에만 돌림
  //   (far_plan) → 그 회전(세운 자세 → 쥐는 자세의 joint5 차)이 작은 후보부터. 경로를 못 푸는 후보는 뒤로 (순서만 바꿈)
  void by_wrist_turn(ScoreGrasps::Response & res) {
    std::vector<std::pair<double, size_t>> key;
    for (size_t i = 0; i < res.grasps.size(); ++i) {
      far::Plan pl;
      double vs = 0.0;
      const bool ok = make_plan(res.grasps[i], pl, vs) && pl.path.size() >= 2;
      key.emplace_back(ok ? std::abs(pl.path.back()[4] - pl.path[pl.path.size() - 2][4]) : 1e9, i);
    }
    std::stable_sort(key.begin(), key.end());
    ScoreGrasps::Response sorted = res;
    for (size_t k = 0; k < key.size(); ++k) {
      sorted.grasps[k] = res.grasps[key[k].second];
      sorted.scores[k] = res.scores[key[k].second];
    }
    res = sorted;
    why_.clear();  // 미리 풀어 본 실패 이유는 지움 (실제 시도에서 다시 정함)
  }

  void try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i, int tag, Done done) {
    if (i >= cands->grasps.size() || static_cast<int>(i) >= max_tries_) {
      done(false, arm_.contact() ? "arm_contact" : arm_.grasp_empty() ? "grasp_empty" : !why_.empty() ? why_ : "grasp_failed_all");
      return;
    }
    stage("후보 " + std::to_string(i) + " (점수 " + std::to_string(cands->scores[i]).substr(0, 4) + ")");
    grasp(cands->grasps[i], tag, [this, cands, i, tag, done](bool ok) {
      if (ok) done(true, "");
      else try_grasp(cands, i + 1, tag, done);
    });
  }

  // 후보 하나: map 쥘 손끝 g · 손 방향 ψ → base_footprint 퍽 바닥 · 닫는 축 → far::plan
  bool make_plan(const geometry_msgs::msg::PoseStamped & g, far::Plan & out, double & vs) {
    double psi;
    if (!hand_yaw(g.pose.orientation, psi)) { why_ = "pick_far_no_yaw"; return false; }  // 손 방향 없는 후보는 못 씀
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = arm_.tf().lookupTransform("base_footprint", g.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &) {
      why_ = "pick_far_tf";
      return false;
    }
    const auto & r = tf.transform.rotation;
    const auto & tr = tf.transform.translation;
    const tf2::Transform t(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(tr.x, tr.y, tr.z));
    const double z = stand::standing_grasp_z(g.pose.position.z);  // 쥘 손끝 높이 (pick과 같은 규칙)
    const tf2::Vector3 p = t * tf2::Vector3(g.pose.position.x, g.pose.position.y, z - stand::GRASP_Z);  // 퍽 바닥 가운데
    const double base_yaw = std::atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z));
    const double close_yaw = psi + base_yaw - M_PI / 2;  // ArmIO::ik_ee_yaw와 같은 변환 (닫는 축 = ψ − 90°)
    std::string why;
    if (!far::plan({p.x(), p.y(), p.z()}, close_yaw, fp_, out, why)) {
      RCLCPP_WARN(get_logger(), "pick_far: 경로 못 풂 (%s) — 퍽 바닥 base (%.3f, %.3f, %.3f)", why.c_str(), p.x(), p.y(), p.z());
      why_ = why;
      return false;
    }
    vs = p.x() > slow_x_m_ ? contact_vs_far_ : contact_vs_;  // 박스 끝 퍽만 천천히 (위 파라미터 설명)
    RCLCPP_INFO(get_logger(), "pick_far: 퍽 x %.0f mm → 닿는 구간 속도 %.0f %%", 1000 * p.x(), 100 * vs);
    return true;
  }

  void grasp(const geometry_msgs::msg::PoseStamped & g, int tag, ArmIO::Cb done) {
    auto plan = std::make_shared<far::Plan>();
    double vs = contact_vs_;
    if (!make_plan(g, *plan, vs)) { done(false); return; }
    // 퍽 쥘 손끝 (map) — hand_relative가 손 기준 퍽 위치(insp_rel)를 재게 (pick과 같음)
    const double gx = g.pose.position.x, gy = g.pose.position.y, gz = stand::standing_grasp_z(g.pose.position.z);
    const auto open_fail = [this, done](bool ok) {
      if (ok) { done(true); return; }
      arm_.set_gripper(GRIPPER_OPEN_M, [done](bool) { done(false); });  // 실패하면 벌리고 다음 후보
    };
    const auto after_retreat = [this, plan, vs, tag, gx, gy, gz, open_fail](bool ok) {
      if (!ok) { open_fail(false); return; }
      const double open_j = std::min(stand::gap_joint(get_parameter("far_open_gap_m").as_double()), GRIPPER_OPEN_M);
      arm_.set_gripper(open_j, [this, plan, vs, tag, gx, gy, gz, open_fail](bool ok2) {
        if (!ok2) { open_fail(false); return; }
        arm_.hand_target("puck", tag, -1, gx, gy, gz);
        contact(plan, vs, open_fail);
      });
    };
    // 뒤로 뺀 자세까지 — 적재함 쪽(|joint1| > 1.2)이면 carry를 지나 (pick과 같은 이유: 손이 차체 위를 낮게 지나지 않게)
    ArmIO::Q5 now;
    const Q5v retreat(plan->retreat().begin(), plan->retreat().end());
    if (via_carry_ && arm_.arm_now(now) && std::abs(now[0]) > 1.2) {
      const auto & carry = now[0] > 0 ? carry_left_ : carry_right_;
      if (carry.size() == 5) {
        ArmIO::Q5 via{};
        std::copy_n(carry.begin(), 5, via.begin());
        const auto q_r = plan->retreat();
        arm_.traj_step([this, via, q_r](const ArmIO::Q5 & from) {
          return stand::sample_via(from, via, q_r, arm_.velocity_scale(), TRAJ_DT);
        })(after_retreat);
        return;
      }
    }
    arm_.move_joints(retreat, after_retreat);
  }

  // 닿는 구간 — arm_io contact_step과 같은 단계·시간, 내려가기만 경유점 경로 (시뮬 _apply_action 먼 퍽)
  void contact(const std::shared_ptr<far::Plan> & plan, double vs_in, ArmIO::Cb done) {
    const ArmIO::Q5 above = plan->retreat(), down = plan->down(), lift = plan->lift;
    const double vs = std::clamp(vs_in, 0.1, 1.0);
    const auto p = std::make_shared<SkillModel>(policy_);
    const auto ctx = [this, p, above, down](uint8_t phase, bool holding, bool live) -> ArmIO::Step {
      return [this, p, phase, holding, live, above, down](ArmIO::Cb c) { arm_.skill_ctx(p.get(), phase, holding, live, above, down); c(true); };
    };
    // 밀고 들어가는 궤적(첫 traj_step)은 팔이 멈출 때까지 기다린 뒤 보냄 — 0 단계 동안 처짐 적분·보정으로 팔이 움직이면
    // 출발 자세가 어긋나 MoveIt이 거부함 (Isaac 11/12: start point deviates … joint3). 뒤로 뺀 자세로 '맞추는' 짧은 궤적은
    // 보내지 않음 — 0.02 s에 처진 만큼 확 당겨 팔이 흔들렸고, 바로 뒤 궤적이 같은 이유로 거부됐음
    arm_.settle_next();
    arm_.new_run();
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      ctx(0, true, true), arm_.wait_step(HOLD_S),                              // 0 멈춤 (처짐 적분)
      ctx(1, false, true),                                                     // 1 밀고 들어가기 → 세우기 → 손목 돌리기
      arm_.traj_step([path = plan->path, vs](const ArmIO::Q5 &) { return far::sample_path(path, vs, TRAJ_DT); }),
      ctx(2, true, true), arm_.wait_step(SETTLE_S),                            // 2 안정
      ctx(3, true, false),                                                     // 3 쥐기 (보정 고정)
      arm_.grip_timed_step(arm_.grip_close_m(), arm_.grip_current_a(), CLOSE_S),
      arm_.grip_stop_step(STOP_S),
      arm_.held_step(),                                                        // 잡기 판정 (빈손이면 내려간 채 실패 — pick과 같음)
      ctx(4, false, false),                                                    // 4 들어올리기
      arm_.traj_step([down, lift, vs](const ArmIO::Q5 &) { return stand::sample_move(down, lift, vs, TRAJ_DT); }),
      ctx(5, false, false)});                                                  // 5 끝 (실기에선 안 기다림 — contact_step과 같음)
    RCLCPP_INFO(get_logger(), "pick_far: 닿는 구간 (시뮬 기준 동작 + %s) · 들어올리며 %.0f mm 당김",
                policy_.use ? "학습 보정" : "보정 없음", 1000 * plan->lift_in_m);
    arm_.run_steps(steps, 0, [this, done, use = policy_.use](bool ok) {
      ok = ok && !arm_.grasp_empty();
      arm_.ctx_off();  // 보정 끔
      arm_.hand_target("none", -1, -1);
      if (use) arm_.settle_next();  // 보정 Δ가 0으로 빠지는 동안 다음 동작은 기다림 (contact_step과 같음)
      done(ok);
    });
  }

  using Q5v = std::vector<double>;
  ArmIO arm_;
  SkillModel policy_;
  far::Params fp_;
  double contact_vs_ = 0.7;      // far_contact_vel_scale (가까운 퍽)
  double contact_vs_far_ = 0.5;  // far_contact_vel_scale_far (박스 끝 퍽)
  double slow_x_m_ = 0.27;       // far_slow_x_m
  int max_tries_ = 2;
  bool via_carry_ = true;
  std::string why_;  // 마지막 후보가 경로를 못 푼 이유 (다른 실패가 없을 때 결과 이유)
  std::vector<double> carry_left_, carry_right_;
  rclcpp::Client<ScoreGrasps>::SharedPtr score_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PickFarSkill)
