// skill_stand — 누운 퍽 세우기와 그 자리에서 다시 쥐기 (팀원 right_env "tilt", 경유점: stand_plan.hpp — stand_waypoints ik | table)
//   /skills/stand (goal.pose = 누운 퍽 map 자세 · 긴 축 = orientation · goal.shape · goal.tag):
//     벌림(40mm) → MoveIt으로 먼 끝 위 → 멈춤 → 내려감 → 가볍게 쥠(stand_pinch_current_a) → 가까운 아래 모서리 축으로 호 0→120° ·
//     멈춤 · 90°로 되돌려 눌러 놓음 → 멈춤 → 벌림 → 위로. 손은 놓고 끝 (경유점을 기억 — 다음 stand_pick)
//   /skills/stand_pick: 직전 stand의 경유점에서 같은 손목으로 선 퍽 위 → 바로 아래로 → 멈춤 → 제 힘으로 닫기 → 잡기 판정 → 들어올림
//     (reset_sag 안 함 — 세운 직후 처짐 보정을 0으로 되돌리면 손이 처진 만큼 떨어짐)
//   정책: model/skills/regrasp_after_stand/ (동료 regrasp_stood = 세우기 + 다시 쥐기 한 에피소드) — 로봇 입력 모양(stand)이
//     rl_arm_correction에 생기기 전까지 meta use: false (기준 동작만)
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace skills {

namespace {
constexpr double STAND_HOLD_S = 0.3, STAND_CLOSE_S = 0.4, STAND_SETTLE_S = 0.3, STAND_OPEN_S = 0.3, REGRASP_CLOSE_S = 0.5;
constexpr double STAND_HANG_S = 0.5;  // 호 안 멈춤
}  // namespace

class StandSkill : public SkillNode {
public:
  explicit StandSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_stand", options), arm_(*this) {
    waypoints_ = declare_parameter<std::string>("stand_waypoints", "ik");
    box_top_m_ = declare_parameter<double>("stand_box_top_m", 0.07);           // 시작 박스 윗면 높이 — 누운 퍽 바닥
    pinch_a_ = declare_parameter<double>("stand_pinch_current_a", 0.0055);     // 세울 때 가볍게 쥐는 힘 [A] (mA↔N 실측 필요)
    vs_ = declare_parameter<double>("stand_vel_scale", 0.3);                   // 내려가기·위로 × 팀 관절 한계
    arc_vs_ = declare_parameter<double>("stand_arc_vel_scale", 0.5);           // 호 × 팀 관절 한계
    table_.yaw_deg = declare_parameter<std::vector<double>>("stand_table_yaw_deg", std::vector<double>{});
    table_.ycs = declare_parameter<std::vector<double>>("stand_table_ycs", std::vector<double>{});
    table_.box_cx = declare_parameter<double>("stand_table_box_cx", 0.0);
    table_.box_h = declare_parameter<double>("stand_table_box_h", 0.0);
    table_.w = static_cast<int>(declare_parameter<int64_t>("stand_table_w", 0));
    table_.q[0] = declare_parameter<std::vector<double>>("stand_table_cyl_q", std::vector<double>{});
    table_.P[0] = declare_parameter<std::vector<double>>("stand_table_cyl_p", std::vector<double>{});
    table_.q[1] = declare_parameter<std::vector<double>>("stand_table_box_q", std::vector<double>{});
    table_.P[1] = declare_parameter<std::vector<double>>("stand_table_box_p", std::vector<double>{});
    std::string why;
    if (waypoints_ != "ik" && waypoints_ != "table") { RCLCPP_WARN(get_logger(), "stand_waypoints '%s' 모름 — ik로", waypoints_.c_str()); waypoints_ = "ik"; }
    if (waypoints_ == "table" && !table_.ready(why))
      RCLCPP_WARN(get_logger(), "stand_waypoints table인데 표가 없음 (params/stand_table.yaml) — stand는 stand_table_missing으로 실패");
    policy_ = load_model("regrasp_after_stand");
    serve("stand", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, g, done]() { stand(g, done); });
    });
    serve("stand_pick", [this](const SkillGoal &, Done done) {
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done,
                 [this, done]() { stand_pick(done); });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void stand(const SkillGoal & g, Done done) {
    // 퍽 (map) → 로봇 root (base_footprint): 가운데 xy · 긴 축 yaw (180° 대칭 → 로봇에서 먼 쪽으로)
    tf2::Transform t;
    try {
      const auto m = arm_.tf().lookupTransform("base_footprint", g.pose.header.frame_id.empty() ? "map" : g.pose.header.frame_id,
                                               tf2::TimePointZero).transform;
      t = tf2::Transform(tf2::Quaternion(m.rotation.x, m.rotation.y, m.rotation.z, m.rotation.w),
                         tf2::Vector3(m.translation.x, m.translation.y, m.translation.z));
    } catch (const std::exception &) {
      done(false, "stand_no_tf");
      return;
    }
    const auto & pp = g.pose.pose.position;
    const auto & q = g.pose.pose.orientation;
    const tf2::Vector3 c = t * tf2::Vector3(pp.x, pp.y, pp.z);
    const tf2::Vector3 ax = t.getBasis() * tf2::Vector3(1.0 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z), 0.0);
    const double yaw = stand::away_yaw(std::atan2(ax.y(), ax.x()));
    const stand::V3 centre = {c.x(), c.y(), box_top_m_};
    const bool is_box = g.shape != "cyl";
    std::vector<stand::Q5> qs;
    std::string why;
    const bool ok = waypoints_ == "table" ? stand::plan_table(table_, centre, yaw, is_box, qs, why)
                                          : stand::plan_ik(centre, yaw, is_box, sp_, qs, why);
    if (!ok) { done(false, why); return; }
    if (waypoints_ == "table" && std::abs(centre[0] - table_.box_cx) > 0.02)
      RCLCPP_WARN(get_logger(), "stand 표: 퍽이 표 기준 x에서 %.0fmm — dock 자리 확인", (centre[0] - table_.box_cx) * 1000);
    stage(std::string(is_box ? "사각" : "원기둥") + " 퍽 (" + std::to_string(centre[0]).substr(0, 5) + ", " +
          std::to_string(centre[1]).substr(0, 5) + ") 축 " + std::to_string(static_cast<int>(yaw * 180 / M_PI)) + "° — 경유점 " +
          waypoints_ + " " + std::to_string(qs.size()) + "개");
    done_qs_.clear();
    arm_.clear_flags();
    arm_.reset_sag();
    const double open = stand::gap_joint(0.040);
    const std::vector<stand::Q5> wp(qs.begin(), qs.end() - 2);  // [위 · 시작 · 호 … · 놓기 · 위로] (뒤 2개 = 다시 잡기)
    const stand::Q5 above = wp.front(), start = wp[1], away = wp.back();
    const double vs = vs_, avs = arc_vs_;
    const stand::Params sp = sp_;
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      arm_.gripper_step(open),
      arm_.joints_step(std::vector<double>(above.begin(), above.end())),  // 어디서든 MoveIt 계획으로 (충돌 확인)
      arm_.wait_step(STAND_HOLD_S),
      arm_.traj_step([start, vs](const stand::Q5 & now) { return stand::sample_move(now, start, vs, TRAJ_DT); }),
      arm_.grip_timed_step(0.0, pinch_a_, STAND_CLOSE_S),
      arm_.traj_step([wp, sp, avs](const stand::Q5 &) { return stand::sample_arc(wp, sp, avs, STAND_HANG_S, TRAJ_DT); }),
      arm_.wait_step(STAND_SETTLE_S),
      arm_.grip_timed_step(open, pinch_a_, STAND_OPEN_S),
      arm_.traj_step([away, vs](const stand::Q5 & now) { return stand::sample_move(now, away, vs, TRAJ_DT); })});
    arm_.run_steps(steps, 0, [this, qs, done](bool ok2) {
      if (ok2) done_qs_ = qs;
      done(ok2, ok2 ? "" : arm_.contact() ? "arm_contact" : "stand_failed");
    });
  }

  void stand_pick(Done done) {
    if (done_qs_.size() < 2) { done(false, "stand_pick_no_stand"); return; }
    const stand::Q5 grasp = done_qs_[done_qs_.size() - 2], lift = done_qs_.back();
    const double vs = vs_;
    arm_.clear_flags();
    stage("선 퍽 다시 쥐기");
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      arm_.traj_step([lift, vs](const stand::Q5 & now) { return stand::sample_move(now, lift, vs, TRAJ_DT); }),   // 선 퍽 위
      arm_.traj_step([grasp, vs](const stand::Q5 & now) { return stand::sample_move(now, grasp, vs, TRAJ_DT); }),  // 바로 아래로
      arm_.wait_step(STAND_HOLD_S),                                                                                // 처짐 보정 쌓임
      arm_.grip_timed_step(GRIPPER_CLOSED_M, arm_.grip_current_a(), REGRASP_CLOSE_S), arm_.held_step(),
      arm_.traj_step([lift, vs](const stand::Q5 & now) { return stand::sample_move(now, lift, vs, TRAJ_DT); })});
    arm_.run_steps(steps, 0, [this, done](bool ok) {
      if (!ok) {
        if (arm_.grasp_empty()) arm_.set_gripper(stand::gap_joint(0.040), [](bool) {});
        done(false, arm_.grasp_empty() ? "grasp_empty" : "stand_pick_failed");
        return;
      }
      done_qs_.clear();
      done(true, "");
    });
  }

  ArmIO arm_;
  SkillModel policy_;
  std::string waypoints_ = "ik";
  double box_top_m_ = 0.07, pinch_a_ = 0.0055, vs_ = 0.3, arc_vs_ = 0.5;
  stand::Params sp_;
  stand::Table table_;
  std::vector<stand::Q5> done_qs_;  // 마지막 stand 경유점 — stand_pick이 같은 손목으로 다시 잡음
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::StandSkill)
