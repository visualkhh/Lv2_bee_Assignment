// skill_place — /skills/place: 손에 든 퍽 내려놓기. goal.pose = 놓을 면 (map, z = 면 높이 — 손 안 퍽 바닥이 여기 닿음)
//   target "cell" (주사위 칸 · 좌표 place):
//     contact_mode sim: 칸 위 5 cm까지 MoveIt → 닿는 구간 (place_cell 정책: 내려가기 → 벌림 → 올리기) — 손 안 퍽 바닥이 면에 닿는 자리에서 벌림 (시뮬 place_m 0)
//     moveit: 위 place_approach_m → 퍽 바닥 + place_release_m까지 내려가 벌림 → 다시 위 (예전)
//   target "over" (물체 위 — location_box · 판): 면 + held_bottom + place_release 위로 values[0](다가가는 높이)만큼 위에서 MoveIt → 벌림
//   정책: model/skills/place_cell/ (팀원 pip_env) — 상수 오프셋만으로 시뮬 Δ = 0 99.7 %라 meta.yaml use: false
//   손가락은 늘 로봇 y 방향으로 닫고(시뮬 place_env와 같음), 놓을 때 36 mm만 벌리고, 목표를 로봇 앞쪽으로 place_offset_x_m만큼 뒤로 옮긴다
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "skills/stand_plan.hpp"

#include <cmath>

namespace skills {

class PlaceSkill : public SkillNode {
public:
  explicit PlaceSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_place", options), arm_(*this) {
    held_bottom_m_ = declare_parameter<double>("held_bottom_m", 0.0606);      // 손끝 → 손 안 퍽 바닥
    release_m_ = declare_parameter<double>("place_release_m", 0.02);          // 벌리는 높이 (퍽 바닥이 면 위 이만큼)
    approach_m_ = declare_parameter<double>("place_approach_m", 0.08);        // moveit 방식: 위에서 내려오는 높이
    // sim 방식: 손 안 퍽 바닥이 면 위 이만큼에서 다가와 수직으로 내려감. 이웃 퍽(높이 60 mm)보다 높아야 옆으로 지나갈 때 안 침
    //   (11/08 실기 주사위 5: 5 cm로 가운데 눈을 놓으러 가다 먼 줄 퍽을 넘어뜨림). 손이 못 닿으면 5 cm(CONTACT_ABOVE_M)로
    above_m_ = declare_parameter<double>("place_above_m", 0.08);
    // 놓을 때 손가락 벌림 [m] — 0.036(시뮬 place_env release_gap)이면 사각 퍽(30 mm, 대각선 42 mm)이 손 안에서 조금만 기울어도
    //   손가락 사이에 끼어, 박스에 닿은 뒤 퍽을 쥔 채 다시 올라가 공중에서 돌다 떨어져 누움 (11-10 Isaac 퍽 정답 시간선 · 3회 재현).
    //   0.046이면 같은 퍽이 목표 4 mm 안에 섬. 주사위 칸 이웃 퍽(틈 30 mm) 사이에서 손가락이 걸리는지는 아직 시험 전
    open_gap_m_ = declare_parameter<double>("place_open_gap_m", 0.046);
    offset_x_m_ = declare_parameter<double>("place_offset_x_m", 0.0037);      // 시뮬 Δ = 0: 퍽이 칸 가운데보다 로봇 앞으로 이만큼 놓임 → 목표를 뒤로 (실기에서 다시 재서 바꿈)
    // 팔이 뒤(적재함 쪽, |joint1| > 1.2 rad — unload_end_pose over)면 그쪽 carry를 멈추지 않고 지나 놓을 자리 위로 (pick과 같음)
    via_carry_ = declare_parameter<bool>("place_via_carry", true);
    carry_left_ = declare_parameter<std::vector<double>>("pose_carry_left", std::vector<double>{});
    carry_right_ = declare_parameter<std::vector<double>>("pose_carry_right", std::vector<double>{});
    policy_ = load_model("place_cell");
    serve("place", [this](const SkillGoal & g, Done done) {
      if (g.target != "cell" && g.target != "over") { done(false, "place_bad_target"); return; }
      when_ready([this]() { return arm_.moveit_ready() && arm_.gripper_ready(); }, "arm_unavailable", done, [this, g, done]() {
        arm_.clear_flags();
        // 처짐 적분 0으로 되돌리기는 정책 입력이 시뮬처럼 0에서 시작하게 하려는 것 — 정책이 꺼져 있으면 안 함
        //   (되돌리면 팔이 처졌다 다시 올라와 멈출 때까지 ~0.45 s 기다림 · 11/08 실기 unload → place 사이 멈춤)
        if (policy_.use) arm_.reset_sag();
        auto release = g.pose;  // 벌릴 손끝 자리
        if (release.header.frame_id.empty()) release.header.frame_id = "map";
        release.header.stamp = now();
        release.pose.orientation = geometry_msgs::msg::Quaternion();
        release.pose.position.z += held_bottom_m_ + release_m_;
        double yb = 0.0;  // 로봇 방향 [rad, release 좌표계] — 손가락 닫는 축을 로봇 y에 맞추고 목표를 앞뒤로 옮길 때 씀
        if (g.target == "cell") {
          if (!base_yaw(release.header.frame_id, yb)) { done(false, "place_tf"); return; }
          release.pose.position.x -= offset_x_m_ * std::cos(yb);
          release.pose.position.y -= offset_x_m_ * std::sin(yb);
        }
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
        place_cell(release, g.pose.pose.position.z, yb, fin);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  // ψ = 로봇 yaw → 손가락 닫는 축 = ψ − 90° = 로봇 y (ik_ee_yaw · move_arm yaw와 같은 정의). 퍽 놓기는 모든 칸을 이 방향으로 한다
  bool base_yaw(const std::string & frame, double & yaw) {
    try {
      const auto tf = arm_.tf().lookupTransform(frame, "base_footprint", tf2::TimePointZero);
      const auto & r = tf.transform.rotation;
      yaw = std::atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void place_cell(const geometry_msgs::msg::PoseStamped & release, double top, double psi, ArmIO::Cb done) {
    const double open_m = stand::gap_joint(open_gap_m_);
    if (arm_.sim_contact()) {
      auto down = release;
      down.pose.position.z -= release_m_;  // 퍽 바닥이 면에 닿는 자리
      auto above = down;
      above.pose.position.z += above_m_;
      {
        ArmIO::Q5 now{}, q{};
        if (arm_.arm_now(now) && !arm_.ik_ee_yaw(above, psi, now, q)) {
          RCLCPP_WARN(get_logger(), "place: 놓을 자리 위 %.0f cm IK 실패 — %.0f cm로 (이웃 퍽을 칠 수 있음)", 100 * above_m_, 100 * CONTACT_ABOVE_M);
          above.pose.position.z = down.pose.position.z + CONTACT_ABOVE_M;
        }
      }
      arm_.hand_target("cell", -1, -1, down.pose.position.x, down.pose.position.y, top);
      stage("칸 위");
      const auto descend = [this, down, open_m, done](bool ok2) {
        if (!ok2) { done(false); return; }
        arm_.contact_step(policy_, [this, down](ArmIO::Q5 & a, ArmIO::Q5 & d) { return arm_.arm_now(a) && arm_.ik_ee(down, a, d); },
                          false, open_m)(done);
      };
      ArmIO::Q5 now{}, q_up{};
      if (via_carry_ && arm_.arm_now(now) && std::abs(now[0]) > 1.2 && (now[0] > 0 ? carry_left_ : carry_right_).size() == 5) {
        if (!arm_.ik_ee_yaw(above, psi, now, q_up)) { RCLCPP_WARN(get_logger(), "place: 놓을 자리 위 IK 실패 (팔이 못 닿음)"); done(false); return; }
        ArmIO::Q5 via{};
        const auto & c = now[0] > 0 ? carry_left_ : carry_right_;
        std::copy_n(c.begin(), 5, via.begin());
        arm_.traj_step([this, via, q_up](const ArmIO::Q5 & from) {
          return stand::sample_via(from, via, q_up, arm_.velocity_scale(), TRAJ_DT);
        })(descend);
        return;
      }
      arm_.move_arm(above, [this, down, above, psi, open_m, done](bool ok) {
        if (!ok) { done(false); return; }
        ArmIO::Q5 now{}, q_above{};  // MoveIt은 손 방향을 ±0.15 rad까지만 맞춤 — 닫는 축을 로봇 y에 정확히 (pick과 같은 방식)
        if (!arm_.arm_now(now) || !arm_.ik_ee_yaw(above, psi, now, q_above)) { done(false); return; }
        arm_.move_joints(std::vector<double>(q_above.begin(), q_above.end()), [this, down, open_m, done](bool ok2) {
          if (!ok2) { done(false); return; }
          arm_.contact_step(policy_, [this, down](ArmIO::Q5 & a, ArmIO::Q5 & d) { return arm_.arm_now(a) && arm_.ik_ee(down, a, d); },
                            false, open_m)(done);
        });
      }, psi);
      return;
    }
    auto above = release;
    above.pose.position.z += approach_m_;
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      [this, above, psi](ArmIO::Cb c) { arm_.move_arm(above, c, psi); },     // 위
      [this, release, psi](ArmIO::Cb c) { arm_.move_arm(release, c, psi); }, // 내려가기
      arm_.gripper_step(open_m),                                             // 벌리기 (36 mm)
      [this, above, psi](ArmIO::Cb c) { arm_.move_arm(above, c, psi); }});   // 다시 위 (놓은 퍽을 건드리지 않게)
    arm_.run_steps(steps, 0, done);
  }


  ArmIO arm_;
  SkillModel policy_;
  bool via_carry_ = true;
  std::vector<double> carry_left_, carry_right_;
  double held_bottom_m_, release_m_, approach_m_, offset_x_m_, open_gap_m_, above_m_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PlaceSkill)
