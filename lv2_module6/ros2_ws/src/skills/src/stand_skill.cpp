// skill_stand — 누운 퍽 세우기와 그 자리에서 다시 쥐기 (팀원 right_env "tilt", 경유점: stand_plan.hpp — stand_waypoints ik | table)
//   /skills/stand (goal.pose = 누운 퍽 map 자세 · 긴 축 = orientation · goal.shape · goal.tag):
//     벌림(stand_open_m) → MoveIt으로 먼 끝 위 → 멈춤 → 내려감 → 위치형 쥐기: 목표 stand_pinch_m을 닫는 전류로 → 쥠 확인 → 쥐는 전류로 낮춤 →
//     대각선 끌기(stand_drag_m > 0, 실기 방식) 또는 호 · 멈춤 · 눌러 놓음 → 멈춤 → 벌림 → 위로. 손은 놓고 끝 (경유점을 기억 — 다음 stand_pick)
//   /skills/stand_pick: 직전 stand의 경유점에서 같은 손목으로 선 퍽 위 → 바로 아래로 → 멈춤 → 제 힘으로 닫기 → 잡기 판정 → 들어올림
//     (reset_sag 안 함 — 세운 직후 처짐 보정을 0으로 되돌리면 손이 처진 만큼 떨어짐)
//   정책: model/skills/regrasp_after_stand/ (동료 regrasp_stood = 세우기 + 다시 쥐기 한 에피소드) — 로봇 입력 모양(stand)이
//     rl_arm_correction에 생기기 전까지 meta use: false (기준 동작만)
//   방식 C: model/skills/stand_lying_c/ (kind hand, use: true면 호 대신) — 시뮬 stand_c_env와 같은 단계 규칙을 50 Hz로:
//     위(MoveIt) → 0 멈춤 → 1 정책 손 속도로 쥘 자리까지 → 2 쥐기(stand_close_current_a) → 3 정책 손 속도로 세우기(stand_pinch_current_a)
//     → 4 안정 → 5 벌리기 → 6 위로 5 cm → 7 대기. 손 목표 → ik_down(손 아래 · 닫는 축 고정) → /arm_controller/joint_trajectory (3 rad/s 제한)
//     퍽 축·각속도는 손 위치로 구함 (ml/stand_c_obs.hpp pinch_phi — 가까운 끝 바닥 모서리 경첩)
//     쥐기 전(0·1)엔 손 카메라(/hand_relative puck: 누운 퍽 가운데 − 손끝 · 축)로 쥘 자리를 계속 고침 — 시뮬 입력도 인식값 (11/08 실기:
//     inspect 한 번으로 정하면 내려가며 전혀 안 맞춤). 한 번에 stand_c_cam_step_m · 기준에서 stand_c_cam_max_m 넘게는 안 옮김
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "ml/mlp_policy.hpp"
#include "ml/stand_c_obs.hpp"
#include "perception/msg/hand_relative.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace skills {

namespace {
constexpr double STAND_HOLD_S = 0.3, STAND_PINCH_S = 0.3, STAND_SETTLE_S = 0.3, STAND_OPEN_S = 0.3, REGRASP_CLOSE_S = 0.5;
constexpr double STAND_HANG_S = 0.5;  // 호 안 멈춤
// 방식 C (isaac/rl/bee_isaac/tasks/stand_c_env.py StandCEnvCfg · RightEnvCfg)
constexpr double C_DT = 0.02, C_STEP_M = 0.012, C_VMAX = 3.0, C_HOLD_S = 0.3, C_CLOSE_S = 0.4, C_SETTLE_S = 0.3, C_OPEN_S = 0.3;
constexpr double C_LIFT_M = 0.05, C_LIFT_S = 0.5, C_WAIT_S = 0.5, C_T_APPROACH_S = 4.0, C_T_STAND_S = 6.0, C_EPISODE_S = 16.0;
constexpr double C_TOL_Z = 0.004, C_TOL_LAT = 0.004, C_TOL_AX = 0.008, C_Z_FLOOR = 0.012, C_UP_DEG = 5.0, C_W_STAND = 0.5;
}  // namespace

class StandSkill : public SkillNode {
public:
  explicit StandSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_stand", options), arm_(*this) {
    waypoints_ = declare_parameter<std::string>("stand_waypoints", "ik");
    box_top_m_ = declare_parameter<double>("stand_box_top_m", 0.07);           // 시작 박스 윗면 높이 — 누운 퍽 바닥
    open_m_ = declare_parameter<double>("stand_open_m", 0.012);                // [m] 벌림 (손가락 하나)
    pinch_m_ = declare_parameter<double>("stand_pinch_m", 0.006);              // [m] 쥐는 목표 — 힘은 전류 상한과 이 위치 오차로 정함
                                                                               //   (11/08: 0.007은 30 mm 퍽 닿는 자리 0.0062보다 넓어 안 쥠 → 0.006)
    close_a_ = declare_parameter<double>("stand_close_current_a", 0.01614);    // [A] 목표까지 닫을 때
    pinch_a_ = declare_parameter<double>("stand_pinch_current_a", 0.00538);    // [A] 닫은 뒤 낮춰 쥐는 전류
    grip_a_ = declare_parameter<double>("stand_grip_current_a", 0.007);        // [A] 이 전류 이상 + 안 움직임 = 쥠
    empty_m_ = declare_parameter<double>("stand_empty_m", 0.003);              // [m] 이보다 닫히면 빈손
    grip_timeout_s_ = declare_parameter<double>("stand_grip_timeout_s", 4.0);
    sp_.drag_m = declare_parameter<double>("stand_drag_m", sp_.drag_m);
    sp_.drag_lift_m = declare_parameter<double>("stand_drag_lift_m", sp_.drag_lift_m);
    sp_.arc_end_deg = declare_parameter<double>("stand_arc_end_deg", sp_.arc_end_deg);
    sp_.set_dz_m = declare_parameter<double>("stand_set_dz_m", sp_.set_dz_m);
    sp_.tilt_over_m = declare_parameter<double>("stand_tilt_over_m", 0.030);   // 호 끝까지 더 올림 (시뮬 0.010 · 실기 11/08 0.030)
    sp_.arc_return = declare_parameter<bool>("stand_arc_return", false);       // 호 끝에서 90°로 되돌리지 않고 바로 벌림 (실기 11/08)
    sp_.above_m = declare_parameter<double>("stand_above_m", sp_.above_m);
    sp_.lift_away_m = declare_parameter<double>("stand_lift_away_m", sp_.lift_away_m);
    sp_.contact_h_box = declare_parameter<double>("stand_contact_h_box_m", sp_.contact_h_box);
    sp_.contact_h_cyl = declare_parameter<double>("stand_contact_h_cyl_m", sp_.contact_h_cyl);
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
    c_model_ = load_model("stand_lying_c");
    if (c_model_.use && (!c_net_.load(c_model_.policy, why) || c_net_.obs_dim() != ml::STAND_C_OBS || c_net_.act_dim() != 3)) {
      RCLCPP_ERROR(get_logger(), "stand_lying_c: 정책을 못 씀 — %s (호로)", why.c_str());
      c_model_.use = false;
    }
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", rclcpp::QoS(10));
    declare_parameter<double>("stand_puck_dx_m", -0.010);      // [m] inspect 퍽 위치에 더함 (base_footprint x) — 실기 11/08 inspect가 손 카메라보다 10 mm 앞
    declare_parameter<double>("stand_arc_k", 1.0);           // 호의 퍽 축 방향 배율 (1 원 · 0 그대로 위로)
    // 호 방식: 내려가기 전 멈춰 손 카메라로 퍽을 다시 재고 경유점을 다시 풂 — 쥐는 높이 근처는 카메라–퍽 10 cm 안이라 depth가 비어서
    declare_parameter<bool>("stand_cam", true);
    declare_parameter<double>("stand_cam_above_m", 0.06);    // [m] 잴 때 손끝 높이 (쥐는 자리 위) — 6 cm면 카메라–퍽 10 cm 넘음
    declare_parameter<double>("stand_cam_back_m", 0.025);    // [m] 잴 때 쥐는 자리에서 로봇 쪽(퍽 가까운 끝 쪽)으로 — 퍽 전체가 화면에 들어오게
    declare_parameter<double>("stand_cam_wait_s", 1.2);      // [s] 재는 시간 (손 카메라 검출 ≤ 4 Hz)
    declare_parameter<int>("stand_cam_min_n", 2);            // 이만큼 못 보면 inspect 값 그대로
    declare_parameter<double>("stand_cam_max_m", 0.03);      // [m] inspect 자리에서 이보다 멀리는 안 옮김 (잘못 본 퍽)
    declare_parameter<double>("stand_cam_max_deg", 30.0);    // 축도 이보다 많이는 안 돌림
    declare_parameter<bool>("stand_c_cam", true);              // 쥐기 전 손 카메라로 쥘 자리 고침
    declare_parameter<double>("stand_c_cam_max_m", 0.03);      // inspect 위치에서 이보다 멀리는 안 옮김 (잘못 본 퍽)
    declare_parameter<double>("stand_c_cam_max_deg", 30.0);    // 축도 이보다 많이는 안 돌림
    declare_parameter<double>("stand_c_cam_gain", 0.3);        // 새 값 쪽으로 이 비율씩 (한 정책 스텝)
    hr_sub_ = create_subscription<perception::msg::HandRelative>("/hand_relative", rclcpp::QoS(10),
      [this](perception::msg::HandRelative::ConstSharedPtr m) { if (m->kind == "puck") hr_ = m; });
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
  void stop() override {
    if (c_timer_) c_timer_->cancel();
    if (cam_timer_) cam_timer_->cancel();
    arm_.stop();
  }

private:
  void stand(const SkillGoal & g, Done done) {
    refresh_params();  // 실행 중 ros2 param set /skill_stand stand_… 이 다음 goal부터 먹게
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
    const stand::V3 centre = {c.x() + get_parameter("stand_puck_dx_m").as_double(), c.y(), box_top_m_};
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
    if (c_model_.use) {
      const auto & gp = g.pose.pose.position;
      arm_.hand_target("puck", g.tag, -1, gp.x, gp.y, gp.z);  // 손 카메라가 이 퍽을 잼
      stand_c(centre, yaw, is_box, qs, done);
      return;
    }
    const double open = open_m_, pinch = pinch_m_;
    const double vs = vs_, avs = arc_vs_;
    const stand::Params sp = waypoints_ == "table" ? stand::Params{} : sp_;  // 표는 right_env 호 그대로
    // 경유점은 손 카메라로 다시 잰 뒤 바뀔 수 있어서 뒤 단계들은 run->qs를 실행 때 읽음
    auto run = std::make_shared<SRun>();
    run->qs = qs;
    run->centre0 = centre;
    run->yaw0 = yaw;
    run->is_box = is_box;
    const auto wp = [run]() { return std::vector<stand::Q5>(run->qs.begin(), run->qs.end() - 2); };  // [위 · 시작 · 호 … · 놓기 · 위로] (뒤 2개 = 다시 잡기)
    stand::Q5 look;
    const bool cam = get_parameter("stand_cam").as_bool() && waypoints_ == "ik" && look_pose(centre, yaw, is_box, qs.front(), look);
    const stand::Q5 first = cam ? look : qs.front();
    std::vector<ArmIO::Step> st = {
      arm_.gripper_step(open),
      arm_.joints_step(std::vector<double>(first.begin(), first.end())),  // 어디서든 MoveIt 계획으로 (충돌 확인)
      arm_.wait_step(STAND_HOLD_S)};
    if (cam) {
      const auto & gp = g.pose.pose.position;
      st.push_back(cam_step(run, g.tag, gp.x, gp.y, gp.z));
      st.push_back(arm_.traj_step([run, vs](const stand::Q5 & now) { return stand::sample_move(now, run->qs.front(), vs, TRAJ_DT); }));
    }
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(st);
    steps->insert(steps->end(), {
      arm_.traj_step([run, vs](const stand::Q5 & now) { return stand::sample_move(now, run->qs[1], vs, TRAJ_DT); }),
      arm_.grip_timed_step(pinch, close_a_, 0.0),
      arm_.grip_settle_step(grip_a_, empty_m_, grip_timeout_s_),
      arm_.grip_timed_step(pinch, pinch_a_, STAND_PINCH_S),  // 퍽이 손가락 사이에서 돌 만큼 낮춤
      arm_.traj_step([wp, sp, avs](const stand::Q5 &) { return stand::sample_arc(wp(), sp, avs, STAND_HANG_S, TRAJ_DT); }),
      arm_.wait_step(STAND_SETTLE_S),
      arm_.grip_timed_step(open, -1.0, STAND_OPEN_S),
      arm_.traj_step([wp, vs](const stand::Q5 & now) { return stand::sample_move(now, wp().back(), vs, TRAJ_DT); })});
    arm_.run_steps(steps, 0, [this, run, done](bool ok2) {
      if (ok2) { done_qs_ = run->qs; done(true, ""); return; }
      const std::string why2 = !arm_.grip_fail().empty() ? arm_.grip_fail() : arm_.contact() ? "arm_contact" : "stand_failed";
      arm_.set_gripper(open_m_, [done, why2](bool) { done(false, why2); });  // 잡은 채 멈추지 않게 벌리고 실패
    });
  }

  // ---------- 호 방식: 내려가기 전 손 카메라로 다시 재기
  struct SRun {
    std::vector<stand::Q5> qs;
    stand::V3 centre0{};
    double yaw0 = 0;
    bool is_box = true;
  };

  // 잴 자세: 쥐는 자리에서 로봇 쪽으로 stand_cam_back_m, 위로 stand_cam_above_m (못 닿으면 1 cm씩 낮춤, 4 cm 아래면 안 잼)
  bool look_pose(const stand::V3 & centre, double yaw, bool is_box, const stand::Q5 & near, stand::Q5 & out) {
    double close_yaw;
    const auto pts = stand::waypoints(centre, yaw, is_box, sp_, close_yaw);
    const double back = get_parameter("stand_cam_back_m").as_double();
    for (double h = get_parameter("stand_cam_above_m").as_double(); h > 0.04 - 1e-9; h -= 0.01) {
      const stand::V3 p = {pts[1][0] - back * std::cos(yaw), pts[1][1] - back * std::sin(yaw), pts[1][2] + h};
      if (stand::ik_down(p, close_yaw, near, out)) return true;
    }
    stage("손 카메라: 잴 자세에 못 닿음 — inspect 값 그대로");
    return false;
  }

  ArmIO::Step cam_step(std::shared_ptr<SRun> run, int tag, double gx, double gy, double gz) {
    return [this, run, tag, gx, gy, gz](ArmIO::Cb next) {
      arm_.hand_target("puck", tag, -1, gx, gy, gz);
      struct Acc { std::vector<double> x, y, dyaw; int64_t last = 0; std::chrono::steady_clock::time_point t0; };
      auto acc = std::make_shared<Acc>();
      acc->t0 = std::chrono::steady_clock::now();
      const double wait_s = get_parameter("stand_cam_wait_s").as_double();
      cam_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this, run, acc, wait_s, next]() {
        const auto m = hr_;
        stand::Q5 q;
        if (m && m->valid && m->age < 0.45 && arm_.arm_now(q)) {
          const int64_t t = rclcpp::Time(m->header.stamp).nanoseconds();
          if (t != acc->last) {  // hand_relative는 같은 검출을 새 검출이 올 때까지 다시 보냄 — 영상 시각이 바뀐 것만
            acc->last = t;
            const auto e = stand::fk(q);  // rel은 base_link 기준 차이 — base_footprint와 방향이 같아 그대로 더함
            acc->x.push_back(e[0] + m->rel.x);
            acc->y.push_back(e[1] + m->rel.y);
            // 손 카메라가 축을 못 재면 axis = (0, 0, 1) (실기 11/08: 누운 퍽도 늘 그랬음) — 위치만 쓰고 축은 inspect 값
            if (std::abs(m->axis.z) < 0.5)
              acc->dyaw.push_back(std::remainder(stand::away_yaw(std::atan2(m->axis.y, m->axis.x)) - run->yaw0, M_PI));  // 축은 180° 대칭
          }
        }
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - acc->t0).count() < wait_s) return;
        cam_timer_->cancel();
        arm_.hand_target("none", -1, -1);
        cam_fix(*run, acc->x, acc->y, acc->dyaw);
        next(true);
      });
    };
  }

  static double median(std::vector<double> v) {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  // 잰 값(중앙값)으로 경유점을 다시 풂 — 못 보거나 못 풀면 inspect 값 그대로
  void cam_fix(SRun & run, const std::vector<double> & xs, const std::vector<double> & ys, const std::vector<double> & dyaws) {
    const int n = static_cast<int>(xs.size());
    if (n < get_parameter("stand_cam_min_n").as_int()) {
      stage("손 카메라: 퍽을 " + std::to_string(n) + "번만 봄 — inspect 값 그대로");
      return;
    }
    const double max_m = get_parameter("stand_cam_max_m").as_double();
    const double max_a = get_parameter("stand_cam_max_deg").as_double() * M_PI / 180;
    double dx = median(xs) - run.centre0[0], dy = median(ys) - run.centre0[1];
    const double d = std::hypot(dx, dy);
    if (d > max_m) { dx *= max_m / d; dy *= max_m / d; }
    const double dyaw = dyaws.size() >= xs.size() / 2 + 1 ? std::clamp(median(dyaws), -max_a, max_a) : 0.0;  // 축은 반 넘게 잰 때만
    const stand::V3 c = {run.centre0[0] + dx, run.centre0[1] + dy, run.centre0[2]};
    std::vector<stand::Q5> qs;
    std::string why;
    if (!stand::plan_ik(c, run.yaw0 + dyaw, run.is_box, sp_, qs, why)) {
      stage("손 카메라: 고친 자리 " + why + " — inspect 값 그대로");
      return;
    }
    run.qs = qs;
    char buf[160];
    std::snprintf(buf, sizeof buf, "손 카메라 %d번: 퍽 (%+.1f, %+.1f) mm · 축 %+.1f°%s 고침%s", n, dx * 1000, dy * 1000,
                  dyaw * 180 / M_PI, dyaws.empty() ? " (축 못 잼)" : "", d > max_m ? " (한도에서 자름)" : "");
    stage(buf);
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
      arm_.grip_timed_step(arm_.grip_close_m(), arm_.grip_current_a(), REGRASP_CLOSE_S), arm_.held_step(),
      arm_.traj_step([lift, vs](const stand::Q5 & now) { return stand::sample_move(now, lift, vs, TRAJ_DT); })});
    arm_.run_steps(steps, 0, [this, done](bool ok) {
      if (!ok) {
        if (arm_.grasp_empty()) arm_.set_gripper(open_m_, [](bool) {});
        done(false, arm_.grasp_empty() ? "grasp_empty" : "stand_pick_failed");
        return;
      }
      done_qs_.clear();
      done(true, "");
    });
  }

  void refresh_params() {
    const auto d = [this](const char * n) { return get_parameter(n).as_double(); };
    open_m_ = d("stand_open_m"); pinch_m_ = d("stand_pinch_m");
    close_a_ = d("stand_close_current_a"); pinch_a_ = d("stand_pinch_current_a"); grip_a_ = d("stand_grip_current_a");
    sp_.drag_m = d("stand_drag_m"); sp_.drag_lift_m = d("stand_drag_lift_m"); sp_.arc_end_deg = d("stand_arc_end_deg");
    sp_.set_dz_m = d("stand_set_dz_m"); sp_.tilt_over_m = d("stand_tilt_over_m"); sp_.arc_return = get_parameter("stand_arc_return").as_bool();
    sp_.above_m = d("stand_above_m"); sp_.lift_away_m = d("stand_lift_away_m"); sp_.arc_k = d("stand_arc_k");
    sp_.contact_h_box = d("stand_contact_h_box_m"); sp_.contact_h_cyl = d("stand_contact_h_cyl_m");
    vs_ = d("stand_vel_scale"); arc_vs_ = d("stand_arc_vel_scale");
  }

  // ---------- 방식 C: 정책이 손 속도를 냄 (시뮬 stand_c_env _pre_physics_step · _get_observations와 같은 순서)
  struct CRun {
    ml::V3c u{}, pinch{}, pivot{}, x_t{}, ee_prev{};
    stand::Q5 q_goal{};
    stand::V3 centre0{}, centre{};  // inspect 퍽 가운데 · 지금 쓰는 값 (손 카메라로 고침)
    double yaw0 = 0, yaw = 0;
    bool is_box = true;
    int cam_n = 0;
    double close_yaw = 0, a = stand::PUCK_L, b = 0.03, phi = 0, t_in = 0, t = 0;
    int ph = 0;
    std::array<double, 3> act{};
    bool started = false;
    std::vector<stand::Q5> qs;
    Done done;
  };

  void stand_c(const stand::V3 & centre, double yaw, bool is_box, const std::vector<stand::Q5> & qs, Done done) {
    auto r = std::make_shared<CRun>();
    r->centre0 = centre;
    r->yaw0 = yaw;
    r->is_box = is_box;
    c_geom(*r, centre, yaw);
    r->a = stand::PUCK_L - sp_.pinch_from_end_m - sp_.press_m;
    r->b = is_box ? sp_.contact_h_box : sp_.contact_h_cyl;
    r->q_goal = qs.front();
    const auto x0 = stand::fk(qs.front());
    r->x_t = {x0[0], x0[1], x0[2]};
    r->qs = qs;
    r->done = done;
    stage("방식 C 정책 (stand_lying_c) — 위에서 시작");
    auto steps = std::make_shared<std::vector<ArmIO::Step>>(std::vector<ArmIO::Step>{
      arm_.gripper_step(open_m_),
      arm_.joints_step(std::vector<double>(qs.front().begin(), qs.front().end()))});  // 먼 끝 위 (MoveIt, 충돌 확인)
    arm_.run_steps(steps, 0, [this, r](bool ok) {
      if (!ok) { r->done(false, arm_.contact() ? "arm_contact" : "stand_failed"); return; }
      c_timer_ = create_wall_timer(std::chrono::duration<double>(C_DT), [this, r]() { c_tick(r); });
    });
  }

  // 누운 퍽 가운데·축 → 쥘 자리 · 경첩 · 손가락 닫는 축 (stand_plan waypoints와 같은 호 φ = 0)
  void c_geom(CRun & r, const stand::V3 & centre, double yaw) {
    double close_yaw;
    const auto pts = stand::waypoints(centre, yaw, r.is_box, sp_, close_yaw);
    r.centre = centre;
    r.yaw = yaw;
    r.u = {std::cos(yaw), std::sin(yaw), 0.0};
    r.pinch = pts[1];  // 손가락 중심이 먼 끝 윗면
    r.pivot = {centre[0] - r.u[0] * stand::PUCK_L / 2, centre[1] - r.u[1] * stand::PUCK_L / 2, centre[2]};
    r.close_yaw = close_yaw;
  }

  // 쥐기 전: 손 카메라 퍽 (가운데 − 손끝 · 축) → 쥘 자리를 조금씩 그쪽으로
  void c_cam(CRun & r, const ml::V3c & ee) {
    const auto m = hr_;
    if (!m || !m->valid || m->age > 0.45 || std::abs(m->axis.z) > 0.5 || !get_parameter("stand_c_cam").as_bool()) return;
    const double gain = get_parameter("stand_c_cam_gain").as_double();
    const double max_m = get_parameter("stand_c_cam_max_m").as_double(), max_a = get_parameter("stand_c_cam_max_deg").as_double() * M_PI / 180;
    // rel은 base_link 기준 차이 — base_footprint와 방향이 같아 그대로 더함
    double cx = ee[0] + m->rel.x, cy = ee[1] + m->rel.y;
    double dyaw = std::remainder(stand::away_yaw(std::atan2(m->axis.y, m->axis.x)) - r.yaw0, M_PI);  // 축은 180° 대칭
    const double dx = cx - r.centre0[0], dy = cy - r.centre0[1], d = std::hypot(dx, dy);
    if (d > max_m) { cx = r.centre0[0] + dx * max_m / d; cy = r.centre0[1] + dy * max_m / d; }
    dyaw = std::clamp(dyaw, -max_a, max_a);
    const double yaw_cur = std::remainder(r.yaw - r.yaw0, M_PI);
    const stand::V3 c = {r.centre[0] + gain * (cx - r.centre[0]), r.centre[1] + gain * (cy - r.centre[1]), r.centre0[2]};
    c_geom(r, c, r.yaw0 + yaw_cur + gain * (dyaw - yaw_cur));
    ++r.cam_n;
  }

  void c_end(const std::shared_ptr<CRun> & r, bool ok, const std::string & why) {
    c_timer_->cancel();
    arm_.hand_target("none", -1, -1);
    if (ok) {
      std::vector<stand::Q5> qs;  // 다시 쥐기 경유점은 고친 퍽 자리로
      std::string w;
      done_qs_ = stand::plan_ik(r->centre, r->yaw, r->is_box, sp_, qs, w) ? qs : r->qs;
      r->done(true, "");
      return;
    }
    RCLCPP_WARN(get_logger(), "stand C 실패 %s (단계 %d)", why.c_str(), r->ph);
    arm_.set_gripper(open_m_, [r, why](bool) { r->done(false, why); });  // 쥔 채 멈추지 않게
  }

  void c_tick(const std::shared_ptr<CRun> & r) {
    stand::Q5 q;
    double grip = 0.0;
    if (!arm_.arm_now(q)) return;
    arm_.joint("gripper_left_joint", grip);
    const auto e = stand::fk(q);
    const ml::V3c ee = {e[0], e[1], e[2]};
    if (!r->started) { r->ee_prev = ee; r->started = true; }
    if (r->ph <= 1) c_cam(*r, ee);
    const double ee_up = stand::GRASP_Z - stand::PUCK_H + stand::GRIP_DEPTH / 2;
    // 퍽 기울기: 쥔 동안(2~4)은 손 위치로 · 쥐기 전은 누움 · 벌린 뒤는 마지막 값
    double phi_dot = 0.0;
    if (r->ph >= 2 && r->ph <= 4) {
      const double phi = std::max(0.0, ml::pinch_phi(r->u, r->pivot, ee, ee_up, r->a, r->b));
      phi_dot = (phi - r->phi) / C_DT;
      r->phi = phi;
    }
    ml::StandCObsIn in;
    in.u = r->u;
    in.pinch = r->pinch;
    in.pivot = r->pivot;
    in.ee = ee;
    in.ee_prev = r->ee_prev;
    in.axis = ml::axis_of(r->u, r->phi);
    in.w = ml::w_of(r->u, phi_dot);
    in.grip = grip;
    in.phase = r->ph;
    in.prev_act = r->act;
    in.t_in = r->t_in;
    in.dt = C_DT;
    double obs[ml::STAND_C_OBS], a[3];
    ml::stand_c_obs(in, obs);
    c_net_.act_into(obs, a);
    for (double & v : a) v = std::clamp(v, -1.0, 1.0);
    r->ee_prev = ee;

    // 단계 (시뮬 _pre_physics_step: 시간 더하고 → 넘어갈지 → 넘어가면 단계 시간 0)
    r->t_in += C_DT;
    r->t += C_DT;
    const ml::V3c lat = ml::lat_of(r->u);
    const ml::V3c d = {r->pinch[0] - ee[0], r->pinch[1] - ee[1], r->pinch[2] - ee[2]};
    const bool near = (ee[2] - r->pinch[2]) < C_TOL_Z && std::abs(d[0] * lat[0] + d[1] * lat[1]) < C_TOL_LAT &&
                      std::abs(d[0] * r->u[0] + d[1] * r->u[1]) < C_TOL_AX;
    const double up_z = std::sin(r->phi), up_ok = std::cos(C_UP_DEG * M_PI / 180);
    const bool stood = up_z > up_ok && std::abs(phi_dot) < C_W_STAND;
    const bool relapse = r->ph == 4 && up_z < up_ok;
    const int ph = r->ph;
    const bool adv = (ph == 0 && r->t_in >= C_HOLD_S) || (ph == 1 && near) || (ph == 2 && r->t_in >= C_CLOSE_S) ||
                     (ph == 3 && stood) || (ph == 4 && r->t_in >= C_SETTLE_S && !relapse) || (ph == 5 && r->t_in >= C_OPEN_S) ||
                     (ph == 6 && r->t_in >= C_LIFT_S) || (ph == 7 && r->t_in >= C_WAIT_S);
    const int nph = relapse ? 3 : ph + (adv ? 1 : 0);
    if (nph != ph) {
      r->t_in = 0.0;
      if (nph == 2) {
        arm_.set_gripper(pinch_m_, [](bool) {}, close_a_);                     // 쥐기 (시뮬 힘 상한 0.4 N ↔ 닫는 전류)
        RCLCPP_INFO(get_logger(), "stand C 쥠: 손 카메라로 쥘 자리 %d번 고침 — inspect보다 (%.0f, %.0f) mm · 축 %.0f°", r->cam_n,
                    (r->centre[0] - r->centre0[0]) * 1000, (r->centre[1] - r->centre0[1]) * 1000,
                    std::remainder(r->yaw - r->yaw0, M_PI) * 180 / M_PI);
      }
      else if (nph == 3) arm_.set_gripper(pinch_m_, [](bool) {}, pinch_a_);     // 퍽이 손가락 사이에서 돌 만큼 낮춤
      else if (nph == 5) arm_.set_gripper(open_m_, [](bool) {});
      if (nph == 8) { c_end(r, true, ""); return; }
    }
    r->ph = nph;
    if ((nph == 1 && r->t_in > C_T_APPROACH_S) || (nph == 3 && r->t_in > C_T_STAND_S)) { c_end(r, false, nph == 1 ? "stand_c_approach_timeout" : "stand_c_not_standing"); return; }
    if (r->t > C_EPISODE_S) { c_end(r, false, "stand_c_timeout"); return; }
    if (arm_.contact()) { c_end(r, false, "arm_contact"); return; }

    // 손 목표: 정책 손 속도 (살아 있는 단계만) · 위로 · 쥘 자리 둘레 상자 안으로
    const bool live = nph == 1 || nph == 3;
    r->act = {a[0], a[1], a[2]};
    ml::V3c x = r->x_t;
    for (int i = 0; i < 3; ++i) {
      const double s = live ? C_STEP_M * a[i] : 0.0;
      x[0] += s * (i == 0 ? r->u[0] : i == 1 ? lat[0] : 0.0);
      x[1] += s * (i == 0 ? r->u[1] : i == 1 ? lat[1] : 0.0);
      x[2] += i == 2 ? s : 0.0;
    }
    if (nph == 6) x[2] += C_LIFT_M / C_LIFT_S * C_DT;
    const ml::V3c dx = {x[0] - r->pinch[0], x[1] - r->pinch[1], x[2] - r->pinch[2]};
    const double dr = std::clamp(dx[0] * r->u[0] + dx[1] * r->u[1], -0.08, 0.04);
    const double dl = std::clamp(dx[0] * lat[0] + dx[1] * lat[1], -0.03, 0.03);
    const double dz = std::clamp(dx[2], -C_Z_FLOOR, 0.12);
    r->x_t = {r->pinch[0] + dr * r->u[0] + dl * lat[0], r->pinch[1] + dr * r->u[1] + dl * lat[1], r->pinch[2] + dz};
    stand::Q5 qn;
    if (stand::ik_down({r->x_t[0], r->x_t[1], r->x_t[2]}, r->close_yaw, r->q_goal, qn)) r->q_goal = qn;  // 못 닿으면 직전 목표

    // 관절 목표 → JTC (3 rad/s 제한 = 시뮬 rate_vmax)
    double step = 0.0;
    for (int i = 0; i < 5; ++i) step = std::max(step, std::abs(r->q_goal[i] - q[i]));
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(r->q_goal.begin(), r->q_goal.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(std::max(C_DT, step / C_VMAX));
    jt.points.push_back(pt);
    traj_pub_->publish(jt);
  }

  ArmIO arm_;
  SkillModel policy_, c_model_;
  ml::MlpPolicy c_net_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr c_timer_, cam_timer_;
  rclcpp::Subscription<perception::msg::HandRelative>::SharedPtr hr_sub_;
  perception::msg::HandRelative::ConstSharedPtr hr_;
  std::string waypoints_ = "ik";
  double box_top_m_ = 0.07, vs_ = 0.3, arc_vs_ = 0.5;
  double open_m_ = 0.012, pinch_m_ = 0.007, close_a_ = 0.01614, pinch_a_ = 0.00538, grip_a_ = 0.007, empty_m_ = 0.003, grip_timeout_s_ = 4.0;
  stand::Params sp_;
  stand::Table table_;
  std::vector<stand::Q5> done_qs_;  // 마지막 stand 경유점 — stand_pick이 같은 손목으로 다시 잡음
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::StandSkill)
