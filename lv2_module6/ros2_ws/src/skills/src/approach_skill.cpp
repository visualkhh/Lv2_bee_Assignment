// skill_approach — /skills/approach: 목표 자세(map x,y,yaw) 앞 마지막 몇 cm를 저속 정밀 주행 (Nav2는 로봇을 원 0.18로 봐서 못 붙는 자리)
//   goal.target = 쓸 주행 정책: dock_box(기본, 상자 앞 도킹) · park_home(출발 구역) · drive_arc · drive_straight
//   approach_mode sim: 목표 방향 직선 따라가기(직선 위 carrot 조향 ≤ approach_v_max → 마지막 방향, 시뮬 drive_env 기준 제어를 11-12 고침) + 정책 보정 (dv, dw)
//     — 이 노드가 정책을 직접 계산 (20 Hz). 정책: models/skills/<정책>/ (meta use: false면 기준 제어만)
//     도착 판정은 정책 meta의 tol_m · tol_deg · hold_s (시뮬 drive_env TASKS와 같음)
//   approach_mode legacy: 예전 coordinator approach (목표 뒤 approach_pre_m로 → 돌기 → 직선)
//   /cmd_vel_nav로 냄 (Nav2와 같은 입구 → twist_mux 비상정지·collision_monitor 그대로). 진행이 멈추면 approach_blocked
//   상자 도킹 (goal.values = [gap_m, half_m] — mission_manager go_near): goal.pose는 도면 힌트일 뿐, 목표는 인지로 정함 (11-12 결정: 도면은 안 믿음)
//     시작하면 팔을 천천히 pose_look으로 (arm_velocity_scale, launch) · /tracked_objects에서 힌트 상자 가운데 box_match_m 안의
//     start_box·location_box → 목표 = 상자 가운데 − (half + gap)·면 법선 (윗면 방향 90° 주기 중 힌트 방향에 가까운 것)
//     첫 측정 전에는 안 움직임 (box_wait_s 넘으면 approach_box_unseen) · 측정은 지연·오차가 있어 그대로 안 씀:
//     오래된 것(box_stale_s) · 윗면이 잘려 작게 잡힌 것(짧은 변 < box_min_side_m — 가운데가 보이는 쪽으로 쏠림) · 추정에서 box_jump_m
//     넘게 튄 것 버림 · box_alpha로 섞고 초당 box_rate_mps · box_rate_dps까지만 (11-12 실측: 측정마다 1 cm씩 따라가 목표가 8 cm/s로
//     옆으로 움직여 상자 앞에서 크게 돎) · 목표 box_freeze_m 안부터는 고정 (가까우면 잘려 보임 — 마지막은 odom)
//     상자 도킹은 odom 프레임에서: map은 cuVSLAM 보정으로 움직임 (11-12 Isaac 정답 대비 map→odom이 한 번에 21 cm · 5.5° 밀림,
//     odom→base_footprint는 1 cm) → 목표를 map에 고정하면 붙은 뒤에 '지나침'으로 보고 후진. 측정(map)은 그 관측 시각의
//     map→odom으로 odom에 옮김 (인지가 쓴 것과 같은 보정이라 상쇄) · 도면 힌트는 시작 때 한 번 옮김
//   goal.target "exit" (상자에서 빠져나오기, 11-13): 지금 방향 그대로 values[0] m(없으면 approach_exit_m) 똑바로 후진 — odom 기준,
//     정책 보정 없음 · 뒤는 카메라가 못 보므로 느리게 (기준 제어 후진 상한 0.05 m/s)
#include <algorithm>
#include <cmath>
#include <map>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ml/policy.hpp"
#include "ml/nav_obs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace skills {

class ApproachSkill : public SkillNode {
public:
  explicit ApproachSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_approach", options), arm_(*this) {
    mode_ = declare_parameter<std::string>("approach_mode", "sim");
    speed_ = declare_parameter<double>("approach_speed", 0.05);      // legacy 직선 속도 [m/s]
    pre_m_ = declare_parameter<double>("approach_pre_m", 0.10);      // legacy: 목표 뒤 이만큼에서 돌고 직선으로
    pos_tol_ = declare_parameter<double>("approach_pos_tol", 0.005); // legacy 도착
    yaw_tol_ = declare_parameter<double>("approach_yaw_tol", 0.03);
    timeout_s_ = declare_parameter<double>("approach_timeout_s", 30.0);
    res_v_ = declare_parameter<double>("residual_v", 0.02);          // 시뮬 res_v · res_w (행동 1 = 이만큼)
    res_w_ = declare_parameter<double>("residual_w", 0.1);
    exit_m_ = declare_parameter<double>("approach_exit_m", 0.165);  // exit 기본 후진 거리 [m] (= standoff 0.25 − gap 0.085)
    release_m_ = declare_parameter<double>("approach_release_m", 0.03);  // sim: 도착(tol_m×0.6) 뒤 이만큼 벗어나야 다시 위치 맞춤
    carrot_m_ = declare_parameter<double>("approach_carrot_m", 0.05);    // sim: 목표 직선 위 이만큼 앞 점을 향해 조향
    v_max_ = declare_parameter<double>("approach_v_max", 0.06);          // sim: 전진 상한 [m/s]
    w_max_ = declare_parameter<double>("approach_w_max", 0.5);           // sim: 회전 상한 [rad/s]
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
    box_match_m_ = declare_parameter<double>("box_match_m", 0.35);  // 상자 도킹: 힌트(도면) 상자 가운데에서 이만큼 안의 상자만
    box_wait_s_ = declare_parameter<double>("box_wait_s", 8.0);     // 첫 측정까지 기다림 (그동안 정지)
    box_stale_s_ = declare_parameter<double>("box_stale_s", 1.0);   // last_seen이 이보다 오래된 측정은 버림
    box_jump_m_ = declare_parameter<double>("box_jump_m", 0.08);    // 추정에서 이만큼 넘게 튄 측정은 버림
    box_alpha_ = declare_parameter<double>("box_alpha", 0.3);       // 새 측정을 섞는 비율
    box_rate_mps_ = declare_parameter<double>("box_rate_mps", 0.02);  // 목표를 옮기는 최대 속도 [m/s]
    box_rate_dps_ = declare_parameter<double>("box_rate_dps", 3.0);   // 목표 방향을 돌리는 최대 속도 [deg/s]
    box_min_side_ = declare_parameter<double>("box_min_side_m", 0.19);  // 윗면 짧은 변이 이보다 작으면 잘린 것 — 버림
    box_freeze_m_ = declare_parameter<double>("box_freeze_m", 0.10);  // 목표까지 이보다 가까우면 목표 고정
    for (const char * name : {"dock_box", "park_home", "drive_arc", "drive_straight"}) {
      Policy p;
      p.model = load_model(name);
      std::string why;
      if (p.model.use && (!p.net.load(p.model.policy, why) || p.net.inputs() != size_t(ml::NAV_OBS) || p.net.outputs() != 2)) {
        RCLCPP_ERROR(get_logger(), "%s: 정책을 못 씀 — %s", name, why.c_str());
        p.model.use = false;
      }
      policies_[name] = std::move(p);
    }
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", rclcpp::QoS(10));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_ = m; });
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
      [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { on_objects(*m); });
    serve("approach", [this](const SkillGoal & g, Done done) { start(g, done); });
  }

protected:
  void stop() override {
    if (timer_) timer_->cancel();
    box_mode_ = false;
    arm_.stop();
    halt();
  }

private:
  struct Policy { SkillModel model; ml::Policy net; };

  void start(const SkillGoal & g, Done done) {
    double x, y, yaw;
    box_mode_ = false;  // 이전 goal의 측정이 끼지 않게 (아래에서 다시 정함)
    exit_ = g.target == "exit";
    if (exit_) { start_exit(g, done); return; }
    frame_ = g.values.size() >= 2 ? "odom" : "map";
    if (!robot_pose(x, y, yaw)) { done(false, "approach_no_tf"); return; }
    const std::string name = g.target.empty() ? "dock_box" : g.target;
    const auto it = policies_.find(name);
    if (it == policies_.end()) { done(false, "approach_bad_policy"); return; }
    pol_ = &it->second;
    const auto & q = g.pose.pose.orientation;
    gx_ = g.pose.pose.position.x;
    gy_ = g.pose.pose.position.y;
    gyaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    if (!from_map(rclcpp::Time(0, 0, RCL_ROS_TIME), gx_, gy_, gyaw_)) { done(false, "approach_no_tf"); return; }
    tol_m_ = pol_->model.num("tol_m", 0.01);
    tol_yaw_ = pol_->model.num("tol_deg", 1.5) * M_PI / 180.0;
    hold_s_ = pol_->model.num("hold_s", 0.5);
    limit_s_ = pol_->model.num("limit_s", 25.0);
    // 이미 목표 방향이고 옆 어긋남이 작으면 바로 직선 (legacy)
    const double lat0 = -(gx_ - x) * std::sin(gyaw_) + (gy_ - y) * std::cos(gyaw_);
    stage_ = std::abs(std::remainder(gyaw_ - yaw, 2 * M_PI)) < 0.15 && std::abs(lat0) < 0.03 ? 2 : 0;
    best_ = 1e9;
    still_s_ = act0_ = act1_ = last_v_ = last_w_ = 0.0;
    pos_done_ = false;
    started_ = progress_ = last_meas_ = now();
    done_ = done;
    box_mode_ = g.values.size() >= 2;
    have_box_ = frozen_ = false;
    if (box_mode_) {
      gap_ = g.values[0];
      half_ = g.values[1];
      hint_x_ = gx_ + (half_ + gap_) * std::cos(gyaw_);  // 도면 상자 가운데 — 고를 상자를 찾는 데만 씀
      hint_y_ = gy_ + (half_ + gap_) * std::sin(gyaw_);
      bx_ = hint_x_;
      by_ = hint_y_;
      bh_ = gyaw_;
      if (arm_.moveit_ready())
        arm_.move_joints(look_q_, [this](bool ok) { if (!ok) RCLCPP_WARN(get_logger(), "look 자세 실패 — pole_front_cam만으로 봄"); });
      else RCLCPP_WARN(get_logger(), "MoveIt 없음 — look 자세 없이 pole_front_cam만으로 봄");
    }
    stage(name + (mode_ == "sim" && pol_->model.use ? " (기준 제어 + 학습 보정)" : " (기준 제어)") + " → (" +
          std::to_string(gx_).substr(0, 5) + ", " + std::to_string(gy_).substr(0, 5) + ")");
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      if (!box_ready()) return;
      mode_ == "sim" ? tick_sim() : tick_legacy();
    });
  }

  // 상자에서 빠져나오기 (헤더 설명): 목표 = 지금 자리에서 뒤로 d · 같은 방향 → tick_sim의 '지나침' 후진 가지로 똑바로 물러남
  void start_exit(const SkillGoal & g, Done done) {
    double x, y, yaw;
    frame_ = "odom";
    if (!robot_pose(x, y, yaw)) { done(false, "approach_no_tf"); return; }
    const double d = g.values.empty() ? exit_m_ : g.values[0];
    if (!(d > 0.0) || d > 1.0) { done(false, "approach_bad_target"); return; }
    gx_ = x - d * std::cos(yaw);
    gy_ = y - d * std::sin(yaw);
    gyaw_ = yaw;
    tol_m_ = 0.01;
    tol_yaw_ = 3.0 * M_PI / 180.0;
    hold_s_ = 0.2;
    best_ = 1e9;
    still_s_ = 0.0;
    pos_done_ = false;
    started_ = progress_ = now();
    done_ = done;
    stage("exit — 뒤로 " + std::to_string(d).substr(0, 5) + " m");
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { tick_sim(); });
  }

  // 상자 측정 → 걸러서 목표 갱신 (헤더 설명)
  void on_objects(const perception::msg::TrackedObjectArray & m) {
    if (!box_mode_ || frozen_ || !timer_ || timer_->is_canceled()) return;
    const auto t = now();
    const double ref_x = have_box_ ? bx_ : hint_x_, ref_y = have_box_ ? by_ : hint_y_;
    // 후보마다 map → frame_ (그 관측 시각의 map→odom) 뒤에 비교 — 힌트·추정이 frame_에 있음
    bool found = false, has_yaw = false;
    double mx = 0, my = 0, psi = 0, best_d = box_match_m_;
    for (const auto & o : m.objects) {
      if (o.class_name != "start_box" && o.class_name != "location_box") continue;
      if ((t - rclcpp::Time(o.last_seen, t.get_clock_type())).seconds() > box_stale_s_) continue;
      if (std::min(o.size.x, o.size.y) < box_min_side_) continue;
      const auto & q = o.pose.pose.orientation;
      double x = o.pose.pose.position.x, y = o.pose.pose.position.y, a = 2.0 * std::atan2(q.z, q.w);
      if (!from_map(rclcpp::Time(o.last_seen, RCL_ROS_TIME), x, y, a)) continue;
      if (std::hypot(x - hint_x_, y - hint_y_) > box_match_m_) continue;
      const double dr = std::hypot(x - ref_x, y - ref_y);
      if (dr < best_d) {
        best_d = dr; found = true; mx = x; my = y; psi = a;
        has_yaw = std::abs(q.z) > 1e-6 || std::abs(q.w) < 1.0 - 1e-9;  // 단위 쿼터니언 = 방향 모름
      }
    }
    if (!found) return;
    double h = bh_;  // 윗면 방향을 모르면 지금 방향 그대로
    if (has_yaw)  // 윗면 변 방향 → 다가갈 방향 = 그 90° 주기 중 지금 방향에 가까운 것
      h = psi + std::round(std::remainder(bh_ - psi, 2 * M_PI) / (M_PI / 2)) * (M_PI / 2);
    const double dt = std::clamp((t - last_meas_).seconds(), 0.0, 0.5);  // 측정 사이 시간 — 초당 한계를 이만큼만
    last_meas_ = t;
    if (!have_box_) {
      have_box_ = true;
      bx_ = mx; by_ = my; bh_ = std::remainder(h, 2 * M_PI);
      stage(fmt_box("상자 봄", mx, my));
    } else {
      const double dx = mx - bx_, dy = my - by_;
      if (std::hypot(dx, dy) > box_jump_m_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "상자 측정 튐 %.3f m — 버림", std::hypot(dx, dy));
        return;
      }
      double sx = box_alpha_ * dx, sy = box_alpha_ * dy;
      const double n = std::hypot(sx, sy), lim = box_rate_mps_ * dt;
      if (n > lim) { sx *= lim / n; sy *= lim / n; }
      const double step = box_rate_dps_ * M_PI / 180.0 * dt;
      bx_ += sx; by_ += sy;
      bh_ = std::remainder(bh_ + std::clamp(box_alpha_ * std::remainder(h - bh_, 2 * M_PI), -step, step), 2 * M_PI);
    }
    gx_ = bx_ - (half_ + gap_) * std::cos(bh_);
    gy_ = by_ - (half_ + gap_) * std::sin(bh_);
    gyaw_ = bh_;
  }

  std::string fmt_box(const char * what, double x, double y) const {
    char b[160];
    std::snprintf(b, sizeof b, "%s (%.3f, %.3f) — 도면과 %.1f cm", what, x, y, 100 * std::hypot(x - hint_x_, y - hint_y_));
    return b;
  }

  // 상자 도킹: 첫 측정 전 정지 (못 보면 실패) · 목표 가까이 오면 고정. false면 이번 틱은 끝
  bool box_ready() {
    if (!box_mode_) return true;
    if (!have_box_) {
      halt();
      progress_ = now();
      if ((now() - started_).seconds() > box_wait_s_) {
        RCLCPP_WARN(get_logger(), "approach: %.1f s 동안 상자를 못 봄 (도면 (%.3f, %.3f) 근처)", box_wait_s_, hint_x_, hint_y_);
        end(false, "approach_box_unseen");
      }
      return false;
    }
    double x, y, yaw;
    if (!frozen_ && robot_pose(x, y, yaw) && std::hypot(gx_ - x, gy_ - y) < box_freeze_m_) {
      frozen_ = true;
      RCLCPP_INFO(get_logger(), "%s", fmt_box("목표 고정 — 상자", bx_, by_).c_str());
    }
    return true;
  }

  // map 점·방향 → frame_ (odom이면 stamp 때의 map→odom, 그 시각이 버퍼에 없으면 최신) · map이면 그대로
  bool from_map(const rclcpp::Time & stamp, double & x, double & y, double & yaw) const {
    if (frame_ == "map") return true;
    geometry_msgs::msg::Transform t;
    try {
      t = tf_buffer_->lookupTransform(frame_, "map", stamp.nanoseconds() ? tf2_ros::fromRclcpp(stamp) : tf2::TimePointZero).transform;
    } catch (const std::exception &) {
      try {
        t = tf_buffer_->lookupTransform(frame_, "map", tf2::TimePointZero).transform;
      } catch (const std::exception &) {
        return false;
      }
    }
    const double r = std::atan2(2 * (t.rotation.w * t.rotation.z + t.rotation.x * t.rotation.y),
                                1 - 2 * (t.rotation.y * t.rotation.y + t.rotation.z * t.rotation.z));
    const double c = std::cos(r), s = std::sin(r), x0 = x;
    x = t.translation.x + c * x0 - s * y;
    y = t.translation.y + s * x0 + c * y;
    yaw = std::remainder(yaw + r, 2 * M_PI);
    return true;
  }

  bool robot_pose(double & x, double & y, double & yaw) const {
    try {
      const auto t = tf_buffer_->lookupTransform(frame_, "base_footprint", tf2::TimePointZero).transform;
      x = t.translation.x;
      y = t.translation.y;
      yaw = std::atan2(2 * (t.rotation.w * t.rotation.z + t.rotation.x * t.rotation.y),
                       1 - 2 * (t.rotation.y * t.rotation.y + t.rotation.z * t.rotation.z));
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void halt() {
    geometry_msgs::msg::TwistStamped z;
    z.header.stamp = now();
    z.header.frame_id = "base_footprint";
    cmd_pub_->publish(z);
  }

  void end(bool ok, const std::string & why) {
    timer_->cancel();
    box_mode_ = false;
    halt();
    done_(ok, why);
  }

  void send(double v, double w) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_footprint";
    cmd.twist.linear.x = v;
    cmd.twist.angular.z = w;
    cmd_pub_->publish(cmd);
    last_v_ = v;
    last_w_ = w;
  }

  // 정책 보정 (시뮬 drive_env: base + res·a, 행동은 −1~1) — 입력 13개는 ml/nav_obs.hpp (odom · imu · 직전 명령·행동 · 시간 비율)
  void residual(double x, double y, double yaw, double & dv, double & dw) {
    dv = dw = 0.0;
    if (!pol_->model.use || !odom_) return;
    ml::NavObsIn in;
    in.x = x; in.y = y; in.yaw = yaw;
    in.gx = gx_; in.gy = gy_; in.gyaw = gyaw_;
    in.vx = odom_->twist.twist.linear.x;
    in.vy = odom_->twist.twist.linear.y;
    in.wz = odom_->twist.twist.angular.z;
    in.cmd_v = last_v_; in.cmd_w = last_w_;
    in.act0 = act0_; in.act1 = act1_;
    in.elapsed_s = (now() - started_).seconds();
    in.limit_s = limit_s_;
    if (imu_) in.tilt = ml::tilt_of(imu_->orientation.x, imu_->orientation.y, imu_->orientation.z, imu_->orientation.w);
    double obs[ml::NAV_OBS], act[2];
    ml::nav_obs(in, obs);
    pol_->net.act_into(obs, act);
    act0_ = std::clamp(act[0], -1.0, 1.0);
    act1_ = std::clamp(act[1], -1.0, 1.0);
    dv = std::clamp(res_v_ * act0_, -0.05, 0.05);
    dw = std::clamp(res_w_ * act1_, -0.10, 0.10);
  }

  // 시뮬 drive_env._baseline과 같은 식 (isaac/rl/bee_isaac/tasks/drive_env.py, demo_cycle drive_to)
  void tick_sim() {
    double x, y, yaw;
    if (!robot_pose(x, y, yaw)) return;
    const auto wrap = [](double a) { return std::remainder(a, 2 * M_PI); };
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double gx = c * (gx_ - x) + s * (gy_ - y), gy = -s * (gx_ - x) + c * (gy_ - y), gyaw = wrap(gyaw_ - yaw);
    const double dist = std::hypot(gx, gy);
    // 시뮬 식(점을 향해 돌기 → 직진 → 마지막 방향)은 Nav2가 옆으로 몇 cm 어긋나게 세우면 점에 닿을 때 방향이 틀어져 있고,
    // 1~2 cm 안에서 점을 향해 조향(head · 4·gy)하다 상자 앞에서 계속 돌았다 (scene_approach 실측 11-12).
    // → 목표 방향 직선을 따라감: 로봇 발(직선 위 투영)에서 carrot_m 앞 점을 향해 조향(목표는 안 넘김) → 옆 어긋남이 도착 전에 사라짐
    //   pos_done_: latch_m(= tol_m의 0.6) 안에 들면 release 밖까지 방향만 · 지나치면 직선 따라 후진 · 등지면(45° 넘게) 제자리 회전 (멀 때만)
    const double latch = 0.6 * tol_m_, release = std::max(release_m_, 2.0 * tol_m_);
    if (dist <= latch) pos_done_ = true;
    else if (dist > release) pos_done_ = false;
    const double along = gx * std::cos(gyaw) + gy * std::sin(gyaw);  // 목표까지 남은 길이 (목표 방향으로) — 음수면 지나침
    const double lead = std::max(0.0, along - carrot_m_);              // 당근 = 목표에서 이만큼 뒤 (직선 위)
    const double cx = gx - lead * std::cos(gyaw), cy = gy - lead * std::sin(gyaw);
    const double head_c = std::atan2(cy, cx);
    double v = 0.0, w = 0.0;
    if (pos_done_) {
      w = std::clamp(3.0 * gyaw, -w_max_, w_max_);
    } else if (along < -0.003) {
      v = std::max(-0.05, 1.5 * along);
      w = std::clamp(1.5 * gyaw, -0.3, 0.3);
    } else if (std::abs(head_c) > 45.0 * M_PI / 180.0 && dist > release) {
      w = std::clamp(3.0 * head_c, -w_max_, w_max_);
    } else {
      v = std::clamp(1.5 * along, 0.0, v_max_) * std::max(0.0, std::cos(head_c));
      if (along > 0.003) v = std::max(v, 0.01);
      w = std::clamp(3.0 * head_c, -w_max_, w_max_);
    }
    if (dist <= tol_m_ && std::abs(gyaw) < tol_yaw_) still_s_ += 0.05;
    else still_s_ = 0.0;
    if (still_s_ >= hold_s_) {
      RCLCPP_INFO(get_logger(), "approach 도착 — 앞 %.4f m · 옆 %.4f m · 방향 %.3f rad", gx, gy, gyaw);
      end(true, "");
      return;
    }
    const auto t = now();
    const double remain = dist + 0.1 * std::abs(gyaw);
    if (remain < best_ - 0.003) { best_ = remain; progress_ = t; }
    if ((t - progress_).seconds() > 4.0 || (t - started_).seconds() > timeout_s_) {
      RCLCPP_WARN(get_logger(), "approach 멈춤 (남은 %.3f m, %.3f rad)", dist, gyaw);
      end(false, "approach_blocked");
      return;
    }
    double dv, dw;
    if (exit_) dv = dw = 0.0;
    else residual(x, y, yaw, dv, dw);
    send(v + dv, w + dw);
  }

  void tick_legacy() {
    double x, y, yaw;
    if (!robot_pose(x, y, yaw)) return;
    const double c = std::cos(gyaw_), s = std::sin(gyaw_);
    const double px = gx_ - pre_m_ * c, py = gy_ - pre_m_ * s;  // 목표 뒤 자리 (여기서 돈다)
    const auto wrap = [](double a) { return std::remainder(a, 2 * M_PI); };
    double v = 0.0, w = 0.0, remain;
    const int stage0 = stage_;
    if (stage_ == 0) {  // 앞 자리로: 그쪽을 보고 → 직진 (이미 2 cm 안이면 건너뜀)
      remain = std::hypot(px - x, py - y);
      const double head = wrap(std::atan2(py - y, px - x) - yaw);
      if (remain < 0.02) stage_ = 1;
      else if (std::abs(head) > 0.15) w = std::clamp(1.5 * head, -0.4, 0.4);
      else { v = std::min(speed_ * 2, 0.8 * remain); w = std::clamp(1.5 * head, -0.3, 0.3); }
    } else if (stage_ == 1) {  // 목표 방향으로 돌기
      const double e = wrap(gyaw_ - yaw);
      remain = std::abs(e) * 0.1;
      if (std::abs(e) < yaw_tol_) stage_ = 2;
      else w = std::clamp(1.5 * e, -0.4, 0.4) + (e > 0 ? 0.05 : -0.05);
    } else {  // 직선 전진 — 옆 어긋남은 방향을 살짝 틀어 줄임
      const double along = (gx_ - x) * c + (gy_ - y) * s, lat = -(gx_ - x) * s + (gy_ - y) * c;
      remain = std::abs(along);
      if (remain < pos_tol_) {
        RCLCPP_INFO(get_logger(), "approach 도착 — 남은 %.4f m · 옆 %.4f m · 방향 %.3f rad", along, lat, wrap(gyaw_ - yaw));
        end(true, "");
        return;
      }
      v = std::clamp(0.8 * along, -speed_, speed_);
      w = std::clamp(1.5 * wrap(gyaw_ - yaw) + (along > 0 ? 3.0 : -3.0) * lat, -0.2, 0.2);
    }
    const auto t = now();
    if (stage_ != stage0) { best_ = 1e9; progress_ = t; }
    else if (remain < best_ - 0.003) { best_ = remain; progress_ = t; }
    if ((t - progress_).seconds() > 4.0 || (t - started_).seconds() > timeout_s_) {
      RCLCPP_WARN(get_logger(), "approach 멈춤 (단계 %d · 남은 %.3f)", stage_, remain);
      end(false, "approach_blocked");
      return;
    }
    send(v, w);
  }

  std::string mode_;
  double speed_, pre_m_, pos_tol_, yaw_tol_, timeout_s_, res_v_, res_w_, release_m_, carrot_m_, v_max_, w_max_;
  bool pos_done_ = false;
  bool exit_ = false;
  double exit_m_ = 0.165;
  std::string frame_ = "map";  // 목표·로봇 자세 프레임 — 상자 도킹은 odom
  ArmIO arm_;
  std::vector<double> look_q_;
  double box_match_m_, box_wait_s_, box_stale_s_, box_jump_m_, box_alpha_, box_rate_mps_, box_rate_dps_, box_min_side_, box_freeze_m_;
  rclcpp::Time last_meas_{0, 0, RCL_ROS_TIME};
  bool box_mode_ = false, have_box_ = false, frozen_ = false;
  double gap_ = 0, half_ = 0, hint_x_ = 0, hint_y_ = 0, bx_ = 0, by_ = 0, bh_ = 0;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  std::map<std::string, Policy> policies_;
  Policy * pol_ = nullptr;
  double gx_ = 0, gy_ = 0, gyaw_ = 0, tol_m_ = 0.01, tol_yaw_ = 0.026, hold_s_ = 0.5, limit_s_ = 25.0;
  double best_ = 1e9, still_s_ = 0, act0_ = 0, act1_ = 0, last_v_ = 0, last_w_ = 0;
  int stage_ = 0;
  rclcpp::Time started_{0, 0, RCL_ROS_TIME}, progress_{0, 0, RCL_ROS_TIME};
  Done done_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::ApproachSkill)
