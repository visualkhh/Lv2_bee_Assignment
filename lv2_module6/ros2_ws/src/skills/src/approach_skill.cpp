// skill_approach — /skills/approach: 목표 자세(map x,y,yaw) 앞 마지막 몇 cm를 저속 정밀 주행 (Nav2는 로봇을 원 0.18로 봐서 못 붙는 자리)
//   goal.target = 쓸 주행 정책: dock_box(기본, 상자 앞 도킹) · park_home(출발 구역) · drive_arc · drive_straight
//   approach_mode sim: 시뮬 drive_env 기준 제어(목표 쪽으로 돌기 → 옆 어긋남 고치며 직진 ≤ 0.12 m/s → 마지막 방향) + 그 정책의 보정 (dv, dw)
//     — 이 노드가 정책을 직접 계산 (20 Hz, 예전 rl_nav_correction 도킹 모드). 정책: model/skills/<정책>/ (meta use: false면 기준 제어만)
//     도착 판정은 정책 meta의 tol_m · tol_deg · hold_s (시뮬 drive_env TASKS와 같음)
//   approach_mode legacy: 예전 coordinator approach (목표 뒤 approach_pre_m로 → 돌기 → 직선)
//   /cmd_vel_nav로 냄 (Nav2와 같은 입구 → rl_nav_correction 속도 제한 → twist_mux 비상정지·collision_monitor 그대로). 진행이 멈추면 approach_blocked
#include <algorithm>
#include <cmath>
#include <map>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ml/mlp_policy.hpp"
#include "ml/nav_obs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "skills/skill_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace skills {

class ApproachSkill : public SkillNode {
public:
  explicit ApproachSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_approach", options) {
    mode_ = declare_parameter<std::string>("approach_mode", "sim");
    speed_ = declare_parameter<double>("approach_speed", 0.05);      // legacy 직선 속도 [m/s]
    pre_m_ = declare_parameter<double>("approach_pre_m", 0.10);      // legacy: 목표 뒤 이만큼에서 돌고 직선으로
    pos_tol_ = declare_parameter<double>("approach_pos_tol", 0.005); // legacy 도착
    yaw_tol_ = declare_parameter<double>("approach_yaw_tol", 0.03);
    timeout_s_ = declare_parameter<double>("approach_timeout_s", 30.0);
    res_v_ = declare_parameter<double>("residual_v", 0.02);          // 시뮬 res_v · res_w (행동 1 = 이만큼)
    res_w_ = declare_parameter<double>("residual_w", 0.1);
    for (const char * name : {"dock_box", "park_home", "drive_arc", "drive_straight"}) {
      Policy p;
      p.model = load_model(name);
      std::string why;
      if (p.model.use && (!p.net.load(p.model.policy, why) || p.net.obs_dim() != ml::NAV_OBS || p.net.act_dim() != 2)) {
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
    serve("approach", [this](const SkillGoal & g, Done done) { start(g, done); });
  }

protected:
  void stop() override {
    if (timer_) timer_->cancel();
    halt();
  }

private:
  struct Policy { SkillModel model; ml::MlpPolicy net; };

  void start(const SkillGoal & g, Done done) {
    double x, y, yaw;
    if (!robot_pose(x, y, yaw)) { done(false, "approach_no_tf"); return; }
    const std::string name = g.target.empty() ? "dock_box" : g.target;
    const auto it = policies_.find(name);
    if (it == policies_.end()) { done(false, "approach_bad_policy"); return; }
    pol_ = &it->second;
    const auto & q = g.pose.pose.orientation;
    gx_ = g.pose.pose.position.x;
    gy_ = g.pose.pose.position.y;
    gyaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    tol_m_ = pol_->model.num("tol_m", 0.01);
    tol_yaw_ = pol_->model.num("tol_deg", 1.5) * M_PI / 180.0;
    hold_s_ = pol_->model.num("hold_s", 0.5);
    limit_s_ = pol_->model.num("limit_s", 25.0);
    // 이미 목표 방향이고 옆 어긋남이 작으면 바로 직선 (legacy)
    const double lat0 = -(gx_ - x) * std::sin(gyaw_) + (gy_ - y) * std::cos(gyaw_);
    stage_ = std::abs(std::remainder(gyaw_ - yaw, 2 * M_PI)) < 0.15 && std::abs(lat0) < 0.03 ? 2 : 0;
    best_ = 1e9;
    still_s_ = act0_ = act1_ = last_v_ = last_w_ = 0.0;
    started_ = progress_ = now();
    done_ = done;
    stage(name + (mode_ == "sim" && pol_->model.use ? " (기준 제어 + 학습 보정)" : " (기준 제어)") + " → (" +
          std::to_string(gx_).substr(0, 5) + ", " + std::to_string(gy_).substr(0, 5) + ")");
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { mode_ == "sim" ? tick_sim() : tick_legacy(); });
  }

  bool robot_pose(double & x, double & y, double & yaw) const {
    try {
      const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero).transform;
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

  // 시뮬 drive_env._baseline과 같은 식 (isaac/lab/drive_env.py, demo_cycle drive_to)
  void tick_sim() {
    double x, y, yaw;
    if (!robot_pose(x, y, yaw)) return;
    const auto wrap = [](double a) { return std::remainder(a, 2 * M_PI); };
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double gx = c * (gx_ - x) + s * (gy_ - y), gy = -s * (gx_ - x) + c * (gy_ - y), gyaw = wrap(gyaw_ - yaw);
    const double dist = std::hypot(gx, gy), head = std::atan2(gy, gx);
    const bool far = dist > 0.01, turn_first = far && std::abs(head) > 10.0 * M_PI / 180.0;
    double v = far && !turn_first ? std::clamp(1.5 * gx, -0.12, 0.12) : 0.0;
    double w = turn_first ? std::clamp(3.0 * head, -1.0, 1.0) : far ? std::clamp(4.0 * gy, -0.5, 0.5) : std::clamp(3.0 * gyaw, -1.0, 1.0);
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
    residual(x, y, yaw, dv, dw);
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
  double speed_, pre_m_, pos_tol_, yaw_tol_, timeout_s_, res_v_, res_w_;
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
