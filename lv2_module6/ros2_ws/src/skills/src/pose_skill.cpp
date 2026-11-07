// skill_pose — /skills/pose: 이름 있는 관절 자세로 (stow · carry_left · carry_right · look · scan) — MoveIt 관절 목표. 학습 정책 없음 (move_pose 기준선 100 %)
//   팔을 맡는 기본 일도 이 노드 (예전 coordinator):
//   - 시작 자세: bringup 직후 한 번 팔을 initial_pose로 — MoveIt 없이 /arm_controller/joint_trajectory 토픽.
//     못 가면 initial_pose_retries번만 다시 보내고 지금 자세로 멈춤 (걸린 채 계속 밀면 과부하). initial_pose_enabled는 실행 중에도 바꿈
//   - 스캔 자세: 지도를 만들 때(/state/map_mode MAPPING · 자동 탐사 EXPLORE) 작업이 없으면 팔을 scan_pose로 (2초마다 확인)
//     — gripper_cam이 왼쪽 뒤를 보는 두 번째 고정 카메라 (ros_system/config/scan_pose.yaml · scan_cam_gate)
//   작업 중인지는 /task/status (RUNNING) 와 이 노드의 goal로 봄. 자세 값: skills/params/basket.yaml (isaac-lab slot_table)
#include <chrono>
#include <cmath>
#include <map>

#include "plan/msg/task_status.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/skill_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace skills {

class PoseSkill : public SkillNode {
public:
  explicit PoseSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_pose", options), arm_(*this) {
    for (const char * n : {"stow", "carry_left", "carry_right", "look"})  // look = inspect 상자 보기 1차 자세 (skill_inspect도 같은 pose_look)
      poses_[n] = declare_parameter<std::vector<double>>(std::string("pose_") + n, std::vector<double>{});
    // 값의 정본은 ros_system/config/scan_pose.yaml — skills.launch가 그 파일을 넘긴다.
    // 아래 기본값은 ros_system 없이 skill만 띄울 때의 fallback이다.
    scan_joints_ = declare_parameter<std::vector<std::string>>("scan_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    scan_pose_ = declare_parameter<std::vector<double>>("scan_pose", {2.618, 0.681, -1.414, 1.082, 0.0});
    scan_tol_ = declare_parameter<double>("scan_pose_tolerance", 0.05);
    // 매핑 중 자동 스캔 자세는 기본 끔 — 팔을 뒤로 크게 돌리면 무게중심이 왼쪽으로 쏠린다.
    // 필요하면 `ros2 param set /skill_pose scan_pose_enabled true`로 켠다 (pose scan 액션은 항상 사용 가능).
    declare_parameter<bool>("scan_pose_enabled", false);
    poses_["scan"] = scan_pose_;  // pose scan (mission_manager — 바코드·신호 볼 때 팔이 pole_front_cam 시야를 안 가림)
    initial_enabled_ = declare_parameter<bool>("initial_pose_enabled", true);
    initial_joints_ = declare_parameter<std::vector<std::string>>("initial_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
    initial_pose_ = declare_parameter<std::vector<double>>("initial_pose", {0.0, -1.9024, 1.344, 0.9023, 0.0});
    initial_tol_ = declare_parameter<double>("initial_pose_tolerance", 0.05);
    initial_move_s_ = declare_parameter<double>("initial_pose_move_s", 3.0);
    initial_retries_ = declare_parameter<int>("initial_pose_retries", 1);
    initial_gripper_ = declare_parameter<double>("initial_gripper", 0.0);  // [m] 음수 = 안 건드림
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", rclcpp::QoS(10));
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
      mode_ = m->data;
      if (mode_ == "TEACH") {  // 손으로 자세를 잡는 중 — 시작 자세로 끌고 가지 않는다
        initial_waiting_ = false;
        if (initial_timer_) initial_timer_->cancel();
      }
    });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>("/state/emergency", latched,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { estop_ = m->data; });
    map_mode_sub_ = create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) { map_mode_ = m->data; check_scan_pose(); });
    status_sub_ = create_subscription<plan::msg::TaskStatus>("/task/status", rclcpp::QoS(10).reliable().transient_local(),
      [this](plan::msg::TaskStatus::ConstSharedPtr m) { task_running_ = m->state == plan::msg::TaskStatus::RUNNING; });
    initial_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { check_initial_pose(); });
    scan_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { check_scan_pose(); });
    param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
      for (const auto & p : ps) {
        if (p.get_name() != "initial_pose_enabled") continue;
        initial_enabled_ = p.as_bool();
        if (!initial_enabled_) {
          initial_timer_->cancel();
          if (initial_waiting_) hold_arm();  // 가는 중이면 그 자리에서 멈춤
          initial_waiting_ = false;
          RCLCPP_INFO(get_logger(), "팔 초기 자세 끔");
        } else {
          initial_sent_count_ = 0;
          initial_timer_->reset();
          RCLCPP_INFO(get_logger(), "팔 초기 자세 켬 — 다시 시도");
        }
      }
    });
    serve("pose", [this](const SkillGoal & g, Done done) {
      const auto it = poses_.find(g.target);
      if (it == poses_.end() || it->second.size() != 5) { done(false, "pose_unknown"); return; }
      when_ready([this]() { return arm_.moveit_ready(); }, "arm_unavailable", done, [this, q = it->second, g, done]() {
        stage(g.target);
        arm_.move_joints(q, [done](bool ok) { done(ok, ok ? "" : "pose_failed"); });
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  bool busy_arm() const { return busy() || task_running_; }

  bool at(const std::vector<std::string> & names, const std::vector<double> & q, double tol) const {
    for (size_t i = 0; i < names.size() && i < q.size(); ++i) {
      double p;
      if (!arm_.joint(names[i], p)) return false;
      if (std::abs(std::remainder(p - q[i], 2 * M_PI)) > tol) return false;  // joint1은 ±π가 같은 자리
    }
    return !names.empty();
  }

  std::string initial_error() const {
    std::string out;
    for (size_t i = 0; i < initial_joints_.size() && i < initial_pose_.size(); ++i) {
      double p;
      if (!arm_.joint(initial_joints_[i], p)) { out += (out.empty() ? "" : ", ") + initial_joints_[i] + " 모름"; continue; }
      const double d = std::remainder(p - initial_pose_[i], 2 * M_PI);
      if (std::abs(d) <= initial_tol_) continue;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%s %+.2f", initial_joints_[i].c_str(), d);
      out += (out.empty() ? "" : ", ") + std::string(buf);
    }
    return out.empty() ? "차이 없음" : out + " rad";
  }

  void publish_targets(const std::vector<std::string> & names, const std::vector<double> & pos, double seconds) {
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
      traj.joint_names.push_back(names[i]);
      point.positions.push_back(pos[i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(seconds);
    traj.points.push_back(point);
    traj_pub_->publish(traj);
  }

  void hold_arm() {  // 지금 각도를 목표로 보내 JTC가 더 밀지 않게
    std::vector<std::string> names;
    std::vector<double> pos;
    for (const auto & j : initial_joints_) {
      double p;
      if (!arm_.joint(j, p)) continue;
      names.push_back(j);
      pos.push_back(p);
    }
    if (!names.empty()) publish_targets(names, pos, 0.5);
  }

  void check_initial_pose() {
    if (!initial_enabled_) return;
    double probe;
    const bool have_joints = arm_.joint(initial_joints_.empty() ? "joint1" : initial_joints_[0], probe);
    if (initial_waiting_) {  // 보낸 뒤 도달 확인 — 도달하면 타이머를 끔, 못 가면 한 번 더
      if (at(initial_joints_, initial_pose_, initial_tol_)) {
        initial_waiting_ = false;
        if (initial_gripper_sent_ || initial_gripper_ < 0.0) initial_timer_->cancel();
        RCLCPP_INFO(get_logger(), "팔 초기 자세 완료");
      } else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - initial_sent_).count() > initial_move_s_ + 3.0) {
        initial_waiting_ = false;
        if (initial_sent_count_ > initial_retries_) {
          initial_timer_->cancel();
          hold_arm();
          RCLCPP_ERROR(get_logger(), "팔 초기 자세 %d번 미도달 — 포기하고 지금 자세로 멈춤 (%s). skills.yaml initial_pose 확인",
                       initial_sent_count_, initial_error().c_str());
        } else {
          RCLCPP_WARN(get_logger(), "팔 초기 자세 미도달 (%s) — 다시 보냄", initial_error().c_str());
        }
      }
      return;
    }
    if (estop_ || busy_arm() || !have_joints) return;
    if (!initial_gripper_sent_ && initial_gripper_ >= 0.0 && arm_.gripper_ready()) {
      initial_gripper_sent_ = true;  // 한 번만 — 실패해도 다시 안 보냄 (쥔 채 계속 힘주지 않게)
      RCLCPP_INFO(get_logger(), "그리퍼를 시작 값으로 %.4fm", initial_gripper_);
      arm_.set_gripper(initial_gripper_, [this](bool ok) {
        if (!ok) RCLCPP_WARN(get_logger(), "그리퍼 시작 값 실패 — skills.yaml initial_gripper 확인");
      });
    }
    if (map_mode_ == "MAPPING" || mode_ == "EXPLORE") { initial_timer_->cancel(); return; }  // 스캔 자세가 대신 잡음
    if (at(initial_joints_, initial_pose_, initial_tol_)) {
      if (initial_gripper_sent_ || initial_gripper_ < 0.0) { initial_timer_->cancel(); RCLCPP_INFO(get_logger(), "팔 초기 자세 확인 끝"); }
      return;
    }
    if (traj_pub_->get_subscription_count() == 0) return;  // JTC 아직 없음
    initial_waiting_ = true;
    ++initial_sent_count_;
    initial_sent_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "팔을 초기 자세로 이동 (%.1fs)", initial_move_s_);
    publish_targets(initial_joints_, initial_pose_, initial_move_s_);
  }

  void check_scan_pose() {
    // 지도를 만드는 중이고 작업이 없을 때 — 작업이 끝나면 다음 확인 때 돌아감. 수동 모드여도 (스캔은 보통 수동 주행)
    if (!get_parameter("scan_pose_enabled").as_bool()) return;  // 실행 중 param set도 반영
    if (map_mode_ != "MAPPING" && mode_ != "EXPLORE") return;
    if (estop_ || busy_arm() || scan_moving_ || at(scan_joints_, scan_pose_, scan_tol_) || !arm_.moveit_ready()) return;
    double probe;
    if (!arm_.joint("joint1", probe)) return;
    scan_moving_ = true;
    RCLCPP_INFO(get_logger(), "지도 만드는 중 — 팔을 스캔 자세로 (gripper_cam이 왼쪽 뒤를 봄)");
    arm_.move_joints(scan_pose_, [this](bool ok) {
      scan_moving_ = false;
      if (!ok) RCLCPP_WARN(get_logger(), "스캔 자세로 못 감 — 2초 뒤 다시");
    });
  }

  ArmIO arm_;
  std::map<std::string, std::vector<double>> poses_;
  std::vector<std::string> scan_joints_, initial_joints_;
  std::vector<double> scan_pose_, initial_pose_;
  double scan_tol_ = 0.05, initial_tol_ = 0.05, initial_move_s_ = 3.0, initial_gripper_ = 0.0;
  int initial_retries_ = 1, initial_sent_count_ = 0;
  bool initial_enabled_ = true, initial_waiting_ = false, initial_gripper_sent_ = false, scan_moving_ = false;
  bool estop_ = false, task_running_ = false;
  std::string mode_ = "AUTO", map_mode_;
  std::chrono::steady_clock::time_point initial_sent_{};
  rclcpp::TimerBase::SharedPtr initial_timer_, scan_timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_, map_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<plan::msg::TaskStatus>::SharedPtr status_sub_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::PoseSkill)
