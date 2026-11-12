// RlNavController — Nav2 controller 플러그인 (ml 패키지) (ONNX 정책 + 기본 controller 바이패스).
//   관측 = 웨이포인트 5×2 + 가상 스캔 lidar_rays + v/w/gyro + 이전 행동 2
//   가상 스캔: nvblox 2D ESDF height slice(PointCloud2) → base_footprint lidar_rays방향 최소거리 (LiDAR 없음)
//   ~/<controller>/toggle (std_srvs/SetBool): false(기본)=inner controller, true=정책
//   ~/<controller>/status: [rl_active, reason, …] — CLI 노드 상태가 RL/DWB를 판별한다.
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/timer.hpp"
#include "ml/fallback_controller.hpp"
#include "ml/onnx_policy.hpp"
#include "ml/srv/set_fallback.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ml {

class RlNavController : public nav2_core::Controller {
public:
  void configure(
    const nav2::LifecycleNode::WeakPtr & parent, std::string name,
    nav2::TransformBuffer::SharedPtr tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void newPathReceived(const nav_msgs::msg::Path & raw_global_path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, const nav_msgs::msg::Path & transformed_global_plan,
    const geometry_msgs::msg::PoseStamped & global_goal) override;
  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

  // 관측 (학습 env와 같은 순서) — 테스트용으로 공개
  std::vector<float> waypoints(const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & plan) const;
  std::vector<float> scan_rays(const sensor_msgs::msg::LaserScan & scan, double laser_yaw) const;

private:
  void load_model(const std::string & path);
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  // RL/DWB 상태 — [rl_active, reason, policy_loaded, enable, cloud_ok, cloud_age, scan_min, v, w, inner_kind]
  //   reason: 0=RL, 1=모델 없음/로드 실패, 2=enable=false, 3=센서 없음/stale, 4=추론 실패
  //   inner_kind: 1=DWB, 2=MPPI, 3=RPP, 4=RotationShim, 0=기타
  void publish_status(int rl_active, int reason, const geometry_msgs::msg::TwistStamped & cmd);
  void publish_diagnostics();  // 1Hz /diagnostics — CLI '진단'/노드 상태가 읽는다

  nav2::LifecycleNode::WeakPtr node_;
  std::string name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("RlNavController")};

  pluginlib::ClassLoader<nav2_core::Controller> loader_{"nav2_core", "nav2_core::Controller"};
  nav2_core::Controller::Ptr inner_;
  OnnxPolicy policy_;

  nav2::ServiceServer<std_srvs::srv::SetBool>::SharedPtr toggle_srv_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;  // FollowPath.enable 즉시 적용
  nav2::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  nav2::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  nav2::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::mutex mtx_;
  // inner(바이패스) controller 는 실행 중에 바꿀 수 있다 (~/<name>/set_fallback) — inner_ 를 부르는 동안·바꾸는 동안은 inner_mtx_
  //   잠금 순서: mtx_ → inner_mtx_ (그 반대로 잡지 않는다)
  std::mutex inner_mtx_;
  std::string inner_key_ = "mppi";          // 지금 쓰는 inner 별칭
  bool active_ = false;                    // activate()~deactivate() — 새 inner 도 같은 상태로 만든다
  nav_msgs::msg::Path last_path_;          // 새 inner 에 다시 넘길 마지막 전역 경로
  double last_speed_limit_ = 0.0;
  bool last_speed_pct_ = false;
  bool have_speed_limit_ = false;
  nav2::ServiceServer<ml::srv::SetFallback>::SharedPtr fallback_srv_;
  // 바꾸기: 새 inner 를 먼저 만들고 설정·활성화까지 성공해야 교체한다 (실패하면 이전 inner 유지). {성공, 메시지}
  std::pair<bool, std::string> switch_inner(const std::string & want);
  bool enable_ = false;
  bool enable_requested_ = false;  // YAML/toggle 의도 — 모델이 없어 enable_을 내려도 이유를 구분한다
  bool use_scan_ = false;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  std::vector<float> ranges_;  // 가상 스캔 (m) — lidar_rays_개
  rclcpp::Time cloud_stamp_{0, 0, RCL_ROS_TIME};
  bool cloud_ok_ = false;
  size_t cloud_tf_fail_ = 0;
  double gyro_z_ = 0.0;
  bool gyro_seen_ = false;
  double laser_yaw_ = 0.0;
  bool laser_yaw_known_ = false;
  float prev_a_[2] = {0.0f, 0.0f};
  bool has_cmd_ = false;              // 한 번이라도 제어 주기가 돌았는지 (진단 'idle' 구분)
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};  // 마지막 제어 주기 — 1초 넘으면 진단은 idle
  int last_rl_active_ = 0;
  int last_reason_ = 2;               // publish_status가 갱신 — 진단에 그대로 노출
  float last_v_ = 0.0f, last_w_ = 0.0f;

  // 파라미터 (기본값 = 학습 env와 같게; 모델이 다르면 yaml에서 맞춘다)
  int inner_kind_ = 0;  // 1=DWB, 2=MPPI, 3=RPP, 4=RotationShim — status/CLI 표시용
  int lookahead_ = 5;
  double wp_step_ = 0.30, wp_scale_ = 2.0;
  int lidar_rays_ = 36;
  double lidar_max_ = 3.5;
  double lidar_inflation_ = 0.02;  // [m] 사용자 안전 여유 — 기본 2 cm (학습 1~5 cm)
  double max_lin_ = 0.26, max_ang_ = 1.82, v_min_ratio_ = -0.2;
  double speed_ratio_ = 1.0;
  bool heading_cap_ = true;     // 경로 방향 속도 상한 (heading_speed_cap)
  double extra_lin_vel_ = 0.0;  // [m/s] 정책 출력에 더하는 추가 선속도 (0~0.10, max_lin 상한 유지)
  double extra_ang_vel_ = 0.0;  // [rad/s] 정책 출력에 더하는 추가 각속도 (0~0.50, max_ang 상한 유지)
  int points_stride_ = 4;
  double z_min_ = 0.03, z_max_ = 1.2;
  double esdf_max_distance_ = 0.20;  // nvblox ESDF intensity(장애물까지 거리) 필터 [m]
  double cloud_timeout_ = 0.5;
};

}  // namespace ml
