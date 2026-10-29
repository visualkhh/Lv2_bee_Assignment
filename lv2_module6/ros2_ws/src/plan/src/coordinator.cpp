// coordinator 구현 — 노드 설명·입출력·멤버 선언은 include/plan/coordinator.hpp, main은 coordinator_main.cpp
#include "plan/coordinator.hpp"

#include <algorithm>

#include <cmath>
#include <cstdio>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace plan {

std::vector<std::pair<double, double>> tray_slots(const std::vector<double> & flat) {
  std::vector<std::pair<double, double>> out;
  for (size_t i = 0; i + 1 < flat.size(); i += 2) out.push_back({flat[i], flat[i + 1]});
  return out;
}

geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff) {
  double dx = ox - rx, dy = oy - ry;
  const double d = std::hypot(dx, dy);
  if (d < 1e-3) { dx = 1.0; dy = 0.0; } else { dx /= d; dy /= d; }  // 로봇 → 물체 방향
  geometry_msgs::msg::Pose p;
  p.position.x = ox - dx * standoff;
  p.position.y = oy - dy * standoff;
  const double yaw = std::atan2(dy, dx);  // 물체를 바라봄
  p.orientation.z = std::sin(yaw / 2);
  p.orientation.w = std::cos(yaw / 2);
  return p;
}

std::pair<double, double> spread_offset(int n, double s) {
  static const int k[][2] = {{0, 0}, {0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  const int i = n % 9, ring = n / 9 + 1;  // 9개 넘으면 한 칸 더 바깥으로
  return {k[i][0] * s * ring, k[i][1] * s * ring};
}

Coordinator::Coordinator()
: rclcpp::Node("coordinator") {
  max_grasp_tries_ = declare_parameter<int>("max_grasp_tries", 2);
  max_step_failures_ = declare_parameter<int>("max_step_failures", 2);
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  arm_plan_tries_ = declare_parameter<int>("arm_plan_tries", 3);
  tray_slots_ = tray_slots(declare_parameter<std::vector<double>>(
    "tray_slots", {-0.135, 0.05, -0.135, -0.05, -0.095, 0.05, -0.095, -0.05}));
  tray_full_.assign(tray_slots_.size(), false);
  tray_release_z_ = declare_parameter<double>("tray_release_z", 0.22);
  tray_grasp_z_ = declare_parameter<double>("tray_grasp_z", 0.17);
  tray_approach_m_ = declare_parameter<double>("tray_approach_m", 0.05);
  place_spacing_ = declare_parameter<double>("place_spacing", 0.08);
  place_approach_m_ = declare_parameter<double>("place_approach_m", 0.08);
  place_release_dz_ = declare_parameter<double>("place_release_dz", 0.04);
  approach_speed_ = declare_parameter<double>("approach_speed", 0.05);
  approach_pre_m_ = declare_parameter<double>("approach_pre_m", 0.10);
  approach_pos_tol_ = declare_parameter<double>("approach_pos_tol", 0.005);
  approach_yaw_tol_ = declare_parameter<double>("approach_yaw_tol", 0.03);
  approach_timeout_s_ = declare_parameter<double>("approach_timeout_s", 30.0);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav", rclcpp::QoS(10));
  reach_m_ = declare_parameter<double>("reach_m", 0.30);
  approach_m_ = declare_parameter<double>("approach_m", 0.24);
  inspect_height_m_ = declare_parameter<double>("inspect_height_m", 0.15);
  inspect_wait_s_ = declare_parameter<double>("inspect_wait_s", 1.0);
  initial_enabled_ = declare_parameter<bool>("initial_pose_enabled", true);
  initial_joints_ = declare_parameter<std::vector<std::string>>(
    "initial_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  initial_pose_ = declare_parameter<std::vector<double>>(
    "initial_pose", {0.0, 0.0, 0.0, 0.0, 0.0});
  initial_tol_ = declare_parameter<double>("initial_pose_tolerance", 0.05);
  initial_move_s_ = declare_parameter<double>("initial_pose_move_s", 3.0);
  initial_retries_ = declare_parameter<int>("initial_pose_retries", 1);
  initial_gripper_ = declare_parameter<double>("initial_gripper", 0.0);
  arm_traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", rclcpp::QoS(10));
  scan_joints_ = declare_parameter<std::vector<std::string>>("scan_pose_joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  scan_pose_ = declare_parameter<std::vector<double>>("scan_pose", {2.618, 0.681, -1.414, 1.082, 0.0});
  scan_tol_ = declare_parameter<double>("scan_pose_tolerance", 0.05);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  status_pub_ = create_publisher<TaskStatus>("/task/status", latched);
  say_pub_ = create_publisher<std_msgs::msg::String>("/robot/say", rclcpp::QoS(10).reliable());
  goals_sub_ = create_subscription<SubGoalArray>("/task/sub_goals", latched,
    [this](SubGoalArray::ConstSharedPtr m) { on_plan(*m); });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      const bool first = !objects_;
      objects_ = m;
      if (first && waiting_plan_) {  // 물체 목록보다 계획이 먼저 왔으면 지금 검증
        auto plan = *waiting_plan_;
        waiting_plan_.reset();
        on_plan(plan);
      }
    });
  safety_sub_ = create_subscription<ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable(),
    [this](ArmSafetyEvent::ConstSharedPtr m) {
      RCLCPP_WARN(get_logger(), "팔 접촉 %s %.2fA", m->joint_name.c_str(), m->current_a);
      arm_contact_ = true;  // 진행 중인 MTC 결과가 실패로 오면 reason을 arm_contact로
    });
  mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_mode(m->data); });
  estop_sub_ = create_subscription<std_msgs::msg::Bool>("/state/emergency", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr m) {
      const bool e = m->data;
      if (e == estop_) return;
      estop_ = e;
      RCLCPP_WARN(get_logger(), "비상정지 %s", e ? "ON — 진행 중 전부 취소" : "OFF");
      if (e && running_) {
        cancel_all();
        running_ = false;
        status(plan_.goals.empty() ? 0 : plan_.goals[idx_].step, TaskStatus::FAILED, "estop");
      }
    });
  map_mode_sub_ = create_subscription<std_msgs::msg::String>("/state/map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { map_mode_ = m->data; check_scan_pose(); });
  joints_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) joints_[m->name[i]] = m->position[i];
    });
  // 시작 자세는 1초부터 자주 확인 (팔이 늦게 뜨는 경우 대비), 스캔 자세는 기존대로 2초
  initial_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { check_initial_pose(); });
  scan_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { check_scan_pose(); });
  // initial_pose_enabled 실행 중 변경 — ros2 param set /coordinator initial_pose_enabled false
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
  nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  motion_plan_ = create_client<GetMotionPlan>("/plan_kinematic_path");
  exec_ = rclcpp_action::create_client<ExecuteTrajectory>(this, "/execute_trajectory");
  gripper_ = rclcpp_action::create_client<ParallelGripperCommand>(this, "/gripper_controller/gripper_cmd");
  score_ = create_client<ScoreGrasps>("/score_grasps");
  clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
  apply_scene_ = create_client<ApplyPlanningScene>("/apply_planning_scene");
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (mode_ != "AUTO") s.summary(DiagnosticStatus::WARN, mode_ + " 모드 — 자동 실행 멈춤");
    else if (waited_s_ > 0) s.summary(DiagnosticStatus::WARN, "Nav2·MoveIt 서버 대기");
    else if (running_) s.summary(DiagnosticStatus::OK, "실행 중: " + step_text());
    else s.summary(DiagnosticStatus::OK, "대기");
    s.add("모드", mode_);
    s.add("plan_id", plan_.plan_id);
    s.add("단계", step_text());
    s.add("실행 중", running_);
    s.add("연속 실패", failures_.size());
    s.add("적재함", std::to_string(std::count(tray_full_.begin(), tray_full_.end(), true)) + "/" + std::to_string(tray_slots_.size()));
  });
}

void Coordinator::on_mode(const std::string & mode) {
  if (mode == mode_) return;
  mode_ = mode;
  RCLCPP_WARN(get_logger(), "운영 모드: %s", mode.c_str());
  if (mode == "TEACH") {  // 손으로 자세를 잡는 중 — 시작 자세로 끌고 가지 않는다 (나온 뒤에도 그 자세 그대로)
    initial_waiting_ = false;
    if (initial_timer_) initial_timer_->cancel();
  }
  if (mode != "AUTO" && running_) {  // 수동 · 자동 탐사(EXPLORE) · TEACH — 작업 실행과 동시에 하면 안 됨
    cancel_all();
    running_ = false;
    status(idx_ < plan_.goals.size() ? plan_.goals[idx_].step : 0, TaskStatus::FAILED,
           mode == "MANUAL" ? "manual_override" : mode == "TEACH" ? "manual_teach_override" : "explore_override");
  }
}

std::string Coordinator::step_text() const {
  if (idx_ >= plan_.goals.size()) return "-";
  const auto & g = plan_.goals[idx_];
  return std::to_string(g.step) + " " + g.skill + "(" + g.target + ")";
}

namespace {
bool same_goals(const SubGoalArray & a, const SubGoalArray & b) {
  if (a.goals.size() != b.goals.size()) return false;
  for (size_t i = 0; i < a.goals.size(); ++i) {
    const auto & x = a.goals[i], & y = b.goals[i];
    if (x.step != y.step || x.skill != y.skill || x.target != y.target ||
        x.relation != y.relation || x.offset_m != y.offset_m)
      return false;
  }
  return true;
}
}  // namespace

void Coordinator::on_plan(const SubGoalArray & plan) {
  // transient_local 재수신은 무시 — 단, 같은 id에 내용이 다르면 다른 계획 (VLM 카운터와 sim 직접 발행 충돌 대비)
  if (estop_) {  // 비상정지 중엔 새 계획도 안 받음
    RCLCPP_WARN(get_logger(), "계획 %u 거부: 비상정지 중", plan.plan_id);
    status(0, TaskStatus::FAILED, "estop_blocked");
    return;
  }
  if (plan.plan_id == plan_.plan_id && running_ && same_goals(plan, plan_)) return;
  if (plan.plan_id == 0 && plan.goals.empty()) {  // 명시적 정지 (sim 정지 버튼 — VLM 카운터는 1부터라 충돌 없음)
    if (running_) cancel_all();
    running_ = false;
    plan_ = plan;
    RCLCPP_WARN(get_logger(), "정지 요청 — 진행 중 전부 취소");
    status(0, TaskStatus::FAILED, "cancelled");
    return;
  }
  if (mode_ != "AUTO") {
    plan_ = plan;
    RCLCPP_WARN(get_logger(), "계획 %u 거부: %s 모드", plan.plan_id, mode_.c_str());
    status(0, TaskStatus::FAILED, mode_ == "MANUAL" ? "manual_mode" : mode_ == "TEACH" ? "manual_teach_mode" : "explore_mode");
    return;
  }
  if (!objects_) {  // 대상 id를 검증할 수 없음 — 첫 /tracked_objects까지 미룬다
    RCLCPP_INFO(get_logger(), "계획 %u: /tracked_objects 대기 후 검증", plan.plan_id);
    waiting_plan_ = std::make_shared<SubGoalArray>(plan);
    return;
  }
  std::string why;
  if (!validate(plan, why)) {
    plan_ = plan;
    RCLCPP_ERROR(get_logger(), "계획 %u 거부: %s", plan.plan_id, why.c_str());
    status(0, TaskStatus::FAILED, "invalid_plan:" + why);
    return;
  }
  RCLCPP_INFO(get_logger(), "계획 %u 수락 (%zu steps)", plan.plan_id, plan.goals.size());
  if (running_) cancel_all();  // 진행 중 goal 전부 취소 — 옛 결과가 새 계획을 건드리지 않게
  plan_ = plan;
  idx_ = 0;
  approached_ = static_cast<size_t>(-1);
  placed_.clear();
  running_ = true;
  run_step();
}

bool Coordinator::validate(const SubGoalArray & plan, std::string & why) const {
  static const std::set<std::string> allowed = {"navigate", "approach", "pick", "place", "inspect", "pose", "say"};
  if (plan.goals.empty()) { why = "empty"; return false; }
  for (const auto & g : plan.goals) {
    if (!allowed.count(g.skill)) { why = "skill " + g.skill; return false; }
    if (is_id(g.target) && !find(std::stoul(g.target))) { why = "target " + g.target + " not tracked"; return false; }
  }
  return true;
}

bool Coordinator::is_id(const std::string & s) {
  return !s.empty() && s.find_first_not_of("0123456789") == std::string::npos;
}

const perception::msg::TrackedObject * Coordinator::find(uint32_t id) const {
  if (objects_) for (const auto & o : objects_->objects) if (o.id == id) return &o;
  return nullptr;
}

void Coordinator::run_step() {
  if (!running_ || mode_ != "AUTO" || estop_) return;
  if (idx_ >= plan_.goals.size()) {
    running_ = false;
    status(plan_.goals.back().step, TaskStatus::SUCCEEDED, "");
    const bool carried = std::any_of(plan_.goals.begin(), plan_.goals.end(),
      [](const SubGoal & s) { return s.skill == "place"; });
    if (carried) say("퍽을 옮겼어요");  // inspect만 한 계획(퍽 번호 보기)은 말 없이 — planner가 이어서 계획
    return;
  }
  const auto & g = plan_.goals[idx_];
  status(g.step, TaskStatus::RUNNING, "");
  RCLCPP_INFO(get_logger(), "step %u: %s %s", g.step, g.skill.c_str(), g.target.c_str());
  if (g.skill == "navigate") navigate(g);
  else if (g.skill == "approach") approach(g);
  else if (g.skill == "pick") pick(g);
  else if (g.skill == "inspect") inspect(g);
  else if (g.skill == "pose") {  // 팔 관절 자세 — scan (pole_front_cam 시야를 안 가림) · initial
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready(), "arm_unavailable")) return;
    const auto & joints = g.target == "scan" ? scan_joints_ : initial_joints_;
    const auto & pos = g.target == "scan" ? scan_pose_ : initial_pose_;
    if (g.target != "scan" && g.target != "initial") { fail("pose_unknown"); return; }
    std::map<std::string, double> target;
    for (size_t i = 0; i < joints.size() && i < pos.size(); ++i) target[joints[i]] = pos[i];
    const uint64_t gen = gen_;
    move_joints(target, [this, gen](bool ok) { if (gen == gen_) ok ? next() : fail("pose_failed"); }, false);
  }
  else if (g.skill == "place") {
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready() &&
        gripper_->action_server_is_ready(), "arm_unavailable")) return;
    if (g.target.rfind("tray", 0) == 0) {  // 적재함에 싣기 — "tray" 첫 빈 칸 · "tray:<칸>" 그 칸. 칸 위 tray_release_z에서 벌림
      const int slot = tray_slot(g.target, true);
      if (slot < 0) { fail(slot == -1 ? "tray_full" : "tray_slot_bad"); return; }
      execute_place(tray_pose(slot, tray_release_z_), [this, slot](bool ok) {
        if (!ok) { fail("tray_place_failed"); return; }
        tray_full_[slot] = true;
        next();
      }, 0.0);
      return;
    }
    if (const auto v = parse_numbers(g.target); v.size() == 3) {  // 좌표 place — 퍽 바닥이 z가 되게
      geometry_msgs::msg::PoseStamped release;
      release.header.frame_id = "map";
      release.header.stamp = now();
      release.pose.position.x = v[0];
      release.pose.position.y = v[1];
      release.pose.position.z = v[2] + place_release_dz_;
      release.pose.orientation.w = 1.0;
      if (double rx, ry; robot_xy(rx, ry))
        RCLCPP_INFO(get_logger(), "place 점까지 수평 %.3fm (로봇 %.3f,%.3f)", std::hypot(v[0] - rx, v[1] - ry), rx, ry);
      place_at(release, [this](bool ok) { ok ? next() : fail("place_failed"); });
      return;
    }
    if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("place_no_target"); return; }
    if (approach_if_far(g)) return;  // 내려놓을 곳이 팔 밖이면 먼저 다가감
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = now();
    target.pose = find(std::stoul(g.target))->pose.pose;
    const auto [dx, dy] = spread_offset(placed_[g.target], place_spacing_);  // 같은 곳에 여러 개면 비켜서
    target.pose.position.x += dx;
    target.pose.position.y += dy;
    execute_place(target, [this, id = g.target](bool ok) {
      if (!ok) { fail("place_failed"); return; }
      ++placed_[id];
      next();
    });
  } else { say(g.target); next(); }
}

bool Coordinator::ready_or_retry(bool ready, const std::string & reason) {
  if (ready) { waited_s_ = 0; return true; }
  if (waited_s_ >= server_wait_s_) { waited_s_ = 0; fail(reason); return false; }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s — 서버 대기 중", reason.c_str());
  waited_s_ += 1.0;
  retry_ = create_wall_timer(std::chrono::seconds(1), [this]() { retry_->cancel(); run_step(); });
  return false;
}

void Coordinator::next() {
  if (!running_) return;  // 수동 전환 등으로 이미 멈춘 뒤 도착한 결과
  failures_.erase(key(plan_.goals[idx_]));
  ++idx_;
  run_step();
}

void Coordinator::fail(const std::string & reason) {
  if (!running_) return;  // 이미 멈춤 (수동 전환으로 취소된 goal의 결과 등)
  const auto & g = plan_.goals[idx_];
  const int n = ++failures_[key(g)];
  running_ = false;
  RCLCPP_WARN(get_logger(), "step %u 실패 (%s) %d/%d", g.step, reason.c_str(), n, max_step_failures_);
  status(g.step, TaskStatus::FAILED, reason);
  if (n >= max_step_failures_) {
    failures_.clear();
    say(g.skill == "navigate" ? "목표까지 갈 수 없어요" : "퍽을 잡을 수 없어요");
  }
}

std::string Coordinator::key(const SubGoal & g) { return g.skill + ":" + g.target; }

std::vector<double> parse_numbers(const std::string & target) {
  std::vector<double> out;
  size_t start = 0;
  while (start <= target.size()) {
    const size_t comma = std::min(target.find(',', start), target.size());
    const std::string part = target.substr(start, comma - start);
    try {
      size_t used = 0;
      out.push_back(std::stod(part, &used));
      if (part.find_first_not_of(" ", used) != std::string::npos) return {};
    } catch (const std::exception &) {
      return {};
    }
    start = comma + 1;
  }
  return out.size() >= 2 ? out : std::vector<double>{};
}

bool Coordinator::at_scan_pose() const {
  for (size_t i = 0; i < scan_joints_.size() && i < scan_pose_.size(); ++i) {
    const auto it = joints_.find(scan_joints_[i]);
    if (it == joints_.end()) return false;
    const double d = std::remainder(it->second - scan_pose_[i], 2 * M_PI);  // joint1은 ±π가 같은 자리
    if (std::abs(d) > scan_tol_) return false;
  }
  return true;
}

bool Coordinator::at_initial_pose() const {
  for (size_t i = 0; i < initial_joints_.size() && i < initial_pose_.size(); ++i) {
    const auto it = joints_.find(initial_joints_[i]);
    if (it == joints_.end()) return false;
    if (std::abs(std::remainder(it->second - initial_pose_[i], 2 * M_PI)) > initial_tol_) return false;
  }
  return !joints_.empty();
}

std::string Coordinator::initial_pose_error() const {
  std::string out;
  for (size_t i = 0; i < initial_joints_.size() && i < initial_pose_.size(); ++i) {
    const auto it = joints_.find(initial_joints_[i]);
    if (it == joints_.end()) {
      out += (out.empty() ? "" : ", ") + initial_joints_[i] + " 모름";
      continue;
    }
    const double d = std::remainder(it->second - initial_pose_[i], 2 * M_PI);
    if (std::abs(d) <= initial_tol_) continue;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s %+.2f", initial_joints_[i].c_str(), d);
    out += (out.empty() ? "" : ", ") + std::string(buf);
  }
  return out.empty() ? "차이 없음" : out + " rad";
}

void Coordinator::hold_arm() {
  std::vector<std::string> names;
  std::vector<double> pos;
  for (const auto & j : initial_joints_) {
    const auto it = joints_.find(j);
    if (it == joints_.end()) continue;
    names.push_back(j);
    pos.push_back(it->second);
  }
  if (!names.empty()) publish_arm_targets(names, pos, 0.5);
}

void Coordinator::publish_arm_targets(const std::vector<std::string> & names,
                                      const std::vector<double> & pos, double seconds) {
  if (!arm_traj_pub_) return;
  trajectory_msgs::msg::JointTrajectory traj;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  for (size_t i = 0; i < names.size() && i < pos.size(); ++i) {
    traj.joint_names.push_back(names[i]);
    point.positions.push_back(pos[i]);
  }
  point.time_from_start = rclcpp::Duration::from_seconds(seconds);
  traj.points.push_back(point);
  arm_traj_pub_->publish(traj);
}

void Coordinator::check_initial_pose() {
  // bringup 직후 한 번, 손으로 잡아 둔 초기 자세로 팔을 옮긴다.
  //  - MAPPING/EXPLORE면 스캔 자세가 우선이라 건너뜀
  //  - estop·진행 중 작업·비상정지 중에는 안 함
  //  - 이미 그 자세면 바로 완료
  if (!initial_enabled_) return;
  if (initial_waiting_) {
    // 보낸 뒤 도달 확인 — 도달하면 타이머 자체를 끔, 6초 넘게 못 가면 한 번 더 보냄 (JTC 재시작·일시 거절 대비)
    if (at_initial_pose()) {
      initial_waiting_ = false;
      if (initial_gripper_sent_ || initial_gripper_ < 0.0) initial_timer_->cancel();  // 그리퍼가 남았으면 다음 확인 때
      RCLCPP_INFO(get_logger(), "팔 초기 자세 완료");
    } else if (std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - initial_sent_).count() > initial_move_s_ + 3.0) {
      initial_waiting_ = false;
      if (initial_sent_count_ > initial_retries_) {
        // 어딘가에 걸려 못 가는 것 — 계속 밀면 모터 과부하(Hardware Error)로 멈춘다
        initial_timer_->cancel();
        hold_arm();
        RCLCPP_ERROR(get_logger(), "팔 초기 자세 %d번 미도달 — 포기하고 지금 자세로 멈춤 (%s). "
                     "plan.yaml initial_pose 확인", initial_sent_count_, initial_pose_error().c_str());
      } else {
        RCLCPP_WARN(get_logger(), "팔 초기 자세 미도달 (%s) — 다시 보냄", initial_pose_error().c_str());
      }
    }
    return;
  }
  if (estop_ || running_ || joints_.empty()) return;
  if (!initial_gripper_sent_ && initial_gripper_ >= 0.0 && gripper_->action_server_is_ready()) {
    initial_gripper_sent_ = true;  // 한 번만 — 실패해도 다시 안 보냄 (쥔 채 계속 힘주지 않게)
    RCLCPP_INFO(get_logger(), "그리퍼를 시작 값으로 %.4fm", initial_gripper_);
    set_gripper(initial_gripper_, [this](bool ok) {
      if (!ok) RCLCPP_WARN(get_logger(), "그리퍼 시작 값 실패 — plan.yaml initial_gripper 확인");
    });
  }
  if (map_mode_ == "MAPPING" || mode_ == "EXPLORE") {
    initial_timer_->cancel();  // 스캔 자세(coordinator check_scan_pose)가 대신 잡는다
    return;
  }
  if (at_initial_pose()) {
    if (initial_gripper_sent_ || initial_gripper_ < 0.0) {  // 그리퍼 서버가 아직 없으면 1초마다 다시 확인
      initial_timer_->cancel();
      RCLCPP_INFO(get_logger(), "팔 초기 자세 확인 끝");
    }
    return;
  }
  if (arm_traj_pub_->get_subscription_count() == 0) return;  // JTC 아직 없음
  initial_waiting_ = true;
  ++initial_sent_count_;
  initial_sent_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "팔을 초기 자세로 이동 (%.1fs)", initial_move_s_);
  publish_arm_targets(initial_joints_, initial_pose_, initial_move_s_);
}

void Coordinator::check_scan_pose() {
  // 지도를 만드는 중(스캔 · 자동 탐사)이고 작업이 없을 때 — 작업(pick 등)이 끝나면 다음 확인 때 돌아감.
  // 수동 모드여도 (스캔은 보통 수동으로 몰고 다님 — 팔 수동 조작은 MAPPING 동안 manual_control·service_action_input이 막음)
  if (map_mode_ != "MAPPING" && mode_ != "EXPLORE") return;
  if (estop_ || running_ || scan_moving_ || joints_.empty() || at_scan_pose()) return;
  if (!motion_plan_->service_is_ready() || !exec_->action_server_is_ready()) return;
  std::map<std::string, double> target;
  for (size_t i = 0; i < scan_joints_.size() && i < scan_pose_.size(); ++i) target[scan_joints_[i]] = scan_pose_[i];
  scan_moving_ = true;
  RCLCPP_INFO(get_logger(), "지도 만드는 중 — 팔을 스캔 자세로 (gripper_cam이 왼쪽 뒤를 봄)");
  move_joints(target, [this](bool ok) {
    scan_moving_ = false;
    if (!ok) RCLCPP_WARN(get_logger(), "스캔 자세로 못 감 — 2초 뒤 다시");
  });
}

void Coordinator::move_joints(const std::map<std::string, double> & target, std::function<void(bool)> done, bool background) {
  auto req = std::make_shared<GetMotionPlan::Request>();
  auto & r = req->motion_plan_request;
  r.group_name = "arm";
  r.start_state.is_diff = true;
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 5.0;
  r.max_velocity_scaling_factor = 0.3;
  r.max_acceleration_scaling_factor = 0.3;
  moveit_msgs::msg::Constraints c;
  for (const auto & [name, pos] : target) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.position = pos;
    jc.tolerance_above = jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    c.joint_constraints.push_back(jc);
  }
  r.goal_constraints.push_back(c);
  // 작업 취소(gen_)와 상관없이 끝은 늘 알림 — scan_moving_이 걸린 채 남지 않게
  motion_plan_->async_send_request(req, [this, done, background](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    const auto res = f.get();
    // 배경(스캔 자세 유지)이면 그새 작업이 시작됐을 때 실행하지 않음 · skill(pose)이면 그 작업 자체
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS || (background && running_)) {
      done(false);
      return;
    }
    ExecuteTrajectory::Goal goal;
    goal.trajectory = res->motion_plan_response.trajectory;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
    opt.goal_response_callback = [done](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr & h) {
      if (!h) done(false);
    };
    opt.result_callback = [done](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & res2) {
      done(res2.code == rclcpp_action::ResultCode::SUCCEEDED &&
        res2.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    };
    exec_->async_send_goal(goal, opt);
  });
}

void Coordinator::cancel_all() {
  ++gen_;
  if (retry_) retry_->cancel();
  if (approach_timer_ && !approach_timer_->is_canceled()) { approach_timer_->cancel(); stop_base(); }
  nav_->async_cancel_all_goals();
  exec_->async_cancel_all_goals();
  gripper_->async_cancel_all_goals();
}

bool Coordinator::robot_pose(double & x, double & y, double & yaw) const {
  try {
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    const auto & q = t.transform.rotation;
    yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void Coordinator::approach(const SubGoal & g) {
  const auto v = parse_numbers(g.target);
  double x, y, yaw;
  if (v.size() != 3) { fail("approach_bad_target"); return; }
  if (!robot_pose(x, y, yaw)) { fail("approach_no_tf"); return; }
  ap_x_ = v[0];
  ap_y_ = v[1];
  ap_yaw_ = v[2];
  // 이미 목표 방향이고 옆 어긋남이 작으면 바로 직선 (Nav2가 변 앞에 정렬해 준 경우 · 박스에서 똑바로 물러날 때)
  const double lat0 = -(ap_x_ - x) * std::sin(ap_yaw_) + (ap_y_ - y) * std::cos(ap_yaw_);
  ap_stage_ = std::abs(std::remainder(ap_yaw_ - yaw, 2 * M_PI)) < 0.15 && std::abs(lat0) < 0.03 ? 2 : 0;
  ap_best_ = 1e9;
  ap_started_ = ap_progress_ = now();
  RCLCPP_INFO(get_logger(), "approach (%.3f, %.3f, %.2f) — 지금 (%.3f, %.3f, %.2f)", ap_x_, ap_y_, ap_yaw_, x, y, yaw);
  approach_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { approach_tick(); });
}

void Coordinator::stop_base() {
  geometry_msgs::msg::TwistStamped z;
  z.header.stamp = now();
  z.header.frame_id = "base_footprint";
  cmd_vel_pub_->publish(z);
}

void Coordinator::approach_tick() {
  double x, y, yaw;
  if (!running_ || !robot_pose(x, y, yaw)) return;
  const double c = std::cos(ap_yaw_), s = std::sin(ap_yaw_);
  const double px = ap_x_ - approach_pre_m_ * c, py = ap_y_ - approach_pre_m_ * s;  // 목표 뒤 자리 (여기서 돈다)
  const auto wrap = [](double a) { return std::remainder(a, 2 * M_PI); };
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_footprint";
  double remain;
  const int stage0 = ap_stage_;
  if (ap_stage_ == 0) {  // 앞 자리로: 그쪽을 보고 → 직진 (이미 2 cm 안이면 건너뜀)
    remain = std::hypot(px - x, py - y);
    const double head = wrap(std::atan2(py - y, px - x) - yaw);
    if (remain < 0.02) ap_stage_ = 1;
    else if (std::abs(head) > 0.15) cmd.twist.angular.z = std::clamp(1.5 * head, -0.4, 0.4);
    else {
      cmd.twist.linear.x = std::min(approach_speed_ * 2, 0.8 * remain);
      cmd.twist.angular.z = std::clamp(1.5 * head, -0.3, 0.3);
    }
  } else if (ap_stage_ == 1) {  // 목표 방향으로 돌기
    const double e = wrap(ap_yaw_ - yaw);
    remain = std::abs(e) * 0.1;  // 진행 판정용 (회전 0.1 rad ≈ 1 cm로 셈)
    if (std::abs(e) < approach_yaw_tol_) ap_stage_ = 2;
    else cmd.twist.angular.z = std::clamp(1.5 * e, -0.4, 0.4) + (e > 0 ? 0.05 : -0.05);
  } else {  // 직선 전진 — 남은 거리는 목표 방향 축으로 · 옆 어긋남은 방향을 살짝 틀어 줄임
    const double along = (ap_x_ - x) * c + (ap_y_ - y) * s;
    const double lat = -(ap_x_ - x) * s + (ap_y_ - y) * c;
    remain = std::abs(along);
    if (remain < approach_pos_tol_) {
      approach_timer_->cancel();
      stop_base();
      RCLCPP_INFO(get_logger(), "approach 도착 — 남은 %.4f m · 옆 %.4f m · 방향 %.3f rad", along, lat, wrap(ap_yaw_ - yaw));
      next();
      return;
    }
    cmd.twist.linear.x = std::clamp(0.8 * along, -approach_speed_, approach_speed_);
    cmd.twist.angular.z = std::clamp(1.5 * wrap(ap_yaw_ - yaw) + (along > 0 ? 3.0 : -3.0) * lat, -0.2, 0.2);
  }
  const auto t = now();
  if (ap_stage_ != stage0) { ap_best_ = 1e9; ap_progress_ = t; }  // 단계가 바뀌면 진행 판정 처음부터 (남은 값 단위가 다름)
  else if (remain < ap_best_ - 0.003) { ap_best_ = remain; ap_progress_ = t; }  // 3 mm 넘게 줄면 진행
  if ((t - ap_progress_).seconds() > 4.0 || (t - ap_started_).seconds() > approach_timeout_s_) {
    approach_timer_->cancel();
    stop_base();
    RCLCPP_WARN(get_logger(), "approach 멈춤 (단계 %d · 남은 %.3f)", ap_stage_, remain);
    fail("approach_blocked");
    return;
  }
  cmd_vel_pub_->publish(cmd);
}

bool Coordinator::robot_xy(double & x, double & y) const {
  try {
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool Coordinator::approach_if_far(const SubGoal & g) {
  if (approached_ == idx_ || !is_id(g.target)) return false;  // 이 단계는 이미 다가감 → 그대로 실행 (안 닿으면 그 단계가 실패)
  const auto * o = find(std::stoul(g.target));
  double rx, ry;
  if (!o || !robot_xy(rx, ry)) return false;  // 로봇 위치를 모르면 판단 못 함 — 그대로 실행
  const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
  const double d = std::hypot(ox - rx, oy - ry);
  if (d <= reach_m_) return false;
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return true;
  approached_ = idx_;
  RCLCPP_INFO(get_logger(), "%s %s: 대상이 %.2fm — 팔이 닿게 %.2fm 앞까지 다가감", g.skill.c_str(), g.target.c_str(), d, approach_m_);
  send_nav(approach_pose(ox, oy, rx, ry, approach_m_), [this](bool ok) {
    if (ok) run_step();  // 같은 단계를 다시 (이번엔 다가가기를 건너뜀)
    else fail("approach_failed");
  });
  return true;
}

void Coordinator::navigate(const SubGoal & g) {
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return;
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  if (is_id(g.target)) {
    // 대상 앞 (물체 반 크기 + offset + 0.1) — 로봇이 있는 쪽에서 물체를 바라봄. TODO: 통로 방향 고려 (미로 벽에 붙지 않게)
    const auto * o = find(std::stoul(g.target));
    const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
    double rx = ox - 1.0, ry = oy;  // 로봇 위치를 모르면 물체의 −x 쪽 (예전과 같게)
    robot_xy(rx, ry);
    pose = approach_pose(ox, oy, rx, ry, std::max(o->size.x, o->size.y) / 2 + g.offset_m + 0.1);
  } else {
    const auto v = parse_numbers(g.target);  // 직접 지정 "x,y" (시뮬레이터 지도 클릭) · "x,y,yaw" (mission_manager)
    if (v.empty()) { fail("nav_bad_target"); return; }
    pose.position.x = v[0];
    pose.position.y = v[1];
    if (v.size() >= 3) {
      pose.orientation.z = std::sin(v[2] / 2);
      pose.orientation.w = std::cos(v[2] / 2);
    }
  }
  send_nav(pose, [this](bool ok) { ok ? next() : fail("nav_failed"); });
}

void Coordinator::send_nav(const geometry_msgs::msg::Pose & pose, std::function<void(bool)> done) {
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose = pose;
  const uint64_t gen = gen_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
  opt.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & h) {
      if (h) nav_goal_ = h;
    };
  opt.result_callback = [this, gen, done](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    done(r.code == rclcpp_action::ResultCode::SUCCEEDED);
  };
  nav_->async_send_goal(goal, opt);
}

void Coordinator::pick(const SubGoal & g) {
  if (g.target.rfind("tray", 0) == 0) {  // 적재함에서 꺼내기 — "tray" 마지막 찬 칸 · "tray:<칸>" 그 칸. 칸 위치를 알아서 grasp_scorer 없이
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready() &&
        gripper_->action_server_is_ready(), "pick_unavailable")) return;
    const int slot = tray_slot(g.target, false);
    if (slot < 0) { fail(slot == -1 ? "tray_empty" : "tray_slot_bad"); return; }
    execute_pick(tray_pose(slot, tray_grasp_z_), [this, slot](bool ok) {
      if (!ok) { fail("tray_pick_failed"); return; }
      tray_full_[slot] = false;
      next();
    }, tray_approach_m_);
    return;
  }
  if (!ready_or_retry(clear_octomap_->service_is_ready() && apply_scene_->service_is_ready() &&
      score_->service_is_ready() && motion_plan_->service_is_ready() &&
      exec_->action_server_is_ready() && gripper_->action_server_is_ready(), "pick_unavailable")) return;
  if (approach_if_far(g)) return;  // 집을 퍽이 팔 밖이면 먼저 다가감 (VLM이 navigate를 빼먹어도)
  const uint32_t object_id = std::stoul(g.target);
  clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
    [this, object_id](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
      auto scene = std::make_shared<ApplyPlanningScene::Request>();
      scene->scene.is_diff = true;  // TODO: 퍽·주변 장애물 CollisionObject 등록
      apply_scene_->async_send_request(scene, [this, object_id](rclcpp::Client<ApplyPlanningScene>::SharedFuture) {
        auto req = std::make_shared<ScoreGrasps::Request>();
        req->object_id = object_id;
        req->max_candidates = 3;
        score_->async_send_request(req, [this](rclcpp::Client<ScoreGrasps>::SharedFuture f) {
          const auto res = f.get();
          if (!res->success || res->grasps.empty()) { fail("no_grasp"); return; }
          try_grasp(res, 0);
        });
      });
    });
}

void Coordinator::inspect(const SubGoal & g) {
  if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready(), "arm_unavailable")) return;
  geometry_msgs::msg::PoseStamped above;
  above.header.frame_id = "map";
  above.header.stamp = now();
  if (const auto v = parse_numbers(g.target); v.size() == 3) {  // 좌표 inspect — 그 점(퍽 윗면 등) 위
    above.pose.position.x = v[0];
    above.pose.position.y = v[1];
    above.pose.position.z = v[2] + inspect_height_m_;
    above.pose.orientation.w = 1.0;
    double look = std::numeric_limits<double>::quiet_NaN();  // 손이 로봇 → 점 방향을 보게 (카메라가 그 점을 화면 가운데에)
    if (double rx, ry; robot_xy(rx, ry)) {
      look = std::atan2(v[1] - ry, v[0] - rx);
      RCLCPP_INFO(get_logger(), "inspect 점까지 수평 %.3fm (로봇 %.3f,%.3f)", std::hypot(v[0] - rx, v[1] - ry), rx, ry);
    }
    move_arm(above, [this, gen = gen_](bool ok) {
      if (!ok) { fail("inspect_failed"); return; }
      inspect_timer_ = create_wall_timer(std::chrono::duration<double>(inspect_wait_s_), [this, gen]() {
        inspect_timer_->cancel();
        if (gen == gen_) next();
      });
    }, -1, look);
    return;
  }
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("inspect_no_target"); return; }
  if (approach_if_far(g)) return;  // 팔 밖이면 먼저 다가감
  const auto * o = find(std::stoul(g.target));
  above.pose = o->pose.pose;
  above.pose.position.z += o->size.z / 2 + inspect_height_m_;  // 윗면 위 — 손가락 아래라 gripper_cam이 윗면을 봄
  const uint64_t gen = gen_;
  move_arm(above, [this, gen, id = g.target](bool ok) {
    if (!ok) { fail("inspect_failed"); return; }
    inspect_timer_ = create_wall_timer(std::chrono::duration<double>(inspect_wait_s_), [this, gen, id]() {
      inspect_timer_->cancel();
      if (gen != gen_) return;  // 그새 취소됨
      const auto * obj = find(std::stoul(id));
      const bool read = obj && std::any_of(obj->details.begin(), obj->details.end(),
        [](const diagnostic_msgs::msg::KeyValue & kv) { return kv.key == "tag"; });
      RCLCPP_INFO(get_logger(), "inspect %s: 태그 %s", id.c_str(), read ? "읽음" : "못 읽음");
      next();
    });
  });
}

void Coordinator::try_grasp(const ScoreGrasps::Response::SharedPtr & cands, size_t i) {
  if (i >= cands->grasps.size() || static_cast<int>(i) >= max_grasp_tries_) {
    fail(arm_contact_ ? "arm_contact" : "grasp_failed_all");
    return;
  }
  RCLCPP_INFO(get_logger(), "파지 후보 %zu (score %.2f)", i, cands->scores[i]);
  execute_pick(cands->grasps[i], [this, cands, i](bool ok) { ok ? next() : try_grasp(cands, i + 1); });
}

namespace {
constexpr double GRIPPER_OPEN_M = 0.0265;   // URDF 한계
constexpr double GRIPPER_CLOSED_M = 0.0;
}  // namespace

int Coordinator::tray_slot(const std::string & target, bool for_place) const {
  const int n = static_cast<int>(tray_full_.size());
  if (target == "tray") {  // 칸을 안 정하면 — 싣기는 첫 빈 칸, 꺼내기는 마지막 찬 칸 (예전 순서 그대로)
    if (for_place) { for (int i = 0; i < n; ++i) if (!tray_full_[i]) return i; }
    else { for (int i = n - 1; i >= 0; --i) if (tray_full_[i]) return i; }
    return -1;
  }
  const std::string k = target.substr(5);  // "tray:<칸>" (mission_manager — 칸 → tag 대응표를 그쪽이 가짐)
  if (target[4] != ':' || !is_id(k) || std::stoi(k) >= n) return -2;
  const int i = std::stoi(k);
  return tray_full_[i] != for_place ? i : -1;
}

geometry_msgs::msg::PoseStamped Coordinator::tray_pose(size_t slot, double z) const {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "base_footprint";  // 적재함은 로봇에 붙어 있음 — 로봇 기준
  p.pose.position.x = tray_slots_[slot].first;
  p.pose.position.y = tray_slots_[slot].second;
  p.pose.position.z = z;
  p.pose.orientation.w = 1.0;  // 방향은 move_arm이 위→아래로 정함
  return p;
}

void Coordinator::execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done,
                               double approach) {
  arm_contact_ = false;
  auto pre = grasp;
  pre.pose.position.z += approach;
  move_arm(pre, [this, grasp, done, approach](bool ok) {  // 접근
    if (!ok) { done(false); return; }
    set_gripper(GRIPPER_OPEN_M, [this, grasp, done, approach](bool ok2) {  // 벌리기
      if (!ok2) { done(false); return; }
      move_arm(grasp, [this, grasp, done, approach](bool ok3) {  // 파지 위치
        if (!ok3) { done(false); return; }
        set_gripper(GRIPPER_CLOSED_M, [this, grasp, done, approach](bool ok4) {  // 닫기
          if (!ok4) { done(false); return; }
          auto lift = grasp;
          lift.pose.position.z += approach;
          move_arm(lift, done);  // 들어올리기
        });
      });
    });
  });
}

void Coordinator::execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done,
                                double approach) {
  arm_contact_ = false;
  auto above = target;
  above.pose.position.z += approach;
  move_arm(above, [this, done](bool ok) {  // 위로 이동
    if (!ok) { done(false); return; }
    set_gripper(GRIPPER_OPEN_M, done);  // 벌리기
  });
}

void Coordinator::place_at(const geometry_msgs::msg::PoseStamped & release, std::function<void(bool)> done) {
  arm_contact_ = false;
  auto above = release;
  above.pose.position.z += place_approach_m_;
  move_arm(above, [this, release, above, done](bool ok) {  // 위
    if (!ok) { done(false); return; }
    move_arm(release, [this, above, done](bool ok2) {  // 내려가기
      if (!ok2) { done(false); return; }
      set_gripper(GRIPPER_OPEN_M, [this, above, done](bool ok3) {  // 벌리기
        if (!ok3) { done(false); return; }
        move_arm(above, done);  // 다시 위 (놓은 퍽을 건드리지 않게)
      });
    });
  });
}

void Coordinator::move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done, int tries_left,
                           double yaw) {
  if (tries_left < 0) tries_left = arm_plan_tries_ - 1;
  auto req = std::make_shared<GetMotionPlan::Request>();
  req->motion_plan_request.group_name = "arm";
  req->motion_plan_request.start_state.is_diff = true;
  req->motion_plan_request.num_planning_attempts = 3;
  req->motion_plan_request.allowed_planning_time = 5.0;
  req->motion_plan_request.max_velocity_scaling_factor = 0.3;
  req->motion_plan_request.max_acceleration_scaling_factor = 0.3;
  moveit_msgs::msg::Constraints goal;
  moveit_msgs::msg::PositionConstraint pc;
  pc.header = target.header;
  // 목표 = 손가락 끝(end_effector_link, gripper_link에서 0.109m) — 퍽 위치에 손목을 대면 손가락이 바닥을 뚫는다
  pc.link_name = "end_effector_link";
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.02, 0.02, 0.02};
  pc.constraint_region.primitives.push_back(box);
  pc.constraint_region.primitive_poses.push_back(target.pose);
  pc.weight = 1.0;
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header = target.header;
  oc.link_name = "end_effector_link";
  // 방향 = 항상 위에서 아래로 (손가락이 바닥을 봄: 손 z축 → 지면 아래, x축 180° 회전). 목표 물체의 방향(인지 = 항등)은 쓰지 않는다 —
  // 항등이면 손가락이 하늘을 봐서 바닥 퍽에 닿을 수 없다. 수직축 둘레 회전은 joint1·joint5 조합으로 정해지므로 풀어 둔다 (z = π)
  oc.orientation.x = 1.0;
  oc.orientation.y = 0.0;
  oc.orientation.z = 0.0;
  oc.orientation.w = 0.0;
  oc.absolute_x_axis_tolerance = 0.15;
  oc.absolute_y_axis_tolerance = 0.15;
  oc.absolute_z_axis_tolerance = M_PI;
  if (!std::isnan(yaw)) {  // 손 방향 고정: Rz(yaw) · Rx(π) = (cos(yaw/2), sin(yaw/2), 0, 0)
    oc.orientation.x = std::cos(yaw / 2);
    oc.orientation.y = std::sin(yaw / 2);
    oc.absolute_z_axis_tolerance = 0.15;
  }
  oc.weight = 1.0;
  goal.position_constraints.push_back(pc);
  goal.orientation_constraints.push_back(oc);
  req->motion_plan_request.goal_constraints.push_back(goal);
  const uint64_t gen0 = gen_;
  motion_plan_->async_send_request(req, [this, done, target, tries_left, gen0, yaw](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    if (gen0 != gen_) return;  // 그새 취소됨
    const auto res = f.get();
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(), "플랜 실패 (%d) — 남은 재시도 %d", res->motion_plan_response.error_code.val, tries_left);
      // IK·OMPL이 무작위라 닿는 곳도 가끔 못 찾음 (적재함 뒷줄 실측 9/12) → 같은 목표로 다시
      if (tries_left > 0) move_arm(target, done, tries_left - 1, yaw);
      else done(false);
      return;
    }
    ExecuteTrajectory::Goal goal;
    goal.trajectory = res->motion_plan_response.trajectory;
    const uint64_t gen = gen_;
    rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
    opt.result_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & r) {
      if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
      done(r.code == rclcpp_action::ResultCode::SUCCEEDED &&
        r.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    };
    exec_->async_send_goal(goal, opt);
  });
}

void Coordinator::set_gripper(double width_m, std::function<void(bool)> done) {
  ParallelGripperCommand::Goal goal;
  goal.command.position = {width_m};
  goal.command.effort = {1.0};
  const uint64_t gen = gen_;
  rclcpp_action::Client<ParallelGripperCommand>::SendGoalOptions opt;
  opt.result_callback = [this, done, width_m, gen](const rclcpp_action::ClientGoalHandle<ParallelGripperCommand>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
    if (!ok) RCLCPP_WARN(get_logger(), "그리퍼 %.3fm 실패", width_m);
    done(ok);
  };
  gripper_->async_send_goal(goal, opt);
}

void Coordinator::status(uint32_t step, uint8_t state, const std::string & reason) {
  TaskStatus s;
  s.plan_id = plan_.plan_id;
  s.step = step;
  s.state = state;
  s.reason = reason;
  status_pub_->publish(s);
}

void Coordinator::say(const std::string & text) {
  std_msgs::msg::String m;
  m.data = text;
  say_pub_->publish(m);
}

}  // namespace plan
