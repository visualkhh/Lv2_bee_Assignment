// 팔 skill 공통 — 설명은 include/skills/arm_io.hpp (예전 coordinator.cpp move_arm · move_joints · set_gripper · contact_step …)
#include "skills/arm_io.hpp"

#include <algorithm>
#include <cmath>

#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace skills {

bool grasp_held(double pos, double effort, double min_pos, double min_effort) {
  if (pos < min_pos) return false;  // 끝까지 닫힘 — 쥔 것 없음
  return std::isnan(effort) || std::abs(effort) >= min_effort;
}

namespace {
constexpr double CONTACT_VS = 0.3;  // 팀 관절 한계 × (시뮬 vel_scale)
constexpr double CONTACT_HOLD_S = 0.3, CONTACT_SETTLE_S = 0.3, CONTACT_CLOSE_S = 0.5, CONTACT_RELEASE_S = 0.4;
const char * ARM[] = {"joint1", "joint2", "joint3", "joint4", "joint5"};
}  // namespace

ArmIO::ArmIO(rclcpp::Node & node)
: node_(node) {
  arm_plan_tries_ = node.declare_parameter<int>("arm_plan_tries", 3);
  grip_current_a_ = node.declare_parameter<double>("grip_current_a", 0.35);  // 닫을 때 쥐는 힘 [A] — 팀 결정 0.3~0.4 A (22~29 N)
  grasp_check_ = node.declare_parameter<bool>("grasp_check", true);
  grasp_min_pos_m_ = node.declare_parameter<double>("grasp_min_pos_m", 0.003);
  grasp_min_effort_ = node.declare_parameter<double>("grasp_min_effort", 0.25);
  // sim: 닿는 구간은 시뮬과 같은 기준 동작 + 학습 보정 (2026-11-03 배포 결정) · moveit: 예전처럼 MoveIt만
  contact_mode_ = node.declare_parameter<std::string>("contact_mode", "sim");
  plan_ = node.create_client<GetMotionPlan>("/plan_kinematic_path");
  exec_ = rclcpp_action::create_client<ExecuteTrajectory>(&node, "/execute_trajectory");
  gripper_ = rclcpp_action::create_client<Gripper>(&node, "/gripper_controller/gripper_cmd");
  reset_sag_ = node.create_client<std_srvs::srv::Trigger>("/rl_arm_correction/reset_sag");
  ctx_pub_ = node.create_publisher<ml::msg::SkillContext>("/skill_context", rclcpp::QoS(10).reliable());
  hand_pub_ = node.create_publisher<perception::msg::HandTarget>("/hand_relative/target", rclcpp::QoS(1).reliable().transient_local());
  joints_sub_ = node.create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) pos_[m->name[i]] = m->position[i];
      for (size_t i = 0; i < m->name.size() && i < m->effort.size(); ++i) effort_[m->name[i]] = m->effort[i];
    });
  safety_sub_ = node.create_subscription<control::msg::ArmSafetyEvent>("/arm_safety_event", rclcpp::QoS(10).reliable(),
    [this](control::msg::ArmSafetyEvent::ConstSharedPtr m) {
      RCLCPP_WARN(node_.get_logger(), "팔 접촉 %s %.2fA", m->joint_name.c_str(), m->current_a);
      contact_ = true;  // 진행 중 결과가 실패로 오면 reason을 arm_contact로
    });
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool ArmIO::moveit_ready() const { return plan_->service_is_ready() && exec_->action_server_is_ready(); }
bool ArmIO::gripper_ready() const { return gripper_->action_server_is_ready(); }

void ArmIO::stop() {
  ++gen_;
  if (timer_) timer_->cancel();
  exec_->async_cancel_all_goals();
  gripper_->async_cancel_all_goals();
  ctx_off();
}

bool ArmIO::joint(const std::string & name, double & pos) const {
  const auto it = pos_.find(name);
  if (it == pos_.end()) return false;
  pos = it->second;
  return true;
}

bool ArmIO::arm_now(Q5 & q) const {
  for (int i = 0; i < 5; ++i)
    if (!joint(ARM[i], q[i])) return false;
  return true;
}

void ArmIO::execute(const moveit_msgs::msg::RobotTrajectory & traj, Cb done) {
  ExecuteTrajectory::Goal goal;
  goal.trajectory = traj;
  const uint64_t gen = gen_;
  rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr & h) {
    if (!h && gen == gen_) done(false);
  };
  opt.result_callback = [this, done, gen](const rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    done(r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  };
  exec_->async_send_goal(goal, opt);
}

void ArmIO::move_arm(const geometry_msgs::msg::PoseStamped & target, Cb done, double yaw, int tries_left) {
  if (tries_left < 0) tries_left = arm_plan_tries_ - 1;
  auto req = std::make_shared<GetMotionPlan::Request>();
  auto & r = req->motion_plan_request;
  r.group_name = "arm";
  r.start_state.is_diff = true;
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 5.0;
  r.max_velocity_scaling_factor = 0.3;
  r.max_acceleration_scaling_factor = 0.3;
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
  // 방향 = 늘 위에서 아래로 (손 z축 → 지면 아래, x축 180° 회전). 수직축 둘레는 joint1·joint5 조합이라 풀어 둠 (z = π)
  oc.orientation.x = 1.0;
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
  r.goal_constraints.push_back(goal);
  const uint64_t gen = gen_;
  plan_->async_send_request(req, [this, done, target, tries_left, gen, yaw](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    if (gen != gen_) return;  // 그새 취소됨
    const auto res = f.get();
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(node_.get_logger(), "플랜 실패 (%d) — 남은 재시도 %d", res->motion_plan_response.error_code.val, tries_left);
      if (tries_left > 0) move_arm(target, done, yaw, tries_left - 1);
      else done(false);
      return;
    }
    execute(res->motion_plan_response.trajectory, done);
  });
}

void ArmIO::move_joints(const std::vector<double> & q5, Cb done) {
  if (q5.size() != 5) { done(false); return; }
  auto req = std::make_shared<GetMotionPlan::Request>();
  auto & r = req->motion_plan_request;
  r.group_name = "arm";
  r.start_state.is_diff = true;
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 5.0;
  r.max_velocity_scaling_factor = 0.3;
  r.max_acceleration_scaling_factor = 0.3;
  moveit_msgs::msg::Constraints c;
  for (int i = 0; i < 5; ++i) {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = ARM[i];
    jc.position = q5[i];
    jc.tolerance_above = jc.tolerance_below = 0.01;
    jc.weight = 1.0;
    c.joint_constraints.push_back(jc);
  }
  r.goal_constraints.push_back(c);
  const uint64_t gen = gen_;
  plan_->async_send_request(req, [this, done, gen](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    if (gen != gen_) return;
    const auto res = f.get();
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) { done(false); return; }
    execute(res->motion_plan_response.trajectory, done);
  });
}

void ArmIO::set_gripper(double width_m, Cb done, double effort_a) {
  Gripper::Goal goal;
  goal.command.position = {width_m};
  // 쥐는 힘 [A]: goal effort → gripper_controller(max_effort_interface) → 펌웨어 Goal Current (0.00269~1.2 A로 자름)
  if (effort_a > 0.0) goal.command.effort = {effort_a};
  const uint64_t gen = gen_;
  rclcpp_action::Client<Gripper>::SendGoalOptions opt;
  opt.result_callback = [this, done, width_m, gen](const rclcpp_action::ClientGoalHandle<Gripper>::WrappedResult & r) {
    if (gen != gen_) return;
    const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
    if (!ok) RCLCPP_WARN(node_.get_logger(), "그리퍼 %.3fm 실패", width_m);
    done(ok);
  };
  gripper_->async_send_goal(goal, opt);
}

void ArmIO::run_steps(std::shared_ptr<std::vector<Step>> steps, size_t i, Cb done) {
  if (i >= steps->size()) { done(true); return; }
  (*steps)[i]([this, steps, i, done](bool ok) {
    if (!ok) { done(false); return; }
    run_steps(steps, i + 1, done);
  });
}

ArmIO::Step ArmIO::joints_step(const std::vector<double> & q5) {
  return [this, q5](Cb cb) { move_joints(q5, cb); };
}

ArmIO::Step ArmIO::gripper_step(double width_m, double effort_a) {
  return [this, width_m, effort_a](Cb cb) { set_gripper(width_m, cb, effort_a); };
}

ArmIO::Step ArmIO::held_step() {
  return [this](Cb cb) {
    if (!grasp_check_) { cb(true); return; }
    const auto p = pos_.find("gripper_left_joint");
    const auto e = effort_.find("gripper_left_joint");
    const double pos = p == pos_.end() ? 0.0 : p->second;
    const double eff = e == effort_.end() ? std::nan("") : e->second;
    const bool ok = grasp_held(pos, eff, grasp_min_pos_m_, grasp_min_effort_);
    RCLCPP_INFO(node_.get_logger(), "잡기 판정 %s — 그리퍼 %.4fm · %.2fA", ok ? "쥠" : "빈손", pos, eff);
    grasp_empty_ = !ok;
    cb(ok);
  };
}

ArmIO::Step ArmIO::traj_step(std::function<std::vector<Q5>(const Q5 &)> make) {
  return [this, make](Cb cb) {
    Q5 now;
    if (!arm_now(now)) { cb(false); return; }
    const auto samples = make(now);
    moveit_msgs::msg::RobotTrajectory traj;
    auto & jt = traj.joint_trajectory;
    jt.joint_names.assign(std::begin(ARM), std::end(ARM));
    const auto point = [](const Q5 & q, double t) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions.assign(q.begin(), q.end());
      p.time_from_start = rclcpp::Duration::from_seconds(t);
      return p;
    };
    jt.points.push_back(point(now, 0.0));
    for (size_t k = 0; k < samples.size(); ++k) jt.points.push_back(point(samples[k], (k + 1) * TRAJ_DT));
    execute(traj, cb);
  };
}

ArmIO::Step ArmIO::grip_timed_step(double joint_m, double effort_a, double secs) {
  return [this, joint_m, effort_a, secs](Cb cb) {
    Gripper::Goal goal;
    goal.command.position = {joint_m};
    if (effort_a > 0.0) goal.command.effort = {effort_a};
    gripper_->async_send_goal(goal);
    wait_step(secs)(cb);
  };
}

ArmIO::Step ArmIO::wait_step(double secs) {
  return [this, secs](Cb cb) {
    const uint64_t gen = gen_;
    timer_ = node_.create_wall_timer(std::chrono::duration<double>(secs), [this, cb, gen]() {
      timer_->cancel();
      if (gen == gen_) cb(true);
    });
  };
}

void ArmIO::skill_ctx(const SkillModel * policy, uint8_t phase, bool holding, bool live, const Q5 & above, const Q5 & down) {
  ml::msg::SkillContext m;
  m.header.stamp = node_.now();  // 이 단계 시작 시각 (정책 입력 '진행'의 기준)
  m.run_id = run_;
  if (policy) {
    m.skill = policy->name;
    if (policy->use) { m.policy = policy->policy; m.layout = policy->layout; }  // 안 쓰면 "" → rl_arm_correction Δ = 0
    m.phase = phase;
    m.holding = holding;
    m.residual_live = live;
    m.q_above.assign(above.begin(), above.end());
    m.q_down.assign(down.begin(), down.end());
  }
  ctx_pub_->publish(m);
}

void ArmIO::ctx_off() { skill_ctx(nullptr, 0, false, false, {}, {}); }

void ArmIO::hand_target(const std::string & kind, int puck_tag, int slot, double x, double y, double z) {
  perception::msg::HandTarget t;
  t.kind = kind;
  t.puck_tag = puck_tag;
  t.box_tag = -1;  // ponytail: 상자 밀림 보정은 상자 태그·계획 상자 위치를 넘길 때 (mission_manager가 아는 값)
  t.point.x = x;
  t.point.y = y;
  t.point.z = z;
  t.slot = slot;
  hand_pub_->publish(t);
}

void ArmIO::reset_sag() {
  if (reset_sag_->service_is_ready()) {
    reset_sag_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  } else if (!sag_warned_) {
    sag_warned_ = true;
    RCLCPP_WARN(node_.get_logger(), "/rl_arm_correction/reset_sag 없음 — 처짐 보정 초기화 없이 계속 (rl_arm_correction 확인)");
  }
}

bool ArmIO::ik_ee(const geometry_msgs::msg::PoseStamped & ee, const Q5 & near, Q5 & out) {
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("base_footprint", ee.header.frame_id, tf2::TimePointZero);
  } catch (const std::exception &) {
    return false;
  }
  const auto & r = tf.transform.rotation;
  const auto & tr = tf.transform.translation;
  const tf2::Transform t(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(tr.x, tr.y, tr.z));
  const tf2::Vector3 p = t * tf2::Vector3(ee.pose.position.x, ee.pose.position.y, ee.pose.position.z);
  stand::V3 axis{};
  stand::fk(near, &axis);  // 손가락이 닫히는 방향 — 지금 손목 그대로
  return stand::ik_down({p.x(), p.y(), p.z()}, std::atan2(axis[1], axis[0]), near, out);
}

ArmIO::Step ArmIO::contact_step(const SkillModel & policy, std::function<bool(Q5 &, Q5 &)> plan, bool close) {
  return [this, policy, plan, close](Cb cb) {
    Q5 above{}, down{};
    if (!plan(above, down)) {
      RCLCPP_WARN(node_.get_logger(), "%s: 닿는 구간 자세를 못 구함 (IK·TF)", policy.name.c_str());
      cb(false);
      return;
    }
    ++run_;
    const auto p = std::make_shared<SkillModel>(policy);
    const auto ctx = [this, p, above, down](uint8_t phase, bool holding, bool live) -> Step {
      return [this, p, phase, holding, live, above, down](Cb c) { skill_ctx(p.get(), phase, holding, live, above, down); c(true); };
    };
    const auto move = [this](Q5 from, Q5 to) {  // 시뮬처럼 from→to 부드러운 보간 (팀 한계 × 0.3)
      return traj_step([from, to](const Q5 &) { return stand::sample_move(from, to, CONTACT_VS, TRAJ_DT); });
    };
    auto steps = std::make_shared<std::vector<Step>>(std::vector<Step>{
      ctx(0, true, true), wait_step(CONTACT_HOLD_S),       // 0 위에서 멈춤 (처짐 적분)
      ctx(1, false, true), move(above, down),             // 1 내려가기
      ctx(2, true, true), wait_step(CONTACT_SETTLE_S),    // 2 안정
      ctx(3, true, false),                                // 3 쥐기 · 놓기 (보정 고정)
      close ? grip_timed_step(GRIPPER_CLOSED_M, grip_current_a_, CONTACT_CLOSE_S)
            : grip_timed_step(GRIPPER_OPEN_M, -1.0, CONTACT_RELEASE_S)});
    if (close) steps->push_back(held_step());
    steps->push_back(ctx(4, false, false));               // 4 올리기
    steps->push_back(move(down, above));
    steps->push_back(ctx(5, !close, false));              // 5 대기
    steps->push_back(wait_step(close ? 0.3 : 0.5));
    RCLCPP_INFO(node_.get_logger(), "%s: 닿는 구간 (시뮬 기준 동작 + %s) run %u", policy.name.c_str(),
                policy.use ? "학습 보정" : "보정 없음", run_);
    run_steps(steps, 0, [this, cb](bool ok) {
      ctx_off();  // 보정 끔
      hand_target("none", -1, -1);
      cb(ok);
    });
  };
}

}  // namespace skills
