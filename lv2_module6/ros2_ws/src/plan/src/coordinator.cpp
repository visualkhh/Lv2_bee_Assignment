// coordinator 구현 — 노드 설명·입출력·멤버 선언은 include/plan/coordinator.hpp, main은 coordinator_main.cpp
#include "plan/coordinator.hpp"

#include <cmath>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace plan {

Coordinator::Coordinator()
: rclcpp::Node("coordinator") {
  max_grasp_tries_ = declare_parameter<int>("max_grasp_tries", 2);
  max_step_failures_ = declare_parameter<int>("max_step_failures", 2);
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  status_pub_ = create_publisher<TaskStatus>("/task_status", latched);
  say_pub_ = create_publisher<std_msgs::msg::String>("/robot_say", rclcpp::QoS(10).reliable());
  goals_sub_ = create_subscription<SubGoalArray>("/sub_goals", latched,
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
  });
}

void Coordinator::on_mode(const std::string & mode) {
  if (mode == mode_) return;
  mode_ = mode;
  RCLCPP_WARN(get_logger(), "운영 모드: %s", mode.c_str());
  if (mode != "AUTO" && running_) {  // 수동 · 자동 탐사(EXPLORE) — 둘 다 작업 실행과 동시에 하면 안 됨
    cancel_all();
    running_ = false;
    status(idx_ < plan_.goals.size() ? plan_.goals[idx_].step : 0, TaskStatus::FAILED, mode == "MANUAL" ? "manual_override" : "explore_override");
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
    status(0, TaskStatus::FAILED, mode_ == "MANUAL" ? "manual_mode" : "explore_mode");
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
  running_ = true;
  run_step();
}

bool Coordinator::validate(const SubGoalArray & plan, std::string & why) const {
  static const std::set<std::string> allowed = {"navigate", "pick", "place", "say"};
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
    say("퍽을 옮겼어요");
    return;
  }
  const auto & g = plan_.goals[idx_];
  status(g.step, TaskStatus::RUNNING, "");
  RCLCPP_INFO(get_logger(), "step %u: %s %s", g.step, g.skill.c_str(), g.target.c_str());
  if (g.skill == "navigate") navigate(g);
  else if (g.skill == "pick") pick(g);
  else if (g.skill == "place") {
    if (!ready_or_retry(motion_plan_->service_is_ready() && exec_->action_server_is_ready() &&
        gripper_->action_server_is_ready(), "arm_unavailable")) return;
    if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("place_no_target"); return; }
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = now();
    target.pose = find(std::stoul(g.target))->pose.pose;
    execute_place(target, [this](bool ok) { ok ? next() : fail("place_failed"); });
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

bool Coordinator::parse_point(const std::string & target, double & x, double & y) {
  const auto comma = target.find(',');
  if (comma == std::string::npos || comma == 0 || comma + 1 >= target.size()) return false;
  try {
    x = std::stod(target.substr(0, comma));
    y = std::stod(target.substr(comma + 1));
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

void Coordinator::cancel_all() {
  ++gen_;
  if (retry_) retry_->cancel();
  nav_->async_cancel_all_goals();
  exec_->async_cancel_all_goals();
  gripper_->async_cancel_all_goals();
}

void Coordinator::navigate(const SubGoal & g) {
  if (!ready_or_retry(nav_->action_server_is_ready(), "nav_unavailable")) return;
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose.orientation.w = 1.0;
  if (is_id(g.target)) {
    // TODO: 대상 앞 offset + 로봇 반경만큼 떨어진 접근 자세 (통로 방향 고려 — 미로 벽에 붙지 않게)
    const auto * o = find(std::stoul(g.target));
    goal.pose.pose.position.x = o->pose.pose.position.x - o->size.x / 2 - g.offset_m - 0.1;
    goal.pose.pose.position.y = o->pose.pose.position.y;
  } else {
    double x, y;
    if (parse_point(g.target, x, y)) {  // 직접 지정 "x,y" (시뮬레이터 지도 클릭)
      goal.pose.pose.position.x = x;
      goal.pose.pose.position.y = y;
    }
  }
  const uint64_t gen = gen_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
  opt.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & h) {
      if (h) nav_goal_ = h;
    };
  opt.result_callback = [this, gen](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    r.code == rclcpp_action::ResultCode::SUCCEEDED ? next() : fail("nav_failed");
  };
  nav_->async_send_goal(goal, opt);
}

void Coordinator::pick(const SubGoal & g) {
  if (!ready_or_retry(clear_octomap_->service_is_ready() && apply_scene_->service_is_ready() &&
      score_->service_is_ready() && motion_plan_->service_is_ready() &&
      exec_->action_server_is_ready() && gripper_->action_server_is_ready(), "pick_unavailable")) return;
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
constexpr double APPROACH_M = 0.10;          // 접근·들어올리기 높이 (map = 지면 수평)
}  // namespace

void Coordinator::execute_pick(const geometry_msgs::msg::PoseStamped & grasp, std::function<void(bool)> done) {
  arm_contact_ = false;
  auto pre = grasp;
  pre.pose.position.z += APPROACH_M;
  move_arm(pre, [this, grasp, done](bool ok) {  // 접근
    if (!ok) { done(false); return; }
    set_gripper(GRIPPER_OPEN_M, [this, grasp, done](bool ok2) {  // 벌리기
      if (!ok2) { done(false); return; }
      move_arm(grasp, [this, grasp, done](bool ok3) {  // 파지 위치
        if (!ok3) { done(false); return; }
        set_gripper(GRIPPER_CLOSED_M, [this, grasp, done](bool ok4) {  // 닫기
          if (!ok4) { done(false); return; }
          auto lift = grasp;
          lift.pose.position.z += APPROACH_M;
          move_arm(lift, done);  // 들어올리기
        });
      });
    });
  });
}

void Coordinator::execute_place(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done) {
  arm_contact_ = false;
  auto above = target;
  above.pose.position.z += APPROACH_M;
  move_arm(above, [this, done](bool ok) {  // 위로 이동
    if (!ok) { done(false); return; }
    set_gripper(GRIPPER_OPEN_M, done);  // 벌리기
  });
}

void Coordinator::move_arm(const geometry_msgs::msg::PoseStamped & target, std::function<void(bool)> done) {
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
  oc.weight = 1.0;
  goal.position_constraints.push_back(pc);
  goal.orientation_constraints.push_back(oc);
  req->motion_plan_request.goal_constraints.push_back(goal);
  motion_plan_->async_send_request(req, [this, done](rclcpp::Client<GetMotionPlan>::SharedFuture f) {
    const auto res = f.get();
    if (res->motion_plan_response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_WARN(get_logger(), "플랜 실패 (%d)", res->motion_plan_response.error_code.val);
      done(false);
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
