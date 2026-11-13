// coordinator 구현 — 노드 설명·입출력·멤버 선언은 include/plan/coordinator.hpp, main은 coordinator_main.cpp
#include "plan/coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace plan {

int basket_slot(const std::string & target, const std::vector<std::string> & filled, const std::vector<int> & order,
                bool load, std::string & why, const std::vector<bool> & usable) {
  const int n = static_cast<int>(filled.size());
  int slot = -1;
  if (target == (load ? "next" : "last") || target == "tray") {
    if (load) {
      for (int i = 0; i < n && slot < 0; ++i)
        if (filled[i].empty() && (usable.empty() || (i < static_cast<int>(usable.size()) && usable[i]))) slot = i;
      if (slot < 0) { why = "slot_full"; return -1; }
    } else {
      if (order.empty()) { why = "slot_empty"; return -1; }
      slot = order.back();
    }
    return slot;
  }
  if (target.empty() || target.size() > 2 || target.find_first_not_of("0123456789") != std::string::npos ||
      (slot = std::stoi(target)) >= n) {
    why = "slot_invalid";
    return -1;
  }
  if (load && !filled[slot].empty()) { why = "slot_full"; return -1; }
  if (!load && filled[slot].empty()) { why = "slot_empty"; return -1; }
  return slot;
}

std::pair<double, double> box_spread(int n, double spacing, double half_x, double half_y, double margin, double yaw) {
  auto [dx, dy] = spread_offset(n, spacing);  // 상자 축 기준
  const double lx = std::max(half_x - margin, 0.0), ly = std::max(half_y - margin, 0.0);
  dx = std::clamp(dx, -lx, lx);
  dy = std::clamp(dy, -ly, ly);
  return {dx * std::cos(yaw) - dy * std::sin(yaw), dx * std::sin(yaw) + dy * std::cos(yaw)};
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


Coordinator::Coordinator(const rclcpp::NodeOptions & options)
: rclcpp::Node("coordinator", options) {
  server_wait_s_ = declare_parameter<double>("server_wait_s", 30.0);
  reach_m_ = declare_parameter<double>("reach_m", 0.30);
  remember_s_ = declare_parameter<double>("remember_s", 120.0);  // [s] 안 보이게 된 퍽을 이만큼 기억 (0 = 안 함) — remembered_
  approach_m_ = declare_parameter<double>("approach_m", 0.24);
  place_spacing_ = declare_parameter<double>("place_spacing", 0.08);
  box_margin_m_ = declare_parameter<double>("box_margin_m", 0.03);
  for (const auto & p : declare_parameter<std::vector<std::string>>("pose_names", {"stow", "carry_left", "carry_right", "gaze", "look"}))
    pose_names_.insert(p);
  basket_.assign(static_cast<size_t>(declare_parameter<int>("basket_slots", 12)), PuckInfo{});
  {
    const auto use = declare_parameter<std::vector<int64_t>>("basket_use", std::vector<int64_t>{});
    basket_use_.assign(basket_.size(), use.empty());
    for (const auto i : use) if (i >= 0 && i < static_cast<int64_t>(basket_use_.size())) basket_use_[i] = true;
  }
  basket_file_ = declare_parameter<std::string>("basket_file", "~/.ros/beebot_basket.txt");
  if (basket_file_.rfind("~/", 0) == 0 && std::getenv("HOME")) basket_file_ = std::string(std::getenv("HOME")) + basket_file_.substr(1);
  basket_pub_ = create_publisher<plan::msg::BasketState>("/coordinator/basket", rclcpp::QoS(1).reliable().transient_local());
  basket_clear_srv_ = create_service<std_srvs::srv::Trigger>("/coordinator/basket_clear",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      std::fill(basket_.begin(), basket_.end(), PuckInfo{});
      basket_order_.clear();
      held_ = PuckInfo{};
      basket_changed();
      RCLCPP_WARN(get_logger(), "적재함 장부 비움 (basket_clear)");
      res->success = true;
      res->message = "적재함 장부를 비웠습니다";
    });
  basket_load();
  basket_changed();
  // 단계 시간 제한 기본값 [s] — 실기 한 단계(팔 skill 약 13~20 s · 경기장 끝에서 끝 주행 약 40 s)의 몇 배. 시뮬 시계 기준
  static const std::map<std::string, double> timeout_default = {
    {"navigate", 120.0}, {"approach", 60.0}, {"pose", 30.0}, {"inspect", 60.0}, {"pick", 90.0},
    {"place", 90.0}, {"load", 90.0}, {"unload", 90.0}, {"stand", 90.0}, {"stand_pick", 90.0}};
  for (const auto & [a, t] : timeout_default) {
    skills_[a] = rclcpp_action::create_client<Skill>(this, "/skills/" + a);
    step_timeout_s_[a] = declare_parameter<double>("step_timeout_s." + a, t);
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(10).reliable().transient_local();
  status_pub_ = create_publisher<TaskStatus>("/task/status", latched);
  goals_sub_ = create_subscription<SubGoalArray>("/task/sub_goals", latched,
    [this](SubGoalArray::ConstSharedPtr m) { on_plan(*m); });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      const bool first = !objects_;
      objects_ = m;
      remember(*m);
      if (first && waiting_plan_) {  // 물체 목록보다 계획이 먼저 왔으면 지금 검증
        auto plan = *waiting_plan_;
        waiting_plan_.reset();
        on_plan(plan);
      }
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
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (mode_ != "AUTO") s.summary(DiagnosticStatus::WARN, mode_ + " 모드 — 자동 실행 멈춤");
    else if (waited_s_ > 0) s.summary(DiagnosticStatus::WARN, "skill 노드 대기 (" + active_action_ + ")");
    else if (running_) s.summary(DiagnosticStatus::OK, "실행 중: " + step_text());
    else s.summary(DiagnosticStatus::OK, "대기");
    s.add("모드", mode_);
    s.add("plan_id", plan_.plan_id);
    s.add("단계", step_text());
    s.add("실행 중", running_);
    std::string slots;
    for (const auto & b : basket_) slots += b.shape.empty() ? "·" : (b.check ? "?" : b.shape.substr(0, 1));
    s.add("적재함", std::to_string(basket_order_.size()) + "/" + std::to_string(basket_.size()) + " " + slots);
    s.add("손", held_.shape.empty() ? "-" : held_.shape + " tag " + std::to_string(held_.tag));
    std::string down;
    for (const auto & [name, c] : skills_) if (!c->action_server_is_ready()) down += (down.empty() ? "" : " · ") + name;
    s.add("skill 노드 없음", down.empty() ? "-" : down);
  });
}

void Coordinator::on_mode(const std::string & mode) {
  if (mode == mode_) return;
  mode_ = mode;
  RCLCPP_WARN(get_logger(), "운영 모드: %s", mode.c_str());
  if (mode != "AUTO" && running_) {  // 수동 · TEACH — 작업 실행과 동시에 하면 안 됨
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
    if (x.step != y.step || x.skill != y.skill || x.target != y.target || x.relation != y.relation || x.offset_m != y.offset_m)
      return false;
  }
  return true;
}
}  // namespace

void Coordinator::on_plan(const SubGoalArray & plan) {
  // transient_local 재수신은 무시 — 단, 같은 id에 내용이 다르면 다른 계획 (mission_manager 카운터와 sim 직접 발행 충돌 대비)
  if (estop_) {
    RCLCPP_WARN(get_logger(), "계획 %u 거부: 비상정지 중", plan.plan_id);
    status(0, TaskStatus::FAILED, "estop_blocked");
    return;
  }
  if (plan.plan_id == plan_.plan_id && running_ && same_goals(plan, plan_)) return;
  if (plan.plan_id == 0 && plan.goals.empty()) {  // 명시적 정지 (sim 정지 버튼 · mission_manager abort)
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
  if (running_) cancel_all();  // 진행 중 goal 취소 — 옛 결과가 새 계획을 건드리지 않게
  plan_ = plan;
  idx_ = 0;
  approached_ = static_cast<size_t>(-1);
  placed_.clear();
  running_ = true;
  run_step();
}

bool Coordinator::validate(const SubGoalArray & plan, std::string & why) const {
  static const std::set<std::string> allowed = {"navigate", "approach", "pick", "place", "inspect", "pose", "load", "unload",
                                                "stand", "stand_pick"};
  // target이 물체 id가 아님 (stand_pick은 직전 stand의 퍽 — 세운 사이 인지 id가 바뀌어도 됨)
  static const std::set<std::string> not_object = {"pose", "load", "unload", "stand_pick"};
  if (plan.goals.empty()) { why = "empty"; return false; }
  for (const auto & g : plan.goals) {
    if (!allowed.count(g.skill)) { why = "skill " + g.skill; return false; }
    if (g.skill == "pose" && !pose_names_.count(g.target)) { why = "pose " + g.target; return false; }
    if (!not_object.count(g.skill) && is_id(g.target) && !find(std::stoul(g.target))) {
      why = "target " + g.target + " not tracked";
      return false;
    }
  }
  return true;
}

bool Coordinator::is_id(const std::string & s) {
  return !s.empty() && s.size() < 10 && s.find_first_not_of("0123456789") == std::string::npos;
}

const perception::msg::TrackedObject * Coordinator::find(uint32_t id) const {
  if (objects_) for (const auto & o : objects_->objects) if (o.id == id) return &o;
  if (const auto it = remembered_.find(id); it != remembered_.end()) return &it->second.o;  // 지금은 안 보이는 퍽
  return nullptr;
}

void Coordinator::remember(const TrackedObjectArray & m) {
  const rclcpp::Time t = now();
  // 믿을 만한 관측만 기억에 씀: 계획이 없을 때(팔 멈춤) · inspect 단계. pick·load 중에는 팔이 빨리 움직여(TF 시각 어긋남)
  //   트랙이 엉뚱한 자리로 끌려갔음 (11/07 Isaac: 적재 중 옆 퍽 트랙이 14 cm 밀려 그 자리로 pick → IK 실패)
  const bool trust = !running_ || (idx_ < plan_.goals.size() && plan_.goals[idx_].skill == "inspect");
  for (auto it = remembered_.begin(); it != remembered_.end();) {  // 오래됨 · 같은 자리(3 cm)에 다른 id 트랙이 다시 생김 → 잊음
    bool replaced = (t - it->second.t).seconds() > remember_s_;
    for (const auto & o : m.objects)
      if (trust && !replaced && o.id != it->first && o.class_name.rfind("puck", 0) == 0 &&
          std::hypot(o.pose.pose.position.x - it->second.o.pose.pose.position.x,
                     o.pose.pose.position.y - it->second.o.pose.pose.position.y) < 0.03)
        replaced = true;
    it = replaced ? remembered_.erase(it) : std::next(it);
  }
  if (remember_s_ <= 0 || !trust) return;
  for (const auto & o : m.objects)
    if (o.class_name.rfind("puck", 0) == 0) remembered_[o.id] = {o, t};
}

std::string Coordinator::shape_of(const perception::msg::TrackedObject & o) const {
  for (const auto & kv : o.details) if (kv.key == "shape" && (kv.value == "box" || kv.value == "cyl")) return kv.value;
  return "box";  // 모름 — 사각 퍽 표 (손 방향을 칸 벽에 맞춘 자세라 원기둥도 들어감)
}

int Coordinator::tag_of(const perception::msg::TrackedObject & o) const {
  for (const auto & kv : o.details) if (kv.key == "tag" && is_id(kv.value)) return std::stoi(kv.value);
  return -1;
}

void Coordinator::run_step() {
  if (!running_ || mode_ != "AUTO" || estop_) return;
  if (idx_ >= plan_.goals.size()) {
    running_ = false;
    status(plan_.goals.back().step, TaskStatus::SUCCEEDED, "");
    return;
  }
  const auto & g = plan_.goals[idx_];
  status(g.step, TaskStatus::RUNNING, "");
  RCLCPP_INFO(get_logger(), "step %u: %s %s", g.step, g.skill.c_str(), g.target.c_str());
  if (g.skill == "navigate") navigate(g);
  else if (g.skill == "approach") approach(g);
  else if (g.skill == "pick") g.target == "tray" ? unload(g) : pick(g);   // pick tray = unload last (호환)
  else if (g.skill == "place") g.target == "tray" ? load(g) : place(g);   // place tray = load next (호환)
  else if (g.skill == "inspect") inspect(g);
  else if (g.skill == "load") load(g);
  else if (g.skill == "unload") unload(g);
  else if (g.skill == "stand") stand(g);
  else if (g.skill == "stand_pick") stand_pick(g);
  else if (g.skill == "pose") {
    Skill::Goal sg;
    sg.target = g.target;
    send("pose", sg, [this](const Skill::Result &) { next(); });
  }
}

void Coordinator::send(const std::string & action, const Skill::Goal & goal, Ok ok, Fail on_fail) {
  const auto & client = skills_.at(action);
  active_action_ = action;
  if (!client->action_server_is_ready()) {  // skill 노드가 아직 없음 (기동 중) — 1초 뒤 같은 단계를 다시
    if (waited_s_ >= server_wait_s_) { waited_s_ = 0; fail("skill_unavailable:" + action); return; }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "/skills/%s 대기 중", action.c_str());
    waited_s_ += 1.0;
    retry_ = create_wall_timer(std::chrono::seconds(1), [this]() { retry_->cancel(); run_step(); });
    return;
  }
  waited_s_ = 0;
  const uint64_t gen = gen_;
  rclcpp_action::Client<Skill>::SendGoalOptions opt;
  opt.goal_response_callback = [this, gen, action](const rclcpp_action::ClientGoalHandle<Skill>::SharedPtr & h) {
    if (gen != gen_) return;
    if (!h) { fail(action + "_busy"); return; }  // skill 노드가 다른 goal 실행 중
    active_ = h;
  };
  opt.result_callback = [this, gen, action, ok, on_fail](const rclcpp_action::ClientGoalHandle<Skill>::WrappedResult & r) {
    if (gen != gen_) return;  // 취소된 옛 goal의 늦은 결과
    active_.reset();
    if (step_timer_) step_timer_->cancel();
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->success) { ok(*r.result); return; }
    if (r.code == rclcpp_action::ResultCode::CANCELED) return;  // 취소한 쪽(모드·비상정지·새 계획)이 이미 처리
    const std::string why = r.result && !r.result->reason.empty() ? r.result->reason : action + "_failed";
    if (on_fail) on_fail(why);
    fail(why);
  };
  client->async_send_goal(goal, opt);
  if (step_timer_) step_timer_->cancel();
  const double limit = step_timeout_s_.at(action);
  if (limit > 0)  // 노드 시계(시뮬이면 /clock) 기준 — skill이 결과 없이 멈춰도 계획이 그 단계에 갇히지 않게 (규칙 3-1)
    step_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(limit), [this, gen, action, limit]() {
      step_timer_->cancel();
      if (gen != gen_) return;
      RCLCPP_ERROR(get_logger(), "/skills/%s %.0f s 안에 결과 없음 — 취소", action.c_str(), limit);
      cancel_all();
      skills_.at(action)->async_cancel_all_goals();  // goal 응답 전이라 active_가 없어도 그 skill의 goal을 멈춤
      fail(action + "_timeout");
    });
}

void Coordinator::cancel_all() {
  ++gen_;
  if (retry_) retry_->cancel();
  if (step_timer_) step_timer_->cancel();
  if (active_) skills_.at(active_action_)->async_cancel_goal(active_);
  active_.reset();
}

void Coordinator::next() {
  if (!running_) return;  // 수동 전환 등으로 이미 멈춘 뒤 도착한 결과
  ++idx_;
  run_step();
}

// 규칙 3: 실패는 FAILED + reason으로 바로 알리고 멈춤 — 다시 할지는 계획을 낸 쪽(mission_manager retries · approach_tries)
void Coordinator::fail(const std::string & reason) {
  if (!running_) return;  // 이미 멈춤 (수동 전환으로 취소된 goal의 결과 등)
  const auto & g = plan_.goals[idx_];
  running_ = false;
  RCLCPP_WARN(get_logger(), "step %u 실패 (%s)", g.step, reason.c_str());
  status(g.step, TaskStatus::FAILED, reason);
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

namespace {
geometry_msgs::msg::PoseStamped map_pose(double x, double y, double z, double yaw = 0.0) {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
}  // namespace

bool Coordinator::approach_if_far(const SubGoal & g) {
  if (approached_ == idx_ || !is_id(g.target)) return false;  // 이 단계는 이미 다가감 → 그대로 실행 (안 닿으면 그 단계가 실패)
  const auto * o = find(std::stoul(g.target));
  double rx, ry;
  if (!o || !robot_xy(rx, ry)) return false;  // 로봇 위치를 모르면 판단 못 함 — 그대로 실행
  const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
  const double d = std::hypot(ox - rx, oy - ry);
  if (d <= reach_m_) return false;
  if (skills_.at("navigate")->action_server_is_ready()) approached_ = idx_;  // 서버를 기다리는 동안은 다시 이 길로
  RCLCPP_INFO(get_logger(), "%s %s: 대상이 %.2fm — 팔이 닿게 %.2fm 앞까지 다가감", g.skill.c_str(), g.target.c_str(), d, approach_m_);
  Skill::Goal sg;
  sg.pose.header.frame_id = "map";
  sg.pose.pose = approach_pose(ox, oy, rx, ry, approach_m_);
  send("navigate", sg, [this](const Skill::Result &) { run_step(); },  // 같은 단계를 다시 (이번엔 다가가기를 건너뜀)
       [this](const std::string &) {});
  return true;
}

void Coordinator::navigate(const SubGoal & g) {
  Skill::Goal sg;
  sg.pose.header.frame_id = "map";
  sg.pose.pose.orientation.w = 1.0;
  if (is_id(g.target)) {
    // 대상 앞 (물체 반 크기 + offset + 0.1) — 로봇이 있는 쪽에서 물체를 바라봄
    const auto * o = find(std::stoul(g.target));
    if (!o) { fail("nav_bad_target"); return; }
    const double ox = o->pose.pose.position.x, oy = o->pose.pose.position.y;
    double rx = ox - 1.0, ry = oy;  // 로봇 위치를 모르면 물체의 −x 쪽
    robot_xy(rx, ry);
    sg.pose.pose = approach_pose(ox, oy, rx, ry, std::max(o->size.x, o->size.y) / 2 + g.offset_m + 0.1);
  } else {
    const auto v = parse_numbers(g.target);  // "x,y" (시뮬레이터 지도 클릭) · "x,y,yaw" (mission_manager)
    if (v.empty()) { fail("nav_bad_target"); return; }
    sg.pose = map_pose(v[0], v[1], 0.0, v.size() >= 3 ? v[2] : 0.0);
  }
  send("navigate", sg, [this](const Skill::Result &) { next(); });
}

void Coordinator::approach(const SubGoal & g) {
  const auto v = parse_numbers(g.target);
  if (v.size() != 3) { fail("approach_bad_target"); return; }
  Skill::Goal sg;
  sg.target = g.relation;  // 주행 정책: dock_box(기본) · park_home · drive_arc · drive_straight
  sg.pose = map_pose(v[0], v[1], 0.0, v[2]);
  send("approach", sg, [this](const Skill::Result &) { next(); });
}

void Coordinator::pick(const SubGoal & g) {
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("pick_no_target"); return; }
  if (approach_if_far(g)) return;  // 집을 퍽이 팔 밖이면 먼저 다가감 (계획에 navigate가 빠져 있어도)
  const auto o = *find(std::stoul(g.target));
  Skill::Goal sg;
  sg.target = g.target;
  sg.shape = shape_of(o);
  sg.tag = tag_of(o);
  const bool visible = objects_ && std::any_of(objects_->objects.begin(), objects_->objects.end(),
                                               [&o](const auto & x) { return x.id == o.id; });
  if (!visible) {  // 기억한 퍽 — 대상과 그 옆 퍽(기억한 것)을 실어 보냄: grasp_scorer가 지금 인지 대신 씀
    sg.objects.push_back(o);
    for (const auto & [id, s] : remembered_)
      if (id != o.id) sg.objects.push_back(s.o);
    RCLCPP_INFO(get_logger(), "pick %s: 지금 안 보임 — 기억한 자리 (%.3f, %.3f)로", g.target.c_str(),
                o.pose.pose.position.x, o.pose.pose.position.y);
  }
  send("pick", sg, [this, o](const Skill::Result &) {
    remembered_.erase(o.id);  // 손에 있음
    held_ = PuckInfo{shape_of(o), o.color, tag_of(o), false};  // 집은 퍽 — 칸에 실을 때 장부에 그대로 적음
    basket_changed();
    next();
  });
}

void Coordinator::place(const SubGoal & g) {
  Skill::Goal sg;
  if (const auto v = parse_numbers(g.target); v.size() == 3) {  // 좌표 place — 손 안 퍽 바닥이 z가 되게 (주사위 칸)
    sg.target = "cell";
    sg.pose = map_pose(v[0], v[1], v[2]);
    if (double rx, ry; robot_xy(rx, ry))
      RCLCPP_INFO(get_logger(), "place 점까지 수평 %.3fm (로봇 %.3f,%.3f)", std::hypot(v[0] - rx, v[1] - ry), rx, ry);
    send("place", sg, [this](const Skill::Result &) {
      held_ = PuckInfo{};  // 손 비움 — 장부 발행·저장
      basket_changed();
      next();
    });
    return;
  }
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("place_no_target"); return; }
  if (approach_if_far(g)) return;  // 내려놓을 곳이 팔 밖이면 먼저 다가감
  const auto * o = find(std::stoul(g.target));
  sg.target = "over";
  sg.pose.header.frame_id = "map";
  sg.pose.pose.position = o->pose.pose.position;  // 놓을 면 (물체 위치)
  sg.pose.pose.orientation.w = 1.0;
  if (o->class_name == "location_box") {
    // 상자 윗면 — 윗면을 못 찾았으면(태그만) z가 태그 높이라 size.z(바닥~윗면)로. 여러 개면 상자 안·상자 축으로 비켜 놓음
    sg.pose.pose.position.z = std::max(o->pose.pose.position.z, static_cast<double>(o->size.z));
    const auto & q = o->pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const auto [dx, dy] = box_spread(placed_[g.target], place_spacing_, o->size.x / 2, o->size.y / 2, box_margin_m_, yaw);
    sg.pose.pose.position.x += dx;
    sg.pose.pose.position.y += dy;
    sg.values = {0.0};
  } else {
    const auto [dx, dy] = spread_offset(placed_[g.target], place_spacing_);  // 같은 곳에 여러 개면 비켜서
    sg.pose.pose.position.x += dx;
    sg.pose.pose.position.y += dy;
    sg.values = {0.10};
  }
  send("place", sg, [this, id = g.target](const Skill::Result &) {
    ++placed_[id];
    held_ = PuckInfo{};
    basket_changed();
    next();
  });
}

void Coordinator::inspect(const SubGoal & g) {
  Skill::Goal sg;
  if (const auto v = parse_numbers(g.target); v.size() == 3) {  // 좌표 inspect — 그 점(퍽 윗면 등) 위
    sg.pose = map_pose(v[0], v[1], v[2]);
    if (g.relation == "box") {  // 상자 보기 — 점 = 상자 가운데(퍽 윗면 높이): look 자세 → 퍽이 모자라면 다시 담기 (skill_inspect)
      sg.target = "box";
      send("inspect", sg, [this](const Skill::Result &) { next(); });
      return;
    }
    if (double rx, ry; robot_xy(rx, ry)) {  // 손이 로봇 → 점 방향을 보게 (gripper_cam이 그 점을 화면 가운데에)
      sg.values = {std::atan2(v[1] - ry, v[0] - rx)};
      RCLCPP_INFO(get_logger(), "inspect 점까지 수평 %.3fm (로봇 %.3f,%.3f)", std::hypot(v[0] - rx, v[1] - ry), rx, ry);
    }
    send("inspect", sg, [this](const Skill::Result &) { next(); });
    return;
  }
  if (!is_id(g.target) || !find(std::stoul(g.target))) { fail("inspect_no_target"); return; }
  if (approach_if_far(g)) return;  // 팔 밖이면 먼저 다가감
  const auto * o = find(std::stoul(g.target));
  sg.pose.header.frame_id = "map";
  sg.pose.pose.position = o->pose.pose.position;
  sg.pose.pose.position.z += o->size.z / 2;  // 윗면 — 손가락 아래라 gripper_cam이 윗면을 봄
  sg.pose.pose.orientation.w = 1.0;
  send("inspect", sg, [this, id = g.target](const Skill::Result &) {
    const auto * obj = find(std::stoul(id));
    RCLCPP_INFO(get_logger(), "inspect %s: 태그 %s", id.c_str(), obj && tag_of(*obj) >= 0 ? "읽음" : "못 읽음");
    next();
  });
}

void Coordinator::load(const SubGoal & g) {
  if (basket_.empty()) { fail("basket_table_missing"); return; }
  std::string why;
  const int slot = basket_slot(g.target, basket_shapes(), basket_order_, true, why, basket_use_);
  if (slot < 0) { fail(why); return; }
  Skill::Goal sg;
  sg.slot = slot;
  sg.shape = held_.shape.empty() ? "box" : held_.shape;
  sg.tag = held_.tag;
  send("load", sg, [this, slot, shape = sg.shape](const Skill::Result &) {
    basket_[slot] = held_;
    basket_[slot].shape = shape;
    basket_[slot].check = false;
    basket_order_.push_back(slot);
    held_ = PuckInfo{};
    basket_changed();
    next();
  });
}

void Coordinator::unload(const SubGoal & g) {
  if (basket_.empty()) { fail("basket_table_missing"); return; }
  std::string why;
  const int slot = basket_slot(g.target, basket_shapes(), basket_order_, false, why);
  if (slot < 0) { fail(why); return; }
  Skill::Goal sg;
  sg.slot = slot;
  sg.shape = basket_[slot].shape;
  sg.tag = basket_[slot].tag;
  send("unload", sg, [this, slot](const Skill::Result &) {
    held_ = basket_[slot];
    held_.check = false;
    basket_[slot] = PuckInfo{};
    basket_order_.erase(std::remove(basket_order_.begin(), basket_order_.end(), slot), basket_order_.end());
    basket_changed();
    next();
  }, [this, slot](const std::string & reason) {
    if (reason == "grasp_empty") { basket_[slot].check = true; basket_changed(); }  // 빈손 — 칸은 비우지 않고 확인 필요로
  });
}

void Coordinator::stand(const SubGoal & g) {
  const auto * o = is_id(g.target) ? find(std::stoul(g.target)) : nullptr;
  if (!o) { fail("stand_no_target"); return; }
  if (o->class_name != "puck_lying") { fail("stand_not_lying"); return; }
  if (!std::any_of(o->details.begin(), o->details.end(), [](const auto & kv) { return kv.key == "yaw_period"; })) {
    fail("stand_no_yaw");  // 긴 축 방향을 못 구함 — 다른 자리에서 다시 보기
    return;
  }
  if (approach_if_far(g)) return;
  Skill::Goal sg;
  sg.pose.header.frame_id = objects_->header.frame_id.empty() ? "map" : objects_->header.frame_id;
  sg.pose.pose = o->pose.pose;  // 가운데 · 긴 축 = orientation
  sg.shape = shape_of(*o);
  sg.tag = tag_of(*o);
  const PuckInfo puck{sg.shape, o->color, sg.tag, false};
  send("stand", sg, [this, puck](const Skill::Result &) { stood_ = puck; next(); });
}

void Coordinator::stand_pick(const SubGoal &) {
  if (!held_.shape.empty()) { fail("hand_full"); return; }
  send("stand_pick", Skill::Goal(), [this](const Skill::Result &) {
    held_ = stood_;
    stood_ = PuckInfo{};
    basket_changed();
    next();
  });
}

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

std::vector<std::string> Coordinator::basket_shapes() const {
  std::vector<std::string> v;
  for (const auto & b : basket_) v.push_back(b.shape);
  return v;
}

void Coordinator::basket_changed() {
  plan::msg::BasketState m;
  m.header.stamp = now();
  auto fill = [](plan::msg::BasketSlot & s, int i, const PuckInfo & b) {
    s.slot = i;
    s.filled = !b.shape.empty();
    s.shape = b.shape;
    s.color = b.color;
    s.tag = b.tag;
    s.check = b.check;
  };
  for (size_t i = 0; i < basket_.size(); ++i) {
    plan::msg::BasketSlot s;
    fill(s, static_cast<int>(i), basket_[i]);
    m.slots.push_back(s);
  }
  for (int o : basket_order_) m.order.push_back(o);
  fill(m.held, -1, held_);
  basket_pub_->publish(m);
  basket_save();
}

// 파일 형식 (한 줄 = 칸 하나): "slot <칸> <모양> <색|-> <tag> <check 0|1>" · "order <칸> <칸> …" · "held <모양|-> <색|-> <tag>"
void Coordinator::basket_save() const {
  if (basket_file_.empty()) return;
  std::ofstream f(basket_file_);
  if (!f) return;  // 저장 실패는 진단에만 — 동작은 계속
  for (size_t i = 0; i < basket_.size(); ++i)
    if (!basket_[i].shape.empty())
      f << "slot " << i << " " << basket_[i].shape << " " << (basket_[i].color.empty() ? "-" : basket_[i].color) << " "
        << basket_[i].tag << " " << (basket_[i].check ? 1 : 0) << "\n";
  f << "order";
  for (int o : basket_order_) f << " " << o;
  f << "\nheld " << (held_.shape.empty() ? "-" : held_.shape) << " " << (held_.color.empty() ? "-" : held_.color) << " " << held_.tag << "\n";
}

void Coordinator::basket_load() {
  if (basket_file_.empty()) return;
  std::ifstream f(basket_file_);
  if (!f) return;
  std::string line;
  int n = 0;
  while (std::getline(f, line)) {
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    if (kind == "slot") {
      int i = -1, check = 0;
      PuckInfo b;
      in >> i >> b.shape >> b.color >> b.tag >> check;
      if (b.color == "-") b.color.clear();
      b.check = check != 0;
      if (i >= 0 && i < static_cast<int>(basket_.size()) && (b.shape == "box" || b.shape == "cyl")) { basket_[i] = b; ++n; }
    } else if (kind == "order") {
      int o;
      while (in >> o) if (o >= 0 && o < static_cast<int>(basket_.size()) && !basket_[o].shape.empty()) basket_order_.push_back(o);
    } else if (kind == "held") {
      in >> held_.shape >> held_.color >> held_.tag;
      if (held_.shape == "-") held_.shape.clear();
      if (held_.color == "-") held_.color.clear();
    }
  }
  for (size_t i = 0; i < basket_.size(); ++i)  // order에 빠진 칸은 뒤에 (파일이 손상돼도 꺼낼 수 있게)
    if (!basket_[i].shape.empty() && std::find(basket_order_.begin(), basket_order_.end(), static_cast<int>(i)) == basket_order_.end())
      basket_order_.push_back(static_cast<int>(i));
  if (n > 0 || !held_.shape.empty())
    RCLCPP_WARN(get_logger(), "적재함 장부 이어 감 (%s): 칸 %d개%s — 실제와 다르면 /coordinator/basket_clear", basket_file_.c_str(), n,
                held_.shape.empty() ? "" : " · 손에 퍽");
}


void Coordinator::status(uint32_t step, uint8_t state, const std::string & reason) {
  TaskStatus s;
  s.plan_id = plan_.plan_id;
  s.step = step;
  s.state = state;
  s.reason = reason;
  status_pub_->publish(s);
}

}  // namespace plan
