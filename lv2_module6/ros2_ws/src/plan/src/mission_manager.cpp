// mission_manager 구현 — 노드 설명·입출력은 include/plan/mission_manager.hpp, main은 mission_manager_main.cpp
#include "plan/mission_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <set>

#include "plan/vlm_task_planner.hpp"  // detail()

namespace plan {

namespace {
constexpr uint32_t PLAN_ID_BASE = 1000000;  // vlm_task_planner(1부터)와 plan_id가 겹치지 않게

std::vector<Box> boxes(const std::vector<double> & flat, double half) {  // [x, y, …] 또는 half < 0이면 [x, y, half, …]
  std::vector<Box> out;
  const size_t n = half < 0 ? 3 : 2;
  for (size_t i = 0; i + n <= flat.size(); i += n) out.push_back({flat[i], flat[i + 1], half < 0 ? flat[i + 2] : half});
  return out;
}

std::string fmt(const char * f, double a, double b, double c) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

std::string q(const std::string & s) { return "\"" + s + "\""; }
}  // namespace

MissionManager::MissionManager()
: rclcpp::Node("mission_manager") {
  const auto sp = declare_parameter<std::vector<double>>("start_pose_arena", {0.734, 0.950, M_PI});
  start_ = {sp.at(0), sp.at(1), sp.at(2)};
  start_zone_ = declare_parameter<std::vector<double>>("start_zone", {0.434, 0.650, 1.034, 1.250});
  const double box_half = declare_parameter<double>("box_half", 0.1025);
  start_boxes_ = boxes(declare_parameter<std::vector<double>>("start_boxes",
    {1.468, 0.950, 3.670, 3.152, 2.936, 5.354, 0.734, 5.354, 2.202, 5.354, 3.670, 3.886}), box_half);
  for (const auto & b : boxes(declare_parameter<std::vector<double>>("locations",
         {3.670, 0.950, 2.202, 3.700, 3.670, 5.354}), 0.0)) locations_.push_back({b.x, b.y});
  loc_boxes_ = boxes(declare_parameter<std::vector<double>>("location_boxes",
    {3.670, 0.950, 0.1025, 2.202, 3.495, 0.41, 3.670, 5.354, 0.1025}), -1.0);
  puck_dx_ = declare_parameter<std::vector<double>>("puck_offsets_x", {-0.040, 0.040});
  box_top_z_ = declare_parameter<double>("box_top_z", 0.070);
  puck_h_ = declare_parameter<double>("puck_height", 0.060);
  grid_step_ = declare_parameter<double>("grid_step", 0.060);
  standoff_ = declare_parameter<double>("standoff", 0.25);
  approach_gap_ = declare_parameter<double>("approach_gap", 0.085);
  approach_tries_ = static_cast<int>(declare_parameter<int64_t>("approach_tries", 3));
  match_radius_ = declare_parameter<double>("match_radius", 0.04);
  verify_radius_ = declare_parameter<double>("verify_radius", 0.15);
  signal_timeout_s_ = declare_parameter<double>("signal_timeout_s", 90.0);
  barcode_timeout_s_ = declare_parameter<double>("barcode_timeout_s", 20.0);
  const auto bp = declare_parameter<std::vector<double>>("barcode_pose_arena", {0.950, 0.800, M_PI + 0.171});
  barcode_pose_ = {bp.at(0), bp.at(1), bp.at(2)};
  home_hold_s_ = declare_parameter<double>("home_hold_s", 3.0);
  log_dir_ = declare_parameter<std::string>("log_dir", "~/.ros/mission");
  // plan_id를 노드가 뜰 때마다 다른 구간에서 시작 — /task/status가 latched라 재시작하면 지난 실행의 같은 id 결과가 바로 들어왔음
  plan_seq_ = static_cast<uint32_t>(std::time(nullptr) % 1000) * 1000;
  if (log_dir_.rfind("~/", 0) == 0 && std::getenv("HOME")) log_dir_ = std::string(std::getenv("HOME")) + log_dir_.substr(1);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  goals_pub_ = create_publisher<SubGoalArray>("/task/sub_goals", rclcpp::QoS(10).reliable().transient_local());
  state_pub_ = create_publisher<std_msgs::msg::String>("/mission/state", latched);
  reset_pub_ = create_publisher<std_msgs::msg::Empty>("/mission/signal/reset", rclcpp::QoS(1).reliable());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mission/markers", latched);
  cmd_sub_ = create_subscription<std_msgs::msg::String>("/cmd/mission", rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_command(m->data); });
  barcode_sub_ = create_subscription<std_msgs::msg::String>("/mission/barcode", latched,
    [this](std_msgs::msg::String::ConstSharedPtr m) { barcode_msg_ = m->data; barcode_rx_ = now(); });
  signal_sub_ = create_subscription<perception::msg::MissionSignal>("/mission/signal", latched,
    [this](perception::msg::MissionSignal::ConstSharedPtr m) {
      // 이번 READ 이전에 발행된 것(지난 회차 DONE 등)은 버림
      if (phase_ == "READ" && rclcpp::Time(m->header.stamp, RCL_ROS_TIME) >= read_started_) signal_ = *m;
    });
  objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { objects_ = m; });
  status_sub_ = create_subscription<TaskStatus>("/task/status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr s) {
      if (!waiting_plan_ || s->plan_id != waiting_plan_ || s->state == TaskStatus::RUNNING) return;
      waiting_plan_ = 0;
      auto done = std::move(waiting_done_);
      done(s->state == TaskStatus::SUCCEEDED, s->reason);
    });
  markers_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_markers(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    s.summary(phase_ == "ERROR" ? DiagnosticStatus::ERROR : DiagnosticStatus::OK, phase_);
    s.add("배정 L1·L2·L3", order_.empty() ? "-" : order_);
    s.add("조사한 퍽", pucks_.size());
    s.add("바코드", barcode_.empty() ? "-" : barcode_);
  });
  phase("IDLE", "/cmd/mission start 대기");
}

// ---------- 진행
void MissionManager::on_command(const std::string & cmd) {
  if (cmd == "abort") { if (active_) fail("aborted"); return; }
  if (cmd != "start" || active_) return;
  active_ = true;
  pucks_.clear();
  puck_at_.clear();
  alloc_.clear();
  jobs_.clear();
  order_.clear();
  verify_json_.clear();
  barcode_.clear();
  back_.clear();
  char stamp[32];
  const std::time_t t = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
  run_dir_ = log_dir_ + "/" + stamp;
  std::filesystem::create_directories(run_dir_);
  phase("GO_OBSERVE", "팔 scan 자세 · 바코드 보는 자세로 (출발 구역 안)");
  const auto p = to_map(barcode_pose_.x, barcode_pose_.y, barcode_pose_.yaw);
  run({SubGoal().set__skill("pose").set__target("scan"),
       SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", p.x, p.y, p.yaw))},
    [this](bool ok, const std::string & why) { ok ? barcode_phase() : fail("observe_nav_failed:" + why); });
}

void MissionManager::barcode_phase() {
  // 이번에 실제로 본 바코드만 — /tracked_objects의 barcode(last_seen ≥ 시작) 또는 시작 뒤 받은 /mission/barcode
  phase("BARCODE", "바코드 읽는 중");
  phase_started_ = now();
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
    if (phase_ != "BARCODE") { timer_->cancel(); return; }
    if (objects_) for (const auto & o : objects_->objects)
      if (o.class_name == "barcode" && rclcpp::Time(o.last_seen, RCL_ROS_TIME) >= phase_started_) barcode_ = detail(o, "content");
    if (barcode_.empty() && barcode_rx_ >= phase_started_) barcode_ = barcode_msg_;
    if (barcode_.empty()) {
      if ((now() - phase_started_).seconds() > barcode_timeout_s_) { timer_->cancel(); fail("barcode_missing"); }
      return;
    }
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "바코드 %s", barcode_.c_str());
    const auto p = to_map(start_.x, start_.y, M_PI);  // 관측 = 출발 자리에서 −X(π)를 봄
    run({SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", p.x, p.y, p.yaw))},
      [this](bool ok, const std::string & why) { ok ? read_phase() : fail("observe_nav_failed:" + why); });
  });
}

void MissionManager::read_phase() {
  phase("READ", "신호 12회 관측 중");
  signal_ = perception::msg::MissionSignal();
  read_started_ = now();
  reset_pub_->publish(std_msgs::msg::Empty());
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
    using perception::msg::MissionSignal;
    if (phase_ != "READ") { timer_->cancel(); return; }
    if (signal_.state == MissionSignal::ERROR) { timer_->cancel(); fail("signal_" + signal_.error); return; }
    if (signal_.state != MissionSignal::DONE) {
      if ((now() - read_started_).seconds() > signal_timeout_s_) { timer_->cancel(); fail("signal_timeout"); }
      return;
    }
    timer_->cancel();
    counts_ = {signal_.count_r, signal_.count_y, signal_.count_g};
    std::string why, seq, onsets;
    for (size_t i = 0; i < signal_.sequence.size(); ++i) {
      seq += (i ? "," : "") + q(signal_.sequence[i]);
      onsets += (i ? "," : "") + fmt("%.3f", rclcpp::Time(signal_.onsets[i]).seconds(), 0, 0);
    }
    bool asc = false;
    const bool code_ok = parse_barcode(barcode_, asc);
    if (code_ok) order_ = assign_locations(counts_, asc);
    write_log("signal.json", "{\"barcode\": " + q(barcode_) + ", \"ascending\": " + (asc ? "true" : "false") +
      ", \"sequence\": [" + seq + "], \"onsets\": [" + onsets + "], \"counts\": {\"R\": " + std::to_string(counts_.r) +
      ", \"Y\": " + std::to_string(counts_.y) + ", \"G\": " + std::to_string(counts_.g) + "}, \"order\": " + q(order_) + "}\n");
    if (!valid_counts(counts_, why)) { fail(why); return; }
    if (!code_ok) { fail("unknown_barcode:" + barcode_); return; }
    RCLCPP_INFO(get_logger(), "신호 R%d Y%d G%d · %s → L1 %c · L2 %c · L3 %c", counts_.r, counts_.y, counts_.g,
                barcode_.c_str(), order_[0], order_[1], order_[2]);
    phase("SURVEY", "시작 박스 퍽 조사");
    survey(0);
  });
}

void MissionManager::survey(size_t b) {
  if (b >= start_boxes_.size()) { solve_allocation(); return; }
  const Box box = start_boxes_[b];
  phase("SURVEY", "시작 박스 " + std::to_string(b + 1));
  go_near(box, box.x, box.y, [this, b, box](bool ok, const std::string & why) {
    if (!ok) { RCLCPP_WARN(get_logger(), "시작 박스 %zu 접근 실패 (%s) — 건너뜀", b + 1, why.c_str()); survey(b + 1); return; }
    std::vector<SubGoal> goals;
    for (double dx : puck_dx_)
      goals.push_back(SubGoal().set__skill("inspect").set__target(point(box.x + dx, box.y, box_top_z_ + puck_h_)));
    run(goals, [this, b, box](bool ok2, const std::string & why2) {
      if (!ok2) RCLCPP_WARN(get_logger(), "시작 박스 %zu inspect 실패 (%s)", b + 1, why2.c_str());
      for (double dx : puck_dx_) {  // 그 자리(map)에서 match_radius 안 퍽 중 tag를 읽은 것
        const auto m = to_map(box.x + dx, box.y);
        const perception::msg::TrackedObject * best = nullptr;
        double best_d = match_radius_;
        if (objects_) for (const auto & o : objects_->objects) {
          if (o.class_name.rfind("puck", 0) != 0 || detail(o, "tag").empty()) continue;
          const double d = std::hypot(o.pose.pose.position.x - m.x, o.pose.pose.position.y - m.y);
          if (d < best_d) { best_d = d; best = &o; }
        }
        if (!best) { RCLCPP_WARN(get_logger(), "박스 %zu (%+.2f): 퍽 tag 못 읽음", b + 1, dx); continue; }
        const int tag = std::stoi(detail(*best, "tag"));
        if (puck_at_.count(tag)) continue;
        pucks_.push_back({tag, best->color, detail(*best, "shape")});
        puck_at_[tag] = {b, dx};
        RCLCPP_INFO(get_logger(), "퍽 %d: %s %s (박스 %zu)", tag, best->color.c_str(), detail(*best, "shape").c_str(), b + 1);
      }
      survey(b + 1);
    });
  });
}

void MissionManager::solve_allocation() {
  std::string why;
  std::string list;
  for (const auto & p : pucks_)
    list += std::string(list.empty() ? "" : ", ") + "{\"tag\": " + std::to_string(p.tag) + ", \"color\": " + q(p.color) +
            ", \"shape\": " + q(p.shape) + ", \"box\": " + std::to_string(puck_at_[p.tag].first + 1) + "}";
  const bool ok = allocate(pucks_, counts_, alloc_, why);
  std::string groups;
  if (ok) {
    for (size_t li = 0; li < 3; ++li) {
      const char g = order_[li];
      const auto cells = dice_cells(group_count(counts_, g));
      std::vector<int> tags;
      for (const auto & [tag, grp] : alloc_) if (grp == g) tags.push_back(tag);
      std::string t;
      for (size_t k = 0; k < cells.size() && k < tags.size(); ++k) {
        jobs_.push_back({tags[k], li, cells[k]});
        t += std::string(t.empty() ? "" : ", ") + "{\"tag\": " + std::to_string(tags[k]) + ", \"cell\": " + std::to_string(cells[k]) + "}";
      }
      groups += std::string(groups.empty() ? "" : ", ") + "\"L" + std::to_string(li + 1) + "\": {\"group\": " + q(std::string(1, g)) +
                ", \"pucks\": [" + t + "]}";
    }
  }
  write_log("allocation.json", "{\"pucks\": [" + list + "], \"ok\": " + (ok ? "true" : "false") + ", \"why\": " + q(why) +
            ", \"locations\": {" + groups + "}}\n");
  if (!ok) { fail(why + (pucks_.size() != 12 ? ":" + std::to_string(pucks_.size()) : "")); return; }
  phase("BUILD", "퍽 " + std::to_string(jobs_.size()) + "개 배치");
  build(0);
}

void MissionManager::build(size_t i) {
  if (i >= jobs_.size()) { verify_json_.clear(); phase("VERIFY", "장소 재검증"); verify(0); return; }
  const Job job = jobs_[i];
  phase("BUILD", "퍽 " + std::to_string(job.tag) + " → L" + std::to_string(job.loc + 1) + " 칸 " + std::to_string(job.cell) +
        " (" + std::to_string(i + 1) + "/" + std::to_string(jobs_.size()) + ")");
  const auto [b, dx] = puck_at_[job.tag];
  const Box box = start_boxes_[b];
  go_near(box, box.x + dx, box.y, [this, i, job](bool ok, const std::string & why) {
    if (!ok) { fail("puck_approach:" + std::to_string(job.tag) + ":" + why); return; }
    const perception::msg::TrackedObject * o = nullptr;  // 집는 순간의 트랙 id — 퍽은 tag로 찾는다
    if (objects_) for (const auto & x : objects_->objects)
      if (x.class_name.rfind("puck", 0) == 0 && detail(x, "tag") == std::to_string(job.tag)) o = &x;
    if (!o) { fail("puck_lost:" + std::to_string(job.tag)); return; }
    run({SubGoal().set__skill("pick").set__target(std::to_string(o->id))}, [this, i, job](bool ok2, const std::string & why2) {
      if (!ok2) { fail("pick:" + std::to_string(job.tag) + ":" + why2); return; }
      const auto [dx2, dy2] = cell_offset(job.cell, grid_step_);
      const double tx = locations_[job.loc].first + dx2, ty = locations_[job.loc].second + dy2;
      go_near(loc_boxes_[job.loc], tx, ty, [this, i, job, tx, ty](bool ok3, const std::string & why3) {
        if (!ok3) { fail("place_approach:" + std::to_string(job.tag) + ":" + why3); return; }
        run({SubGoal().set__skill("place").set__target(point(tx, ty, box_top_z_))}, [this, i, job](bool ok4, const std::string & why4) {
          if (!ok4) { fail("place:" + std::to_string(job.tag) + ":" + why4); return; }
          build(i + 1);
        });
      });
    });
  });
}

void MissionManager::verify(size_t li) {
  if (li >= locations_.size()) {
    write_log("verify.json", "{\"locations\": [" + verify_json_ + "]}\n");
    go_home();
    return;
  }
  const double lx = locations_[li].first, ly = locations_[li].second;
  go_near(loc_boxes_[li], lx, ly, [this, li, lx, ly](bool ok, const std::string & why) {
    if (!ok) { fail("verify_approach:L" + std::to_string(li + 1) + ":" + why); return; }
    const rclcpp::Time t0 = now();  // 이 시각 뒤에 실제로 본 퍽만 — 목표 상태 복사 금지 (발제 p7)
    run({SubGoal().set__skill("inspect").set__target(point(lx, ly, box_top_z_ + puck_h_))},
      [this, li, lx, ly, t0](bool ok2, const std::string & why2) {
        if (!ok2) RCLCPP_WARN(get_logger(), "L%zu inspect 실패 (%s) — 본 것만 기록", li + 1, why2.c_str());
        const auto c = to_map(lx, ly);
        std::set<int> seen, expected;
        for (const auto & j : jobs_) if (j.loc == li) expected.insert(j.tag);
        std::string obs;
        if (objects_) for (const auto & o : objects_->objects) {
          if (o.class_name.rfind("puck", 0) != 0 || rclcpp::Time(o.last_seen, RCL_ROS_TIME) < t0) continue;
          const double mx = o.pose.pose.position.x, my = o.pose.pose.position.y;
          if (std::hypot(mx - c.x, my - c.y) > verify_radius_) continue;
          // map → arena (기록은 발제 좌표로)
          const double ax = start_.x + std::cos(start_.yaw) * mx - std::sin(start_.yaw) * my;
          const double ay = start_.y + std::sin(start_.yaw) * mx + std::cos(start_.yaw) * my;
          const std::string tag = detail(o, "tag");
          if (!tag.empty()) seen.insert(std::stoi(tag));
          obs += std::string(obs.empty() ? "" : ", ") + "{\"tag\": " + (tag.empty() ? "null" : tag) +
                 fmt(", \"x_mm\": %.0f, \"y_mm\": %.0f", ax * 1000, ay * 1000, 0) + ", \"color\": " + q(o.color) + "}";
        }
        std::string exp;
        for (int t : expected) exp += std::string(exp.empty() ? "" : ", ") + std::to_string(t);
        verify_json_ += std::string(verify_json_.empty() ? "" : ", ") + "{\"location\": \"L" + std::to_string(li + 1) +
          "\", \"group\": " + q(std::string(1, order_[li])) + ", \"expected\": [" + exp + "], \"observed\": [" + obs +
          "], \"match\": " + (seen == expected ? "true" : "false") + "}";
        RCLCPP_INFO(get_logger(), "재검증 L%zu: 기대 %zu개 · 새로 본 tag %zu개 %s", li + 1, expected.size(), seen.size(),
                    seen == expected ? "일치" : "불일치");
        verify(li + 1);
      });
  });
}

void MissionManager::go_home() {
  if (!back_.empty()) {  // 마지막 장소에 붙어 있음 — 먼저 똑바로 물러남
    const std::string back = back_;
    back_.clear();
    run({SubGoal().set__skill("approach").set__target(back)}, [this](bool ok, const std::string & why) {
      ok ? go_home() : fail("retreat:" + why);
    });
    return;
  }
  phase("RETURN", "출발 구역으로");
  const auto p = to_map(start_.x, start_.y, M_PI);
  run({SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", p.x, p.y, p.yaw))},
    [this](bool ok, const std::string & why) {
      if (!ok) { fail("return_nav:" + why); return; }
      phase("RETURN", "출발 구역에서 정지");
      timer_ = create_wall_timer(std::chrono::duration<double>(home_hold_s_), [this]() {
        timer_->cancel();
        active_ = false;
        phase("DONE", "기록: " + run_dir_);
      });
    });
}

void MissionManager::fail(const std::string & why) {
  active_ = false;
  waiting_plan_ = 0;
  if (timer_) timer_->cancel();
  SubGoalArray stop;  // plan_id 0 + 빈 목록 = coordinator 정지 (진행 중 goal 취소)
  stop.header.stamp = now();
  goals_pub_->publish(stop);
  write_log("error.json", "{\"phase\": " + q(phase_) + ", \"why\": " + q(why) + "}\n");
  RCLCPP_ERROR(get_logger(), "미션 오류 (%s): %s — 안전 정지", phase_.c_str(), why.c_str());
  phase("ERROR", why);
}

void MissionManager::phase(const std::string & name, const std::string & note) {
  phase_ = name;
  std_msgs::msg::String m;
  m.data = name + (note.empty() ? "" : ": " + note);
  state_pub_->publish(m);
  RCLCPP_INFO(get_logger(), "[%s] %s", name.c_str(), note.c_str());
}

// ---------- skill 요청
void MissionManager::run(std::vector<SubGoal> goals, Done done) {
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = PLAN_ID_BASE + ++plan_seq_;
  for (size_t i = 0; i < goals.size(); ++i) goals[i].step = static_cast<uint32_t>(i + 1);
  plan.goals = std::move(goals);
  waiting_plan_ = plan.plan_id;
  waiting_done_ = std::move(done);
  goals_pub_->publish(plan);
}

void MissionManager::go_near(const Box & box, double px, double py, Done done, size_t k, std::vector<Pose2> poses) {
  if (k == 0 && poses.empty() && !back_.empty()) {  // 지금 박스에 붙어 있으면 먼저 standoff까지 똑바로 물러남 (Nav2는 원 0.18이라 붙은 자리에선 출발 못 함)
    const std::string back = back_;
    back_.clear();
    run({SubGoal().set__skill("approach").set__target(back)}, [this, box, px, py, done](bool ok, const std::string & why) {
      if (!ok) { done(false, "retreat:" + why); return; }
      go_near(box, px, py, done);
    });
    return;
  }
  if (poses.empty()) {
    poses = approach_poses(box.x, box.y, box.half, px, py, standoff_);
    double rx, ry;
    if (robot_arena(rx, ry)) {  // 점에서 가까운 변 먼저, 같으면(박스 가운데 등) 로봇에서 가까운 변
      std::stable_sort(poses.begin(), poses.end(), [&](const Pose2 & a, const Pose2 & b) {
        const long da = std::lround(std::hypot(a.x - px, a.y - py) * 100), db = std::lround(std::hypot(b.x - px, b.y - py) * 100);
        return da != db ? da < db : std::hypot(a.x - rx, a.y - ry) < std::hypot(b.x - rx, b.y - ry);
      });
    }
  }
  if (k >= poses.size() || static_cast<int>(k) >= approach_tries_) { done(false, "approach_failed"); return; }
  // Nav2로 변 앞 standoff까지 → approach로 approach_gap까지 직선으로 붙음 (Nav2 원형 footprint로는 못 붙는 거리)
  const auto m = to_map(poses[k].x, poses[k].y, poses[k].yaw);
  const double push = standoff_ - approach_gap_;  // 실패하면 Nav2가 어디서 멈췄는지 모름 — back_ 없음 (다음 navigate가 알아서)
  const auto f = to_map(poses[k].x + push * std::cos(poses[k].yaw), poses[k].y + push * std::sin(poses[k].yaw), poses[k].yaw);
  run({SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", m.x, m.y, m.yaw)),
       SubGoal().set__skill("approach").set__target(fmt("%.4f,%.4f,%.4f", f.x, f.y, f.yaw))},
    [this, box, px, py, done, k, poses, m](bool ok, const std::string & why) {
      if (ok) { back_ = fmt("%.4f,%.4f,%.4f", m.x, m.y, m.yaw); done(true, ""); return; }
      RCLCPP_WARN(get_logger(), "접근 자리 %zu 실패 (%s) — 다음 변", k + 1, why.c_str());
      go_near(box, px, py, done, k + 1, poses);
    });
}

// ---------- 좌표
Pose2 MissionManager::to_map(double x, double y, double yaw) const { return arena_to_map({x, y, yaw}, start_); }

std::string MissionManager::point(double ax, double ay, double z) const {
  const auto m = to_map(ax, ay);
  return fmt("%.4f,%.4f,%.4f", m.x, m.y, z);
}

bool MissionManager::robot_arena(double & x, double & y) const {
  try {
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const double mx = t.transform.translation.x, my = t.transform.translation.y;
    x = start_.x + std::cos(start_.yaw) * mx - std::sin(start_.yaw) * my;
    y = start_.y + std::sin(start_.yaw) * mx + std::cos(start_.yaw) * my;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void MissionManager::write_log(const std::string & name, const std::string & json) const {
  if (run_dir_.empty()) return;
  std::ofstream(run_dir_ + "/" + name) << json;
}

// ---------- RViz — 출발 구역 · 박스 · 장소 칸 (배정 뒤엔 그룹 색)
void MissionManager::publish_markers() {
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray arr;
  int id = 0;
  const auto add = [&](int type, double ax, double ay, double z, double sx, double sy, double sz,
                       float r, float g, float b, float a, const std::string & text = "") {
    Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "mission";
    m.id = id++;
    m.type = type;
    const auto p = to_map(ax, ay);
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = z;
    m.pose.orientation.z = std::sin(-start_.yaw / 2);  // 박스는 arena 축 정렬
    m.pose.orientation.w = std::cos(-start_.yaw / 2);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.text = text;
    arr.markers.push_back(m);
  };
  if (start_zone_.size() == 4)
    add(Marker::CUBE, (start_zone_[0] + start_zone_[2]) / 2, (start_zone_[1] + start_zone_[3]) / 2, 0.005,
        start_zone_[2] - start_zone_[0], start_zone_[3] - start_zone_[1], 0.01, 0.3f, 0.55f, 0.9f, 0.35f);
  for (const auto & b : start_boxes_)
    add(Marker::CUBE, b.x, b.y, box_top_z_ / 2, 2 * b.half, 2 * b.half, box_top_z_, 0.8f, 0.7f, 0.5f, 0.8f);
  for (size_t li = 0; li < loc_boxes_.size(); ++li) {
    const auto & b = loc_boxes_[li];
    add(Marker::CUBE, b.x, b.y, box_top_z_ / 2, 2 * b.half, 2 * b.half, box_top_z_, 0.55f, 0.8f, 0.65f, 0.6f);
    const char g = order_.size() == 3 ? order_[li] : '?';
    add(Marker::TEXT_VIEW_FACING, locations_[li].first, locations_[li].second, 0.35, 0, 0, 0.12, 1, 1, 1, 1,
        "L" + std::to_string(li + 1) + (g == '?' ? "" : " · " + std::string(1, g) + std::to_string(group_count(counts_, g))));
    const auto cells = g == '?' ? std::vector<int>{} : dice_cells(group_count(counts_, g));
    for (int c = 0; c < 9; ++c) {  // 3×3 칸 — 쓰는 칸은 그룹 색, 아니면 흐린 회색
      const auto [dx, dy] = cell_offset(c, grid_step_);
      const bool on = std::find(cells.begin(), cells.end(), c) != cells.end();
      const float r = g == 'R' ? 0.9f : g == 'Y' ? 0.95f : 0.2f, gg = g == 'G' ? 0.75f : g == 'Y' ? 0.85f : 0.2f;
      add(Marker::CYLINDER, locations_[li].first + dx, locations_[li].second + dy, box_top_z_ + 0.03, 0.03, 0.03, 0.06,
          on ? r : 0.6f, on ? gg : 0.6f, on ? 0.2f : 0.6f, on ? 0.9f : 0.25f);
    }
  }
  markers_pub_->publish(arr);
}

}  // namespace plan
