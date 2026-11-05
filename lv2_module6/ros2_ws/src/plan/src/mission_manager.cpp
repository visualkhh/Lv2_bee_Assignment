// mission_manager 구현 — 노드 설명·입출력은 include/plan/mission_manager.hpp, main은 mission_manager_main.cpp
#include "plan/mission_manager.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <set>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

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

std::string fmt(const char * f, double a, double b = 0, double c = 0) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

std::string q(const std::string & s) { return "\"" + s + "\""; }
std::string str(int v) { return std::to_string(v); }
std::string join(const std::vector<std::string> & v) {
  std::string out;
  for (const auto & s : v) out += (out.empty() ? "" : ", ") + s;
  return out;
}
bool is_puck(const perception::msg::TrackedObject & o) { return o.class_name.rfind("puck", 0) == 0; }
bool is_lying(const perception::msg::TrackedObject & o) { return o.class_name == "puck_lying"; }
// coordinator가 사람·안전 때문에 멈춘 실패 — 다시 시도하지 않고 미션을 멈춤 (estop · 수동/탐사 전환 · 다른 계획이 끼어듦)
bool interrupted(const std::string & why) {
  for (const char * p : {"estop", "manual", "explore", "cancelled"})
    if (why.rfind(p, 0) == 0) return true;
  return false;
}
// 주행 전 팔을 접음 (05-arm.md — 주행 동안 팔은 stow. 놓기·보기 뒤 팔이 박스 위에 나가 있음)
SubGoal stow() { return SubGoal().set__skill("pose").set__target("stow"); }
}  // namespace

MissionManager::MissionManager()
: rclcpp::Node("mission_manager") {
  const auto sp = declare_parameter<std::vector<double>>("start_pose_arena", {0.734, 0.950, M_PI});
  start_ = {sp.at(0), sp.at(1), sp.at(2)};
  start_zone_ = declare_parameter<std::vector<double>>("start_zone", {0.434, 0.650, 1.034, 1.250});
  footprint_ = declare_parameter<std::vector<double>>("footprint", {0.069, -0.233, 0.153});
  const double box_half = declare_parameter<double>("box_half", 0.1025);
  start_boxes_ = boxes(declare_parameter<std::vector<double>>("start_boxes",
    {1.468, 0.950, 3.670, 3.152, 2.936, 5.354, 0.734, 5.354, 2.202, 5.354, 3.670, 3.886}), box_half);
  for (const auto & b : boxes(declare_parameter<std::vector<double>>("locations",
         {3.670, 0.950, 2.202, 3.700, 3.670, 5.354}), 0.0)) locations_.push_back({b.x, b.y});
  loc_boxes_ = boxes(declare_parameter<std::vector<double>>("location_boxes",
    {3.670, 0.950, 0.1025, 2.202, 3.700, 0.1025, 3.670, 5.354, 0.1025}), -1.0);
  for (const auto v : declare_parameter<std::vector<int64_t>>("location_tags", {12, 13, 14})) loc_tags_.push_back(static_cast<int>(v));
  puck_dx_ = declare_parameter<std::vector<double>>("puck_offsets_x", {-0.040, 0.040});
  box_top_z_ = declare_parameter<double>("box_top_z", 0.070);
  puck_h_ = declare_parameter<double>("puck_height", 0.060);
  grid_step_ = declare_parameter<double>("grid_step", 0.060);
  standoff_ = declare_parameter<double>("standoff", 0.25);
  approach_gap_ = declare_parameter<double>("approach_gap", 0.085);
  approach_tries_ = static_cast<int>(declare_parameter<int64_t>("approach_tries", 3));
  retries_ = static_cast<int>(declare_parameter<int64_t>("retries", 2));
  match_radius_ = declare_parameter<double>("match_radius", 0.04);
  stand_check_s_ = declare_parameter<double>("stand_check_s", 1.0);
  verify_radius_ = declare_parameter<double>("verify_radius", 0.15);
  place_tol_ = declare_parameter<double>("place_tol", 0.010);
  place_search_ = declare_parameter<double>("place_search", 0.05);
  corr_max_ = declare_parameter<double>("tag_correction_max", 0.05);
  signal_timeout_s_ = declare_parameter<double>("signal_timeout_s", 150.0);
  signal_error_cycles_ = static_cast<int>(declare_parameter<int64_t>("signal_error_cycles", 2));
  barcode_timeout_s_ = declare_parameter<double>("barcode_timeout_s", 20.0);
  const auto bp = declare_parameter<std::vector<double>>("barcode_pose_arena", {0.950, 0.800, M_PI + 0.171});
  barcode_pose_ = {bp.at(0), bp.at(1), bp.at(2)};
  home_hold_s_ = declare_parameter<double>("home_hold_s", 3.0);
  log_dir_ = declare_parameter<std::string>("log_dir", "~/.ros/mission");
  photo_topic_ = declare_parameter<std::string>("photo_topic", "/gripper_cam/color/image_raw");
  // 학습된 플래너 (learning/plan/mission_sim.py에서 학습 · ml/model/plan/plan_next.policy) — 기본은 지금 규칙 순서
  planner_ = declare_parameter<std::string>("planner", "rule");
  std::string pn_path = declare_parameter<std::string>("plan_model", "");
  if (pn_path.empty()) pn_path = ament_index_cpp::get_package_share_directory("ml") + "/model/plan/plan_next.policy";
  if (planner_ == "plan_next") {
    std::string why;
    plan_next_ok_ = plan_next_.load(pn_path, why);
    if (plan_next_ok_) RCLCPP_INFO(get_logger(), "planner plan_next: %s", pn_path.c_str());
    else RCLCPP_WARN(get_logger(), "plan_next를 못 씀 (%s) — 규칙 순서로", why.c_str());
  }
  // plan_id를 노드가 뜰 때마다 다른 구간에서 시작 — /task/status가 latched라 재시작하면 지난 실행의 같은 id 결과가 바로 들어왔음
  plan_seq_ = static_cast<uint32_t>(std::time(nullptr) % 1000) * 1000;
  if (log_dir_.rfind("~/", 0) == 0 && std::getenv("HOME")) log_dir_ = std::string(std::getenv("HOME")) + log_dir_.substr(1);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  goals_pub_ = create_publisher<SubGoalArray>("/task/sub_goals", rclcpp::QoS(10).reliable().transient_local());
  state_pub_ = create_publisher<MissionState>("/mission/state", latched);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mission/markers", latched);
  cmd_sub_ = create_subscription<std_msgs::msg::String>("/cmd/mission", rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_command(m->data); });
  barcode_sub_ = create_subscription<std_msgs::msg::String>("/mission/qr", latched,
    [this](std_msgs::msg::String::ConstSharedPtr m) { barcode_msg_ = m->data; barcode_rx_ = now(); });
  // 신호 — 이번 READ 시작 뒤에 끝난 사이클만 (지난 회차의 latched 답은 버림).
  //   /mission/signal은 새 답일 때만 나와서 지난 회차와 같은 신호면 다시 안 옴 → 사이클마다 오는 /mission/signal_status의 ok도 확정으로 봄
  const auto on_signal = [this](const perception::msg::MissionSignal & m) {
    if (phase_ != Phase::READ || signal_ready_ || !signal_error_.empty()) return;
    if (rclcpp::Time(m.header.stamp, RCL_ROS_TIME) < read_started_) return;
    const auto a = signal_action(m.ok, m.error, signal_streak_, signal_error_cycles_);
    if (a == SignalAction::ACCEPT) { signal_ = m; signal_ready_ = true; }
    else if (a == SignalAction::STOP) { signal_ = m; signal_error_ = m.error; }
    else if (!m.ok) RCLCPP_WARN(get_logger(), "신호 사이클 %s [%s] — 다음 사이클을 봄", m.error.c_str(), m.raw.c_str());
  };
  signal_sub_ = create_subscription<perception::msg::MissionSignal>("/mission/signal", latched,
    [on_signal](perception::msg::MissionSignal::ConstSharedPtr m) { on_signal(*m); });
  signal_status_sub_ = create_subscription<perception::msg::MissionSignal>("/mission/signal_status", rclcpp::QoS(10).reliable(),
    [on_signal](perception::msg::MissionSignal::ConstSharedPtr m) { on_signal(*m); });
  objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { objects_ = m; });
  basket_sub_ = create_subscription<plan::msg::BasketState>("/coordinator/basket", rclcpp::QoS(1).reliable().transient_local(),
    [this](plan::msg::BasketState::ConstSharedPtr m) { basket_ = m; if (active_) publish_state(); });
  status_sub_ = create_subscription<TaskStatus>("/task/status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr s) {
      if (!waiting_plan_ || s->plan_id != waiting_plan_ || s->state == TaskStatus::RUNNING) return;
      waiting_plan_ = 0;
      auto done = std::move(waiting_done_);
      const bool ok = s->state == TaskStatus::SUCCEEDED;
      if (!ok && interrupted(s->reason)) { stop("interrupted: " + s->reason); return; }
      done(ok, s->reason);
    });
  markers_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_markers(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    s.summary(phase_ == Phase::ERROR ? DiagnosticStatus::ERROR : DiagnosticStatus::OK, phase_name(phase_));
    s.add("배정 L1·L2·L3", order_.empty() ? "-" : order_);
    s.add("조사한 퍽", pucks_.size());
    int n = 0;
    if (basket_) for (const auto & b : basket_->slots) n += b.filled;
    s.add("적재함", n);
    s.add("놓음", str(placed_) + "/" + str(static_cast<int>(jobs_.size())));
    s.add("바코드", barcode_.empty() ? "-" : barcode_);
  });
  note("/cmd/mission start 대기");
}

// ---------- 단계
void MissionManager::on_command(const std::string & cmd) {
  if (cmd == "abort") { if (active_) stop("aborted"); return; }
  if (cmd != "start" || active_) return;
  int left = 0;
  if (basket_) {
    for (const auto & b : basket_->slots) left += b.filled;
    left += basket_->held.filled;
  }
  if (left) {  // 지난 미션 장부(파일)가 남음 — 적재함이 정말 비었는지 사람이 보고 비워야 함
    RCLCPP_WARN(get_logger(), "적재함 장부에 퍽 %d개가 남아 있음 — 시작 거부", left);
    note("시작 거부: 적재함 장부에 퍽 " + str(left) + "개 — 확인 뒤 ros2 service call /coordinator/basket_clear std_srvs/srv/Trigger");
    return;
  }
  active_ = true;
  pn_action_ = -1;
  pucks_.clear();
  puck_at_.clear();
  alloc_.clear();
  jobs_.clear();
  order_.clear();
  results_.clear();
  build_json_.clear();
  verify_json_.clear();
  seen_tags_.clear();
  lying_.clear();
  corr_.assign(locations_.size(), {0.0, 0.0});
  barcode_.clear();
  error_.clear();
  back_.clear();
  placed_ = 0;
  photo_n_ = 0;
  char stamp[32];
  const std::time_t t = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
  run_dir_ = log_dir_ + "/" + stamp;
  std::filesystem::create_directories(run_dir_ + "/photos");
  enter(next_phase(Phase::IDLE, true));
}

void MissionManager::finish(bool ok, const std::string & why) {
  if (!active_) return;
  const Phase n = next_phase(phase_, ok);
  if (!ok) result(std::string(phase_name(phase_)) + ": " + why);
  enter(n, why);
}

void MissionManager::enter(Phase p, const std::string & why) {
  phase_ = p;
  switch (p) {
    case Phase::GO_OBSERVE: go_observe(); break;
    case Phase::BARCODE: barcode_phase(); break;
    case Phase::READ: read_phase(); break;
    case Phase::SURVEY: survey(0); break;
    case Phase::BUILD: build(0); break;
    case Phase::VERIFY: verify(0); break;
    case Phase::RETURN: go_home(); break;
    case Phase::DONE:
      active_ = false;
      note("기록: " + run_dir_);
      break;
    case Phase::ERROR: stop(why); break;
    default: break;
  }
}

void MissionManager::go_observe() {
  note("팔 scan 자세 · 바코드 보는 자세로 (출발 구역 안)");
  const auto p = to_map(barcode_pose_.x, barcode_pose_.y, barcode_pose_.yaw);
  run({SubGoal().set__skill("pose").set__target("scan"),
       SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", p.x, p.y, p.yaw))},
    [this](bool ok, const std::string & why) { finish(ok, "observe_nav_failed:" + why); });
}

void MissionManager::barcode_phase() {
  // 이번에 실제로 본 바코드만 — /tracked_objects의 barcode(last_seen ≥ 시작) 또는 시작 뒤 받은 /mission/qr
  note("바코드 읽는 중");
  phase_started_ = now();
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
    if (phase_ != Phase::BARCODE) { timer_->cancel(); return; }
    if (objects_) for (const auto & o : objects_->objects)
      if (o.class_name == "barcode" && rclcpp::Time(o.last_seen, RCL_ROS_TIME) >= phase_started_) barcode_ = detail(o, "content");
    if (barcode_.empty() && barcode_rx_ >= phase_started_) barcode_ = barcode_msg_;
    if (barcode_.empty()) {
      if ((now() - phase_started_).seconds() > barcode_timeout_s_) { timer_->cancel(); finish(false, "barcode_missing"); }
      return;
    }
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "바코드 %s", barcode_.c_str());
    snap("barcode");
    const auto p = to_map(start_.x, start_.y, M_PI);  // 관측 = 출발 자리에서 −X(π)를 봄
    run({SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", p.x, p.y, p.yaw))},
      [this](bool ok, const std::string & why) { finish(ok, "observe_nav_failed:" + why); });
  });
}

void MissionManager::read_phase() {
  note("신호 12회 관측 중");
  signal_ = perception::msg::MissionSignal();
  signal_ready_ = false;
  signal_streak_ = 0;
  signal_error_.clear();
  read_started_ = now();
  timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
    if (phase_ != Phase::READ) { timer_->cancel(); return; }
    if (!signal_error_.empty()) {  // 오류 사이클이 이어짐 — 추정하지 않고 정지
      timer_->cancel();
      write_log("signal.json", "{\"ok\": false, \"error\": " + q(signal_error_) + ", \"raw\": " + q(signal_.raw) + "}\n");
      finish(false, "signal_" + signal_error_);
      return;
    }
    if (!signal_ready_) {
      if ((now() - read_started_).seconds() > signal_timeout_s_) { timer_->cancel(); finish(false, "signal_timeout"); }
      return;
    }
    timer_->cancel();
    counts_ = {signal_.red, signal_.yellow, signal_.green};
    std::string why, seq;
    for (size_t i = 0; i < signal_.sequence.size(); ++i) seq += (i ? "," : "") + q(signal_.sequence[i]);
    bool asc = false;
    const bool code_ok = parse_barcode(barcode_, asc);
    if (code_ok) order_ = assign_locations(counts_, asc);
    write_log("signal.json", "{\"barcode\": " + q(barcode_) + ", \"ascending\": " + (asc ? "true" : "false") +
      ", \"sequence\": [" + seq + "], \"raw\": " + q(signal_.raw) + ", \"cycles\": " + str(signal_.cycles) + ", \"counts\": {\"R\": " + str(counts_.r) +
      ", \"Y\": " + str(counts_.y) + ", \"G\": " + str(counts_.g) + "}, \"order\": " + q(order_) + "}\n");
    if (!valid_counts(counts_, why)) { finish(false, why); return; }
    if (!code_ok) { finish(false, "unknown_barcode:" + barcode_); return; }
    RCLCPP_INFO(get_logger(), "신호 R%d Y%d G%d · %s → L1 %c · L2 %c · L3 %c", counts_.r, counts_.y, counts_.g,
                barcode_.c_str(), order_[0], order_[1], order_[2]);
    finish(true);
  });
}

// ---------- SURVEY — 시작 박스 6곳 전부 조사하면서 적재함에 싣기
void MissionManager::survey(size_t b) {
  if (b >= start_boxes_.size()) { solve_allocation(); return; }
  const Box box = start_boxes_[b];
  note("시작 박스 " + str(static_cast<int>(b) + 1) + " 조사");
  go_near(box, box.x, box.y, [this, b, box](bool ok, const std::string & why) {
    if (!ok) { result("box " + str(static_cast<int>(b) + 1) + ": approach_failed " + why); survey(b + 1); return; }
    std::vector<SubGoal> goals;
    for (double dx : puck_dx_)
      goals.push_back(SubGoal().set__skill("inspect").set__target(point(box.x + dx, box.y, box_top_z_ + puck_h_)));
    run_retry(goals, [this, b, box](bool ok2, const std::string & why2) {
      if (!ok2) RCLCPP_WARN(get_logger(), "시작 박스 %zu inspect 실패 (%s) — 본 것만", b + 1, why2.c_str());
      snap("box" + str(static_cast<int>(b) + 1));
      std::vector<std::pair<int, uint32_t>> found;
      for (double dx : puck_dx_) {  // 그 자리(map)에서 match_radius 안 퍽 중 tag를 읽은 것
        const auto m = to_map(box.x + dx, box.y);
        const TrackedObject * best = nullptr;
        double best_d = match_radius_;
        if (objects_) for (const auto & o : objects_->objects) {
          if (!is_puck(o) || detail(o, "tag").empty()) continue;
          const double d = std::hypot(o.pose.pose.position.x - m.x, o.pose.pose.position.y - m.y);
          if (d < best_d) { best_d = d; best = &o; }
        }
        if (!best) { result("box " + str(static_cast<int>(b) + 1) + fmt(" (%+.2f): tag_unread", dx)); continue; }
        const int tag = std::stoi(detail(*best, "tag"));
        if (puck_at_.count(tag)) continue;
        pucks_.push_back({tag, best->color, detail(*best, "shape")});
        puck_at_[tag] = {b, dx};
        if (is_lying(*best)) lying_.insert(tag);  // 실을 때 세워서 집음 (stand → stand_pick)
        found.push_back({tag, best->id});
        RCLCPP_INFO(get_logger(), "퍽 %d: %s %s (박스 %zu)", tag, best->color.c_str(), detail(*best, "shape").c_str(), b + 1);
      }
      publish_state();
      load(b, found, 0);
    });
  });
}

void MissionManager::load(size_t b, std::vector<std::pair<int, uint32_t>> found, size_t k) {
  if (k >= found.size()) { survey(b + 1); return; }
  if (!basket_has_room()) { load(b, found, found.size()); return; }  // 꽉 참 — 나머지는 BUILD 때 시작 박스에서 직접
  const int tag = found[k].first;
  const uint32_t id = found[k].second;
  note("퍽 " + str(tag) + " → 적재함 (load next)");
  // 칸은 coordinator가 고르고 장부에 tag까지 적음 (pick 때 그 퍽의 tag·색·모양을 기억해 둠)
  grab(tag, id, [this, b, found, k, tag](bool ok, const std::string & why) {
    if (!ok) { result("puck " + str(tag) + ": load_pick_failed " + why + " (직접 운반)"); load(b, found, k + 1); return; }
    run_retry({SubGoal().set__skill("load").set__target("next")}, [this, b, found, k, tag](bool ok2, const std::string & why2) {
      if (ok2) { load(b, found, k + 1); return; }
      // 적재함에 못 실음 — 손에 든 퍽을 박스 제자리에 되돌려 두고 BUILD 때 직접 운반
      const auto [bb, dx] = puck_at_[tag];
      run({SubGoal().set__skill("place").set__target(point(start_boxes_[bb].x + dx, start_boxes_[bb].y, box_top_z_))},
        [this, b, found, k, tag, why2](bool ok3, const std::string &) {
          result("puck " + str(tag) + ": load_tray_failed " + why2 + (ok3 ? " (박스에 되돌림 · 직접 운반)" : " (되돌리기도 실패)"));
          load(b, found, k + 1);
        });
    });
  });
}

void MissionManager::solve_allocation() {
  std::string why;
  std::string list;
  for (const auto & p : pucks_) {
    const int slot = basket_slot_of(p.tag);
    list += std::string(list.empty() ? "" : ", ") + "{\"tag\": " + str(p.tag) + ", \"color\": " + q(p.color) +
            ", \"shape\": " + q(p.shape) + ", \"box\": " + str(static_cast<int>(puck_at_[p.tag].first) + 1) +
            ", \"tray_slot\": " + (slot < 0 ? "null" : str(slot)) + "}";
  }
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
        t += std::string(t.empty() ? "" : ", ") + "{\"tag\": " + str(tags[k]) + ", \"cell\": " + str(cells[k]) + "}";
      }
      groups += std::string(groups.empty() ? "" : ", ") + "\"L" + str(static_cast<int>(li) + 1) + "\": {\"group\": " +
                q(std::string(1, g)) + ", \"pucks\": [" + t + "]}";
    }
  }
  write_log("allocation.json", "{\"pucks\": [" + list + "], \"ok\": " + (ok ? "true" : "false") + ", \"why\": " + q(why) +
            ", \"locations\": {" + groups + "}}\n");
  if (!ok) { finish(false, why + (pucks_.size() != 12 ? ":" + std::to_string(pucks_.size()) : "")); return; }
  finish(true);
}

// ---------- BUILD — 퍽마다 [손에 쥐기 → 놓기 → 확인]. 실패하면 기록하고 다음 퍽
void MissionManager::build(size_t i) {
  if (i == 0 && planner_ == "plan_next" && plan_next_ok_ && build_pn_start()) { build_pn(); return; }
  if (i >= jobs_.size()) {
    write_log("build.json", "{\"placed\": " + str(placed_) + ", \"total\": " + str(static_cast<int>(jobs_.size())) +
              ", \"pucks\": [" + build_json_ + "]}\n");
    finish(true);
    return;
  }
  const Job job = jobs_[i];
  note("퍽 " + str(job.tag) + " → L" + str(static_cast<int>(job.loc) + 1) + " 칸 " + str(job.cell) +
       " (" + str(static_cast<int>(i) + 1) + "/" + str(static_cast<int>(jobs_.size())) + ")");
  bring(i);
}

void MissionManager::bring(size_t i) {
  const Job job = jobs_[i];
  const auto skip = [this, i, job](const std::string & why) {
    result("puck " + str(job.tag) + ": " + why);
    build_json_ += std::string(build_json_.empty() ? "" : ", ") + "{\"tag\": " + str(job.tag) + ", \"ok\": false, \"why\": " + q(why) + "}";
    job_done(i, false);
  };
  const auto target = cell_target(job.loc, job.cell);
  const double tx = target.first, ty = target.second;
  const int slot = basket_slot_of(job.tag);  // coordinator 장부에서 그 tag가 실린 칸
  if (slot >= 0) {  // 적재함에 있음 — 장소 앞에 가서 꺼냄
    go_near(loc_boxes_[job.loc], tx, ty, [this, i, job, slot, skip](bool ok, const std::string & why) {
      if (!ok) { skip("place_approach_failed " + why); return; }
      correct(job.loc);
      run_retry({SubGoal().set__skill("unload").set__target(str(slot))}, [this, i, skip](bool ok2, const std::string & why2) {
        if (!ok2) { skip("tray_pick_failed " + why2); return; }
        place_check(i, 0);
      });
    });
    return;
  }
  // 못 실은 퍽 — 시작 박스에서 직접 집어 옴
  const auto [b, dx] = puck_at_[job.tag];
  const Box box = start_boxes_[b];
  go_near(box, box.x + dx, box.y, [this, i, job, tx, ty, skip](bool ok, const std::string & why) {
    if (!ok) { skip("puck_approach_failed " + why); return; }
    const TrackedObject * o = find_puck(job.tag);  // 집는 순간의 트랙 id — 퍽은 tag로 찾는다
    if (!o) { skip("puck_lost"); return; }
    if (is_lying(*o)) lying_.insert(job.tag);  // 조사 뒤에 넘어진 퍽도 세워서
    grab(job.tag, o->id, [this, i, job, tx, ty, skip](bool ok2, const std::string & why2) {
      if (!ok2) { skip("pick_failed " + why2); return; }
      go_near(loc_boxes_[job.loc], tx, ty, [this, i, job, skip](bool ok3, const std::string & why3) {
        if (ok3) { correct(job.loc); place_check(i, 0); return; }
        // 장소에 못 감 — 손에 든 퍽은 적재함 빈 칸에 (다음 퍽을 집을 수 있게)
        if (!basket_has_room()) { skip("place_approach_failed " + why3 + " (퍽을 든 채)"); return; }
        run({SubGoal().set__skill("load").set__target("next")}, [why3, skip](bool ok4, const std::string &) {
          skip("place_approach_failed " + why3 + (ok4 ? " (적재함에 둠)" : " (퍽을 든 채)"));
        });
      });
    });
  });
}

void MissionManager::job_done(size_t i, bool ok) {
  if (pn_action_ < 0) { build(i + 1); return; }  // 규칙 순서
  // plan_next: 이 퍽 결과로 상태를 고치고 다시 고름 — 퍽 하나 처리(bring)에 다시 시도가 들어 있어 실패면 포기(LOST)로 봄
  const int p = PlanNext::puck_of(pn_action_);
  pn_.status[p] = ok ? PlanNext::OK : PlanNext::LOST;
  pn_.tries[p] += 1;
  pn_.at = 6 + static_cast<int>(jobs_[i].loc);
  plan_next_.record(pn_action_, true);
  build_pn();
}

bool MissionManager::build_pn_start() {
  // 퍽 i = 시작 박스 i/2 (가상 미션과 같은 번호) — 박스마다 x 오프셋 순. 12개·박스마다 2개가 아니면 규칙 순서로
  std::vector<size_t> order(jobs_.size());
  for (size_t j = 0; j < order.size(); ++j) order[j] = j;
  std::sort(order.begin(), order.end(), [this](size_t a, size_t b) {
    return puck_at_[jobs_[a].tag] < puck_at_[jobs_[b].tag];
  });
  if (order.size() != static_cast<size_t>(PlanNext::N_PUCK)) return false;
  for (size_t k = 0; k < order.size(); ++k)
    if (puck_at_[jobs_[order[k]].tag].first != k / 2) { RCLCPP_WARN(get_logger(), "plan_next: 박스마다 퍽 2개가 아님 — 규칙 순서로"); return false; }
  pn_ = PlanNext::State{};
  pn_job_ = order;
  for (int p = 0; p < PlanNext::N_PUCK; ++p) {
    const Job & job = jobs_[order[p]];
    pn_.loc[p] = static_cast<int>(job.loc);
    pn_.cell[p] = job.cell;
    pn_.status[p] = basket_slot_of(job.tag) >= 0 ? PlanNext::BASKET : lying_.count(job.tag) ? PlanNext::BOX_LYING : PlanNext::BOX_UP;
  }
  plan_next_.reset();
  build_started_ = now();
  return true;
}

void MissionManager::build_pn() {
  pn_.t = (now() - build_started_).seconds();
  pn_action_ = plan_next_.next(pn_);
  if (pn_action_ == PlanNext::RETURN) {
    write_log("build.json", "{\"placed\": " + str(placed_) + ", \"total\": " + str(static_cast<int>(jobs_.size())) +
              ", \"planner\": \"plan_next\", \"pucks\": [" + build_json_ + "]}\n");
    pn_action_ = -1;
    finish(true);
    return;
  }
  const int p = PlanNext::puck_of(pn_action_);
  const size_t j = pn_job_[p];
  static const char * skills[] = {"pick", "stand", "load", "unload", "place", "nudge", "re-pick", "stand_placed"};
  note("plan_next: " + std::string(skills[PlanNext::skill_of(pn_action_)]) + " 퍽 " + str(jobs_[j].tag) + " → L" +
       str(static_cast<int>(jobs_[j].loc) + 1) + " 칸 " + str(jobs_[j].cell));
  bring(j);  // 손에 쥐기(적재함이면 꺼내기, 누웠으면 세우기) → 놓기 → 확인 — 끝나면 job_done
}

void MissionManager::place_check(size_t i, int tries) {
  const Job job = jobs_[i];
  const auto target = cell_target(job.loc, job.cell);
  const double tx = target.first, ty = target.second;
  const auto next = [this, i, job](bool ok, const std::string & why) {
    if (ok) ++placed_;
    else result("puck " + str(job.tag) + ": " + why);
    build_json_ += std::string(build_json_.empty() ? "" : ", ") + "{\"tag\": " + str(job.tag) + ", \"location\": \"L" +
                   str(static_cast<int>(job.loc) + 1) + "\", \"cell\": " + str(job.cell) + ", \"ok\": " + (ok ? "true" : "false") +
                   ", \"why\": " + q(why) + "}";
    publish_state();
    job_done(i, ok);
  };
  run({SubGoal().set__skill("place").set__target(point(tx, ty, box_top_z_))}, [this, i, tries, tx, ty, next](bool ok, const std::string & why) {
    if (!ok) { next(false, "place_failed " + why); return; }
    const rclcpp::Time t0 = now();  // 놓은 뒤에 실제로 본 것만
    run({SubGoal().set__skill("inspect").set__target(point(tx, ty, box_top_z_ + puck_h_))}, [this, i, tries, tx, ty, t0, next](bool, const std::string &) {
      const Job job = jobs_[i];
      const auto m = to_map(tx, ty);
      const TrackedObject * best = nullptr;  // 칸 근처에서 이번에 본 퍽 — tag가 맞거나 못 읽은 것
      double err = place_search_;
      if (objects_) for (const auto & o : objects_->objects) {
        if (!is_puck(o) || rclcpp::Time(o.last_seen, RCL_ROS_TIME) < t0) continue;
        const std::string tag = detail(o, "tag");
        if (!tag.empty() && tag != str(job.tag)) continue;
        const double d = std::hypot(o.pose.pose.position.x - m.x, o.pose.pose.position.y - m.y);
        if (d < err) { err = d; best = &o; }
      }
      snap("L" + str(static_cast<int>(job.loc) + 1) + "_cell" + str(job.cell) + "_tag" + str(job.tag));
      const bool lying = best && best->class_name == "puck_lying";
      if (best && !lying && err <= place_tol_) { next(true, fmt("err %.0fmm", err * 1000)); return; }
      const std::string why = !best ? "place_unseen" : lying ? "place_lying" : fmt("place_off %.0fmm", err * 1000);
      if (!best || tries >= retries_) { next(false, why); return; }
      RCLCPP_WARN(get_logger(), "퍽 %d %s — 다시 놓기 (%d/%d)", job.tag, why.c_str(), tries + 1, retries_);
      const auto again = [this, i, tries, next, why](bool ok2, const std::string & why2) {
        if (!ok2) { next(false, why + " · re-pick_failed " + why2); return; }
        place_check(i, tries + 1);
      };
      if (lying) stand_up(best->id, best->pose.pose.position.x, best->pose.pose.position.y, 0, again);  // 칸에서 누움 → 세워서 다시 집기
      else run({SubGoal().set__skill("pick").set__target(std::to_string(best->id))}, again);
    });
  });
}

// ---------- VERIFY — 장소마다 새로 본 퍽만 (발제 p7: 목표 상태 복사 금지) → 칸별 판정 + 12개 중복·누락
void MissionManager::verify(size_t li) {
  if (li >= locations_.size()) {
    std::vector<int> all, dup, missing;
    for (const auto & j : jobs_) all.push_back(j.tag);
    global_check(seen_tags_, all, dup, missing);
    std::vector<std::string> d, m;
    for (int t : dup) d.push_back(str(t));
    for (int t : missing) m.push_back(str(t));
    if (!dup.empty()) result("verify: duplicate " + join(d));
    if (!missing.empty()) result("verify: missing " + join(m));
    write_log("verify.json", "{\"locations\": [" + verify_json_ + "], \"duplicate\": [" + join(d) + "], \"missing\": [" +
              join(m) + "], \"ok\": " + (dup.empty() && missing.empty() ? "true" : "false") + "}\n");
    finish(true);
    return;
  }
  const std::string name = "L" + str(static_cast<int>(li) + 1);
  note(name + " 재검증");
  const double lx = locations_[li].first, ly = locations_[li].second;
  go_near(loc_boxes_[li], lx, ly, [this, li, name](bool ok, const std::string & why) {
    if (!ok) {
      result(name + ": verify_approach_failed " + why);
      seen_tags_.push_back({});
      verify_json_ += std::string(verify_json_.empty() ? "" : ", ") + "{\"location\": " + q(name) + ", \"ok\": false, \"why\": " + q(why) + "}";
      verify(li + 1);
      return;
    }
    correct(li);
    const auto center = cell_target(li, 4);  // 가운데 칸 = 보정한 장소 중심
    const double cx = center.first, cy = center.second;
    const rclcpp::Time t0 = now();
    run_retry({SubGoal().set__skill("inspect").set__target(point(cx, cy, box_top_z_ + puck_h_))},
      [this, li, name, cx, cy, t0](bool ok2, const std::string & why2) {
        if (!ok2) RCLCPP_WARN(get_logger(), "%s inspect 실패 (%s) — 본 것만 기록", name.c_str(), why2.c_str());
        snap("verify_" + name);
        const auto c = to_map(cx, cy);
        std::vector<Seen> seen;
        std::vector<int> tags;
        std::string obs;
        if (objects_) for (const auto & o : objects_->objects) {
          if (!is_puck(o) || rclcpp::Time(o.last_seen, RCL_ROS_TIME) < t0) continue;
          const double mx = o.pose.pose.position.x, my = o.pose.pose.position.y;
          if (std::hypot(mx - c.x, my - c.y) > verify_radius_) continue;
          const auto a = to_arena(mx, my);  // 기록은 발제 좌표로
          const std::string tag = detail(o, "tag");
          const int t = tag.empty() ? -1 : std::stoi(tag);
          seen.push_back({t, a.x, a.y});
          tags.push_back(t);
          obs += std::string(obs.empty() ? "" : ", ") + "{\"tag\": " + (tag.empty() ? "null" : tag) +
                 fmt(", \"x_mm\": %.0f, \"y_mm\": %.0f", a.x * 1000, a.y * 1000) + ", \"color\": " + q(o.color) +
                 ", \"lying\": " + (o.class_name == "puck_lying" ? "true" : "false") + "}";
        }
        std::vector<std::pair<int, int>> expected;
        for (const auto & j : jobs_) if (j.loc == li) expected.push_back({j.tag, j.cell});
        const auto r = judge_location(expected, cx, cy, grid_step_, seen, place_tol_);
        std::string cells;
        for (const auto & k : r.cells)
          cells += std::string(cells.empty() ? "" : ", ") + "{\"tag\": " + str(k.tag) + ", \"cell\": " + str(k.cell) +
                   ", \"found\": " + (k.found ? "true" : "false") + fmt(", \"err_mm\": %.1f", k.err * 1000) + "}";
        seen_tags_.push_back(tags);
        verify_json_ += std::string(verify_json_.empty() ? "" : ", ") + "{\"location\": " + q(name) + ", \"group\": " +
          q(std::string(1, order_[li])) + fmt(", \"correction_mm\": [%.1f, %.1f]", corr_[li].first * 1000, corr_[li].second * 1000) +
          ", \"cells\": [" + cells + "], \"observed\": [" + obs + "], \"extra\": " + str(static_cast<int>(r.extra.size())) +
          ", \"ok\": " + (r.ok ? "true" : "false") + "}";
        result(name + ": " + (r.ok ? "ok" : "mismatch"));
        RCLCPP_INFO(get_logger(), "재검증 %s: 칸 %zu개 · 새로 본 퍽 %zu개 · 빈 칸 퍽 %zu개 → %s", name.c_str(), r.cells.size(),
                    seen.size(), r.extra.size(), r.ok ? "일치" : "불일치");
        verify(li + 1);
      });
  });
}

// ---------- RETURN — 출발 구역 가운데 → 외곽 직사각형이 구역 안인지 확인, 밖이면 한 번 보정 → 정지
void MissionManager::go_home(bool corrected) {
  if (!back_.empty()) {  // 박스에 붙어 있음 — 먼저 똑바로 물러남
    const std::string back = back_;
    back_.clear();
    run({stow(), SubGoal().set__skill("approach").set__target(back)}, [this, corrected](bool ok, const std::string & why) {
      ok ? go_home(corrected) : finish(false, "retreat:" + why);
    });
    return;
  }
  note(corrected ? "출발 구역 안으로 보정" : "출발 구역으로");
  const double zx = (start_zone_[0] + start_zone_[2]) / 2, zy = (start_zone_[1] + start_zone_[3]) / 2;
  const auto p = to_map(zx, zy, M_PI);
  // 처음엔 Nav2로 · 보정은 approach로 (구역 가운데로 저속 직선 — Nav2 허용 오차 0.10보다 정확). 보정은 park_home 정책
  //   (skill_approach — 시뮬 drive_env park: 5 cm · 10° 안에 3 s 정지, 81.1 → 96.5 %)
  run({stow(), SubGoal().set__skill(corrected ? "approach" : "navigate").set__target(fmt("%.4f,%.4f,%.4f", p.x, p.y, p.yaw))
                 .set__relation(corrected ? "park_home" : "")},
    [this, corrected](bool ok, const std::string & why) {
      if (!ok && !corrected) { finish(false, "return_nav:" + why); return; }
      Pose2 r;
      const bool known = robot_arena(r);
      const bool in = known && footprint_in_zone(r, footprint_[0], footprint_[1], footprint_[2], start_zone_);
      if (!in && !corrected) { go_home(true); return; }
      note(in ? "출발 구역에서 정지" : "출발 구역 밖 — 그대로 정지");
      write_log("return.json", "{\"in_zone\": " + std::string(in ? "true" : "false") + ", \"corrected\": " +
                (corrected ? "true" : "false") + fmt(", \"x_mm\": %.0f, \"y_mm\": %.0f", r.x * 1000, r.y * 1000) +
                fmt(", \"yaw\": %.3f}\n", r.yaw));
      if (!in) result("return: footprint_out_of_zone");
      snap("return");
      timer_ = create_wall_timer(std::chrono::duration<double>(home_hold_s_), [this]() {
        timer_->cancel();
        finish(true);
      });
    });
}

void MissionManager::stop(const std::string & why) {
  active_ = false;
  waiting_plan_ = 0;
  phase_ = Phase::ERROR;
  error_ = why;
  if (timer_) timer_->cancel();
  SubGoalArray stop;  // plan_id 0 + 빈 목록 = coordinator 정지 (진행 중 goal 취소)
  stop.header.stamp = now();
  goals_pub_->publish(stop);
  write_log("error.json", "{\"why\": " + q(why) + ", \"results\": [" + [&] {
    std::vector<std::string> v;
    for (const auto & r : results_) v.push_back(q(r));
    return join(v);
  }() + "]}\n");
  RCLCPP_ERROR(get_logger(), "미션 오류: %s — 안전 정지", why.c_str());
  note(why);
}

void MissionManager::note(const std::string & text) {
  note_ = text;
  RCLCPP_INFO(get_logger(), "[%s] %s", phase_name(phase_), text.c_str());
  publish_state();
}

void MissionManager::result(const std::string & text) {
  results_.push_back(text);
  RCLCPP_WARN(get_logger(), "결과: %s", text.c_str());
  publish_state();
}

void MissionManager::publish_state() {
  MissionState m;
  m.header.stamp = now();
  m.phase = phase_name(phase_);
  m.note = note_;
  m.barcode = barcode_;
  m.count_r = static_cast<uint8_t>(counts_.r);
  m.count_y = static_cast<uint8_t>(counts_.y);
  m.count_g = static_cast<uint8_t>(counts_.g);
  m.order = order_;
  if (basket_) for (const auto & s : basket_->slots)  // coordinator 장부 그대로 — "칸:tag" (tag 모름 ?, 확인 필요 !)
    if (s.filled) m.tray.push_back(str(s.slot) + ":" + (s.tag < 0 ? std::string("?") : str(s.tag)) + (s.check ? "!" : ""));
  m.placed = static_cast<uint16_t>(placed_);
  m.total = static_cast<uint16_t>(jobs_.size());
  m.results = results_;
  m.error = error_;
  state_pub_->publish(m);
}

// ---------- skill 요청
void MissionManager::run(std::vector<SubGoal> goals, Done done) {
  if (!active_) return;  // abort·오류 뒤에 남은 콜백이 새 계획을 보내지 않게
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = PLAN_ID_BASE + ++plan_seq_;
  for (size_t i = 0; i < goals.size(); ++i) goals[i].step = static_cast<uint32_t>(i + 1);
  plan.goals = std::move(goals);
  waiting_plan_ = plan.plan_id;
  waiting_done_ = std::move(done);
  goals_pub_->publish(plan);
}

void MissionManager::run_retry(std::vector<SubGoal> goals, Done done, int tries) {
  run(goals, [this, goals, done, tries](bool ok, const std::string & why) {
    if (ok || tries >= retries_) { done(ok, why); return; }
    RCLCPP_WARN(get_logger(), "%s %s 실패 (%s) — 다시 (%d/%d)", goals.front().skill.c_str(), goals.front().target.c_str(),
                why.c_str(), tries + 1, retries_);
    run_retry(goals, done, tries + 1);
  });
}

void MissionManager::go_near(const Box & box, double px, double py, Done done, size_t k, std::vector<Pose2> poses) {
  if (k == 0 && poses.empty() && !back_.empty()) {  // 지금 박스에 붙어 있으면 먼저 standoff까지 똑바로 물러남 (Nav2는 원 0.18이라 붙은 자리에선 출발 못 함)
    const std::string back = back_;
    back_.clear();
    run({stow(), SubGoal().set__skill("approach").set__target(back)}, [this, box, px, py, done](bool ok, const std::string & why) {
      if (!ok) { done(false, "retreat:" + why); return; }
      go_near(box, px, py, done);
    });
    return;
  }
  if (poses.empty()) {
    poses = approach_poses(box.x, box.y, box.half, px, py, standoff_);
    Pose2 r;
    if (robot_arena(r)) {  // 점에서 가까운 변 먼저, 같으면(박스 가운데 등) 로봇에서 가까운 변
      std::stable_sort(poses.begin(), poses.end(), [&](const Pose2 & a, const Pose2 & b) {
        const long da = std::lround(std::hypot(a.x - px, a.y - py) * 100), db = std::lround(std::hypot(b.x - px, b.y - py) * 100);
        return da != db ? da < db : std::hypot(a.x - r.x, a.y - r.y) < std::hypot(b.x - r.x, b.y - r.y);
      });
    }
  }
  if (k >= poses.size() || static_cast<int>(k) >= approach_tries_) { done(false, "approach_failed"); return; }
  // Nav2로 변 앞 standoff까지 → approach로 approach_gap까지 직선으로 붙음 (Nav2 원형 footprint로는 못 붙는 거리)
  const auto m = to_map(poses[k].x, poses[k].y, poses[k].yaw);
  const double push = standoff_ - approach_gap_;  // 실패하면 Nav2가 어디서 멈췄는지 모름 — back_ 없음 (다음 navigate가 알아서)
  const auto f = to_map(poses[k].x + push * std::cos(poses[k].yaw), poses[k].y + push * std::sin(poses[k].yaw), poses[k].yaw);
  run({stow(), SubGoal().set__skill("navigate").set__target(fmt("%.3f,%.3f,%.3f", m.x, m.y, m.yaw)),
       SubGoal().set__skill("approach").set__target(fmt("%.4f,%.4f,%.4f", f.x, f.y, f.yaw)).set__relation("dock_box")},
    [this, box, px, py, done, k, poses, m](bool ok, const std::string & why) {
      if (ok) { back_ = fmt("%.4f,%.4f,%.4f", m.x, m.y, m.yaw); done(true, ""); return; }
      RCLCPP_WARN(get_logger(), "접근 자리 %zu 실패 (%s) — 다음 변", k + 1, why.c_str());
      go_near(box, px, py, done, k + 1, poses);
    });
}

// ---------- 놓기 기준 보정 — 인지 location_box(장소 상자 윗면 가운데, 태그 12~14)가 예상 상자 중심에서 어긋난 만큼 칸을 옮김
//   윗면을 못 찾고 태그만 본 location_box는 태그 근처라 보통 tag_correction_max를 넘어 보정 안 함
void MissionManager::correct(size_t li) {
  if (li >= loc_tags_.size() || li >= loc_boxes_.size() || !objects_) return;
  const Box & b = loc_boxes_[li];
  const double ex = b.x, ey = b.y;
  for (const auto & o : objects_->objects) {
    if (o.class_name != "location_box" || detail(o, "tag") != str(loc_tags_[li])) continue;
    const auto a = to_arena(o.pose.pose.position.x, o.pose.pose.position.y);
    corr_[li] = tag_correction(ex, ey, a.x, a.y, corr_max_);
    RCLCPP_INFO(get_logger(), "L%zu 상자 보정 (%+.1f, %+.1f) mm", li + 1, corr_[li].first * 1000, corr_[li].second * 1000);
    return;
  }
}

std::pair<double, double> MissionManager::cell_target(size_t li, int cell) const {
  const auto [dx, dy] = cell_offset(cell, grid_step_);
  const auto [cx, cy] = li < corr_.size() ? corr_[li] : std::pair<double, double>{0.0, 0.0};
  return {locations_[li].first + dx + cx, locations_[li].second + dy + cy};
}

int MissionManager::basket_slot_of(int tag) const {
  if (basket_ && tag >= 0) for (const auto & s : basket_->slots) if (s.filled && s.tag == tag) return s.slot;
  return -1;
}

bool MissionManager::basket_has_room() const {
  if (!basket_) return true;  // 아직 장부를 못 받음 — coordinator가 load next에서 slot_full로 알려 줌
  for (const auto & s : basket_->slots) if (!s.filled) return true;
  return false;
}

const perception::msg::TrackedObject * MissionManager::puck_near(double mx, double my) const {
  const TrackedObject * best = nullptr;
  double best_d = match_radius_;
  if (objects_) for (const auto & o : objects_->objects) {
    if (!is_puck(o)) continue;
    const double d = std::hypot(o.pose.pose.position.x - mx, o.pose.pose.position.y - my);
    if (d < best_d) { best_d = d; best = &o; }
  }
  return best;
}

// ---------- 집기 · 세우기
void MissionManager::grab(int tag, uint32_t id, Done done) {
  if (!lying_.count(tag)) { run_retry({SubGoal().set__skill("pick").set__target(std::to_string(id))}, done); return; }
  const TrackedObject * o = find_puck(tag);
  if (!o && objects_) for (const auto & x : objects_->objects) if (x.id == id) o = &x;
  if (!o) { done(false, "puck_lost"); return; }
  if (!is_lying(*o)) {  // 그새 서 있음 (다른 각도에서 다시 봄) — 그냥 집기
    lying_.erase(tag);
    run_retry({SubGoal().set__skill("pick").set__target(std::to_string(o->id))}, done);
    return;
  }
  stand_up(o->id, o->pose.pose.position.x, o->pose.pose.position.y, 0, [this, tag, done](bool ok, const std::string & why) {
    if (ok) lying_.erase(tag);
    done(ok, why);
  });
}

void MissionManager::stand_up(uint32_t id, double mx, double my, int tries, Done done) {
  note("누운 퍽 세우기 (" + str(tries + 1) + "/" + str(retries_ + 1) + ")");
  run({SubGoal().set__skill("stand").set__target(std::to_string(id))}, [this, mx, my, tries, done](bool ok, const std::string & why) {
    if (!ok) { done(false, "stand_failed " + why); return; }
    // 손은 퍽 위 80mm로 비켜 있음 — 인지가 선 퍽을 다시 볼 시간
    timer_ = create_wall_timer(std::chrono::duration<double>(stand_check_s_), [this, mx, my, tries, done]() {
      timer_->cancel();
      if (!active_) return;
      const TrackedObject * o = puck_near(mx, my);
      if (o && is_lying(*o)) {  // 아직 누움 — 같은 퍽을 다시 세움
        if (tries >= retries_) { done(false, "stand_still_lying"); return; }
        stand_up(o->id, o->pose.pose.position.x, o->pose.pose.position.y, tries + 1, done);
        return;
      }
      // 섰음 (또는 손 그늘로 못 봄 — stand_pick 잡기 판정이 가림). 같은 손목으로 바로 아래로
      run({SubGoal().set__skill("stand_pick").set__target("last")}, done);
    });
  });
}

const perception::msg::TrackedObject * MissionManager::find_puck(int tag) const {
  const TrackedObject * best = nullptr;
  if (objects_) for (const auto & o : objects_->objects)
    if (is_puck(o) && detail(o, "tag") == str(tag) &&
        (!best || rclcpp::Time(o.last_seen, RCL_ROS_TIME) > rclcpp::Time(best->last_seen, RCL_ROS_TIME))) best = &o;
  return best;
}

// ---------- 좌표
Pose2 MissionManager::to_map(double x, double y, double yaw) const { return arena_to_map({x, y, yaw}, start_); }

Pose2 MissionManager::to_arena(double mx, double my, double myaw) const {
  return {start_.x + std::cos(start_.yaw) * mx - std::sin(start_.yaw) * my,
          start_.y + std::sin(start_.yaw) * mx + std::cos(start_.yaw) * my, start_.yaw + myaw};
}

std::string MissionManager::point(double ax, double ay, double z) const {
  const auto m = to_map(ax, ay);
  return fmt("%.4f,%.4f,%.4f", m.x, m.y, z);
}

bool MissionManager::robot_arena(Pose2 & p) const {
  try {
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const auto & r = t.transform.rotation;
    const double yaw = std::atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z));
    p = to_arena(t.transform.translation.x, t.transform.translation.y, yaw);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void MissionManager::write_log(const std::string & name, const std::string & json) const {
  if (run_dir_.empty()) return;
  std::ofstream(run_dir_ + "/" + name) << json;
}

// ---------- 사진 — 찍을 때만 구독해서 한 장 받고 끊음 (영상을 계속 받지 않게)
void MissionManager::snap(const std::string & name) {
  if (run_dir_.empty() || photo_topic_.empty()) return;
  char prefix[8];
  std::snprintf(prefix, sizeof(prefix), "%03d_", ++photo_n_);
  const std::string path = run_dir_ + "/photos/" + prefix + name + ".jpg";
  photo_sub_ = create_subscription<sensor_msgs::msg::Image>(photo_topic_, rclcpp::SensorDataQoS(),
    [this, path](sensor_msgs::msg::Image::ConstSharedPtr m) {
      if (!photo_sub_) return;
      const bool rgb = m->encoding == "rgb8", bgr = m->encoding == "bgr8";
      if (rgb || bgr) {
        cv::Mat img(static_cast<int>(m->height), static_cast<int>(m->width), CV_8UC3, const_cast<uint8_t *>(m->data.data()), m->step);
        cv::Mat out;
        if (rgb) cv::cvtColor(img, out, cv::COLOR_RGB2BGR); else out = img;
        cv::imwrite(path, out);
      }
      photo_sub_.reset();
    });
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
        "L" + str(static_cast<int>(li) + 1) + (g == '?' ? "" : " · " + std::string(1, g) + str(group_count(counts_, g))));
    const auto cells = g == '?' ? std::vector<int>{} : dice_cells(group_count(counts_, g));
    for (int c = 0; c < 9; ++c) {  // 3×3 칸 (태그 보정 포함) — 쓰는 칸은 그룹 색, 아니면 흐린 회색
      const auto [x, y] = cell_target(li, c);
      const bool on = std::find(cells.begin(), cells.end(), c) != cells.end();
      const float r = g == 'R' ? 0.9f : g == 'Y' ? 0.95f : 0.2f, gg = g == 'G' ? 0.75f : g == 'Y' ? 0.85f : 0.2f;
      add(Marker::CYLINDER, x, y, box_top_z_ + 0.03, 0.03, 0.03, 0.06,
          on ? r : 0.6f, on ? gg : 0.6f, on ? 0.2f : 0.6f, on ? 0.9f : 0.25f);
    }
  }
  markers_pub_->publish(arr);
}

}  // namespace plan
