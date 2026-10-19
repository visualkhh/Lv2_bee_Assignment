// state_manager 구현 — 설명·선언은 include/manager/state_manager.hpp, main은 state_manager_main.cpp
#include "manager/state_manager.hpp"

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <vector>

#include "manager/restart.hpp"

namespace manager {

using diagnostic_msgs::msg::DiagnosticStatus;

StateManager::StateManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("state_manager", options) {
  map_mode_ = declare_parameter<std::string>("map_mode", "LOCALIZATION");
  map_name_ = declare_parameter<std::string>("map_name", "");
  map_dir_ = declare_parameter<std::string>("map_dir", "");
  restart_prepare_ = declare_parameter<std::string>("restart_prepare", "");
  restart_log_ = declare_parameter<std::string>("restart_log", "/tmp/bringup.log");
  restart_marker_ = declare_parameter<std::string>("restart_marker", "/tmp/beebot_restart.pending");
  restart_wait_s_ = declare_parameter<double>("restart_wait_s", 25.0);
  restart_registry_ = declare_parameter<std::string>("restart_registry", "/tmp/beebot_restarted_nodes");
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  operation_pub_ = create_publisher<std_msgs::msg::String>("/state/operation_mode", latched);
  map_pub_ = create_publisher<std_msgs::msg::String>("/state/map_mode", latched);
  map_name_pub_ = create_publisher<std_msgs::msg::String>("/state/map_name", latched);
  map_list_pub_ = create_publisher<std_msgs::msg::String>("/state/map_list", latched);
  restart_pub_ = create_publisher<std_msgs::msg::String>("/state/restart", latched);
  restartable_pub_ = create_publisher<std_msgs::msg::String>("/state/restartable", latched);
  restartable_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { publish_restartable(); });
  map_list_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { publish_map_list(); });
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/state/emergency", latched);
  mapping_cli_ = create_client<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_mapping");
  localization_cli_ = create_client<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_localization");
  load_db_cli_ = create_client<rtabmap_msgs::srv::LoadDatabase>("/rtabmap/rtabmap/load_database");
  op_req_sub_ = create_subscription<std_msgs::msg::String>("/cmd/operation_mode", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { request_operation(m->data); });
  estop_sub_ = create_subscription<std_msgs::msg::Bool>("/cmd/e_stop", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Bool::ConstSharedPtr m) { set_estop(m->data); });
  map_req_sub_ = create_subscription<std_msgs::msg::String>("/cmd/map_mode", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { request_map_mode(m->data); });
  explore_sub_ = create_subscription<std_msgs::msg::String>("/explore/result", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) {  // explorer: DONE … | FAILED … — 탐사가 끝나면 AUTO로
      explore_note_ = m->data;
      if (operation_ == "EXPLORE") set_operation_mode("AUTO");
    });
  map_name_sub_ = create_subscription<std_msgs::msg::String>("/cmd/map_name", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { request_map_name(m->data); });
  restart_sub_ = create_subscription<std_msgs::msg::String>("/cmd/restart", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { request_restart(m->data); });
  publish(operation_pub_, operation_);
  publish(map_pub_, map_mode_);
  publish(map_name_pub_, map_name_);
  publish_map_list();
  restart_note_ = "시작 (pid " + std::to_string(getpid()) + ")";
  publish(restart_pub_, restart_note_);
  publish_restartable();
  { std_msgs::msg::Bool m; m.data = estop_; estop_pub_->publish(m); }
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (!map_error_.empty()) s.summary(DiagnosticStatus::WARN, "지도 모드 전환 실패 — " + map_error_);
    else s.summary(DiagnosticStatus::OK, operation_ + " · " + map_mode_ + (map_name_.empty() ? "" : " · " + map_name_));
    s.add("조작 모드", operation_);
    s.add("지도 모드", map_mode_);
    s.add("지도 폴더", map_name_.empty() ? "-" : map_name_);
    s.add("비상정지", estop_ ? "ON" : "-");
    s.add("탐사", explore_note_.empty() ? "-" : explore_note_);
    s.add("지도 모드 전환 중", map_pending_.empty() ? "-" : map_pending_);
    s.add("재시작", restart_note_);
  });
  RCLCPP_INFO(get_logger(), "조작 %s · 지도 %s %s", operation_.c_str(), map_mode_.c_str(), map_name_.c_str());
}

void StateManager::publish(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, const std::string & value) {
  std_msgs::msg::String m;
  m.data = value;
  pub->publish(m);
}

void StateManager::set_estop(bool engaged) {
  if (engaged == estop_) return;
  estop_ = engaged;
  RCLCPP_WARN(get_logger(), "비상정지 %s", estop_ ? "ON" : "OFF");
  if (estop_ && operation_ == "EXPLORE") {
    explore_note_ = "비상정지로 탐사 멈춤";
    set_operation_mode("AUTO");
  }
  std_msgs::msg::Bool m;
  m.data = estop_;
  estop_pub_->publish(m);
}

void StateManager::request_operation(const std::string & mode) {
  if (mode != "MANUAL" && mode != "AUTO" && mode != "EXPLORE") {
    RCLCPP_WARN(get_logger(), "조작 모드 요청 '%s' 모름 (MANUAL | AUTO | EXPLORE)", mode.c_str());
    return;
  }
  if (mode == "EXPLORE" && (map_mode_ != "MAPPING" || estop_)) {  // 탐사 = 매핑하며 스스로 돌아다님
    explore_note_ = estop_ ? "비상정지 중엔 탐사 못 함" : "지도 모드가 MAPPING일 때만 탐사 (지금 " + map_mode_ + ")";
    RCLCPP_WARN(get_logger(), "조작 모드 요청 EXPLORE 거절: %s", explore_note_.c_str());
    return;
  }
  set_operation_mode(mode);
}

void StateManager::set_operation_mode(const std::string & mode) {
  if (mode == operation_) return;
  if (mode == "EXPLORE") explore_note_ = "탐사 중";
  operation_ = mode;
  RCLCPP_WARN(get_logger(), "조작 모드 → %s", operation_.c_str());
  publish(operation_pub_, operation_);
}

void StateManager::request_map_mode(const std::string & mode) {
  map_error_.clear();
  if (mode != "MAPPING" && mode != "LOCALIZATION") {
    map_error_ = "모르는 값 '" + mode + "' (MAPPING | LOCALIZATION)";
  } else if (map_mode_ == "COURSE") {
    map_error_ = "COURSE(정지 지도)는 바꿀 수 없음 — faker map:=slam · saved 로 RTAB-Map을 띄울 것";
  } else if (!map_pending_.empty()) {
    map_error_ = map_pending_ + " 전환 중";
  } else if (mode != map_mode_) {
    auto cli = mode == "MAPPING" ? mapping_cli_ : localization_cli_;
    if (!cli->wait_for_service(std::chrono::seconds(0))) {
      map_error_ = std::string(cli->get_service_name()) + " 없음 (RTAB-Map 꺼짐?)";
    } else {
      map_pending_ = mode;
      cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
          map_mode_ = map_pending_;  // RTAB-Map이 응답한 뒤에만 바꾼다
          map_pending_.clear();
          RCLCPP_WARN(get_logger(), "지도 모드 → %s", map_mode_.c_str());
          publish(map_pub_, map_mode_);
          if (map_mode_ != "MAPPING" && operation_ == "EXPLORE") {
            explore_note_ = "MAPPING이 끝나 탐사 멈춤";
            set_operation_mode("AUTO");
          }
        });
    }
  }
  if (!map_error_.empty()) RCLCPP_WARN(get_logger(), "지도 모드 요청 %s: %s", mode.c_str(), map_error_.c_str());
}

void StateManager::publish_map_list() {
  // 가진 지도 = map_dir 안에서 rtabmap.db가 있는 폴더 (이름순, 줄바꿈으로 구분) — 바뀔 때만 발행
  std::vector<std::string> names;
  std::error_code ec;
  if (!map_dir_.empty()) {
    for (const auto & e : std::filesystem::directory_iterator(map_dir_, ec)) {
      if (e.is_directory(ec) && std::filesystem::exists(e.path() / "rtabmap.db", ec)) names.push_back(e.path().filename().string());
    }
  }
  std::sort(names.begin(), names.end());
  std::string list;
  for (const auto & n : names) list += (list.empty() ? "" : "\n") + n;
  if (map_list_published_ && list == map_list_) return;
  map_list_ = list;
  map_list_published_ = true;
  publish(map_list_pub_, map_list_);
}

void StateManager::request_map_name(const std::string & name) {
  map_error_.clear();
  const auto db = std::filesystem::path(map_dir_) / name / "rtabmap.db";
  if (map_dir_.empty()) {
    map_error_ = "map_dir 파라미터 없음 — 지도 폴더를 모름";
  } else if (map_mode_ == "COURSE") {
    map_error_ = "COURSE(정지 지도)에선 지도를 갈아끼울 수 없음 — faker map:=slam · saved";
  } else if (!map_pending_.empty()) {
    map_error_ = map_pending_ + " 전환 중";
  } else if (name.empty() || name.find('/') != std::string::npos || !std::filesystem::exists(db)) {
    map_error_ = "지도 없음: " + db.string();
  } else if (name == map_name_) {
    return;  // 이미 그 지도
  } else if (!load_db_cli_->wait_for_service(std::chrono::seconds(0)) ||
    !localization_cli_->wait_for_service(std::chrono::seconds(0)))
  {
    map_error_ = std::string(load_db_cli_->get_service_name()) + " 없음 (RTAB-Map 꺼짐?)";
  } else {
    // 1) 그 DB를 연다 (clear=false — 지도 그대로) 2) 위치추정 모드 — 고른 지도는 고정해서 쓰는 것
    //    두 응답이 다 온 뒤에만 /state/map_name · /state/map_mode 를 바꾼다 (발행 값 = 진짜 상태)
    map_pending_ = "지도 " + name;
    auto req = std::make_shared<rtabmap_msgs::srv::LoadDatabase::Request>();
    req->database_path = db.string();
    req->clear = false;
    load_db_cli_->async_send_request(req, [this, name](rclcpp::Client<rtabmap_msgs::srv::LoadDatabase>::SharedFuture) {
      localization_cli_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>(),
        [this, name](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture) {
          map_name_ = name;
          map_mode_ = "LOCALIZATION";
          if (operation_ == "EXPLORE") {
            explore_note_ = "지도를 바꿔 탐사 멈춤";
            set_operation_mode("AUTO");
          }
          map_pending_.clear();
          RCLCPP_WARN(get_logger(), "지도 → %s (위치추정)", map_name_.c_str());
          publish(map_name_pub_, map_name_);
          publish(map_pub_, map_mode_);
        });
    });
  }
  if (!map_error_.empty()) RCLCPP_WARN(get_logger(), "지도 바꾸기 %s: %s", name.c_str(), map_error_.c_str());
}

void StateManager::publish_restartable() {
  // node로 재시작할 수 있는 노드 = 이 launch 아래 + node로 따로 다시 띄운 것 중 ROS 노드 프로세스 — "이름 pid" 줄들, 바뀔 때만
  const auto procs = restart::list_procs();
  std::string list;
  for (const auto & [name, pid] : restart::restartable(procs, restart::launch_root(procs, getpid()),
         restart::registered(restart_registry_, procs)))
  {
    list += (list.empty() ? "" : "\n") + name + " " + std::to_string(pid);
  }
  if (restartable_published_ && list == restartable_) return;
  restartable_ = list;
  restartable_published_ = true;
  publish(restartable_pub_, restartable_);
}

void StateManager::request_restart(const std::string & text) {
  namespace rs = restart;
  const auto req = rs::parse(text);
  std::string note;
  rs::Plan plan;
  std::error_code ec;
  const auto procs = rs::list_procs();
  const int root = rs::launch_root(procs, getpid());
  if (req.kind == rs::Request::INVALID) {
    note = "거절: " + req.error;
  } else if (!restart_marker_.empty() && std::filesystem::exists(restart_marker_, ec)) {
    note = "거절: 재시작이 이미 도는 중 (" + restart_marker_ + ")";
  } else if (root == 0) {
    note = "거절: 이 노드를 띄운 ros2 launch를 못 찾음 (ros2 run으로 띄웠나?)";
  } else if (req.kind == rs::Request::ALL) {
    std::string exe;
    if (!rs::read_proc(root, plan.argv, plan.env, plan.cwd, exe)) {
      note = "거절: launch(pid " + std::to_string(root) + ") 명령줄을 못 읽음";
    } else {
      plan.argv = rs::merge_launch_args(plan.argv, req.args);
      plan.stop = {root};
      plan.kill = rs::descendants(procs, root);
      plan.kill.push_back(root);
      for (int p : rs::registered(restart_registry_, procs)) {  // node로 따로 다시 띄운 것도 (launch 자식이 아님)
        plan.stop.push_back(p);
        plan.kill.push_back(p);
      }
      plan.clear = restart_registry_;
      plan.wait_s = restart_wait_s_;
      plan.clean_shm = true;  // 강제 종료된 노드의 Fast DDS 공유메모리 통로가 남으면 보내는 쪽이 막힐 수 있음 (ros-kill과 같게)
      plan.prepare = restart_prepare_;
      plan.marker = restart_marker_;
      plan.log = restart_log_;
      note = "all: 재시작 중 — launch pid " + std::to_string(root) +
        (req.args.empty() ? "" : " · 바꾼 인자 " + text.substr(text.find("all") + 3)) +
        (restart_prepare_.empty() ? "" : " · 준비 " + restart_prepare_) + " · 출력 " + restart_log_;
    }
  } else {
    const auto pids = rs::find_node(procs, root, req.node, rs::registered(restart_registry_, procs));
    std::string exe;
    if (pids.size() != 1) {
      note = "거절: node " + req.node + " — " +
        (pids.empty() ? "이 launch 아래에 그 이름의 노드 프로세스 없음" : "프로세스가 " + std::to_string(pids.size()) + "개 (이름이 겹침)");
    } else if (!rs::read_proc(pids[0], plan.argv, plan.env, plan.cwd, exe)) {
      note = "거절: node " + req.node + " (pid " + std::to_string(pids[0]) + ") 명령줄을 못 읽음";
    } else {
      if (!plan.argv[0].empty() && plan.argv[0][0] != '/' && !exe.empty()) plan.argv[0] = exe;  // PATH 없이도 되게 절대 경로
      plan.stop = {pids[0]};
      plan.kill = rs::descendants(procs, pids[0]);
      plan.kill.push_back(pids[0]);
      plan.wait_s = 10.0;
      plan.log = "/tmp/restart_" + req.node + ".log";
      note = "node " + req.node + ": 재시작 (pid " + std::to_string(pids[0]) + ", 같은 명령줄·파라미터 · 출력 " + plan.log + ")" +
        (exe.find("nav2_") != std::string::npos ? " — Nav2 수명주기 노드: lifecycle_manager가 다시 활성화해야 함" : "");
    }
  }
  std::string err;
  int new_pid = 0;
  if (note.rfind("거절", 0) != 0) {
    if (!rs::spawn(plan, err, &new_pid)) note = "거절: 도우미 실행 실패 — " + err;
    else if (req.kind == rs::Request::NODE) rs::record(restart_registry_, new_pid, plan.argv);  // 다시 뜬 노드 = 같은 pid · 같은 명령줄
  }
  restart_note_ = note;
  RCLCPP_WARN(get_logger(), "재시작 요청 '%s' → %s", text.c_str(), note.c_str());
  publish(restart_pub_, note);
}

}  // namespace manager
