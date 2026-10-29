// state_manager 구현 — 설명·선언은 include/manager/state_manager.hpp, main은 state_manager_main.cpp
#include "manager/state_manager.hpp"

#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <vector>

#include "manager/restart.hpp"

namespace manager {

using diagnostic_msgs::msg::DiagnosticStatus;

namespace {
constexpr const char * kMapFile = "nvblox.nvblx";  // 지도 폴더 = 이 파일이 있는 폴더 (nvblox — bringup MAP_FILE과 같게)

struct DeviceSpec {
  const char * name;
  const char * kind;
  std::vector<std::string> topics;
};

// 상태 판단의 기준이 되는 대표 장치들. 실제 세부 토픽 목록은 /state/topics가 전체 그래프를 따로 낸다.
const std::vector<DeviceSpec> DEVICE_SPECS = {
  {"pole_front_cam", "camera", {
    "/pole_front_cam/color/image_raw",
    "/pole_front_cam/color/camera_info",
    "/pole_front_cam/aligned_depth_to_color/image_raw",
    "/pole_front_cam/depth/color/points"}},
  {"gripper_cam", "camera", {
    "/gripper_cam/color/image_raw",
    "/gripper_cam/color/camera_info",
    "/gripper_cam/aligned_depth_to_color/image_raw",
    "/gripper_cam/depth/color/points"}},
  {"opencr", "controller", {"/joint_states", "/odom", "/imu"}},
  {"imu", "sensor", {"/imu"}},
};
}  // namespace

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
  // 연결된 장치·토픽 상태 — 장치가 없어도 state_manager는 계속 뜨고, 없는 장치는 offline로 보인다.
  devices_pub_ = create_publisher<std_msgs::msg::String>("/state/devices", latched);
  topics_pub_ = create_publisher<std_msgs::msg::String>("/state/topics", latched);
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",
    rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      joint_names_.clear();
      for (const auto & name : m->name) joint_names_.insert(name);
      last_joint_time_ = std::chrono::steady_clock::now();
    });
  // 카메라/IMU는 "publisher가 있다"가 아니라 실제 메시지가 최근에 왔는지로 본다.
  // 카메라 영상(Image)은 state_manager가 받지 않고, 같은 주기로 작은 camera_info만 받는다.
  // 신선도 확인용 — 과거 샘플을 쌓을 이유가 없어 최신 1개만 받는다.
  const auto heartbeat_qos = rclcpp::SensorDataQoS().keep_last(1);
  front_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/pole_front_cam/color/camera_info", heartbeat_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {
      last_seen_["pole_front_cam"] = std::chrono::steady_clock::now();
    });
  gripper_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/gripper_cam/color/camera_info", heartbeat_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {
      last_seen_["gripper_cam"] = std::chrono::steady_clock::now();
    });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", heartbeat_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr) {
      last_seen_["imu"] = std::chrono::steady_clock::now();
    });
  // 모터 Hardware Error — BeebotSystemHardware가 바뀔 때마다 (transient_local — 늦게 떠도 마지막 값)
  motor_sub_ = create_subscription<control::msg::MotorStatus>("/motor_status", rclcpp::QoS(1).reliable().transient_local(),
    [this](control::msg::MotorStatus::ConstSharedPtr m) {
      motor_error_.clear();
      for (size_t i = 0; i < m->joints.size() && i < m->error.size(); ++i) {
        if (m->error[i] != 0) motor_error_[m->joints[i]] = i < m->text.size() ? m->text[i] : std::to_string(m->error[i]);
      }
      publish_devices();  // 2초 주기를 기다리지 않고 바로
    });
  device_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
    publish_devices();
    publish_topics();
  });
  estop_pub_ = create_publisher<std_msgs::msg::Bool>("/state/emergency", latched);
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
  publish_devices();
  publish_topics();
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
  if (mode != "MANUAL" && mode != "AUTO" && mode != "EXPLORE" && mode != "TEACH") {
    RCLCPP_WARN(get_logger(), "조작 모드 요청 '%s' 모름 (MANUAL | AUTO | EXPLORE | TEACH)", mode.c_str());
    return;
  }
  if (mode == "TEACH" && map_mode_ == "MAPPING") {  // 지도 만드는 중엔 팔이 스캔 자세에 고정 (gripper_cam = 두 번째 지도 카메라)
    RCLCPP_WARN(get_logger(), "조작 모드 요청 TEACH 거절: 지도 만드는 중(MAPPING) — 팔 스캔 자세 고정");
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
  // cuVSLAM·nvblox는 실행 중 모드 전환이 없음 → launch 전체를 인자만 바꿔 다시 띄움 (request_restart "all" — 새 프로세스가 새 상태를 발행)
  //   MAPPING → map:=slam (새 지도 폴더 — nvblox는 이어 붙이지 않음) · LOCALIZATION → map:=saved map_name:=<지금 폴더>
  map_error_.clear();
  if (mode != "MAPPING" && mode != "LOCALIZATION") {
    map_error_ = "모르는 값 '" + mode + "' (MAPPING | LOCALIZATION)";
  } else if (map_mode_ == "COURSE") {
    map_error_ = "COURSE(정지 지도)는 바꿀 수 없음 — faker map:=slam · saved 로 띄울 것";
  } else if (mode == map_mode_) {
    return;
  } else if (mode == "LOCALIZATION" && map_name_.empty()) {
    map_error_ = "지금 지도 폴더가 없음 — /cmd/map_name으로 고를 것";
  } else {
    if (operation_ == "EXPLORE") {
      explore_note_ = "지도 모드를 바꿔 탐사 멈춤 (다시 띄움)";
      set_operation_mode("AUTO");
    }
    request_restart(mode == "MAPPING" ? "all map:=slam" : "all map:=saved map_name:=" + map_name_);
  }
  if (!map_error_.empty()) RCLCPP_WARN(get_logger(), "지도 모드 요청 %s: %s", mode.c_str(), map_error_.c_str());
}

void StateManager::publish_map_list() {
  // 가진 지도 = map_dir 안에서 nvblox.nvblx가 있는 폴더 (이름순, 줄바꿈으로 구분) — 바뀔 때만 발행
  std::vector<std::string> names;
  std::error_code ec;
  if (!map_dir_.empty()) {
    for (const auto & e : std::filesystem::directory_iterator(map_dir_, ec)) {
      if (e.is_directory(ec) && std::filesystem::exists(e.path() / kMapFile, ec)) names.push_back(e.path().filename().string());
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
  // 고른 지도로 위치추정 — launch 전체를 map:=saved map_name:=<이름>으로 다시 띄움 (nvblox_map이 시작할 때 그 지도를 불러옴)
  map_error_.clear();
  const auto file = std::filesystem::path(map_dir_) / name / kMapFile;
  if (map_dir_.empty()) {
    map_error_ = "map_dir 파라미터 없음 — 지도 폴더를 모름";
  } else if (map_mode_ == "COURSE") {
    map_error_ = "COURSE(정지 지도)에선 지도를 갈아끼울 수 없음 — faker map:=slam · saved";
  } else if (name.empty() || name.find('/') != std::string::npos || !std::filesystem::exists(file)) {
    map_error_ = "지도 없음: " + file.string();
  } else if (name == map_name_ && map_mode_ == "LOCALIZATION") {
    return;  // 이미 그 지도
  } else {
    if (operation_ == "EXPLORE") {
      explore_note_ = "지도를 바꿔 탐사 멈춤 (다시 띄움)";
      set_operation_mode("AUTO");
    }
    request_restart("all map:=saved map_name:=" + name);
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

bool StateManager::joints_fresh() const {
  if (last_joint_time_ == std::chrono::steady_clock::time_point{}) return false;
  return std::chrono::steady_clock::now() - last_joint_time_ < std::chrono::seconds(1);
}

bool StateManager::recent(const std::string & key) const {
  const auto it = last_seen_.find(key);
  if (it == last_seen_.end()) return false;
  return std::chrono::steady_clock::now() - it->second < std::chrono::seconds(1);
}

void StateManager::publish_devices() {
  const bool fresh = joints_fresh();
  std::vector<std::string> names;
  std::map<std::string, std::string> statuses;
  auto add = [&names, &statuses](const std::string & name, const std::string & kind, bool active,
                                 const std::vector<std::string> & details) {
    std::ostringstream d;
    d << kind << '|' << (active ? "active" : "disconnected") << '|';
    for (size_t i = 0; i < details.size(); ++i) d << (i ? "," : "") << details[i];
    names.push_back(name);
    statuses[name] = d.str();
  };

  for (const auto & spec : DEVICE_SPECS) {
    const std::string name(spec.name);
    bool active = false;
    if (name == "pole_front_cam" || name == "gripper_cam") {
      active = recent(name);
    } else if (name == "opencr") {
      active = fresh;  // joint_states가 실제로 흐르는지로 판단
    } else if (name == "imu") {
      active = recent("imu");
    }
    add(name, spec.kind, active, spec.topics);
  }

  auto joint_active = [&](const std::vector<std::string> & required) {
    if (!fresh) return false;
    for (const auto & name : required) {
      if (joint_names_.count(name) == 0) return false;
    }
    return true;
  };
  auto add_joints = [&](const std::string & name, const std::vector<std::string> & joints) {
    std::vector<std::string> details = joints;
    bool error = false;
    for (const auto & j : joints) {
      const auto it = motor_error_.find(j);
      if (it == motor_error_.end()) continue;
      error = true;
      details.push_back(j + ":" + it->second);
    }
    add(name, "joint", joint_active(joints), details);
    if (error && joint_active(joints)) {  // 연결은 됐는데 모터가 에러 — active 대신 error
      auto & st = statuses[name];
      st.replace(st.find("|active|"), 8, "|error|");
    }
  };
  add_joints("wheels", {"wheel_left_joint", "wheel_right_joint"});
  add_joints("arm", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  add_joints("gripper", {"gripper_left_joint"});

  // 1) 이름 목록 — std_msgs/String 하나에 쉼표로 구분해 넣는다.
  std::string list;
  for (size_t i = 0; i < names.size(); ++i) list += (i ? "," : "") + names[i];
  if (!devices_published_ || list != devices_) {
    publish(devices_pub_, list);
    devices_ = list;
    devices_published_ = true;
  }

  // 2) 장치별 상태 상세 — 활성 여부·세부 토픽/관절
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  for (const auto & name : names) {
    auto & pub = device_pubs_[name];
    if (!pub) {
      pub = create_publisher<std_msgs::msg::String>("/state/devices/" + name, qos);
    }
    const auto & status = statuses[name];
    if (device_status_[name] == status) continue;
    std_msgs::msg::String msg;
    msg.data = status;
    pub->publish(msg);
    device_status_[name] = status;
  }
}

void StateManager::publish_topics() {
  const auto topics = get_topic_names_and_types();
  std::ostringstream out;
  for (const auto & [topic, types] : topics) {
    out << topic << '|';
    for (size_t i = 0; i < types.size(); ++i) out << (i ? "," : "") << types[i];
    out << '|' << count_publishers(topic) << '\n';
  }
  const std::string value = out.str();
  if (topics_published_ && value == topics_) return;
  topics_ = value;
  topics_published_ = true;
  publish(topics_pub_, topics_);
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
