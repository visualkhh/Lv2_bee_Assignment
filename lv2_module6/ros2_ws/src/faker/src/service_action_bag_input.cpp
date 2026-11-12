// service_action_bag_input 구현 — 설명·선언은 include/faker/service_action_bag_input.hpp
#include "faker/service_action_bag_input.hpp"

#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace faker {

namespace {

const char * kServices[] = {"score_grasps", "clear_octomap", "apply_planning_scene",
  "diff_drive_controller/set_odometry", "reset", "fake_opencr/set_parameters",
  "map_saver/save_map", "controller_server/FollowPath/toggle", "collision_monitor/toggle", "set_parameters",
  "controller_server/set_parameters"};
const char * kActions[] = {"navigate_to_pose", "navigate_through_poses", "execute_task_solution", "move_action"};

bool get_double(const rapidjson::Value & v, const char * key, double & out) {
  if (!v.HasMember(key) || !v[key].IsNumber()) return false;
  out = v[key].GetDouble();
  return true;
}

std::string pose_json(const geometry_msgs::msg::PoseStamped & p) {
  char b[512];
  snprintf(b, sizeof(b),
      "{\"frame_id\": \"%s\", \"position\": [%.3f, %.3f, %.3f], "
      "\"orientation\": [%.3f, %.3f, %.3f, %.3f]}",
      p.header.frame_id.c_str(), p.pose.position.x, p.pose.position.y, p.pose.position.z,
      p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
  return b;
}

std::string escape_json(const std::string & s) {
  std::string o;
  for (char c : s) {
    if (c == '"') o += "\\\"";
    else if (c == '\\') o += "\\\\";
    else if (c == '\n') o += "\\n";
    else o += c;
  }
  return o;
}

}  // namespace

ServiceActionBagInput::ServiceActionBagInput()
: rclcpp::Node("service_action_bag_input") {
  debug_dir_ = declare_parameter<std::string>("debug_dir", "/debug");
  const double period = declare_parameter<double>("period_s", 0.2);
  timeout_s_ = declare_parameter<double>("timeout_s", 20.0);
  for (auto s : kServices) svc_[s] = SvcState{};
  for (auto a : kActions) act_[a] = ActState{};
  score_cli_ = create_client<ml::srv::ScoreGrasps>("/score_grasps");
  clear_cli_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
  reset_cli_ = create_client<std_srvs::srv::Empty>("/reset");  // EKF 리셋
  scene_cli_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  odom_cli_ = create_client<control_msgs::srv::SetOdometry>("/diff_drive_controller/set_odometry");
  rl_toggle_cli_ = create_client<std_srvs::srv::SetBool>("/controller_server/FollowPath/toggle");
  opencr_params_cli_ = create_client<rcl_interfaces::srv::SetParameters>("/fake_opencr/set_parameters");
  save_map_cli_ = create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
  collision_toggle_cli_ = create_client<nav2_msgs::srv::Toggle>("/collision_monitor/toggle");
  nav_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
  through_cli_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "/navigate_through_poses");
  mtc_cli_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      this, "/execute_task_solution");
  move_cli_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(this, "/move_action");
  map_mode_sub_ = create_subscription<std_msgs::msg::String>("/state/map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { map_mode_ = m->data; });
  RCLCPP_INFO(get_logger(), "시작 | 서비스·액션: input/service|action · bag: input/topic/<토픽>/bag → output/topic/<토픽>/bags/");
  timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() { poll(); });
  dirs_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { make_dirs(); });  // 떠 있는 서비스·액션마다 input 폴더
}

// ---------- 공용 파일 helpers

fs::path ServiceActionBagInput::svc_dir(const std::string & root, const std::string & name) {
  std::string rel = name;
  if (!rel.empty() && rel[0] == '/') rel.erase(0, 1);
  return fs::path(root) / rel;
}

void ServiceActionBagInput::write_json(const fs::path & out, const std::string & text) {
  std::error_code ec;
  fs::create_directories(out.parent_path(), ec);
  ::chmod(out.parent_path().c_str(), 0777);  // 컨테이너(root)가 만들어도 호스트가 rename으로 교체 가능
  const fs::path tmp = out.parent_path() / (".tmp-" + out.filename().string());
  { std::ofstream f(tmp); f << text; }
  fs::rename(tmp, out, ec);
}

void ServiceActionBagInput::reply(const std::string & kind, const std::string & name, const std::string & what,
                               const std::string & text) {
  // 응답 → debug/output/<kind>/<이름>/ (service: message · action: accept|feedback|result) + 요청 쪽 history에 쌓음 (feedback은 잦아서 뺌)
  const fs::path out = svc_dir(debug_dir_ + "/output/" + kind, name) / (kind == "service" ? std::string("message") : what);
  write_json(out, text);
  if (what != "feedback") history(kind, name, "<<" + what, text);
}

void ServiceActionBagInput::history(const std::string & kind, const std::string & name, const std::string & tag,
                                 const std::string & text) {
  // debug/input/<kind>/<이름>/history 에 덧붙임: '>>request:<시각>>>' · '<<response:<시각><<' 다음 줄에 JSON
  const fs::path file = svc_dir(debug_dir_ + "/input/" + kind, name) / "history";
  std::error_code ec;
  fs::create_directories(file.parent_path(), ec);
  const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char when[32];
  std::strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
  const std::string end = tag.substr(0, 2);  // '>>' 또는 '<<'
  std::ofstream f(file, std::ios::app);
  f << tag << ":" << when << end << "\n" << text << (text.empty() || text.back() != '\n' ? "\n" : "");
}

bool ServiceActionBagInput::changed(const fs::path & file, fs::file_time_type & last) {
  std::error_code ec;
  const auto mtime = fs::last_write_time(file, ec);
  if (ec) return false;
  if (mtime == last) return false;
  last = mtime;
  return true;
}

std::string ServiceActionBagInput::read_all(const fs::path & file) {
  std::ifstream f(file);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string ServiceActionBagInput::get_str(const rapidjson::Value & v, const char * key, const std::string & dflt) {
  return (v.HasMember(key) && v[key].IsString()) ? v[key].GetString() : dflt;
}

// ---------- 폴링

void ServiceActionBagInput::poll() {
  poll_bag();
  for (auto s : kServices) { poll_service(s); check_service(s); }
  for (auto a : kActions) { poll_goal(a); poll_cancel(a); check_action(a); }
  // 폴더는 있지만 타입을 등록 안 한 서비스·액션 — 요청이 오면 조용히 버리지 않고 '지원 안 함'으로 답함
  for (auto & [key, mtime] : unsupported_) {
    const auto slash = key.find(':');
    const std::string kind = key.substr(0, slash), name = key.substr(slash + 1);
    const fs::path in = svc_dir(debug_dir_ + "/input/" + kind, name) / (kind == "service" ? "message" : "goal");
    if (!changed(in, mtime)) continue;
    const std::string text = read_all(in);
    history(kind, name, kind == "service" ? ">>request" : ">>goal", text);
    rapidjson::Document d;
    d.Parse(text.c_str());
    const std::string id = d.IsObject() ? get_str(d, kind == "service" ? "request_id" : "goal_id") : "";
    const std::string err = "지원 안 함 — service_action_bag_input에 이 " + kind + " 타입 변환이 없음 (kServices·kActions에 추가)";
    if (kind == "service") {
      reply(kind, name, "response", "{\"request_id\": \"" + escape_json(id) + "\", \"ok\": false, \"error\": \"" + err + "\"}");
    } else {
      reply(kind, name, "accept", "{\"goal_id\": \"" + escape_json(id) + "\", \"accepted\": false, \"error\": \"" + err + "\"}");
    }
  }
}

void ServiceActionBagInput::make_dirs() {
  std::error_code ec;
  fs::create_directories(fs::path(debug_dir_) / "input" / "topic", ec);
  // ROS 그래프의 서비스·액션마다 debug/input/<service|action>/<이름>/ 를 만든다 (호스트가 어디에 요청을 쓸지 보이게).
  // 액션은 서비스 '<이름>/_action/send_goal'로 찾는다. 노드 전용 파라미터·타입 서비스 같은 것도 다 — 떠 있는 그대로
  const std::string suffix = "/_action/send_goal";
  const auto all_services = get_service_names_and_types();
  std::set<std::string> services, actions;
  for (const auto & [name, _] : all_services) {
    if (name.find("/_action/") != std::string::npos) {
      if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
        actions.insert(name.substr(0, name.size() - suffix.size()));
      }
      continue;
    }
    services.insert(name);
  }
  auto ensure = [this](const std::string & kind, const std::string & name, bool supported) {
    const fs::path dir = svc_dir(debug_dir_ + "/input/" + kind, name);
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
      fs::create_directories(dir, ec);
      for (fs::path p = dir; p.string().size() > (debug_dir_ + "/input").size(); p = p.parent_path()) {
        ::chmod(p.c_str(), 0777);  // 컨테이너(root)가 만들어도 호스트가 요청 파일을 쓸 수 있게
      }
    }
    const std::string key = kind + ":" + name.substr(name[0] == '/' ? 1 : 0);
    if (!supported && !unsupported_.count(key)) {  // 처음 본 미지원 — 지금 있는 요청 파일은 잔재로 보고 무시
      const fs::path in = dir / (kind == "service" ? "message" : "goal");
      unsupported_[key] = fs::exists(in, ec) ? fs::last_write_time(in, ec) : fs::file_time_type{};
    }
  };
  auto listed = [](const char * const * arr, size_t n, const std::string & name) {
    for (size_t i = 0; i < n; ++i) if (name == std::string("/") + arr[i]) return true;
    return false;
  };
  for (const auto & n : services) ensure("service", n, listed(kServices, std::size(kServices), n));
  for (const auto & n : actions) ensure("action", n, listed(kActions, std::size(kActions), n));

  // CLI 노드 메뉴의 서비스 목록 — 이름·타입·호출 지원 여부 + 노드별 매핑.
  std::string list = "{\"services\": [";
  bool first = true;
  for (const auto & [name, types] : all_services) {
    if (name.find("/_action/") != std::string::npos) continue;
    if (!first) list += ", ";
    first = false;
    list += "{\"name\": \"" + escape_json(name) + "\", \"types\": [";
    for (size_t i = 0; i < types.size(); ++i) {
      if (i) list += ", ";
      list += "\"" + escape_json(types[i]) + "\"";
    }
    list += "], \"callable\": ";
    list += listed(kServices, std::size(kServices), name) ? "true" : "false";
    list += "}";
  }
  list += "], \"nodes\": {";
  bool first_node = true;
  for (const auto & [node_name, ns] : get_node_graph_interface()->get_node_names_and_namespaces()) {
    if (node_name.empty() || node_name[0] == '_') continue;
    const std::string key = (ns == "/" || ns.empty()) ? "/" + node_name : ns + "/" + node_name;
    if (!first_node) list += ", ";
    first_node = false;
    list += "\"" + escape_json(key) + "\": [";
    try {
      bool first_svc = true;
      for (const auto & [svc_name, _] : get_service_names_and_types_by_node(node_name, ns)) {
        if (svc_name.find("/_action/") != std::string::npos) continue;
        if (!first_svc) list += ", ";
        first_svc = false;
        list += "\"" + escape_json(svc_name) + "\"";
      }
    } catch (const std::exception &) {
    }
    list += "]";
  }
  list += "}}";
  write_json(fs::path(debug_dir_) / "output" / "service" / "list.json", list);
}


namespace {
std::string bag_stamp() {
  const auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  char b[32];
  std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", std::localtime(&t));
  return b;
}
}  // namespace

void ServiceActionBagInput::write_bag_status(const std::string & topic, const BagState & st,
    const std::string & request_id, bool ok, const std::string & state, const std::string & error) {
  const fs::path out = fs::path(debug_dir_) / "output" / "topic" / topic.substr(1) / "bag_status";
  std::string text = "{\"request_id\": \"" + escape_json(request_id) + "\", \"ok\": " +
      (ok ? "true" : "false") + ", \"state\": \"" + escape_json(state) +
      "\", \"topic\": \"" + escape_json(topic) + "\", \"bag\": \"" + escape_json(st.path) +
      "\", \"pid\": " + std::to_string(st.pid) + ", \"topics\": [\"" + escape_json(topic) + "\"]";
  if (!error.empty()) text += ", \"error\": \"" + escape_json(error) + "\"";
  text += "}";
  write_json(out, text);
}

void ServiceActionBagInput::start_bag(const std::string & topic, BagState & st,
    const std::string & request_id) {
  if (st.pid > 0) {
    write_bag_status(topic, st, request_id, false, "recording", "이미 녹화 중");
    return;
  }
  const std::string stamp = bag_stamp();
  const fs::path bags = fs::path(debug_dir_) / "output" / "topic" / topic.substr(1) / "bags";
  std::error_code ec;
  fs::create_directories(bags, ec);
  const fs::path out = bags / stamp;
  const fs::path log = bags / (stamp + ".log");
  std::vector<std::string> args = {"ros2", "bag", "record", "-s", "mcap",
      "--disable-keyboard-controls", "-o", out.string(), "--topics", topic};
  const pid_t pid = ::fork();
  if (pid == 0) {
    ::setsid();
    const int fd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) { ::dup2(fd, STDOUT_FILENO); ::dup2(fd, STDERR_FILENO); ::close(fd); }
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (auto & a : args) argv.push_back(a.data());
    argv.push_back(nullptr);
    ::execvp("ros2", argv.data());
    ::_exit(127);
  }
  if (pid < 0) {
    write_bag_status(topic, st, request_id, false, "error", "fork 실패");
    return;
  }
  st.pid = pid;
  st.stop = false;
  std::error_code lec;
  st.lease = fs::exists(fs::path(debug_dir_) / "input" / "topic" / topic.substr(1) / "bag_lease", lec);
  st.path = out.string();
  st.log = log.string();
  st.request_id = request_id;
  write_bag_status(topic, st, request_id, true, "recording");
  RCLCPP_INFO(get_logger(), "bag 녹화 시작: %s → %s", topic.c_str(), out.c_str());
}

void ServiceActionBagInput::stop_bag(const std::string & topic, BagState & st, const std::string & request_id) {
  if (st.pid <= 0) {
    write_bag_status(topic, st, request_id, false, "idle", "녹화 중 아님");
    return;
  }
  if (!st.stop) {
    ::kill(-st.pid, SIGINT);
    st.stop = true;
    st.deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  }
  write_bag_status(topic, st, request_id, true, "stopping");
}

void ServiceActionBagInput::poll_bag() {
  const fs::path root = fs::path(debug_dir_) / "input" / "topic";
  std::error_code ec;
  if (fs::is_directory(root, ec)) {
    for (auto it = fs::recursive_directory_iterator(root, ec);
         !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
      if (!it->is_regular_file(ec) || it->path().filename() != "bag") continue;
      const auto rel = fs::relative(it->path().parent_path(), root, ec);
      if (ec || rel.empty()) continue;
      const std::string topic = "/" + rel.generic_string();
      auto & st = bag_[topic];
      if (!changed(it->path(), st.mtime)) continue;
      const std::string text = read_all(it->path());
      rapidjson::Document d;
      d.Parse(text.c_str());
      if (d.HasParseError() || !d.IsObject()) {
        write_bag_status(topic, st, "", false, "error", "JSON 형식 오류");
        continue;
      }
      const std::string id = get_str(d, "request_id");
      const std::string action = get_str(d, "action", "start");
      if (action == "stop") stop_bag(topic, st, id);
      else start_bag(topic, st, id);
    }
  }
  const auto now = std::chrono::steady_clock::now();
  for (auto & [topic, st] : bag_) {
    if (st.pid <= 0) continue;
    int status = 0;
    const pid_t r = ::waitpid(st.pid, &status, WNOHANG);
    if (r == st.pid) {
      st.pid = -1;
      st.stop = false;
      write_bag_status(topic, st, st.request_id, true, "stopped");
    } else if (st.stop && now > st.deadline) {
      ::kill(-st.pid, SIGKILL);
    } else if (st.lease) {
      const fs::path lease = fs::path(debug_dir_) / "input" / "topic" / topic.substr(1) / "bag_lease";
      std::error_code lec;
      if (!fs::exists(lease, lec)) {
        stop_bag(topic, st, "");
      } else {
        const auto mt = fs::last_write_time(lease, lec);
        if (!lec && fs::file_time_type::clock::now() - mt > std::chrono::seconds(10)) {
          stop_bag(topic, st, "");
        }
      }
    }
  }
}

void ServiceActionBagInput::poll_service(const std::string & name) {
  SvcState & st = svc_[name];
  const fs::path req = svc_dir(debug_dir_ + "/input/service", name) / "message";
  std::error_code ec;
  const bool exists = fs::exists(req, ec);
  // 시작 전 잔재 무시: 첫 관측은 기록만
  static std::map<std::string, bool> seen;
  if (!seen[name] && exists) {
    st.mtime = fs::last_write_time(req, ec);
    seen[name] = true;
    return;
  }
  seen[name] = true;
  if (!changed(req, st.mtime)) return;
  if (st.busy) {  // 처리 중 — 최신 요청 id만 에러로 알림
    rapidjson::Document d;
    d.Parse(read_all(req).c_str());
    const std::string id = d.IsObject() ? get_str(d, "request_id") : "";
    reply("service", name, "response",
        "{\"request_id\": \"" + escape_json(id) + "\", \"ok\": false, \"error\": \"busy\"}");
    return;
  }
  const std::string req_text = read_all(req);
  history("service", name, ">>request", req_text);
  rapidjson::Document d;
  d.Parse(req_text.c_str());
  if (d.HasParseError() || !d.IsObject() || !d.HasMember("data")) {
    reply("service", name, "response",
        "{\"request_id\": \"\", \"ok\": false, \"error\": \"JSON 형식 오류 — {\\\"request_id\\\", \\\"data\\\"}\"}");
    return;
  }
  st.request_id = get_str(d, "request_id");
  st.busy = true;
  st.timed_out = false;
  st.deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_s_));
  const rapidjson::Value & data = d["data"];
  std::string err;
  if (name == "score_grasps") {
    if (!score_cli_->wait_for_service(std::chrono::seconds(0))) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
      return;
    }
    auto ros_req = std::make_shared<ml::srv::ScoreGrasps::Request>();
    if (!make_score_request(data, *ros_req, err)) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"" + escape_json(err) + "\"}");
      st.busy = false;
      return;
    }
    score_fut_ = score_cli_->async_send_request(ros_req).future;
    RCLCPP_INFO(get_logger(), "/score_grasps 호출 id=%s", st.request_id.c_str());
  } else if (name == "clear_octomap") {
    if (!clear_cli_->wait_for_service(std::chrono::seconds(0))) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
      return;
    }
    clear_fut_ = clear_cli_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()).future;
  } else if (name == "reset") {
    if (!reset_cli_->wait_for_service(std::chrono::seconds(0))) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
      return;
    }
    reset_fut_ = reset_cli_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>()).future;
  } else if (name == "apply_planning_scene") {
    if (!scene_cli_->wait_for_service(std::chrono::seconds(0))) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
      return;
    }
    auto ros_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    if (data.IsObject() && data.HasMember("is_diff") && data["is_diff"].IsBool()) {
      ros_req->scene.is_diff = data["is_diff"].GetBool();
    }
    scene_fut_ = scene_cli_->async_send_request(ros_req).future;
  } else if (name == "diff_drive_controller/set_odometry") {
    if (!odom_cli_->wait_for_service(std::chrono::seconds(0))) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
      return;
    }
    auto ros_req = std::make_shared<control_msgs::srv::SetOdometry::Request>();
    if (data.IsObject()) {
      if (data.HasMember("x") && data["x"].IsNumber()) ros_req->x = data["x"].GetDouble();
      if (data.HasMember("y") && data["y"].IsNumber()) ros_req->y = data["y"].GetDouble();
      if (data.HasMember("yaw") && data["yaw"].IsNumber()) ros_req->yaw = data["yaw"].GetDouble();
    }
    odom_fut_ = odom_cli_->async_send_request(ros_req).future;
  } else {
    // call()로 보내는 서비스 — 응답 변환은 st.done (check_service)
    bool sent = false;
    if (name == "fake_opencr/set_parameters") {
      // data: {"parameters": [{"name": "slip", "value": {"type": 3, "double_value": 0.3}}, ...]} (rcl_interfaces 그대로)
      auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      if (data.IsObject() && data.HasMember("parameters") && data["parameters"].IsArray()) {
        for (const auto & p : data["parameters"].GetArray()) {
          if (!p.IsObject() || !p.HasMember("value") || !p["value"].IsObject()) continue;
          rcl_interfaces::msg::Parameter m;
          m.name = get_str(p, "name");
          const auto & v = p["value"];
          m.value.type = v.HasMember("type") && v["type"].IsInt() ? v["type"].GetInt() : 0;
          if (v.HasMember("bool_value") && v["bool_value"].IsBool()) m.value.bool_value = v["bool_value"].GetBool();
          if (v.HasMember("integer_value") && v["integer_value"].IsInt64()) m.value.integer_value = v["integer_value"].GetInt64();
          if (v.HasMember("double_value") && v["double_value"].IsNumber()) m.value.double_value = v["double_value"].GetDouble();
          m.value.string_value = get_str(v, "string_value");
          req->parameters.push_back(m);
        }
      }
      sent = call<rcl_interfaces::srv::SetParameters>(st, opencr_params_cli_, req, [](const auto & res) {
        std::string r = "{\"results\": [";
        for (size_t i = 0; i < res.results.size(); ++i) {
          r += std::string(i ? ", " : "") + "{\"successful\": " + (res.results[i].successful ? "true" : "false") +
               ", \"reason\": \"" + escape_json(res.results[i].reason) + "\"}";
        }
        return r + "]}";
      });
    } else if (name == "controller_server/FollowPath/toggle") {
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      if (data.IsObject() && data.HasMember("data") && data["data"].IsBool()) req->data = data["data"].GetBool();
      sent = call<std_srvs::srv::SetBool>(st, rl_toggle_cli_, req, [](const auto & res) {
        return std::string("{\"success\": ") + (res.success ? "true" : "false") +
               ", \"message\": \"" + escape_json(res.message) + "\"}";
      });
    } else if (name == "collision_monitor/toggle") {
      // data: {"enable": true|false} → collision_monitor 켜기/끄기 (충돌 감시)
      auto req = std::make_shared<nav2_msgs::srv::Toggle::Request>();
      req->enable = true;
      if (data.IsObject() && data.HasMember("enable") && data["enable"].IsBool()) req->enable = data["enable"].GetBool();
      sent = call<nav2_msgs::srv::Toggle>(st, collision_toggle_cli_, req, [](const auto & res) {
        return std::string("{\"success\": ") + (res.success ? "true" : "false") +
               ", \"message\": \"" + escape_json(res.message) + "\"}";
      });
    } else if (name == "set_parameters" || name == "controller_server/set_parameters") {
      // data: {"node": "/controller_server", "parameters": [{"name": ..., "value": {...}}]}
      // 노드 전용 이름(/controller_server/set_parameters)이면 data.node 없이도 그 노드로 보낸다.
      std::string node = get_str(data, "node");
      if (node.empty() && name == "controller_server/set_parameters") node = "/controller_server";
      if (node.empty()) {
        reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
            "\", \"ok\": false, \"error\": \"node 없음\"}");
        st.busy = false;
        return;
      }
      const std::string svc = (node[0] == '/' ? node : "/" + node) + "/set_parameters";
      auto & cli = set_param_clis_[svc];
      if (!cli) cli = create_client<rcl_interfaces::srv::SetParameters>(svc);
      auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
      if (data.HasMember("parameters") && data["parameters"].IsArray()) {
        for (const auto & p : data["parameters"].GetArray()) {
          if (!p.IsObject()) continue;
          rcl_interfaces::msg::Parameter m;
          m.name = get_str(p, "name");
          if (!p.HasMember("value") || !p["value"].IsObject()) { req->parameters.push_back(m); continue; }
          const auto & v = p["value"];
          m.value.type = v.HasMember("type") && v["type"].IsInt() ? v["type"].GetInt() : 0;
          if (v.HasMember("bool_value") && v["bool_value"].IsBool()) m.value.bool_value = v["bool_value"].GetBool();
          if (v.HasMember("integer_value") && v["integer_value"].IsInt64()) m.value.integer_value = v["integer_value"].GetInt64();
          if (v.HasMember("double_value") && v["double_value"].IsNumber()) m.value.double_value = v["double_value"].GetDouble();
          if (v.HasMember("string_value") && v["string_value"].IsString()) m.value.string_value = v["string_value"].GetString();
          auto ass = [&](const char * key, auto & out) {
            if (v.HasMember(key) && v[key].IsArray()) for (const auto & x : v[key].GetArray())
              if (x.IsBool()) out.push_back(x.GetBool());
          };
          if (v.HasMember("bool_array_value") && v["bool_array_value"].IsArray())
            for (const auto & x : v["bool_array_value"].GetArray()) if (x.IsBool()) m.value.bool_array_value.push_back(x.GetBool());
          if (v.HasMember("integer_array_value") && v["integer_array_value"].IsArray())
            for (const auto & x : v["integer_array_value"].GetArray()) if (x.IsInt64()) m.value.integer_array_value.push_back(x.GetInt64());
          if (v.HasMember("double_array_value") && v["double_array_value"].IsArray())
            for (const auto & x : v["double_array_value"].GetArray()) if (x.IsNumber()) m.value.double_array_value.push_back(x.GetDouble());
          if (v.HasMember("string_array_value") && v["string_array_value"].IsArray())
            for (const auto & x : v["string_array_value"].GetArray()) if (x.IsString()) m.value.string_array_value.push_back(x.GetString());
          (void)ass;
          req->parameters.push_back(m);
        }
      }
      sent = call<rcl_interfaces::srv::SetParameters>(st, cli, req, [](const auto & res) {
        std::string r = "{\"results\": [";
        for (size_t i = 0; i < res.results.size(); ++i) {
          r += std::string(i ? ", " : "") + "{\"successful\": " + (res.results[i].successful ? "true" : "false") +
               ", \"reason\": \"" + escape_json(res.results[i].reason) + "\"}";
        }
        return r + "]}";
      });
    } else if (name == "map_saver/save_map") {
      // data: {"map_url": "/ws/map/map", ...} — 비운 값은 map_saver 기본 (map_topic /map · pgm · trinary · 0.25/0.65)
      static const rapidjson::Value kEmpty(rapidjson::kObjectType);
      const rapidjson::Value & o = data.IsObject() ? data : kEmpty;
      auto req = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
      req->map_topic = get_str(o, "map_topic", "/map");
      req->map_url = get_str(o, "map_url");
      req->image_format = get_str(o, "image_format", "pgm");
      req->map_mode = get_str(o, "map_mode", "trinary");
      double free_t = 0.25, occ_t = 0.65;
      get_double(o, "free_thresh", free_t);
      get_double(o, "occupied_thresh", occ_t);
      req->free_thresh = static_cast<float>(free_t);
      req->occupied_thresh = static_cast<float>(occ_t);
      sent = call<nav2_msgs::srv::SaveMap>(st, save_map_cli_, req, [](const auto & res) {
        return std::string("{\"result\": ") + (res.result ? "true" : "false") + "}";
      });
    }
    if (!sent) {
      reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
          "\", \"ok\": false, \"error\": \"server 없음\"}");
      st.busy = false;
    }
  }
}

void ServiceActionBagInput::check_service(const std::string & name) {
  SvcState & st = svc_[name];
  if (!st.busy) return;
  bool done = false;
  std::string body;
  if (st.done) {
    std::string data;
    if (st.done(data)) {
      body = "{\"request_id\": \"" + escape_json(st.request_id) + "\", \"ok\": true, \"data\": " + data + "}";
      done = true;
    }
  } else if (name == "score_grasps") {
    if (score_fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      body = score_response_json(st.request_id, score_fut_.get());
      done = true;
    }
  } else if (name == "clear_octomap") {
    if (clear_fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      clear_fut_.get();
      body = "{\"request_id\": \"" + escape_json(st.request_id) + "\", \"ok\": true, \"data\": {}}";
      done = true;
    }
  } else if (name == "diff_drive_controller/set_odometry") {
    if (odom_fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto res = odom_fut_.get();
      body = "{\"request_id\": \"" + escape_json(st.request_id) + "\", \"ok\": true, \"data\": {\"success\": " +
          std::string(res->success ? "true" : "false") + ", \"message\": \"" + escape_json(res->message) + "\"}}";
      done = true;
    }
  } else if (name == "reset") {
    if (reset_fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      reset_fut_.get();
      body = "{\"request_id\": \"" + escape_json(st.request_id) + "\", \"ok\": true, \"data\": {}}";
      done = true;
    }
  } else {
    if (scene_fut_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto res = scene_fut_.get();
      body = "{\"request_id\": \"" + escape_json(st.request_id) + "\", \"ok\": true, \"data\": {\"success\": " +
          std::string(res->success ? "true" : "false") + "}}";
      done = true;
    }
  }
  if (done) {
    if (!st.timed_out) reply("service", name, "response", body);
    st.busy = false;
    st.done = nullptr;
    return;
  }
  if (std::chrono::steady_clock::now() > st.deadline && !st.timed_out) {
    st.timed_out = true;  // 에러 먼저 알리고 늦은 응답은 버림
    reply("service", name, "response", "{\"request_id\": \"" + escape_json(st.request_id) +
        "\", \"ok\": false, \"error\": \"timeout\"}");
  }
}

// ---------- 액션

void ServiceActionBagInput::poll_goal(const std::string & name) {
  ActState & st = act_[name];
  const fs::path goal = svc_dir(debug_dir_ + "/input/action", name) / "goal";
  std::error_code ec;
  static std::map<std::string, bool> seen;
  if (!seen[name] && fs::exists(goal, ec)) {
    st.goal_mtime = fs::last_write_time(goal, ec);
    seen[name] = true;
    return;
  }
  seen[name] = true;
  if (!changed(goal, st.goal_mtime)) return;
  const std::string goal_text = read_all(goal);
  history("action", name, ">>goal", goal_text);
  rapidjson::Document d;
  d.Parse(goal_text.c_str());
  if (d.HasParseError() || !d.IsObject() || !d.HasMember("data")) {
    reply("action", name, "accept", "{\"goal_id\": \"\", \"accepted\": false, \"error\": \"JSON 형식 오류\"}");
    return;
  }
  if (st.waiting_goal || st.active) {
    reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(get_str(d, "goal_id")) +
        "\", \"accepted\": false, \"error\": \"busy\"}");
    return;
  }
  st.goal_id = get_str(d, "goal_id");
  const rapidjson::Value & data = d["data"];
  std::string err;
  st.waiting_goal = true;
  st.deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(timeout_s_));
  if (name == "navigate_to_pose") {
    if (!nav_cli_->wait_for_action_server(std::chrono::seconds(0))) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"server 없음\"}");
      st.waiting_goal = false;
      return;
    }
    nav2_msgs::action::NavigateToPose::Goal g;
    if (!make_nav_goal(data, g, err)) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"" + escape_json(err) + "\"}");
      st.waiting_goal = false;
      return;
    }
    typename NavClient::SendGoalOptions opt;
    opt.feedback_callback = [this, gid = st.goal_id](
        NavClient::GoalHandle::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> fb) {
      ActState & s = act_["navigate_to_pose"];
      const auto now = std::chrono::steady_clock::now();
      if (now - s.last_fb < std::chrono::milliseconds(200)) return;
      s.last_fb = now;
      reply("action", "navigate_to_pose", "feedback", nav_feedback_json(gid, fb));
    };
    nav_goal_fut_ = nav_cli_->async_send_goal(g, opt);
  } else if (name == "navigate_through_poses") {
    if (!through_cli_->wait_for_action_server(std::chrono::seconds(0))) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"server 없음\"}");
      st.waiting_goal = false;
      return;
    }
    nav2_msgs::action::NavigateThroughPoses::Goal g;
    if (!make_nav_through_goal(data, g, err)) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"" + escape_json(err) + "\"}");
      st.waiting_goal = false;
      return;
    }
    typename NavThroughClient::SendGoalOptions opt;
    opt.feedback_callback = [this, gid = st.goal_id](
        NavThroughClient::GoalHandle::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> fb) {
      ActState & s = act_["navigate_through_poses"];
      const auto now = std::chrono::steady_clock::now();
      if (now - s.last_fb < std::chrono::milliseconds(200)) return;
      s.last_fb = now;
      reply("action", "navigate_through_poses", "feedback", nav_through_feedback_json(gid, fb));
    };
    through_goal_fut_ = through_cli_->async_send_goal(g, opt);
  } else if (name == "move_action") {
    if (map_mode_ == "MAPPING") {  // 시뮬레이터 팔 조작(손끝 목표·관절 슬라이더·퍽 집기)은 지도 만드는 동안 막음 — 팔은 스캔 자세 고정
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"지도 만드는 중(MAPPING) — 팔은 스캔 자세로 고정 (gripper_cam = 지도 카메라). [위치추정으로] 뒤에 조작\"}");
      st.waiting_goal = false;
      return;
    }
    if (!move_cli_->wait_for_action_server(std::chrono::seconds(0))) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"server 없음 (move_group)\"}");
      st.waiting_goal = false;
      return;
    }
    moveit_msgs::action::MoveGroup::Goal g;
    if (!make_move_goal(data, g, err)) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"" + escape_json(err) + "\"}");
      st.waiting_goal = false;
      return;
    }
    typename MoveClient::SendGoalOptions opt;
    opt.feedback_callback = [this, gid = st.goal_id](
        MoveClient::GoalHandle::SharedPtr, const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> fb) {
      reply("action", "move_action", "feedback",  // PLANNING · MONITOR · IDLE
          "{\"goal_id\": \"" + escape_json(gid) + "\", \"data\": {\"state\": \"" + escape_json(fb->state) + "\"}}");
    };
    move_goal_fut_ = move_cli_->async_send_goal(g, opt);
  } else {  // execute_task_solution
    if (!mtc_cli_->wait_for_action_server(std::chrono::seconds(0))) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"server 없음\"}");
      st.waiting_goal = false;
      return;
    }
    moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal g;
    if (!make_mtc_goal(data, g, err)) {
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"" + escape_json(err) + "\"}");
      st.waiting_goal = false;
      return;
    }
    typename MtcClient::SendGoalOptions opt;
    opt.feedback_callback = [this, gid = st.goal_id](
        MtcClient::GoalHandle::SharedPtr,
        const std::shared_ptr<const moveit_task_constructor_msgs::action::ExecuteTaskSolution::Feedback> fb) {
      ActState & s = act_["execute_task_solution"];
      const auto now = std::chrono::steady_clock::now();
      if (now - s.last_fb < std::chrono::milliseconds(200)) return;
      s.last_fb = now;
      char b[128];
      snprintf(b, sizeof(b), "{\"goal_id\": \"%s\", \"data\": {\"sub_id\": %u, \"sub_no\": %u}}",
          escape_json(gid).c_str(), fb->sub_id, fb->sub_no);
      reply("action", "execute_task_solution", "feedback", b);
    };
    mtc_goal_fut_ = mtc_cli_->async_send_goal(g, opt);
  }
  RCLCPP_INFO(get_logger(), "/%s goal id=%s", name.c_str(), st.goal_id.c_str());
}

void ServiceActionBagInput::poll_cancel(const std::string & name) {
  ActState & st = act_[name];
  const fs::path cancel = svc_dir(debug_dir_ + "/input/action", name) / "cancel";
  std::error_code ec;
  static std::map<std::string, bool> seen;
  if (!seen[name] && fs::exists(cancel, ec)) {
    st.cancel_mtime = fs::last_write_time(cancel, ec);
    seen[name] = true;
    return;
  }
  seen[name] = true;
  if (!changed(cancel, st.cancel_mtime)) return;
  const std::string cancel_text = read_all(cancel);
  history("action", name, ">>cancel", cancel_text);
  rapidjson::Document d;
  d.Parse(cancel_text.c_str());
  if (d.HasParseError() || !d.IsObject()) return;
  const std::string id = get_str(d, "goal_id");
  if (!st.active || id != st.goal_id) {
    reply("action", name, "result", "{\"goal_id\": \"" + escape_json(id) +
        "\", \"status\": \"unknown_goal\", \"ok\": false}");
    return;
  }
  if (name == "navigate_to_pose" && nav_handle_) {
    nav_cli_->async_cancel_goal(nav_handle_);
  } else if (name == "navigate_through_poses" && through_handle_) {
    through_cli_->async_cancel_goal(through_handle_);
  } else if (name == "execute_task_solution" && mtc_handle_) {
    mtc_cli_->async_cancel_goal(mtc_handle_);
  } else if (name == "move_action" && move_handle_) {
    move_cli_->async_cancel_goal(move_handle_);
  }
  RCLCPP_INFO(get_logger(), "/%s cancel id=%s", name.c_str(), id.c_str());
}

void ServiceActionBagInput::check_action(const std::string & name) {
  ActState & st = act_[name];
  const auto now0 = std::chrono::seconds(0);
  if (st.waiting_goal) {
    bool ready = false;
    if (name == "navigate_to_pose") ready = nav_goal_fut_.wait_for(now0) == std::future_status::ready;
    else if (name == "navigate_through_poses") ready = through_goal_fut_.wait_for(now0) == std::future_status::ready;
    else if (name == "move_action") ready = move_goal_fut_.wait_for(now0) == std::future_status::ready;
    else ready = mtc_goal_fut_.wait_for(now0) == std::future_status::ready;
    if (ready) {
      bool accepted = false;
      if (name == "navigate_to_pose") {
        nav_handle_ = nav_goal_fut_.get();
        accepted = (nav_handle_ != nullptr);
        if (accepted) nav_res_fut_ = nav_cli_->async_get_result(nav_handle_);
      } else if (name == "navigate_through_poses") {
        through_handle_ = through_goal_fut_.get();
        accepted = (through_handle_ != nullptr);
        if (accepted) through_res_fut_ = through_cli_->async_get_result(through_handle_);
      } else if (name == "move_action") {
        move_handle_ = move_goal_fut_.get();
        accepted = (move_handle_ != nullptr);
        if (accepted) move_res_fut_ = move_cli_->async_get_result(move_handle_);
      } else {
        mtc_handle_ = mtc_goal_fut_.get();
        accepted = (mtc_handle_ != nullptr);
        if (accepted) mtc_res_fut_ = mtc_cli_->async_get_result(mtc_handle_);
      }
      st.waiting_goal = false;
      st.active = accepted;
      if (accepted) {
        reply("action", name, "accept",
            "{\"goal_id\": \"" + escape_json(st.goal_id) + "\", \"accepted\": true}");
      } else {
        reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
            "\", \"accepted\": false, \"error\": \"rejected\"}");
        st.goal_id.clear();
      }
    } else if (std::chrono::steady_clock::now() > st.deadline) {
      st.waiting_goal = false;
      reply("action", name, "accept", "{\"goal_id\": \"" + escape_json(st.goal_id) +
          "\", \"accepted\": false, \"error\": \"timeout\"}");
      st.goal_id.clear();
    }
    return;
  }
  if (!st.active) return;
  bool ready = false;
  if (name == "navigate_to_pose") ready = nav_res_fut_.wait_for(now0) == std::future_status::ready;
  else if (name == "navigate_through_poses") ready = through_res_fut_.wait_for(now0) == std::future_status::ready;
  else if (name == "move_action") ready = move_res_fut_.wait_for(now0) == std::future_status::ready;
  else ready = mtc_res_fut_.wait_for(now0) == std::future_status::ready;
  if (!ready) return;
  st.active = false;
  if (name == "navigate_to_pose") {
    reply("action", name, "result", nav_result_json(st.goal_id, nav_res_fut_.get()));
    nav_handle_.reset();
  } else if (name == "navigate_through_poses") {
    reply("action", name, "result", nav_through_result_json(st.goal_id, through_res_fut_.get()));
    through_handle_.reset();
  } else if (name == "move_action") {
    reply("action", name, "result", move_result_json(st.goal_id, move_res_fut_.get()));
    move_handle_.reset();
  } else {
    reply("action", name, "result", mtc_result_json(st.goal_id, mtc_res_fut_.get()));
    mtc_handle_.reset();
  }
  st.goal_id.clear();
}

// ---------- 타입별 변환

bool ServiceActionBagInput::make_score_request(
    const rapidjson::Value & data, ml::srv::ScoreGrasps::Request & req, std::string & err) {
  if (!data.IsObject() || !data.HasMember("object_id") || !data["object_id"].IsUint()) {
    err = "data.object_id(uint) 필요";
    return false;
  }
  req.object_id = data["object_id"].GetUint();
  req.max_candidates =
      (data.HasMember("max_candidates") && data["max_candidates"].IsUint())
      ? static_cast<uint8_t>(data["max_candidates"].GetUint()) : 3;
  return true;
}

std::string ServiceActionBagInput::score_response_json(
    const std::string & id, ml::srv::ScoreGrasps::Response::SharedPtr res) {
  std::string grasps;
  for (size_t i = 0; i < res->grasps.size(); ++i) {
    if (i) grasps += ", ";
    grasps += pose_json(res->grasps[i]);
  }
  std::string scores;
  for (size_t i = 0; i < res->scores.size(); ++i) {
    if (i) scores += ", ";
    char b[32];
    snprintf(b, sizeof(b), "%.2f", res->scores[i]);
    scores += b;
  }
  return "{\"request_id\": \"" + escape_json(id) + "\", \"ok\": true, \"data\": {\"success\": " +
      std::string(res->success ? "true" : "false") + ", \"message\": \"" + escape_json(res->message) +
      "\", \"scores\": [" + scores + "], \"grasps\": [" + grasps + "]}}";
}

bool ServiceActionBagInput::make_nav_goal(
    const rapidjson::Value & data, nav2_msgs::action::NavigateToPose::Goal & goal, std::string & err) {
  if (!data.IsObject()) { err = "data 객체 필요"; return false; }
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  if (data.HasMember("pose") && data["pose"].IsObject()) {  // 전체 pose 지정
    const auto & p = data["pose"];
    double x = 0, y = 0, z = 0, ox = 0, oy = 0, oz = 0, ow = 1;
    if (p.HasMember("position") && p["position"].IsObject()) {
      get_double(p["position"], "x", x);
      get_double(p["position"], "y", y);
      get_double(p["position"], "z", z);
    }
    if (p.HasMember("orientation") && p["orientation"].IsObject()) {
      get_double(p["orientation"], "x", ox);
      get_double(p["orientation"], "y", oy);
      get_double(p["orientation"], "z", oz);
      get_double(p["orientation"], "w", ow);
    }
    if (p.HasMember("frame_id") && p["frame_id"].IsString()) {
      goal.pose.header.frame_id = p["frame_id"].GetString();
    }
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = z;
    goal.pose.pose.orientation.x = ox;
    goal.pose.pose.orientation.y = oy;
    goal.pose.pose.orientation.z = oz;
    goal.pose.pose.orientation.w = ow;
  } else if (data.HasMember("x") && data.HasMember("y")) {  // 간편형 x,y,yaw
    double x = data["x"].GetDouble(), y = data["y"].GetDouble(), yaw = 0;
    get_double(data, "yaw", yaw);
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = sin(yaw / 2);
    goal.pose.pose.orientation.w = cos(yaw / 2);
  } else {
    err = "data.x/y(+yaw) 또는 data.pose 필요";
    return false;
  }
  if (data.HasMember("behavior_tree") && data["behavior_tree"].IsString()) {
    goal.behavior_tree = data["behavior_tree"].GetString();
  }
  return true;
}

bool ServiceActionBagInput::make_nav_through_goal(
    const rapidjson::Value & data, nav2_msgs::action::NavigateThroughPoses::Goal & goal, std::string & err) {
  if (!data.IsObject() || !data.HasMember("poses") || !data["poses"].IsArray()) {
    err = "data.poses(array) 필요";
    return false;
  }
  const auto & arr = data["poses"];
  if (arr.Empty()) {
    err = "data.poses가 비었음";
    return false;
  }
  const std::string frame = (data.HasMember("frame_id") && data["frame_id"].IsString())
      ? data["frame_id"].GetString() : "map";
  goal.poses.header.frame_id = frame;
  goal.poses.header.stamp = now();
  for (const auto & p : arr.GetArray()) {
    if (!p.IsObject()) {
      err = "poses[] 객체 필요";
      return false;
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame;
    ps.header.stamp = now();
    double x = 0, y = 0, z = 0, ox = 0, oy = 0, oz = 0, ow = 1;
    if (p.HasMember("position") && p["position"].IsObject()) {
      get_double(p["position"], "x", x);
      get_double(p["position"], "y", y);
      get_double(p["position"], "z", z);
      if (p.HasMember("orientation") && p["orientation"].IsObject()) {
        get_double(p["orientation"], "x", ox);
        get_double(p["orientation"], "y", oy);
        get_double(p["orientation"], "z", oz);
        get_double(p["orientation"], "w", ow);
      }
    } else if (p.HasMember("x") && p.HasMember("y")) {
      x = p["x"].GetDouble();
      y = p["y"].GetDouble();
      double yaw = 0;
      get_double(p, "yaw", yaw);
      oz = std::sin(yaw / 2.0);
      ow = std::cos(yaw / 2.0);
    } else {
      err = "poses[]에 position 또는 x/y 필요";
      return false;
    }
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
    ps.pose.orientation.x = ox;
    ps.pose.orientation.y = oy;
    ps.pose.orientation.z = oz;
    ps.pose.orientation.w = ow;
    goal.poses.goals.push_back(ps);
  }
  if (data.HasMember("behavior_tree") && data["behavior_tree"].IsString()) {
    goal.behavior_tree = data["behavior_tree"].GetString();
  }
  return true;
}

std::string ServiceActionBagInput::nav_feedback_json(
    const std::string & id, nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr fb) {
  const auto & p = fb->current_pose.pose.position;
  const double yaw = 2 * atan2(fb->current_pose.pose.orientation.z, fb->current_pose.pose.orientation.w);
  char b[512];
  snprintf(b, sizeof(b),
      "{\"goal_id\": \"%s\", \"data\": {\"x\": %.2f, \"y\": %.2f, \"yaw\": %.2f, "
      "\"distance_remaining\": %.2f, \"recoveries\": %d}}",
      escape_json(id).c_str(), p.x, p.y, yaw, fb->distance_remaining, fb->number_of_recoveries);
  return b;
}

std::string ServiceActionBagInput::nav_result_json(
    const std::string & id, const NavClient::WrappedResult & wr) {
  const char * status = "unknown";
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) status = "succeeded";
  else if (wr.code == rclcpp_action::ResultCode::ABORTED) status = "aborted";
  else if (wr.code == rclcpp_action::ResultCode::CANCELED) status = "canceled";
  char b[512];
  snprintf(b, sizeof(b), "{\"goal_id\": \"%s\", \"status\": \"%s\", \"data\": {\"error_code\": %u, \"error_msg\": \"%s\"}}",
      escape_json(id).c_str(), status, wr.result ? wr.result->error_code : 9999,
      wr.result ? escape_json(wr.result->error_msg).c_str() : "no result");
  return b;
}

std::string ServiceActionBagInput::nav_through_feedback_json(
    const std::string & id, nav2_msgs::action::NavigateThroughPoses::Feedback::ConstSharedPtr fb) {
  const auto & p = fb->current_pose.pose.position;
  const double yaw = 2 * atan2(fb->current_pose.pose.orientation.z, fb->current_pose.pose.orientation.w);
  char b[640];
  snprintf(b, sizeof(b),
      "{\"goal_id\": \"%s\", \"data\": {\"x\": %.2f, \"y\": %.2f, \"yaw\": %.2f, "
      "\"distance_remaining\": %.2f, \"recoveries\": %d, \"poses_remaining\": %d}}",
      escape_json(id).c_str(), p.x, p.y, yaw, fb->distance_remaining, fb->number_of_recoveries,
      fb->number_of_poses_remaining);
  return b;
}

std::string ServiceActionBagInput::nav_through_result_json(
    const std::string & id, const NavThroughClient::WrappedResult & wr) {
  const char * status = "unknown";
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) status = "succeeded";
  else if (wr.code == rclcpp_action::ResultCode::ABORTED) status = "aborted";
  else if (wr.code == rclcpp_action::ResultCode::CANCELED) status = "canceled";
  char b[512];
  snprintf(b, sizeof(b), "{\"goal_id\": \"%s\", \"status\": \"%s\", \"data\": {\"error_code\": %u, \"error_msg\": \"%s\"}}",
      escape_json(id).c_str(), status, wr.result ? wr.result->error_code : 9999,
      wr.result ? escape_json(wr.result->error_msg).c_str() : "no result");
  return b;
}

bool ServiceActionBagInput::make_mtc_goal(const rapidjson::Value & data,
    moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal & goal, std::string & err) {
  if (!data.IsObject() || !data.HasMember("task_id") || !data["task_id"].IsString()) {
    err = "data.task_id(string) 필요 — faker는 task_id만 보고 실행";
    return false;
  }
  goal.solution.task_id = data["task_id"].GetString();
  return true;
}

std::string ServiceActionBagInput::mtc_result_json(const std::string & id, const MtcClient::WrappedResult & wr) {
  const char * status = "unknown";
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) status = "succeeded";
  else if (wr.code == rclcpp_action::ResultCode::ABORTED) status = "aborted";
  else if (wr.code == rclcpp_action::ResultCode::CANCELED) status = "canceled";
  char b[256];
  snprintf(b, sizeof(b), "{\"goal_id\": \"%s\", \"status\": \"%s\", \"data\": {\"error_val\": %d}}",
      escape_json(id).c_str(), status, wr.result ? wr.result->error_code.val : -9999);
  return b;
}

bool ServiceActionBagInput::make_move_goal(
    const rapidjson::Value & data, moveit_msgs::action::MoveGroup::Goal & goal, std::string & err) {
  if (data.IsObject() && data.HasMember("joints") && data["joints"].IsObject()) {  // 관절 목표 {"joints": {"joint1": rad, …}}
    double vel = 0.3;
    get_double(data, "velocity_scaling", vel);
    auto & r = goal.request;
    r.group_name = data.HasMember("group") && data["group"].IsString() ? data["group"].GetString() : "arm";
    r.num_planning_attempts = 3;
    r.allowed_planning_time = 5.0;
    r.max_velocity_scaling_factor = vel;
    r.max_acceleration_scaling_factor = vel;
    r.start_state.is_diff = true;
    moveit_msgs::msg::Constraints c;
    for (auto it = data["joints"].MemberBegin(); it != data["joints"].MemberEnd(); ++it) {
      if (!it->value.IsNumber()) continue;
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = it->name.GetString();
      jc.position = it->value.GetDouble();
      jc.tolerance_above = jc.tolerance_below = 0.01;
      jc.weight = 1.0;
      c.joint_constraints.push_back(jc);
    }
    if (c.joint_constraints.empty()) { err = "data.joints 비어 있음"; return false; }
    r.goal_constraints.push_back(c);
    goal.planning_options.plan_only = data.HasMember("plan_only") && data["plan_only"].IsBool() && data["plan_only"].GetBool();
    return true;
  }
  double x = 0, y = 0, z = 0;
  if (!data.IsObject() || !get_double(data, "x", x) || !get_double(data, "y", y) || !get_double(data, "z", z)) {
    err = "data.x·y·z(m) 또는 data.joints 필요 — 손끝 목표 · 관절 목표";
    return false;
  }
  const std::string frame = data.HasMember("frame_id") && data["frame_id"].IsString() ? data["frame_id"].GetString() : "base_footprint";
  const std::string link = data.HasMember("link") && data["link"].IsString() ? data["link"].GetString() : "end_effector_link";
  double ox = 1.0, oy = 0.0, oz = 0.0, ow = 0.0;  // 기본 = 위에서 아래 (손끝 z축이 바닥을 봄 — coordinator pick과 같게)
  double z_tol = M_PI;                            // 수직축 둘레(손목 회전)는 기본 자유
  double yaw = 0.0;
  if (get_double(data, "yaw", yaw)) {  // 손끝 x축의 수평 방위 고정 = 위→아래를 수직축으로 yaw만큼 돌림 (손가락은 손끝 y축으로 벌어짐)
    ox = std::cos(yaw / 2);
    oy = std::sin(yaw / 2);
    oz = ow = 0.0;
    z_tol = 0.05;
    get_double(data, "yaw_tolerance", z_tol);
  }
  if (data.HasMember("orientation") && data["orientation"].IsArray() && data["orientation"].Size() == 4) {
    const auto & o = data["orientation"];
    ox = o[0].GetDouble(); oy = o[1].GetDouble(); oz = o[2].GetDouble(); ow = o[3].GetDouble();
  }
  double vel = 0.3;
  get_double(data, "velocity_scaling", vel);
  auto & r = goal.request;
  r.group_name = data.HasMember("group") && data["group"].IsString() ? data["group"].GetString() : "arm";
  r.num_planning_attempts = 3;
  r.allowed_planning_time = 5.0;
  r.max_velocity_scaling_factor = vel;
  r.max_acceleration_scaling_factor = vel;
  r.start_state.is_diff = true;  // 지금 자세에서
  moveit_msgs::msg::Constraints c;
  moveit_msgs::msg::PositionConstraint pc;
  pc.header.frame_id = frame;
  pc.link_name = link;
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.01, 0.01, 0.01};
  pc.constraint_region.primitives.push_back(box);
  geometry_msgs::msg::Pose at;
  at.position.x = x;
  at.position.y = y;
  at.position.z = z;
  at.orientation.w = 1.0;
  pc.constraint_region.primitive_poses.push_back(at);
  pc.weight = 1.0;
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = frame;
  oc.link_name = link;
  oc.orientation.x = ox;
  oc.orientation.y = oy;
  oc.orientation.z = oz;
  oc.orientation.w = ow;
  oc.absolute_x_axis_tolerance = 0.15;
  oc.absolute_y_axis_tolerance = 0.15;
  oc.absolute_z_axis_tolerance = z_tol;  // 수직축 둘레 회전 — 기본 자유 (joint1·joint5 조합), yaw를 주면 고정
  oc.weight = 1.0;
  c.position_constraints.push_back(pc);
  c.orientation_constraints.push_back(oc);
  r.goal_constraints.push_back(c);
  goal.planning_options.plan_only = data.HasMember("plan_only") && data["plan_only"].IsBool() && data["plan_only"].GetBool();
  goal.planning_options.replan = false;
  return true;
}

std::string ServiceActionBagInput::move_result_json(const std::string & id, const MoveClient::WrappedResult & wr) {
  const char * status = "unknown";
  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) status = "succeeded";
  else if (wr.code == rclcpp_action::ResultCode::ABORTED) status = "aborted";
  else if (wr.code == rclcpp_action::ResultCode::CANCELED) status = "canceled";
  char b[256];  // error_code: 1 성공 · −1 계획 실패 · −2 계획 무효 · −4 실행 중 실패 · −31 IK 없음 … (moveit_msgs/MoveItErrorCodes)
  snprintf(b, sizeof(b), "{\"goal_id\": \"%s\", \"status\": \"%s\", \"data\": {\"error_code\": %d, \"planning_time\": %.2f}}",
      escape_json(id).c_str(), status, wr.result ? wr.result->error_code.val : -9999,
      wr.result ? wr.result->planning_time : 0.0);
  return b;
}

}  // namespace faker
