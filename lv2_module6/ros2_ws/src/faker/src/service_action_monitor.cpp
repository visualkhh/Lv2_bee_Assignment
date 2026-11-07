// service_action_monitor 구현 — 설명·선언은 include/faker/service_action_monitor.hpp
#include "faker/service_action_monitor.hpp"

#include <cstring>
#include <algorithm>
#include <sstream>
#include <sys/stat.h>

namespace faker {

ServiceActionMonitor::ServiceActionMonitor()
: rclcpp::Node("service_action_monitor") {
  debug_dir_ = declare_parameter<std::string>("debug_dir", "/debug");
  const double period = declare_parameter<double>("period_s", 2.0);
  std::error_code ec;
  fs::create_directories(fs::path(debug_dir_) / "service", ec);
  fs::create_directories(fs::path(debug_dir_) / "action", ec);
  RCLCPP_INFO(get_logger(), "시작 | 저장: %s/service,list action/ (period %.1fs)", debug_dir_.c_str(), period);
  timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() { scan(); });
  scan();
  // interface 목록은 빌드해야 바뀌는 정적 정보 — 60초마다 갱신
  interface_timer_ = create_wall_timer(std::chrono::seconds(60), [this]() { dump_interfaces(); });
  dump_interfaces();
}

bool ServiceActionMonitor::is_param_service(const std::string & name) {
  for (const char * k : {"describe_parameters", "get_parameters", "set_parameters",
                         "list_parameters", "get_parameter_types", "set_parameters_atomically",
                         "get_type_description", "get_type_description_sources",
                         "get_logger_levels", "set_logger_levels"}) {
    if (name.size() >= strlen(k) && name.compare(name.size() - strlen(k), strlen(k), k) == 0) return true;
  }
  return false;
}

bool ServiceActionMonitor::is_action_internal(const std::string & name) {
  return name.find("/_action/") != std::string::npos;
}

std::string ServiceActionMonitor::action_name_of(const std::string & send_goal_service) {
  const std::string suf = "/_action/send_goal";
  if (send_goal_service.size() > suf.size() &&
      send_goal_service.compare(send_goal_service.size() - suf.size(), suf.size(), suf) == 0) {
    return send_goal_service.substr(0, send_goal_service.size() - suf.size());
  }
  return "";
}

std::string ServiceActionMonitor::action_type_of(const std::string & send_goal_type) {
  // "nav2_msgs/action/NavigateToPose_SendGoal" -> "nav2_msgs/action/NavigateToPose"
  const std::string suf = "_SendGoal";
  if (send_goal_type.size() > suf.size() &&
      send_goal_type.compare(send_goal_type.size() - suf.size(), suf.size(), suf) == 0) {
    return send_goal_type.substr(0, send_goal_type.size() - suf.size());
  }
  return "";
}

fs::path ServiceActionMonitor::dir_of(const std::string & root, const std::string & name) {
  std::string rel = name;
  if (!rel.empty() && rel[0] == '/') rel.erase(0, 1);
  return fs::path(root) / rel;
}

void ServiceActionMonitor::write_text(const fs::path & out, const std::string & text) {
  std::error_code ec;
  fs::create_directories(out.parent_path(), ec);
  ::chmod(out.parent_path().c_str(), 0777);  // 컨테이너(root)가 만들어도 호스트가 rename으로 교체 가능
  const fs::path tmp = out.parent_path() / (".tmp-" + out.filename().string());
  { std::ofstream f(tmp); f << text; }
  fs::rename(tmp, out, ec);
}

std::string ServiceActionMonitor::definition_of(const std::string & type) {
  // GetMessageDefinition은 msg만 되므로 srv/action은 `ros2 interface show`로 조회.
  // NOTE: moveit_msgs 등 일부 타입은 `ros2 interface show`가 재귀 전개 버그로
  // 수백 KB를 쏟아낸다 (RobotState state 무한 반복) — 120줄에서 자른다.
  std::string cmd = "ros2 interface show " + type + " 2>&1";
  std::string out;
  char buf[512];
  FILE * p = popen(cmd.c_str(), "r");
  if (!p) return "# definition unavailable: popen failed\n";
  size_t lines = 0;
  bool truncated = false;
  while (fgets(buf, sizeof(buf), p)) {
    out += buf;
    if (++lines >= 120) { truncated = true; break; }
  }
  pclose(p);
  if (out.empty()) out = "# definition unavailable: empty output\n";
  else if (truncated) out += "\n# ... [truncated at 120 lines: `ros2 interface show " + type + "` full output]\n";
  return out;
}

void ServiceActionMonitor::scan() {
  // --- 서비스 목록
  std::map<std::string, std::string> services;
  for (const auto & [name, types] : get_service_names_and_types()) {
    if (types.empty() || is_param_service(name) || is_action_internal(name)) continue;
    if (name == "/rosout" || name.find("/_") != std::string::npos) continue;
    services[name] = types.front();
  }
  // --- 액션 목록: /<action>/_action/send_goal 서비스에서 역산
  std::map<std::string, std::string> actions;
  for (const auto & [name, types] : get_service_names_and_types()) {
    if (types.empty()) continue;
    const std::string aname = action_name_of(name);
    if (aname.empty()) continue;
    const std::string atype = action_type_of(types.front());
    if (atype.empty()) continue;
    actions[aname] = atype;
  }

  if (services != services_) {
    services_ = services;
    std::string list = "{\"services\": [";
    bool first = true;
    for (const auto & [name, type] : services_) {
      if (!first) list += ", ";
      first = false;
      list += "{\"name\": \"" + name + "\", \"type\": \"" + type + "\"}";
      const fs::path dir = dir_of(debug_dir_ + "/service", name);
      write_text(dir / "type", type + "\n");
      write_text(dir / "info", definition_of(type));
      ensure_proto(dir, type);
    }
    list += "]}\n";
    write_text(fs::path(debug_dir_) / "service" / "list.json", list);
    RCLCPP_INFO(get_logger(), "service %zu개: %s", services_.size(), list.c_str());
  }
  if (actions != actions_) {
    actions_ = actions;
    std::string list = "{\"actions\": [";
    bool first = true;
    for (const auto & [name, type] : actions_) {
      if (!first) list += ", ";
      first = false;
      list += "{\"name\": \"" + name + "\", \"type\": \"" + type + "\"}";
      const fs::path dir = dir_of(debug_dir_ + "/action", name);
      write_text(dir / "type", type + "\n");
      write_text(dir / "info", definition_of(type));
      ensure_proto(dir, type);
    }
    list += "]}\n";
    write_text(fs::path(debug_dir_) / "action" / "list.json", list);
    RCLCPP_INFO(get_logger(), "action %zu개: %s", actions_.size(), list.c_str());
  }
  poll_interface_requests();  // 60초짜리 dump와 별개로 2초 폴링에서 on-demand 처리
}

void ServiceActionMonitor::dump_interfaces() {
  // 정적 타입 카탈로그 — `ros2 interface list` 그대로 (~500줄)
  std::string out;
  char buf[1024];
  FILE * p = popen("ros2 interface list 2>/dev/null", "r");
  if (!p) return;
  while (fgets(buf, sizeof(buf), p)) out += buf;
  pclose(p);
  if (!out.empty() && out != interfaces_) {
    interfaces_ = out;
    write_text(fs::path(debug_dir_) / "interface" / "list", out);
    // 종류별 목록도 분리 (Messages:/Services:/Actions: 섹션 파싱)
    std::string cur;
    std::map<std::string, std::string> per_kind;
    std::istringstream sec(out);
    std::string line;
    while (std::getline(sec, line)) {
      std::string t = line;
      t.erase(0, t.find_first_not_of(" \t\r"));
      if (t == "Messages:") { cur = "msg"; continue; }
      if (t == "Services:") { cur = "srv"; continue; }
      if (t == "Actions:") { cur = "action"; continue; }
      if (!cur.empty() && !t.empty()) per_kind[cur] += t + "\n";
    }
    for (const auto & [kind, text] : per_kind) {
      write_text(fs::path(debug_dir_) / "interface" / kind / "list", text);
    }
    // 패키지 목록 (`ros2 interface packages`) — 어떤 패키지가 인터페이스를 가졌는지
    {
      std::string pkgs;
      char pch[512];
      FILE * pp = popen("ros2 interface packages 2>/dev/null", "r");
      if (pp) {
        while (fgets(pch, sizeof(pch), pp)) pkgs += pch;
        pclose(pp);
      }
      if (!pkgs.empty()) write_text(fs::path(debug_dir_) / "interface" / "packages", pkgs);
    }
    RCLCPP_INFO(get_logger(), "interface 목록 %zu줄 기록", std::count(out.begin(), out.end(), '\n'));
    // 프로젝트 타입은 폴더뎁스로 미리 생성 (perception·control·ml·plan)
    std::istringstream lines2(out);
    std::string line2;
    while (std::getline(lines2, line2)) {
      std::string t = line2;
      t.erase(0, t.find_first_not_of(" \t"));
      if (t.rfind("perception/", 0) == 0 || t.rfind("control/", 0) == 0 || t.rfind("ml/", 0) == 0 ||
          t.rfind("plan/", 0) == 0) {
        ensure_interface_info(t);
      }
    }
  }
  poll_interface_requests();
}

void ServiceActionMonitor::ensure_interface_info(const std::string & type) {
  const fs::path dir = iface_dir(debug_dir_, type);
  if (dir.empty()) return;
  const fs::path info = dir / "info";
  std::error_code ec;
  if (fs::exists(info, ec)) {
    ensure_proto(dir, type);  // 구버전이 info만 만든 경우 proto 보충
    return;  // 있으면 다시 안 만듦
  }
  write_text(info, definition_of(type));
  ensure_proto(dir, type);
  RCLCPP_INFO(get_logger(), "interface info: %s", type.c_str());
}

void ServiceActionMonitor::ensure_proto(const fs::path & dir, const std::string & type) {
  // 빈 메시지 틀 (`ros2 interface proto`) — request 파일 쓸 때 복사해서 쓴다. 없으면 한 번만 생성
  const fs::path proto = dir / "proto";
  std::error_code ec;
  if (fs::exists(proto, ec)) return;
  std::string cmd = "ros2 interface proto " + type + " 2>&1";
  std::string out;
  char buf[512];
  FILE * p = popen(cmd.c_str(), "r");
  if (p) {
    while (fgets(buf, sizeof(buf), p)) out += buf;
    pclose(p);
  }
  if (out.empty()) out = "# proto unavailable: empty output\n";
  write_text(proto, out);
}

fs::path ServiceActionMonitor::iface_dir(const std::string & debug_dir, const std::string & type) {
  // "ml/srv/ScoreGrasps" → "<debug>/interface/srv/ml/ScoreGrasps" (종류별 폴더뎁스)
  if (type.empty() || type.find("..") != std::string::npos) return {};
  size_t a = type.find('/');
  size_t b = (a == std::string::npos) ? std::string::npos : type.find('/', a + 1);
  if (a == std::string::npos || b == std::string::npos || type.find('/', b + 1) != std::string::npos) return {};
  const std::string pkg = type.substr(0, a);
  const std::string kind = type.substr(a + 1, b - a - 1);
  const std::string name = type.substr(b + 1);
  if (pkg.empty() || name.empty() || (kind != "msg" && kind != "srv" && kind != "action")) return {};
  return fs::path(debug_dir) / "interface" / kind / pkg / name;
}

void ServiceActionMonitor::poll_interface_requests() {
  // interface/<종류>/<패키지>/<이름>/request (mtime 변경) → 옆에 info 생성
  const fs::path root = fs::path(debug_dir_) / "interface";
  std::error_code ec;
  for (auto it = fs::recursive_directory_iterator(root, ec); !ec && it != fs::recursive_directory_iterator();
       it.increment(ec)) {
    if (it->path().filename() != "request" || !it->is_regular_file()) continue;
    // 상대경로 "<종류>/<패키지>/<이름>" → 타입 "패키지/종류/이름" 역산
    const std::string rel = fs::relative(it->path().parent_path(), root, ec).generic_string();
    if (ec || rel.empty() || rel.find("..") != std::string::npos) continue;
    size_t a = rel.find('/');
    size_t b = (a == std::string::npos) ? std::string::npos : rel.find('/', a + 1);
    if (a == std::string::npos || b == std::string::npos || rel.find('/', b + 1) != std::string::npos) continue;
    const std::string kind = rel.substr(0, a);
    const std::string pkg = rel.substr(a + 1, b - a - 1);
    const std::string name = rel.substr(b + 1);
    if (kind != "msg" && kind != "srv" && kind != "action") continue;
    const std::string type = pkg + "/" + kind + "/" + name;
    const auto mtime = fs::last_write_time(it->path(), ec);
    if (ec) continue;
    auto known = iface_req_.find(type);
    if (known != iface_req_.end() && known->second == mtime) continue;
    iface_req_[type] = mtime;
    write_text(it->path().parent_path() / "info", definition_of(type));
    ensure_proto(it->path().parent_path(), type);
    RCLCPP_INFO(get_logger(), "interface info 요청: %s", type.c_str());
  }
}

}  // namespace faker
