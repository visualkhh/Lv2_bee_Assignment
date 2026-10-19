// 재시작 도우미 구현 — 설명은 include/manager/restart.hpp
#include "manager/restart.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace manager::restart {

namespace fs = std::filesystem;

namespace {

std::vector<std::string> split_ws(const std::string & s) {
  std::istringstream in(s);
  std::vector<std::string> out;
  for (std::string t; in >> t;) out.push_back(t);
  return out;
}

std::string key_of(const std::string & arg) {
  const auto i = arg.find(":=");
  return i == std::string::npos ? "" : arg.substr(0, i);
}

std::string base(const std::string & path) {
  const auto i = path.rfind('/');
  return i == std::string::npos ? path : path.substr(i + 1);
}

std::vector<std::string> read_nul(const fs::path & file) {
  std::ifstream in(file, std::ios::binary);
  std::vector<std::string> out;
  for (std::string t; std::getline(in, t, '\0');) out.push_back(t);
  return out;
}

const Proc * find(const std::vector<Proc> & procs, int pid) {
  for (const auto & p : procs) {
    if (p.pid == pid) return &p;
  }
  return nullptr;
}

}  // namespace

Request parse(const std::string & text) {
  Request r;
  auto t = split_ws(text);
  if (t.empty()) {
    r.error = "빈 요청 — \"all [이름:=값 …]\" 또는 \"node <이름>\"";
    return r;
  }
  if (t[0] == "all") {
    for (size_t i = 1; i < t.size(); ++i) {
      if (key_of(t[i]).empty()) {
        r.error = "all 뒤에는 launch 인자(이름:=값)만: '" + t[i] + "'";
        return r;
      }
      r.args.push_back(t[i]);
    }
    r.kind = Request::ALL;
  } else if (t[0] == "node") {
    if (t.size() != 2) {
      r.error = "node 뒤에 노드 이름 하나: \"node <이름>\"";
      return r;
    }
    r.node = t[1];
    while (!r.node.empty() && r.node.front() == '/') r.node.erase(r.node.begin());
    if (r.node.empty()) {
      r.error = "노드 이름이 비었음";
      return r;
    }
    r.kind = Request::NODE;
  } else {
    r.error = "모르는 요청 '" + t[0] + "' — \"all [이름:=값 …]\" 또는 \"node <이름>\"";
  }
  return r;
}

std::vector<std::string> merge_launch_args(const std::vector<std::string> & argv, const std::vector<std::string> & overrides) {
  std::set<std::string> keys;
  for (const auto & o : overrides) keys.insert(key_of(o));
  const bool drop_map_name = keys.count("map") && !keys.count("map_name");
  std::vector<std::string> out;
  for (const auto & a : argv) {
    const auto k = key_of(a);
    if (!k.empty() && (keys.count(k) || (drop_map_name && k == "map_name"))) continue;
    out.push_back(a);
  }
  out.insert(out.end(), overrides.begin(), overrides.end());
  return out;
}

bool is_launch(const Proc & p) {
  for (size_t i = 0; i + 1 < p.argv.size(); ++i) {
    if (base(p.argv[i]) == "ros2" && p.argv[i + 1] == "launch") return true;
  }
  return false;
}

int launch_root(const std::vector<Proc> & procs, int pid) {
  std::set<int> seen;
  for (const Proc * p = find(procs, pid); p && !seen.count(p->pid); p = find(procs, p->ppid)) {
    seen.insert(p->pid);
    if (p->pid != pid && is_launch(*p)) return p->pid;
  }
  return 0;
}

std::vector<int> descendants(const std::vector<Proc> & procs, int root) {
  std::vector<int> out, frontier{root};
  while (!frontier.empty()) {
    std::vector<int> next;
    for (const auto & p : procs) {
      if (std::find(frontier.begin(), frontier.end(), p.ppid) != frontier.end() && p.pid != root &&
        std::find(out.begin(), out.end(), p.pid) == out.end())
      {
        out.push_back(p.pid);
        next.push_back(p.pid);
      }
    }
    frontier = next;
  }
  return out;
}

std::string node_name(const Proc & p) {
  // 실행 파일 기본 노드 이름이 실행 파일 이름과 다른 것 (launch가 name=을 안 주면 이 이름으로 뜸)
  static const std::map<std::string, std::string> DEFAULT_NAMES = {
    {"ros2_control_node", "controller_manager"}, {"aggregator_node", "analyzers"}};
  if (p.argv.empty() || is_launch(p) || std::find(p.argv.begin(), p.argv.end(), "--ros-args") == p.argv.end()) return "";
  std::string remap;
  for (const auto & a : p.argv) {
    if (a.rfind("__node:=", 0) == 0 || a.rfind("__name:=", 0) == 0) remap = a.substr(8);
  }
  if (!remap.empty()) return remap;
  std::string exe = base(p.argv[0]);
  if (exe.rfind("python", 0) == 0 && p.argv.size() > 1) exe = base(p.argv[1]);  // 파이썬 노드: 스크립트 이름
  const auto it = DEFAULT_NAMES.find(exe);
  if (it != DEFAULT_NAMES.end()) return it->second;
  if (exe.size() > 5 && exe.compare(exe.size() - 5, 5, "_node") == 0) exe.erase(exe.size() - 5);
  return exe;
}

namespace {
std::vector<int> candidates(const std::vector<Proc> & procs, int root, const std::vector<int> & extra) {
  std::vector<int> c = descendants(procs, root);
  for (int e : extra) {
    if (std::find(c.begin(), c.end(), e) == c.end()) c.push_back(e);
  }
  return c;
}
}  // namespace

std::vector<int> find_node(const std::vector<Proc> & procs, int root, const std::string & name, const std::vector<int> & extra) {
  const auto leaf = base(name);  // 이름공간이 붙어 와도 마지막 이름으로
  std::vector<int> out;
  for (int pid : candidates(procs, root, extra)) {
    const Proc * p = find(procs, pid);
    if (p && !leaf.empty() && node_name(*p) == leaf) out.push_back(pid);
  }
  return out;
}

std::vector<std::pair<std::string, int>> restartable(const std::vector<Proc> & procs, int root, const std::vector<int> & extra) {
  std::vector<std::pair<std::string, int>> out;
  for (int pid : candidates(procs, root, extra)) {
    const Proc * p = find(procs, pid);
    const auto n = p ? node_name(*p) : "";
    if (!n.empty()) out.emplace_back(n, pid);
  }
  std::sort(out.begin(), out.end());
  return out;
}

namespace {
std::string join_argv(const std::vector<std::string> & argv) {
  std::string s;
  for (const auto & a : argv) s += a + '\x1f';
  return s;
}
}  // namespace

void record(const std::string & file, int pid, const std::vector<std::string> & argv) {
  if (file.empty() || pid <= 0) return;
  std::ofstream out(file, std::ios::app);
  out << pid << ' ' << join_argv(argv) << '\n';
}

std::vector<int> registered(const std::string & file, const std::vector<Proc> & procs) {
  std::vector<int> out;
  std::ifstream in(file);
  for (std::string line; std::getline(in, line);) {
    const auto sp = line.find(' ');
    if (sp == std::string::npos) continue;
    const int pid = std::atoi(line.substr(0, sp).c_str());
    const Proc * p = find(procs, pid);
    if (p && join_argv(p->argv) == line.substr(sp + 1) && std::find(out.begin(), out.end(), pid) == out.end()) out.push_back(pid);
  }
  return out;
}

std::vector<Proc> list_procs() {
  std::vector<Proc> out;
  std::error_code ec;
  for (const auto & e : fs::directory_iterator("/proc", ec)) {
    const auto n = e.path().filename().string();
    if (n.empty() || !std::all_of(n.begin(), n.end(), ::isdigit)) continue;
    std::ifstream stat(e.path() / "stat");
    std::string line;
    if (!std::getline(stat, line)) continue;
    const auto rp = line.rfind(')');  // 2번째 필드(comm)는 괄호 안에 공백이 있을 수 있음
    if (rp == std::string::npos) continue;
    std::istringstream rest(line.substr(rp + 2));
    std::string state;
    Proc p;
    p.pid = std::stoi(n);
    if (!(rest >> state >> p.ppid)) continue;
    p.argv = read_nul(e.path() / "cmdline");
    out.push_back(std::move(p));
  }
  return out;
}

bool read_proc(int pid, std::vector<std::string> & argv, std::vector<std::string> & env, std::string & cwd, std::string & exe) {
  const fs::path d = fs::path("/proc") / std::to_string(pid);
  std::error_code ec;
  argv = read_nul(d / "cmdline");
  env = read_nul(d / "environ");
  cwd = fs::read_symlink(d / "cwd", ec).string();
  if (ec) cwd = "/";
  exe = fs::read_symlink(d / "exe", ec).string();
  if (ec) exe.clear();
  return !argv.empty();
}

std::string helper_script(const Plan &) {
  // 인자는 위치 인자로 받는다 (따옴표·공백 걱정 없음):
  //   $1 marker $2 log $3 cwd $4 prepare $5 wait(0.1초 단위) $6 clean_shm $7 clear  뒤로 [개수 + 목록] × 4: stop · kill · argv · env
  return R"BASH(
marker=$1 log=$2 cwd=$3 prepare=$4 waits=$5 clean=$6 clear=$7; shift 7
n=$1; shift; stop=("${@:1:$n}"); shift "$n"
n=$1; shift; victims=("${@:1:$n}"); shift "$n"
n=$1; shift; cmd=("${@:1:$n}"); shift "$n"
n=$1; shift; envv=("${@:1:$n}"); shift "$n"
[ -n "$marker" ] && : > "$marker"
alive() { [ -d "/proc/$1" ] && ! grep -q '^State:[[:space:]]*Z' "/proc/$1/status" 2>/dev/null; }
echo "$(date +%T) [state_manager 재시작] 끄기 (SIGINT ${stop[*]})" >> "$log"
for p in "${stop[@]}"; do kill -INT "$p" 2>/dev/null; done
# 셸 스크립트가 &로 띄운 launch는 SIGINT를 무시한 채 물려받음(test-faker 0 — 실측 SigIgn) → 3초 뒤 SIGTERM.
# 그런데 SIGTERM을 받은 launch는 자기만 끝나고 노드는 고아로 남김(실측) → 그때 남은 노드에도 직접 SIGINT
# (노드는 rclcpp·rclpy가 SIGINT를 받아 정상 종료 — RTAB-Map이 rtabmap.db를 닫을 수 있게). 그래도 남으면 wait 끝에 SIGKILL
i=0
for _ in $(seq "$waits"); do
  i=$((i + 1))
  if [ "$i" = 30 ]; then
    for p in "${stop[@]}"; do alive "$p" && kill -TERM "$p" 2>/dev/null; done
    for p in "${victims[@]}"; do alive "$p" && kill -INT "$p" 2>/dev/null; done
  fi
  any=0
  for p in "${stop[@]}" "${victims[@]}"; do alive "$p" && { any=1; break; }; done
  [ "$any" = 0 ] && break
  sleep 0.1
done
for p in "${victims[@]}"; do alive "$p" && kill -9 "$p" 2>/dev/null; done
[ "$clean" = 1 ] && rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
[ -n "$clear" ] && rm -f "$clear"
if [ -n "$prepare" ]; then
  echo "$(date +%T) [state_manager 재시작] 준비: $prepare" >> "$log"
  bash -lc "$prepare" >> "$log" 2>&1 || echo "$(date +%T) [state_manager 재시작] 준비 실패 — 그대로 띄움" >> "$log"
fi
echo "$(date +%T) [state_manager 재시작] 띄우기: ${cmd[*]}" >> "$log"
[ -n "$marker" ] && rm -f "$marker"
cd "$cwd" 2>/dev/null || cd /
exec env -i "${envv[@]}" "${cmd[@]}" >> "$log" 2>&1 < /dev/null
)BASH";
}

bool spawn(const Plan & plan, std::string & err, int * pid) {
  if (plan.argv.empty()) {
    err = "다시 띄울 명령줄이 없음";
    return false;
  }
  // fork 뒤(자식)에선 메모리 할당을 하지 않도록 exec 인자를 미리 다 만든다 (여러 스레드 프로세스의 fork)
  std::vector<std::string> args{"bash", "-c", helper_script(plan), "restart-helper", plan.marker, plan.log, plan.cwd,
                                plan.prepare, std::to_string(static_cast<int>(std::ceil(plan.wait_s * 10))),
                                plan.clean_shm ? "1" : "0", plan.clear};
  const auto add = [&](const std::vector<std::string> & v) {
    args.push_back(std::to_string(v.size()));
    args.insert(args.end(), v.begin(), v.end());
  };
  std::vector<std::string> stop, victims;
  for (int p : plan.stop) stop.push_back(std::to_string(p));
  for (int p : plan.kill) victims.push_back(std::to_string(p));
  add(stop);
  add(victims);
  add(plan.argv);
  add(plan.env);
  std::vector<char *> cargv;
  for (auto & a : args) cargv.push_back(a.data());
  cargv.push_back(nullptr);
  const std::string log = plan.log.empty() ? "/dev/null" : plan.log;
  const long max_fd = std::min(sysconf(_SC_OPEN_MAX), 65536L);

  int fds[2];
  if (pipe(fds) != 0) {
    err = "pipe 실패";
    return false;
  }
  const pid_t child = fork();
  if (child < 0) {
    err = "fork 실패";
    close(fds[0]);
    close(fds[1]);
    return false;
  }
  if (child == 0) {  // 1차 자식: 새 세션 → 한 번 더 fork해 init 밑으로 (state_manager가 죽어도 남음, 좀비도 안 남김)
    setsid();
    const pid_t g = fork();
    if (g != 0) {  // 손자 pid = 다시 띄울 프로세스 pid (bash → exec env → exec 명령, 같은 pid) — 부모에게 알리고 끝
      const int v = static_cast<int>(g);
      (void)!write(fds[1], &v, sizeof(v));
      _exit(0);
    }
    const int in = open("/dev/null", O_RDONLY);
    const int out = open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    dup2(in, 0);
    dup2(out >= 0 ? out : in, 1);
    dup2(out >= 0 ? out : in, 2);
    for (long fd = 3; fd < max_fd; ++fd) close(static_cast<int>(fd));  // DDS 소켓 등 state_manager 파일을 물려주지 않음
    // 시그널은 기본값으로 — 다시 띄운 launch·노드가 Ctrl+C·SIGINT로 꺼지게 (무시·막힘을 물려주지 않음)
    sigset_t none;
    sigemptyset(&none);
    sigprocmask(SIG_SETMASK, &none, nullptr);
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    signal(SIGQUIT, SIG_DFL);
    signal(SIGCHLD, SIG_DFL);
    execv("/bin/bash", cargv.data());
    _exit(127);
  }
  close(fds[1]);
  int g = 0;
  if (read(fds[0], &g, sizeof(g)) != static_cast<ssize_t>(sizeof(g))) g = 0;
  close(fds[0]);
  int status = 0;
  waitpid(child, &status, 0);
  if (pid) *pid = g;
  return true;
}

}  // namespace manager::restart
