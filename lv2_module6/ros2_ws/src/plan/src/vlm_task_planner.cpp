// vlm_task_planner 구현 — 노드 설명·입출력·멤버 선언은 include/plan/vlm_task_planner.hpp, main은 vlm_task_planner_main.cpp
#include "plan/vlm_task_planner.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <thread>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace plan {

namespace {

// base64 (요청 본문 images[].data_b64용) — 의존성 없이 직접 인코딩
std::string base64_encode(const std::vector<uint8_t> & in) {
  static const char * tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string o;
  o.reserve((in.size() + 2) / 3 * 4);
  for (size_t i = 0; i < in.size(); i += 3) {
    const uint32_t a = in[i], b = i + 1 < in.size() ? in[i + 1] : 0, c = i + 2 < in.size() ? in[i + 2] : 0;
    const uint32_t v = (a << 16) | (b << 8) | c;
    o += tbl[(v >> 18) & 63];
    o += tbl[(v >> 12) & 63];
    o += (i + 1 < in.size()) ? tbl[(v >> 6) & 63] : '=';
    o += (i + 2 < in.size()) ? tbl[v & 63] : '=';
  }
  return o;
}

// JSON 문자열 이스케이프 (요청 본문용)
std::string json_escape(const std::string & s) {
  std::string o;
  o.reserve(s.size() + 2);
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char b[8];
          snprintf(b, sizeof(b), "\\u%04x", c);
          o += b;
        } else { o += static_cast<char>(c); }
    }
  }
  return o;
}

// "http://host:port"·"host:port" → host/port (https 미지원 — 로컬 http만)
bool split_endpoint(const std::string & ep, std::string & host, int & port) {
  std::string t = ep;
  if (t.rfind("https://", 0) == 0) return false;
  if (t.rfind("http://", 0) == 0) t.erase(0, 7);
  const auto c = t.find(':');
  if (c == std::string::npos) { host = t; port = 80; }
  else {
    host = t.substr(0, c);
    port = std::atoi(t.substr(c + 1).c_str());
  }
  return !host.empty() && port > 0 && port < 65536;
}

bool wait_fd(int fd, bool for_write, double timeout_s) {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  timeval tv{static_cast<long>(timeout_s), 0};
  const int r = select(fd + 1, for_write ? nullptr : &set, for_write ? &set : nullptr, nullptr, &tv);
  return r > 0;
}

// blocking HTTP POST — 200이면 body. select 타임아웃 (워커 스레드에서만 호출)
bool http_post(const std::string & host, int port, const std::string & body,
  std::string & out, double timeout_s) {
  addrinfo hints{};
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * list = nullptr;
  if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &list) != 0 || !list) return false;
  int fd = -1;
  for (addrinfo * a = list; a; a = a->ai_next) {
    fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
    if (fd < 0) continue;
    fcntl(fd, F_SETFL, O_NONBLOCK);
    if (connect(fd, a->ai_addr, a->ai_addrlen) != 0 && errno != EINPROGRESS) {
      close(fd);
      fd = -1;
      continue;
    }
    break;
  }
  freeaddrinfo(list);
  if (fd < 0 || !wait_fd(fd, true, timeout_s)) {
    if (fd >= 0) close(fd);
    return false;
  }
  int err = 0;
  socklen_t len = sizeof(err);
  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
    close(fd);
    return false;
  }
  const std::string req = "POST /plan HTTP/1.0\r\nHost: " + host + "\r\nContent-Type: application/json\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  size_t sent = 0;
  while (sent < req.size()) {
    if (std::chrono::steady_clock::now() > deadline) { close(fd); return false; }
    if (!wait_fd(fd, true, 1.0)) continue;
    const ssize_t n = send(fd, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
    if (n <= 0) { close(fd); return false; }
    sent += static_cast<size_t>(n);
  }
  std::string raw;
  char buf[4096];
  for (;;) {
    if (std::chrono::steady_clock::now() > deadline) { close(fd); return false; }
    if (!wait_fd(fd, false, 1.0)) continue;  // idle — 아직 EOF 아님
    const ssize_t n = recv(fd, buf, sizeof(buf), 0);
    if (n == 0) break;  // 서버가 닫음 (HTTP/1.0 + Connection: close)
    if (n < 0) { close(fd); return false; }
    raw.append(buf, static_cast<size_t>(n));
  }
  close(fd);
  const auto eol = raw.find("\r\n");
  if (eol == std::string::npos || raw.substr(0, eol).find(" 200") == std::string::npos) return false;
  const auto h = raw.find("\r\n\r\n");
  if (h == std::string::npos) return false;
  out = raw.substr(h + 4);
  return true;
}

// {"ok": true, "goals": [{step,skill,target,relation,offset_m}]} 최소 파서
struct Cursor {
  const char * p;
  void ws() { while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p; }
  bool str(std::string & o);
  bool num(double & o);
  bool skip();  // 값 하나 건너뜀 (중첩 포함)
};

bool Cursor::str(std::string & o) {
  if (*p != '"') return false;
  ++p;
  o.clear();
  while (*p && *p != '"') {
    if (*p == '\\') {
      ++p;
      switch (*p) {
        case '"': o += '"'; break;
        case '\\': o += '\\'; break;
        case 'n': o += '\n'; break;
        case 't': o += '\t'; break;
        case 'r': o += '\r'; break;
        case 'b': o += '\b'; break;
        case 'f': o += '\f'; break;
        case 'u': {  // \uXXXX (서로게이트 쌍 포함) → UTF-8 — 없으면 한글이 'uba85…'로 깨짐 (VLM·ai-service가 이스케이프해서 보냄)
          auto hex4 = [](const char * h, unsigned & v) {
            v = 0;
            for (int i = 0; i < 4; ++i) {
              const char ch = h[i];
              if (!std::isxdigit(static_cast<unsigned char>(ch))) return false;
              v = v * 16 + static_cast<unsigned>(std::isdigit(static_cast<unsigned char>(ch)) ? ch - '0' : (std::tolower(ch) - 'a' + 10));
            }
            return true;
          };
          unsigned cp = 0, lo = 0;
          if (!hex4(p + 1, cp)) return false;
          p += 4;
          if (cp >= 0xD800 && cp <= 0xDBFF && p[1] == '\\' && p[2] == 'u' && hex4(p + 3, lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
            p += 6;
          }
          if (cp < 0x80) { o += static_cast<char>(cp); }
          else if (cp < 0x800) { o += static_cast<char>(0xC0 | (cp >> 6)); o += static_cast<char>(0x80 | (cp & 0x3F)); }
          else if (cp < 0x10000) {
            o += static_cast<char>(0xE0 | (cp >> 12)); o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); o += static_cast<char>(0x80 | (cp & 0x3F));
          } else {
            o += static_cast<char>(0xF0 | (cp >> 18)); o += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F)); o += static_cast<char>(0x80 | (cp & 0x3F));
          }
          break;
        }
        default: if (*p) o += *p; break;  // \" \\ \/
      }
      if (*p) ++p;
    } else { o += *p++; }
  }
  if (*p != '"') return false;
  ++p;
  return true;
}

bool Cursor::num(double & o) {
  char * end = nullptr;
  o = std::strtod(p, &end);
  if (end == p) return false;
  p = end;
  return true;
}

bool Cursor::skip() {
  ws();
  if (*p == '"') {
    std::string d;
    return str(d);
  }
  if (*p == '{' || *p == '[') {
    const char open = *p, close = (open == '{' ? '}' : ']');
    ++p;
    ws();
    if (*p == close) { ++p; return true; }
    for (;;) {
      if (open == '{') {
        std::string k;
        if (!str(k)) return false;
        ws();
        if (*p != ':') return false;
        ++p;
      }
      if (!skip()) return false;
      ws();
      if (*p == ',') { ++p; continue; }
      if (*p == close) { ++p; return true; }
      return false;
    }
  }
  if (!std::strncmp(p, "true", 4)) { p += 4; return true; }
  if (!std::strncmp(p, "false", 5)) { p += 5; return true; }
  if (!std::strncmp(p, "null", 4)) { p += 4; return true; }
  double d;
  return num(d);
}

bool parse_goals(const std::string & body, std::vector<SubGoal> & goals, std::string * err = nullptr) {
  Cursor c{body.c_str()};
  c.ws();
  if (*c.p != '{') return false;
  const char * ok = std::strstr(c.p, "\"ok\"");
  if (!ok) return false;
  c.p = ok + 4;
  c.ws();
  if (*c.p != ':') return false;
  ++c.p;
  c.ws();
  if (std::strncmp(c.p, "true", 4) != 0) {  // ok:false → 에러문 추출 (say 계획으로 보여줌)
    if (err) {
      const char * e = std::strstr(c.p, "\"error\"");
      if (e) {
        Cursor ec{e + 7};
        ec.ws();
        if (*ec.p == ':') { ++ec.p; ec.ws(); ec.str(*err); }
      }
    }
    return false;
  }
  const char * g = std::strstr(c.p, "\"goals\"");
  if (!g) return false;
  c.p = g + 7;
  c.ws();
  if (*c.p != ':') return false;
  ++c.p;
  c.ws();
  if (*c.p != '[') return false;
  ++c.p;
  static const std::set<std::string> allowed = {"navigate", "pick", "place", "say"};
  uint32_t step = 0;
  for (;;) {
    c.ws();
    if (*c.p == ']') return !goals.empty();
    if (*c.p != '{') return false;
    ++c.p;
    SubGoal sg;
    uint32_t got_step = 0;
    bool has_step = false, has_skill = false;
    for (;;) {
      c.ws();
      if (*c.p == '}') { ++c.p; break; }
      std::string k;
      if (!c.str(k)) return false;
      c.ws();
      if (*c.p != ':') return false;
      ++c.p;
      c.ws();
      if (k == "step") {
        double v;
        if (!c.num(v)) return false;
        got_step = static_cast<uint32_t>(v);
        has_step = true;
      } else if (k == "skill" || k == "target" || k == "relation") {
        std::string v;
        if (*c.p == '"') {
          if (!c.str(v)) return false;
        } else {  // target이 숫자면 문자열로
          double n;
          if (!c.num(n)) return false;
          v = std::to_string(static_cast<long long>(n));
        }
        if (k == "skill") { sg.skill = v; has_skill = true; }
        else if (k == "target") sg.target = v;
        else sg.relation = v;
      } else if (k == "offset_m") {
        double v;
        if (!c.num(v)) return false;
        sg.offset_m = static_cast<float>(v);
      } else {
        if (!c.skip()) return false;
      }
      c.ws();
      if (*c.p == ',') { ++c.p; continue; }
      if (*c.p == '}') continue;  // 루프 head에서 닫음
      return false;
    }
    if (!has_step || !has_skill || got_step != ++step || !allowed.count(sg.skill)) return false;
    sg.step = got_step;
    goals.push_back(sg);
    c.ws();
    if (*c.p == ',') { ++c.p; continue; }
    if (*c.p == ']') return !goals.empty();
    return false;
  }
}

}  // namespace

std::vector<SubGoal> carry_goals(const std::vector<std::string> & pucks, const std::string & dest) {
  // 퍽마다 [다가가기 → 집기 → 적재함에 싣기] → 목적지로 → 실은 수만큼 [적재함에서 꺼내기 → 목적지에 내려놓기]
  // (같은 곳에 여러 번 place하면 coordinator가 place_spacing으로 비켜 놓음)
  std::vector<SubGoal> g;
  auto add = [&g](const std::string & skill, const std::string & target, const std::string & rel, float off) {
    SubGoal s;
    s.step = static_cast<uint32_t>(g.size() + 1);
    s.skill = skill;
    s.target = target;
    s.relation = rel;
    s.offset_m = off;
    g.push_back(s);
  };
  for (const auto & p : pucks) {
    add("navigate", p, "front", 0.20f);
    add("pick", p, "", 0.0f);
    add("place", "tray", "", 0.0f);
  }
  add("navigate", dest, "front", 0.20f);
  for (size_t i = 0; i < pucks.size(); ++i) {
    add("pick", "tray", "", 0.0f);
    add("place", dest, "on", 0.0f);
  }
  return g;
}

std::vector<size_t> pick_combo(const std::vector<int> & tags, const std::vector<double> & cost, int count, int sum) {
  // 후보는 많아야 12개(퍽 전부) — 모든 부분집합을 본다
  const size_t n = std::min<size_t>(tags.size(), 20);
  std::vector<size_t> best;
  double best_cost = 0.0;
  for (uint32_t m = 0; m < (1u << n); ++m) {
    if (__builtin_popcount(m) != count) continue;
    int s = 0;
    double c = 0.0;
    std::vector<size_t> pick;
    for (size_t i = 0; i < n; ++i) {
      if (!(m & (1u << i))) continue;
      s += tags[i];
      c += i < cost.size() ? cost[i] : 0.0;
      pick.push_back(i);
    }
    if (s == sum && (best.empty() || c < best_cost)) {
      best = pick;
      best_cost = c;
    }
  }
  return best;
}

std::string detail(const perception::msg::TrackedObject & o, const std::string & key) {
  for (const auto & kv : o.details) if (kv.key == key) return kv.value;
  return "";
}

SignalPlan signal_plan(const TrackedObjectArray & objs, const perception::msg::MissionSignal & sig, double rx, double ry,
  const std::set<uint32_t> & inspected, int capacity) {
  SignalPlan r;
  auto dist = [&](const perception::msg::TrackedObject & o) {
    return std::hypot(o.pose.pose.position.x - rx, o.pose.pose.position.y - ry);
  };
  std::string dest;
  std::vector<const perception::msg::TrackedObject *> known, unknown;
  for (const auto & o : objs.objects) {
    const std::string tag = detail(o, "tag");
    if (o.class_name == "location" && tag == std::to_string(sig.dest_tag)) dest = std::to_string(o.id);
    if (o.class_name != "puck" && o.class_name != "puck_lying") continue;
    if (!tag.empty()) {
      if (o.color == sig.color) known.push_back(&o);  // 태그가 있으면 perception이 색도 태그 번호대로 맞춤
    } else if (!inspected.count(o.id) && (o.color == sig.color || o.color.empty() || o.color == "unknown")) {
      unknown.push_back(&o);
    }
  }
  if (sig.count < 1 || sig.count > capacity) {
    r.kind = "fail";
    r.why = std::to_string(sig.count) + "개는 적재함(" + std::to_string(capacity) + "칸)에 못 실음";
    return r;
  }
  std::vector<int> tags;
  std::vector<double> cost;
  for (const auto * o : known) {
    tags.push_back(std::stoi(detail(*o, "tag")));
    cost.push_back(dist(*o));
  }
  const auto combo = pick_combo(tags, cost, sig.count, sig.sum);
  if (!combo.empty()) {
    if (dest.empty()) {
      r.kind = "wait";
      r.why = "목적지 태그 " + std::to_string(sig.dest_tag) + " 아직 못 봄";
      return r;
    }
    std::vector<std::string> pucks;
    std::string list;
    for (size_t i : combo) {
      pucks.push_back(std::to_string(known[i]->id));
      list += (list.empty() ? "" : "+") + std::to_string(tags[i]);
    }
    r.kind = "carry";
    r.goals = carry_goals(pucks, dest);
    for (size_t i : combo) r.carry_tags.push_back(tags[i]);
    r.why = sig.color + " " + list + " = " + std::to_string(sig.sum) + " → 태그 " + std::to_string(sig.dest_tag);
    return r;
  }
  if (!unknown.empty()) {  // 번호를 모르는 퍽부터 본다 — 가까운 순
    std::sort(unknown.begin(), unknown.end(), [&](auto * a, auto * b) { return dist(*a) < dist(*b); });
    r.kind = "inspect";
    for (const auto * o : unknown) {
      SubGoal g;
      g.step = static_cast<uint32_t>(r.goals.size() + 1);
      g.skill = "inspect";
      g.target = std::to_string(o->id);
      r.goals.push_back(g);
      r.inspect_ids.push_back(o->id);
    }
    r.why = "번호 모르는 퍽 " + std::to_string(unknown.size()) + "개 보기";
    return r;
  }
  std::string seen;
  for (int t : tags) seen += (seen.empty() ? "" : ",") + std::to_string(t);
  r.kind = "fail";
  r.why = sig.color + " 번호 [" + seen + "] 중 " + std::to_string(sig.count) + "개로 합 " + std::to_string(sig.sum) + " 못 만듦";
  return r;
}

VerifyPlan verify_plan(const TrackedObjectArray & objs, int dest_tag, const std::vector<int> & tags, double radius,
  bool looked) {
  VerifyPlan r;
  const perception::msg::TrackedObject * dest = nullptr;
  for (const auto & o : objs.objects)
    if (o.class_name == "location" && detail(o, "tag") == std::to_string(dest_tag)) dest = &o;
  if (!dest) {
    r.kind = "wait";
    r.why = "목적지 태그 " + std::to_string(dest_tag) + " 안 보임";
    return r;
  }
  auto near = [&](const perception::msg::TrackedObject & o) {
    return std::hypot(o.pose.pose.position.x - dest->pose.pose.position.x,
                      o.pose.pose.position.y - dest->pose.pose.position.y) <= radius;
  };
  auto is_puck = [](const perception::msg::TrackedObject & o) {
    return o.class_name == "puck" || o.class_name == "puck_lying";
  };
  const std::string dest_id = std::to_string(dest->id);
  if (!looked) {  // 목적지로 가서 근처 퍽 중 번호 모르는 것만 위에서 봄 (번호 아는 건 위치만 갱신되면 됨)
    r.kind = "look";
    SubGoal nav;
    nav.step = 1;
    nav.skill = "navigate";
    nav.target = dest_id;
    nav.relation = "front";
    nav.offset_m = 0.20f;
    r.goals.push_back(nav);
    for (const auto & o : objs.objects) {
      if (!is_puck(o) || !near(o) || !detail(o, "tag").empty()) continue;
      SubGoal g;
      g.step = static_cast<uint32_t>(r.goals.size() + 1);
      g.skill = "inspect";
      g.target = std::to_string(o.id);
      r.goals.push_back(g);
    }
    r.why = "목적지 " + std::to_string(dest_tag) + " 보러 감";
    return r;
  }
  std::vector<std::string> found;  // 빠진 번호를 다른 곳에서 본 퍽
  std::string list;
  for (int t : tags) {
    const std::string tag = std::to_string(t);
    const perception::msg::TrackedObject * at_dest = nullptr, * elsewhere = nullptr;
    for (const auto & o : objs.objects) {
      if (!is_puck(o) || detail(o, "tag") != tag) continue;
      (near(o) ? at_dest : elsewhere) = &o;
    }
    if (at_dest) continue;
    r.missing.push_back(t);
    list += (list.empty() ? "" : ",") + tag;
    if (elsewhere) found.push_back(std::to_string(elsewhere->id));
  }
  if (r.missing.empty()) {
    r.kind = "ok";
    r.why = "목적지 " + std::to_string(dest_tag) + "에 " + std::to_string(tags.size()) + "개 다 있음";
  } else if (found.size() == r.missing.size()) {
    r.kind = "recarry";
    r.goals = carry_goals(found, dest_id);
    r.why = "번호 [" + list + "] 다른 곳에 있음 → 다시 옮김";
  } else {
    r.kind = "missing";
    r.why = "번호 [" + list + "] 목적지 " + std::to_string(dest_tag) + "에 없음";
  }
  return r;
}

VlmTaskPlanner::VlmTaskPlanner()
: rclcpp::Node("vlm_task_planner") {
  max_replans_ = declare_parameter<int>("max_replans", 1);
  vlm_endpoint_ = declare_parameter<std::string>("vlm_endpoint", "");
  vlm_timeout_s_ = declare_parameter<double>("vlm_timeout_s", 120.0);
  tray_capacity_ = declare_parameter<int>("tray_capacity", 4);
  verify_period_s_ = declare_parameter<double>("verify_period_s", 60.0);
  verify_radius_m_ = declare_parameter<double>("verify_radius_m", 0.20);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  goals_pub_ = create_publisher<SubGoalArray>("/task/sub_goals", rclcpp::QoS(10).reliable().transient_local());
  cmd_sub_ = create_subscription<std_msgs::msg::String>("/cmd/user_command", rclcpp::QoS(10).reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_command("명령", m->data); });
  // 장소에 붙은 바코드(작업 지시 문자열, 발제 P2-01 …) · QR — perception이 읽음. 명령과 똑같이 계획 (VLM이 해석)
  // 템플릿(VLM 없음): 가까운 퍽 1개를 장소(location)로
  qr_sub_ = create_subscription<std_msgs::msg::String>("/mission/qr", rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr m) { on_command("QR 문제", m->data); });
  objects_sub_ = create_subscription<TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](TrackedObjectArray::ConstSharedPtr m) {
      std::string sig;
      for (const auto & o : m->objects) sig += std::to_string(o.id) + ":" + o.class_name + ";";
      bool changed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        objects_ = m;
        changed = (sig != last_objs_sig_);
        if (changed) last_objs_sig_ = sig;
      }
      // 물체 집합이 바뀔 때만 재시도 — 같은 목록(빈 목록 포함)으로 매번 VLM을 때리면 초당 요청 폭주
      if (pending_ && changed) make_plan();  // 물체 목록보다 명령이 먼저 왔으면 지금 계획
    });
  signal_sub_ = create_subscription<perception::msg::MissionSignal>("/mission/signal", rclcpp::QoS(1).reliable().transient_local(),
    [this](perception::msg::MissionSignal::ConstSharedPtr m) {
      RCLCPP_INFO(get_logger(), "신호등 미션: %s %d개 · 합 %d · 목적지 태그 %d", m->color.c_str(), m->count, m->sum, m->dest_tag);
      replans_ = 0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        signal_ = *m;
        replan_ = ReplanInfo{};
      }
      signal_active_ = true;
      inspected_.clear();
      stored_dest_ = -1;  // 새 미션 — 이전 수납 확인 대상은 버림
      verify_note_ = "없음";
      pending_ = true;
      make_plan();
    });
  mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode",
    rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::String::ConstSharedPtr m) { mode_ = m->data; });
  if (verify_period_s_ > 0.0)
    verify_timer_ = create_wall_timer(std::chrono::duration<double>(verify_period_s_), [this]() { verify_tick(); });
  status_sub_ = create_subscription<TaskStatus>("/task/status", rclcpp::QoS(10).reliable().transient_local(),
    [this](TaskStatus::ConstSharedPtr m) { on_status(*m); });
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);  // VLM은 최신 장면만 필요
  for (const std::string cam : {"pole_front_cam", "gripper_cam"}) {
    image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
      "/" + cam + "/color/image_raw", sensor_qos,
      [this, cam](sensor_msgs::msg::Image::ConstSharedPtr m) {
        // 복사본으로 보관 — 받은 메시지를 그대로 들고 있으면 (Lyrical: data가 미들웨어 공유메모리 버퍼를 가리킬 수 있음)
        // 그 버퍼가 반납되지 않아 카메라(fake_camera·realsense) 발행이 막혔음 (실측: 이 노드를 끄면 바로 풀림)
        auto copy = std::make_shared<sensor_msgs::msg::Image>(*m);
        std::lock_guard<std::mutex> lock(mutex_);
        images_[cam] = copy;  // 최신 1장만 유지 (VLM 호출 때 꺼냄)
      }));
  }
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    if (pending_) s.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "계획 보류 — 대상 물체 대기");
    else s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, plan_id_ ? "계획 " + std::to_string(plan_id_.load()) : "명령 대기");
    s.add("plan_id", plan_id_.load());
    s.add("재계획", std::to_string(replans_) + "/" + std::to_string(max_replans_));
    s.add("보이는 물체", objects_ ? objects_->objects.size() : 0);
    s.add("vlm", vlm_endpoint_.empty() ? std::string("template") : vlm_endpoint_);
    s.add("vlm 비행", vlm_busy_.load() ? "예" : "아니오");
    s.add("신호등 미션", signal_active_ ? signal_note_ : "없음");
    s.add("수납 확인", verify_note_);
  });
}

void VlmTaskPlanner::on_status(const TaskStatus & s) {
  if (s.state == TaskStatus::RUNNING || s.plan_id != plan_id_.load()) return;
  done_plan_ = s.plan_id;
  if (s.state == TaskStatus::SUCCEEDED) {
    if (plan_kind_ == "inspect" && signal_active_) {  // 번호를 봤으니 다시 판단 (조합이 생겼으면 옮기기)
      pending_ = true;
      make_plan();
    } else if (plan_kind_ == "carry" && signal_active_) {  // 옮기기 끝 → 수납 확인 대상으로 두고 바로 한 번 확인
      signal_active_ = false;
      signal_note_ = "끝";
      stored_dest_ = signal_.dest_tag;
      stored_tags_ = carry_tags_;
      make_verify_plan(false);
    } else if (plan_kind_ == "verify") {
      make_verify_plan(true);
    } else if (plan_kind_ == "recarry") {
      make_verify_plan(false);
    }
    return;
  }
  if (plan_kind_ == "verify" || plan_kind_ == "recarry") {  // 확인 계획은 재계획 없이 다음 주기에 다시
    verify_note_ = "확인 실패(" + s.reason + ") — 다음 주기에 다시";
    RCLCPP_WARN(get_logger(), "수납 확인 계획 실패: %s", s.reason.c_str());
    return;
  }
  if (s.reason.rfind("manual", 0) == 0) return;  // 수동 개입으로 멈춘 것 — 사람이 다시 명령할 때까지
  if (s.reason.rfind("explore", 0) == 0) return;  // 자동 탐사(EXPLORE)로 멈춘 것 — 탐사 끝나고 사람이 다시 명령할 때까지
  if (s.reason.rfind("cancelled", 0) == 0) return;  // 명시적 정지 (sim 정지 버튼) — 재계획 없음
  if (s.reason.rfind("estop", 0) == 0) return;  // 비상정지 — 사람이 풀고 다시 명령할 때까지
  if (replans_ >= max_replans_) return;  // coordinator가 알리고 정지한다
  ++replans_;
  RCLCPP_WARN(get_logger(), "step %u 실패(%s) → 재계획 %d/%d", s.step, s.reason.c_str(), replans_, max_replans_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    replan_ = ReplanInfo{s.plan_id, s.step, s.reason, true};
  }
  pending_ = true;
  make_plan();  // VLM이면 실패 이유를 프롬프트에 넣어 다른 접근 방향으로
}

std::string VlmTaskPlanner::find_in(const TrackedObjectArray & objs, const std::string & cls) {
  for (const auto & o : objs.objects) if (o.class_name == cls) return std::to_string(o.id);
  return "";
}

void VlmTaskPlanner::make_plan() {
  if (signal_active_) {  // 신호등 암호 미션은 규칙으로
    make_signal_plan();
    return;
  }
  TrackedObjectArray::ConstSharedPtr objs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objs = objects_;
  }
  if (vlm_endpoint_.empty()) {  // 템플릿(오프라인)은 퍽·목표 id가 있어야 짜진다
    if (!objs) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/tracked_objects 대기 — 계획 보류");
      return;
    }
    if (find_in(*objs, "puck").empty() || find_in(*objs, "location").empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/tracked_objects에 퍽·장소 태그가 없음 — 계획 보류");
      return;
    }
    std::string cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = cmd_;
    }
    make_template(*objs, cmd);
    return;
  }
  pending_ = true;  // VLM이면 인지 유무와 무관하게 사용자 명령 그대로 전달 (빈 물체+이미지로 판단은 모델 몫)
  try_vlm_async();
}

void VlmTaskPlanner::on_command(const char * source, const std::string & text) {
  RCLCPP_INFO(get_logger(), "%s: \"%s\"", source, text.c_str());
  replans_ = 0;
  stored_dest_ = -1;  // 새 명령이 퍽을 옮길 수 있음 — 이전 수납 확인 대상은 버림
  verify_note_ = "없음";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = text;
    replan_ = ReplanInfo{};
  }
  pending_ = true;
  make_plan();
}

void VlmTaskPlanner::make_template(const TrackedObjectArray & objs, const std::string & cmd) {
  // cmd는 부르는 쪽이 넘김 — VLM 실패 폴백은 mutex_를 잡은 채 부르므로 여기서 잠그면 교착 (실측: 그 뒤 명령을 못 받았음)
  // VLM 없을 때의 최소 계획: 퍽 1개를 장소(location)로 — 지시 문자열 해석은 VLM 몫
  (void)cmd;
  const auto puck = find_in(objs, "puck"), dest = find_in(objs, "location");
  if (puck.empty() || dest.empty()) return;  // 물체 사라짐 — 보류 유지
  pending_ = false;
  plan_kind_ = "template";
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = plan_id_.fetch_add(1) + 1;
  plan.goals.push_back(goal(1, "navigate", puck, "front", 0.20f));
  plan.goals.push_back(goal(2, "pick", puck, "", 0.0f));
  plan.goals.push_back(goal(3, "navigate", dest, "front", 0.20f));
  plan.goals.push_back(goal(4, "place", dest, "on", 0.0f));
  goals_pub_->publish(plan);
  RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps) [template]", plan.plan_id, plan.goals.size());
}

void VlmTaskPlanner::publish_plan(std::vector<SubGoal> goals, const char * how) {
  SubGoalArray plan;
  plan.header.stamp = now();
  plan.plan_id = plan_id_.fetch_add(1) + 1;
  plan.goals = std::move(goals);
  goals_pub_->publish(plan);
  RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps) [%s]", plan.plan_id, plan.goals.size(), how);
}

void VlmTaskPlanner::make_signal_plan() {
  TrackedObjectArray::ConstSharedPtr objs;
  perception::msg::MissionSignal sig;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objs = objects_;
    sig = signal_;
  }
  if (!objs) return;  // 물체 목록이 오면 다시 (pending_)
  const RobotPose robot = robot_pose();
  const auto r = signal_plan(*objs, sig, robot.x, robot.y, inspected_, tray_capacity_);
  signal_note_ = r.kind + " — " + r.why;
  if (r.kind == "wait") {  // 목적지 태그를 볼 때까지 (물체 목록이 바뀌면 다시)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "신호등 미션 보류: %s", r.why.c_str());
    return;
  }
  pending_ = false;
  RCLCPP_INFO(get_logger(), "신호등 미션 %s: %s", r.kind.c_str(), r.why.c_str());
  if (r.kind == "fail") {
    signal_active_ = false;
    plan_kind_ = "say";
    publish_plan({goal(1, "say", "신호등 미션: " + r.why, "", 0.0f)}, "signal");
    return;
  }
  inspected_.insert(r.inspect_ids.begin(), r.inspect_ids.end());
  carry_tags_ = r.carry_tags;
  plan_kind_ = r.kind;
  publish_plan(r.goals, "signal");
}

void VlmTaskPlanner::verify_tick() {
  // 하던 계획이 없고(끝났고) AUTO일 때만 — 수동·탐사 중이거나 다른 계획 중이면 다음 주기에
  if (stored_dest_ < 0 || mode_ != "AUTO" || signal_active_ || pending_ || done_plan_.load() != plan_id_.load()) return;
  make_verify_plan(false);
}

void VlmTaskPlanner::make_verify_plan(bool looked) {
  if (stored_dest_ < 0) return;
  TrackedObjectArray::ConstSharedPtr objs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objs = objects_;
  }
  if (!objs) return;
  const auto r = verify_plan(*objs, stored_dest_, stored_tags_, verify_radius_m_, looked);
  verify_note_ = r.kind + " — " + r.why;
  RCLCPP_INFO(get_logger(), "수납 확인 %s: %s", r.kind.c_str(), r.why.c_str());
  if (r.kind == "wait" || r.kind == "ok") return;
  if (r.kind == "missing") {
    plan_kind_ = "say";
    publish_plan({goal(1, "say", "수납 확인: " + r.why, "", 0.0f)}, "verify");
    return;
  }
  plan_kind_ = r.kind == "look" ? "verify" : "recarry";
  publish_plan(r.goals, "verify");
}

void VlmTaskPlanner::try_vlm_async() {
  bool expected = false;
  if (!vlm_busy_.compare_exchange_strong(expected, true)) return;  // 비행 중 — 끝난 워커가 최신으로 다시 시도
  const uint64_t epoch = epoch_.fetch_add(1) + 1;
  std::string cmd;
  TrackedObjectArray objs;
  ReplanInfo rp;
  std::vector<ImageFrame> imgs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd = cmd_;
    if (objects_) objs = *objects_;
    rp = replan_;
    for (const auto & cam : {"pole_front_cam", "gripper_cam"}) {
      auto it = images_.find(cam);
      if (it != images_.end() && it->second) imgs.push_back({cam, it->second});
    }
  }
  const RobotPose robot = robot_pose();  // 보낼 때 로봇 위치
  std::thread([this, epoch, cmd, objs, rp, imgs, robot]() {
    std::vector<SubGoal> goals;
    std::string vlm_err;
    const bool ok =
      fetch_goals(vlm_endpoint_, plan_request(cmd, objs, rp, imgs, robot), goals, vlm_timeout_s_, &vlm_err);
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stale = (epoch != epoch_.load());
      // 발행 결정까지 잠금 — make_plan의 epoch 증가와 직렬화 (낡은 계획 이중 발행 방지)
      if (!stale) {
        vlm_busy_ = false;
        if (!ok && !vlm_err.empty()) {  // VLM 업무 거절(보류 등) — say로 화면에 보여줌
          pending_ = false;
          SubGoalArray plan;
          plan.header.stamp = now();
          plan.plan_id = plan_id_.fetch_add(1) + 1;
          plan.goals.push_back(goal(1, "say", "VLM: " + vlm_err, "", 0.0f));
          goals_pub_->publish(plan);
          RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (say, VLM 거절) [vlm]", plan.plan_id);
        } else if (!ok) {
          RCLCPP_WARN(get_logger(), "VLM 실패 — 템플릿 폴백");
          make_template(objs, cmd);
        } else {
          pending_ = false;
          SubGoalArray plan;
          plan.header.stamp = now();
          plan.plan_id = plan_id_.fetch_add(1) + 1;
          plan.goals = goals;
          goals_pub_->publish(plan);
          RCLCPP_INFO(get_logger(), "/sub_goals plan=%u (%zu steps) [vlm]", plan.plan_id, plan.goals.size());
        }
      } else {
        vlm_busy_ = false;
      }
    }
    if (stale && pending_.load()) try_vlm_async();  // 최신 스냅샷으로 다시
  }).detach();
}

// sensor_msgs/Image(rgb8) → 320px JPEG base64 (ai-service images[]용)
std::string VlmTaskPlanner::encode_jpeg_base64(const sensor_msgs::msg::Image & msg) {
  if (msg.encoding != "rgb8" || msg.width == 0 || msg.height == 0) return "";
  const size_t expect = static_cast<size_t>(msg.width) * msg.height * 3;
  if (msg.data.size() < expect) return "";
  cv::Mat rgb(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
  cv::Mat small, bgr;
  const double scale = 320.0 / msg.width;
  cv::resize(rgb, small, cv::Size(), scale, scale, cv::INTER_LINEAR);
  // imencode는 BGR로 읽음 — rgb8을 그대로 넣으면 빨강↔파랑이 뒤바뀐 JPEG가 됨 (실측: 모래 바닥·주황 퍽이 파랗게 → VLM이 색을 틀림)
  cv::cvtColor(small, bgr, cv::COLOR_RGB2BGR);
  std::vector<uint8_t> jpeg;
  if (!cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, 70})) return "";
  if (jpeg.size() > 500 * 1024) return "";  // ai-service 장당 제한
  return base64_encode(jpeg);
}

RobotPose VlmTaskPlanner::robot_pose() const {
  RobotPose r;
  try {
    const auto t = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    const auto & q = t.transform.rotation;
    r.x = t.transform.translation.x;
    r.y = t.transform.translation.y;
    r.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    r.ok = true;
  } catch (const std::exception &) {
  }
  return r;
}

std::string VlmTaskPlanner::plan_request(
  const std::string & cmd, const TrackedObjectArray & objs, const ReplanInfo & rp,
  const std::vector<ImageFrame> & imgs, const RobotPose & robot) {
  std::string o = "{\"command\": \"" + json_escape(cmd) + "\", \"objects\": [";
  bool first = true;
  for (const auto & obj : objs.objects) {
    if (!first) o += ", ";
    first = false;
    char pos[128], size[128];
    snprintf(pos, sizeof(pos), "[%.3f, %.3f, %.3f]",
      obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z);
    snprintf(size, sizeof(size), "[%.3f, %.3f, %.3f]", obj.size.x, obj.size.y, obj.size.z);
    char conf[16];
    snprintf(conf, sizeof(conf), "%.2f", obj.confidence);
    const std::string cam = obj.cameras.empty() ? std::string() : obj.cameras.back();  // 마지막으로 본 카메라 (px의 카메라)
    char px[64];  // ai-service 형식 {카메라: [u, v]} — 예전 [u, v]만 보내서 images.py가 번호를 못 찍었음
    snprintf(px, sizeof(px), "[%d, %d]", obj.px[0], obj.px[1]);
    o += std::string("{\"id\": ") + std::to_string(obj.id) + ", \"class_name\": \"" +
      json_escape(obj.class_name) + "\", \"position\": " + pos + ", \"size\": " + size +
      ", \"confidence\": " + conf + ", \"color\": \"" + json_escape(obj.color) +
      "\", \"camera\": \"" + json_escape(cam) + "\", \"px\": " +
      (cam.empty() ? std::string("{}") : "{\"" + json_escape(cam) + "\": " + px + "}");
    for (const char * key : {"tag", "content", "posture"}) {  // 퍽 번호 · 장소 태그 · 바코드 작업 지시 · 자세
      const std::string v = detail(obj, key);
      if (!v.empty()) o += std::string(", \"") + key + "\": \"" + json_escape(v) + "\"";
    }
    o += "}";
  }
  o += "]";
  if (robot.ok) {
    char rb[160];
    snprintf(rb, sizeof(rb), ", \"robot\": {\"x\": %.3f, \"y\": %.3f, \"yaw\": %.3f}", robot.x, robot.y, robot.yaw);
    o += rb;
  }
  if (rp.active) {
    o += ", \"replan\": {\"plan_id\": " + std::to_string(rp.plan_id) + ", \"failed_step\": " +
      std::to_string(rp.step) + ", \"reason\": \"" + json_escape(rp.reason) + "\"}";
  }
  o += ", \"images\": [";
  bool ifirst = true;
  for (const auto & im : imgs) {
    if (!im.msg) continue;
    const std::string b64 = encode_jpeg_base64(*im.msg);
    if (b64.empty()) continue;
    if (!ifirst) o += ", ";
    ifirst = false;
    o += "{\"camera\": \"" + im.camera + "\", \"format\": \"jpeg\", \"data_b64\": \"" + b64 + "\"}";
  }
  o += "]";
  return o + "}";
}

bool VlmTaskPlanner::fetch_goals(const std::string & endpoint, const std::string & body,
  std::vector<SubGoal> & goals, double timeout_s, std::string * err) {
  std::string host;
  int port = 0;
  if (!split_endpoint(endpoint, host, port)) return false;
  std::string resp;
  return http_post(host, port, body, resp, timeout_s) && parse_goals(resp, goals, err);
}

SubGoal VlmTaskPlanner::goal(uint32_t step, const std::string & skill, const std::string & target,
  const std::string & relation, float offset) {
  SubGoal g;
  g.step = step;
  g.skill = skill;
  g.target = target;
  g.relation = relation;
  g.offset_m = offset;
  return g;
}

}  // namespace plan
