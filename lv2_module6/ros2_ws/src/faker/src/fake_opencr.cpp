// fake_opencr 구현 — 설명·선언은 include/faker/fake_opencr.hpp, main은 fake_opencr_main.cpp
// 물리 상수 — 로봇 외곽 반경 ROBOT_R은 simulator/opencr.py와 같게 (퍽 밀기 판정)
#include "faker/fake_opencr.hpp"

#include <array>
#include <cstdio>

namespace faker {

namespace {
constexpr double ROBOT_R = 0.18;
constexpr double WHEEL_R = 0.033;
constexpr double WHEEL_SEP = 0.287;
constexpr double ACCEL_MAX = 8.0;
constexpr double JOINT_VEL_MAX = 4.8;
constexpr double GRIPPER_VEL_MAX = 0.05;
constexpr double COMMAND_TIMEOUT = 0.5;
constexpr double STATE_PERIOD = 0.025;  // 펌웨어 STATE_PERIOD_MS (25ms) — S는 부팅 직후부터 C와 무관하게 항상
}  // namespace

FakeOpenCR::FakeOpenCR() : rclcpp::Node("fake_opencr") {
  serial_in_ = declare_parameter<std::string>("serial_in", "/debug/input/hardware/serial");
  serial_out_ = declare_parameter<std::string>("serial_out", "/debug/output/hardware/serial");
  const auto map_yaml = declare_parameter<std::string>("map_yaml", "");
  x_ = declare_parameter<double>("start_x", 0.0);
  y_ = declare_parameter<double>("start_y", 0.0);
  yaw_ = declare_parameter<double>("start_yaw", 0.0);
  slip_ = declare_parameter<double>("slip", 0.0);
  friction_ = declare_parameter<double>("friction", 1.0);
  noise_ = declare_parameter<double>("noise", 0.01);
  comm_down_ = declare_parameter<bool>("comm_down", false);
  arm_grabbed_ = declare_parameter<bool>("arm_grabbed", false);
  if (!map_yaml.empty() && !load_map(map_yaml)) {
    RCLCPP_WARN(get_logger(), "지도 로드 실패 (%s) — 충돌 없이 동작", map_yaml.c_str());
  }
  cmd_stream_.open(serial_out_, std::ios::in);
  if (cmd_stream_) cmd_stream_.seekg(0, std::ios::end);  // 지금부터 붙는 C만
  last_tick_ = std::chrono::steady_clock::now();
  last_cmd_ = last_tick_ - std::chrono::hours(24);
  last_state_ = last_cmd_;
  timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
  // 시뮬레이터 ↔ OpenCR 대역: 진짜 위치·막힘을 내보내고, 환경(파라미터)·위치(토픽)는 요청으로 받는다
  gt_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/ground_truth", 10);
  bumped_pub_ = create_publisher<std_msgs::msg::Bool>("~/bumped", 10);
  pub_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
    geometry_msgs::msg::PoseStamped p;
    std_msgs::msg::Bool b;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      p.pose.position.x = x_;
      p.pose.position.y = y_;
      p.pose.orientation.z = std::sin(yaw_ / 2);
      p.pose.orientation.w = std::cos(yaw_ / 2);
      b.data = bumped_;
    }
    p.header.stamp = now();
    p.header.frame_id = "map";
    gt_pub_->publish(p);
    bumped_pub_->publish(b);
  });
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/cmd/fake_opencr/set_pose", 10,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      x_ = m->pose.position.x;
      y_ = m->pose.position.y;
      yaw_ = 2.0 * std::atan2(m->pose.orientation.z, m->pose.orientation.w);  // 평면 가정
      wheel_vel_ = {0.0, 0.0};
      yaw_rate_ = 0.0;
      RCLCPP_INFO(get_logger(), "위치 옮김 (%.2f, %.2f, %.2f rad)", x_, y_, yaw_);
    });
  param_cb_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & p : ps) {
      if (p.get_name() == "slip") slip_ = p.as_double();
      else if (p.get_name() == "friction") friction_ = p.as_double();
      else if (p.get_name() == "noise") noise_ = p.as_double();
      else if (p.get_name() == "comm_down") comm_down_ = p.as_bool();
      else if (p.get_name() == "arm_grabbed") arm_grabbed_ = p.as_bool();
      else continue;
      RCLCPP_INFO(get_logger(), "환경 %s = %s", p.get_name().c_str(), p.value_to_string().c_str());
    }
  });
  RCLCPP_INFO(get_logger(), "OpenCR 대역: C %s → S %s (지도 %s)",
    serial_out_.c_str(), serial_in_.c_str(), map_.empty() ? "없음" : "충돌 켬");
}

bool FakeOpenCR::load_map(const std::string & yaml_path) {
  // yaml에서 image·resolution·origin만 읽는다 (nav2 trinary — 0에 가까울수록 점유)
  std::string image;
  {
    std::ifstream yf(yaml_path);
    if (!yf) return false;
    const auto dir = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
    for (std::string line; std::getline(yf, line);) {
      const auto val = [&](const char * key) -> std::string {
        if (line.compare(0, std::strlen(key), key) != 0) return "";
        const auto p = line.find(':');
        return p == std::string::npos ? "" : line.substr(p + 1);
      };
      if (auto v = val("image"); !v.empty()) {
        v.erase(0, v.find_first_not_of(" \t\"'"));
        v.erase(v.find_last_not_of(" \t\r\n\"'") + 1);
        image = (v[0] == '/' ? v : dir + v);
      }
      if (auto v = val("resolution"); !v.empty()) map_res_ = std::stod(v);
      if (auto v = val("origin"); !v.empty()) {
        std::sscanf(v.c_str(), "[%lf, %lf", &map_ox_, &map_oy_);
      }
    }
    if (image.empty()) return false;
  }
  std::ifstream pf(image, std::ios::binary);
  if (!pf) return false;
  const auto next_token = [&]() -> std::string {  // PGM 헤더의 '#' 주석을 건너뜀
    std::string tok;
    while (pf >> tok) {
      if (!tok.empty() && tok[0] == '#') {
        std::string rest;
        std::getline(pf, rest);
        continue;
      }
      return tok;
    }
    return "";
  };
  const std::string magic = next_token();
  map_w_ = std::stoi(next_token());
  map_h_ = std::stoi(next_token());
  next_token();  // maxval
  pf.get();  // 헤더 끝 개행 한 바이트
  if (magic != "P5" || map_w_ <= 0 || map_h_ <= 0) return false;
  map_.resize(static_cast<size_t>(map_w_) * map_h_);
  pf.read(reinterpret_cast<char *>(map_.data()), static_cast<std::streamsize>(map_.size()));
  return pf.good();
}

bool FakeOpenCR::occupied(double x, double y) const {
  if (map_.empty()) return false;
  const int col = static_cast<int>((x - map_ox_) / map_res_);
  const int row = static_cast<int>((map_oy_ + map_h_ * map_res_ - y) / map_res_);  // 0행 = 위 = 큰 y
  if (col < 0 || col >= map_w_ || row < 0 || row >= map_h_) return true;  // 지도 밖 = 벽
  return map_[static_cast<size_t>(row) * map_w_ + col] < 128;
}

bool FakeOpenCR::collide(double x, double y, double r) const {
  if (map_.empty()) return false;
  if (occupied(x, y)) return true;
  for (int k = 0; k < 8; ++k) {
    const double a = k * M_PI / 4;
    if (occupied(x + r * std::cos(a), y + r * std::sin(a))) return true;
  }
  return false;
}

bool FakeOpenCR::read_command(std::array<double, 8> & cmd) {
  if (!cmd_stream_) {
    cmd_stream_.clear();
    cmd_stream_.open(serial_out_, std::ios::in);
    if (!cmd_stream_) return false;
    cmd_stream_.seekg(0, std::ios::end);
    return false;
  }
  // 펌웨어처럼 줄마다 차례로 검사·적용 — 형식이 틀린 줄은 버리고 이전 명령 유지 (결과 = 마지막 유효 줄)
  bool got = false;
  std::string line;
  while (std::getline(cmd_stream_, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::array<double, 8> v{};
    if (parse_command(line, v)) {
      cmd = v;
      got = true;
    }
  }
  cmd_stream_.clear();  // EOF 플래그 해제 — 다음 tick에 이어 읽음
  return got;
}

bool FakeOpenCR::parse_command(const std::string & line, std::array<double, 8> & v) {
  // 펌웨어 parse_fields와 같은 규칙: "C," + 정확히 8값, 각 값은 끝까지 숫자이고 유한, 마지막 값 뒤에 아무것도 없음
  if (line.compare(0, 2, "C,") != 0) return false;
  const char * text = line.c_str() + 2;
  for (int i = 0; i < 8; ++i) {
    char * end = nullptr;
    v[i] = std::strtod(text, &end);
    if (end == text || !std::isfinite(v[i])) return false;
    if (i + 1 < 8) {
      if (*end != ',') return false;
      text = end + 1;
    } else if (*end != '\0') {
      return false;
    }
  }
  return true;
}

void FakeOpenCR::write_state() {
  std::lock_guard<std::mutex> lock(mutex_);
  char buf[32];
  std::string line = "S";
  const auto push = [&](double v) {
    std::snprintf(buf, sizeof(buf), ",%.5f", v);
    line += buf;
  };
  for (int i = 0; i < 2; ++i) {
    push(wheel_pos_[i]);
    push(wheel_vel_[i]);
  }
  std::normal_distribution<double> gauss(0.0, noise_);
  for (int j = 0; j < 6; ++j) {
    const double current = (arm_grabbed_ ? 1.6 : 0.15 + 0.05 * std::abs(joint_vel_[j]));
    push(joint_pos_[j]);
    push(joint_vel_[j]);
    push(current);
  }
  push(0.0);
  push(0.0);
  push(std::sin(yaw_ / 2));
  push(std::cos(yaw_ / 2));
  push(0.0);
  push(0.0);
  push(yaw_rate_ + gauss(rng_));
  push(0.0);
  push(0.0);
  push(9.81);
  line += '\n';
  std::ofstream out(serial_in_, std::ios::app);
  if (out) {
    out << line;
    out.flush();
  }
}

void FakeOpenCR::tick() {
  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_tick_).count();
  last_tick_ = now;
  if (dt <= 0.0) return;
  if (dt > 0.05) dt = 0.05;

  if (!comm_down_) {
    std::array<double, 8> cmd{};
    if (read_command(cmd)) {
      std::lock_guard<std::mutex> lock(mutex_);
      wheel_cmd_ = {cmd[0], cmd[1]};
      for (int j = 0; j < 6; ++j) joint_cmd_[j] = cmd[2 + j];
      last_cmd_ = now;
      watchdog_tripped_ = false;
      ++cmds_;
    }
  }
  const double stale = std::chrono::duration<double>(now - last_cmd_).count();
  if (!watchdog_tripped_ && stale >= COMMAND_TIMEOUT) {  // 펌웨어 check_command_timeout — 한 번만
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_cmd_ = {0.0, 0.0};
    joint_cmd_ = joint_pos_;  // 팔·그리퍼는 지금 자세 유지 (hold_present_pose)
    watchdog_tripped_ = true;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double accel = ACCEL_MAX * friction_ * dt;
    for (int i = 0; i < 2; ++i) {
      const double target = wheel_cmd_[i];  // watchdog이 끊기면 0으로 바꿔 둠
      wheel_vel_[i] += std::max(-accel, std::min(accel, target - wheel_vel_[i]));
      wheel_pos_[i] += wheel_vel_[i] * dt;
    }
    const double ground[2] = {wheel_vel_[0] * WHEEL_R * (1.0 - slip_), wheel_vel_[1] * WHEEL_R * (1.0 - slip_)};
    const double v = (ground[0] + ground[1]) / 2;
    const double w = (ground[1] - ground[0]) / WHEEL_SEP;
    const double nx = x_ + v * std::cos(yaw_) * dt;
    const double ny = y_ + v * std::sin(yaw_) * dt;
    bumped_ = collide(nx, ny, ROBOT_R) && !collide(x_, y_, ROBOT_R);
    if (!bumped_) {
      x_ = nx;
      y_ = ny;
    }
    yaw_ = std::fmod(yaw_ + w * dt + M_PI, 2 * M_PI);
    if (yaw_ < 0) yaw_ += 2 * M_PI;
    yaw_ -= M_PI;
    yaw_rate_ = w;
    for (int j = 0; j < 6; ++j) {
      if (arm_grabbed_ && j != 5) {
        joint_vel_[j] = 0.0;
        continue;
      }
      const double vmax = (j == 5 ? GRIPPER_VEL_MAX : JOINT_VEL_MAX) * dt;
      const double delta = std::max(-vmax, std::min(vmax, joint_cmd_[j] - joint_pos_[j]));
      joint_pos_[j] += delta;
      joint_vel_[j] = dt > 0 ? delta / dt : 0.0;
    }
  }
  if (!comm_down_ && std::chrono::duration<double>(now - last_state_).count() >= STATE_PERIOD) {
    write_state();
    last_state_ = now;
  }
}

}  // namespace faker
