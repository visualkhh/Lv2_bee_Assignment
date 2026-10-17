// beebot_system_hardware 구현 — 설명·선언은 include/control/beebot_system_hardware.hpp
#include "control/beebot_system_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace control {

// 프로토콜의 관절 순서 (URDF에 이 이름들이 있어야 한다)
static const std::array<const char *, 2> WHEELS = {"wheel_left_joint", "wheel_right_joint"};
static const std::array<const char *, 6> ARM = {"joint1", "joint2", "joint3", "joint4", "joint5", "gripper_left_joint"};
static const std::array<const char *, 10> IMU_FIELDS = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
static constexpr size_t STATE_VALUES = WHEELS.size() * 2 + ARM.size() * 3 + IMU_FIELDS.size();  // 32

CallbackReturn BeebotSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  const auto & hp = info_.hardware_parameters;
  mock_ = hp.count("mock") && (hp.at("mock") == "true" || hp.at("mock") == "True");
  mock_effort_ = hp.count("mock_effort") ? std::stod(hp.at("mock_effort")) : 0.0;
  port_ = hp.count("port") ? hp.at("port") : "/dev/opencr";
  imu_ = info_.sensors.empty() ? "imu" : info_.sensors.front().name;
  if (!mock_) {  // 실기 프로토콜은 관절 순서가 고정 — URDF와 다르면 시작하지 않는다
    const auto in_urdf = [this](const char * name) {
      for (const auto & j : info_.joints) if (j.name == name) return true;
      return false;
    };
    for (const auto * name : WHEELS) if (!in_urdf(name)) return missing(name);
    for (const auto * name : ARM) if (!in_urdf(name)) return missing(name);
  }
  RCLCPP_INFO(get_logger(), "관절 %zu개 · 센서 %zu개 · %s", info_.joints.size(), info_.sensors.size(),
    mock_ ? "mock (명령 → 상태 되돌림)" : ("실기 OpenCR " + port_).c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn BeebotSystemHardware::on_configure(const rclcpp_lifecycle::State &) {
  for (const auto & j : info_.joints) {
    for (const auto & s : j.state_interfaces) set_state(j.name + "/" + s.name, 0.0);
  }
  for (const auto & s : info_.sensors) {
    for (const auto & i : s.state_interfaces) set_state(s.name + "/" + i.name, 0.0);
    set_state(s.name + "/orientation.w", 1.0);
    set_state(s.name + "/linear_acceleration.z", 9.81);
  }
  if (!diag_ && get_node()) {
    diag_ = std::make_unique<diagnostic_updater::Updater>(get_node());
    diag_->setHardwareID("beebot");
    diag_->add("OpenCR 연결", [this](diagnostic_updater::DiagnosticStatusWrapper & s) { report(s); });
  }
  return mock_ || open_port() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn BeebotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  close_port();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BeebotSystemHardware::on_shutdown(const rclcpp_lifecycle::State &) {
  close_port();
  return CallbackReturn::SUCCESS;
}

CallbackReturn BeebotSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
  // 활성화 순간 명령 = 현재 상태 (첫 주기에 엉뚱한 곳으로 튀지 않게)
  for (const auto & j : info_.joints) {
    for (const auto & c : j.command_interfaces) {
      const auto key = j.name + "/" + c.name;
      set_command(key, c.name == hardware_interface::HW_IF_POSITION ? get_state<double>(key) : 0.0);
    }
  }
  return CallbackReturn::SUCCESS;
}

return_type BeebotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period) {
  return mock_ ? read_mock(period.seconds()) : read_serial();
}

return_type BeebotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
  return mock_ ? return_type::OK : write_serial();  // mock은 read()에서 명령을 상태로 되돌린다
}

void BeebotSystemHardware::report(diagnostic_updater::DiagnosticStatusWrapper & s) {
  using diagnostic_msgs::msg::DiagnosticStatus;
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_state_.load()).count();
  if (mock_) s.summary(DiagnosticStatus::OK, "mock — 명령을 상태로 되돌림");
  else if (fd_ < 0) s.summary(DiagnosticStatus::ERROR, "포트 닫힘: " + port_);
  else if (age > 0.5) s.summary(DiagnosticStatus::ERROR, "OpenCR 상태(S) 수신 없음");
  else s.summary(DiagnosticStatus::OK, "연결됨 " + port_);
  s.add("mock", mock_);
  s.add("port", port_);
  s.add("마지막 S [s 전]", mock_ ? 0.0 : age);
  s.add("보낸 C 줄", sent_.load());
  s.add("받은 S 줄", received_.load());
  s.add("해석 실패", bad_.load());
}

CallbackReturn BeebotSystemHardware::missing(const char * name) {
  RCLCPP_ERROR(get_logger(), "URDF <ros2_control>에 %s 없음 — 실기 프로토콜 순서와 다름", name);
  return CallbackReturn::ERROR;
}

return_type BeebotSystemHardware::read_mock(double dt) {
  for (const auto & j : info_.joints) {
    const auto pos = j.name + "/" + hardware_interface::HW_IF_POSITION;
    const auto vel = j.name + "/" + hardware_interface::HW_IF_VELOCITY;
    for (const auto & c : j.command_interfaces) {
      const double cmd = get_command<double>(j.name + "/" + c.name);
      if (std::isnan(cmd)) continue;
      if (c.name == hardware_interface::HW_IF_VELOCITY) {  // 바퀴: 속도 명령 → 위치 적분
        set_state(vel, cmd);
        set_state(pos, get_state<double>(pos) + cmd * dt);
      } else if (c.name == hardware_interface::HW_IF_POSITION) {  // 팔: 목표각에 바로 도달
        const double prev = get_state<double>(pos);
        set_state(pos, cmd);
        if (dt > 0) set_state(vel, (cmd - prev) / dt);
      }
    }
    if (has_state(j.name + "/" + hardware_interface::HW_IF_EFFORT)) {
      set_state(j.name + "/" + hardware_interface::HW_IF_EFFORT, mock_effort_);
    }
  }
  return return_type::OK;
}

bool BeebotSystemHardware::open_port() {
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "OpenCR 포트를 열 수 없음: %s (%s) — USB 연결·권한(dialout) 확인", port_.c_str(), strerror(errno));
    return false;
  }
  termios tio{};
  tcgetattr(fd_, &tio);
  cfmakeraw(&tio);
  cfsetspeed(&tio, B115200);  // USB CDC라 속도는 무관
  tio.c_cflag |= CLOCAL | CREAD;
  tcsetattr(fd_, TCSANOW, &tio);
  tcflush(fd_, TCIOFLUSH);
  last_state_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "OpenCR 연결: %s", port_.c_str());
  return true;
}

void BeebotSystemHardware::close_port() {
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

return_type BeebotSystemHardware::read_serial() {
  char buf[1024];
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0) rx_.append(buf, static_cast<size_t>(n));
  // 가장 최근의 완전한 S 줄만 쓴다 (밀린 줄은 버림)
  std::string last;
  for (size_t nl; (nl = rx_.find('\n')) != std::string::npos; rx_.erase(0, nl + 1)) {
    if (rx_.compare(0, 2, "S,") == 0) last = rx_.substr(2, nl - 2);
  }
  if (rx_.size() > 4096) rx_.clear();  // 줄바꿈 없는 쓰레기가 쌓이지 않게
  if (!last.empty()) {
    if (apply_state(last)) {
      last_state_ = std::chrono::steady_clock::now();
      ++received_;
    } else {
      ++bad_;
    }
  }
  if (std::chrono::steady_clock::now() - last_state_.load() > std::chrono::milliseconds(500)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCR 상태(S) 수신 없음 0.5초 넘음 — 펌웨어·배선 확인");
  }
  return return_type::OK;
}

bool BeebotSystemHardware::apply_state(const std::string & text) {
  std::vector<double> v;
  std::stringstream ss(text);
  for (std::string field; std::getline(ss, field, ',');) {
    char * end = nullptr;
    const double d = std::strtod(field.c_str(), &end);
    if (end == field.c_str()) return false;
    v.push_back(d);
  }
  if (v.size() != STATE_VALUES) return false;
  size_t i = 0;
  for (const auto * name : WHEELS) {
    set_state(std::string(name) + "/position", v[i++]);
    set_state(std::string(name) + "/velocity", v[i++]);
  }
  for (const auto * name : ARM) {
    set_state(std::string(name) + "/position", v[i++]);
    set_state(std::string(name) + "/velocity", v[i++]);
    set_state(std::string(name) + "/effort", v[i++]);
  }
  for (const auto * field : IMU_FIELDS) set_state(imu_ + "/" + field, v[i++]);
  return true;
}

return_type BeebotSystemHardware::write_serial() {
  std::string out = "C";
  char num[32];
  for (const auto * name : WHEELS) {
    const double v = get_command<double>(std::string(name) + "/velocity");
    std::snprintf(num, sizeof(num), ",%.4f", std::isnan(v) ? 0.0 : v);
    out += num;
  }
  for (const auto * name : ARM) {
    double p = get_command<double>(std::string(name) + "/position");
    if (std::isnan(p)) p = get_state<double>(std::string(name) + "/position");  // 명령 없으면 현재 자세 유지
    std::snprintf(num, sizeof(num), ",%.4f", p);
    out += num;
  }
  out += '\n';
  if (::write(fd_, out.data(), out.size()) >= 0) ++sent_;
  else if (errno != EAGAIN) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCR 쓰기 실패: %s", strerror(errno));
  }
  return return_type::OK;
}


}  // namespace control

PLUGINLIB_EXPORT_CLASS(control::BeebotSystemHardware, hardware_interface::SystemInterface)
