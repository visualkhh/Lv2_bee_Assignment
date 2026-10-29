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
static constexpr size_t MOTORS = WHEELS.size() + ARM.size();  // 8 — E·R 필드 · S 온도 순서
// S 값 개수: 바퀴 pos·vel + 팔·그리퍼 pos·vel·A + IMU 10 = 32, 그 뒤 모터 온도 °C 8 = 40.
// 온도 없는 예전 펌웨어(32)도 받는다 — 온도만 모름(−1)으로 (업로드 전에 로봇이 안 뜨지 않게)
static constexpr size_t STATE_VALUES_NO_TEMP = WHEELS.size() * 2 + ARM.size() * 3 + IMU_FIELDS.size();  // 32
static constexpr size_t STATE_VALUES = STATE_VALUES_NO_TEMP + MOTORS;  // 40
// 모터 온도 경고 — XM430 Temperature Limit 기본 80 °C (넘으면 과열 Hardware Error로 토크가 꺼짐) 전에 알림
static constexpr int TEMP_WARN_C = 70;
static const char * motor_name(size_t i) { return i < WHEELS.size() ? WHEELS[i] : ARM[i - WHEELS.size()]; }

// DYNAMIXEL Hardware Error Status 비트 → 사람이 읽는 이름
static std::string error_text(int e) {
  if (e < 0) return "응답 없음";
  std::string t;
  const std::pair<int, const char *> bits[] = {{0x01, "입력전압"}, {0x04, "과열"}, {0x08, "엔코더"}, {0x10, "전기충격"}, {0x20, "과부하"}};
  for (const auto & [bit, name] : bits) if (e & bit) t += (t.empty() ? "" : "·") + std::string(name);
  return t.empty() ? "에러 " + std::to_string(e) : t;
}

CallbackReturn BeebotSystemHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  const auto & hp = info_.hardware_parameters;
  for (auto & t : hw_temp_) t = -1;  // S 온도를 받기 전엔 모름 (0 °C로 보이지 않게)
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
  for (const auto & j : info_.joints) {  // 영점 오프셋 — 조립·서보 혼 각도가 URDF 0과 다를 때 (실측해서 URDF에 적는다)
    const auto it = j.parameters.find("offset");
    offset_[j.name] = it != j.parameters.end() ? std::stod(it->second) : 0.0;  // 미리 채워 read/write에서 삽입 없게
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
  if (!motor_pub_ && get_node()) {  // 모터 에러 전용 토픽 — 진단(사람용 문자열) 대신 다른 노드가 기대는 값
    motor_pub_ = get_node()->create_publisher<control::msg::MotorStatus>(
      "/motor_status", rclcpp::QoS(1).reliable().transient_local());
    motor_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(500), [this]() { publish_motor_status(); });
  }
  if (!reboot_srv_ && get_node()) {  // ~/reboot_motors — Hardware Error(과부하 등) 난 모터만 재부팅 (펌웨어 R)
    reboot_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      "~/reboot_motors", [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
                                std_srvs::srv::Trigger::Response::SharedPtr res) { reboot_motors(req, res); });
  }
  if (!torque_srv_ && get_node()) {  // ~/set_arm_torque — 팔(joint1~5)·그리퍼 토크 끄기·켜기 (펌웨어 T) — 손으로 자세 잡기
    torque_srv_ = get_node()->create_service<std_srvs::srv::SetBool>(
      "~/set_arm_torque", [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                                 std_srvs::srv::SetBool::Response::SharedPtr res) { set_arm_torque(req, res); });
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
  // 활성화 순간 명령 = 현재 상태 (첫 주기에 엉뚱한 곳으로 튀지 않게).
  // controller_manager는 configure→activate를 read() 없이 바로 하므로 상태가 아직 on_configure의 0 —
  // 그대로 쓰면 팔이 0rad(서보 중앙)로 최고 속도로 튀었다 (실기). 첫 S 줄을 받아 실제 각도를 채운 뒤 맞춘다.
  // S가 없으면 활성화 실패 — 펌웨어는 모터가 없어도 S를 보내므로 S 없음 = 시리얼 끊김. 0으로 띄우면
  // rl_arm_correction·JTC가 그 0을 "현재 자세"로 붙잡아, 나중에 S가 와도 팔이 0으로 튄다.
  if (!mock_) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (received_.load() == 0 && std::chrono::steady_clock::now() < until) {
      read_serial();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (received_.load() == 0) {
      RCLCPP_ERROR(get_logger(), "OpenCR 상태(S) 3초 동안 없음 — 실제 팔 각도를 몰라 활성화 안 함 (펌웨어·USB 확인, 해석 실패 %zu줄이면 모터 응답 없음 nan — 12V 확인)", bad_.load());
      return CallbackReturn::ERROR;
    }
  }
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
  std::string errors;
  std::string hot;
  for (size_t i = 0; i < MOTORS; ++i) {
    const int e = hw_error_[i].load();
    if (e != 0) errors += (errors.empty() ? "" : ", ") + std::string(motor_name(i)) + " " + error_text(e);
    s.add(std::string("모터 에러 ") + motor_name(i), e == 0 ? std::string("정상") : error_text(e));
    const int t = hw_temp_[i].load();
    if (t >= TEMP_WARN_C) hot += (hot.empty() ? "" : ", ") + std::string(motor_name(i)) + " " + std::to_string(t) + "°C";
    s.add(std::string("모터 온도 ") + motor_name(i), t < 0 ? std::string("모름") : std::to_string(t) + " °C");
  }
  if (!mock_ && !hot.empty())
    s.mergeSummary(DiagnosticStatus::WARN, "모터 온도 높음: " + hot + " (" + std::to_string(TEMP_WARN_C) + "°C 이상 — 80°C면 과열로 멈춤)");
  if (!mock_ && !errors.empty())
    s.mergeSummary(DiagnosticStatus::ERROR, "모터 에러: " + errors + " → ~/reboot_motors 서비스로 재부팅");
  if (!mock_ && !arm_torque_.load()) s.mergeSummary(DiagnosticStatus::WARN, "팔·그리퍼 토크 꺼짐 (손으로 자세 잡기)");
  s.add("팔·그리퍼 토크", arm_torque_.load() ? "켬" : "끔");
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
    else if (rx_.compare(0, 2, "E,") == 0 && !apply_errors(rx_.substr(2, nl - 2))) ++bad_;
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
  if (v.size() != STATE_VALUES && v.size() != STATE_VALUES_NO_TEMP) return false;
  // 펌웨어는 한 번도 못 읽은 모터(부팅 때 12V 꺼짐 등)를 nan으로 보낸다 — 모르는 자세를 상태로 쓰지 않는다
  for (size_t k = 0; k < WHEELS.size() * 2 + ARM.size() * 3; ++k) if (std::isnan(v[k])) return false;
  size_t i = 0;
  for (const auto * name : WHEELS) {
    set_state(std::string(name) + "/position", v[i++]);
    set_state(std::string(name) + "/velocity", v[i++]);
  }
  for (const auto * name : ARM) {
    double p = v[i++] - offset_[name];  // 서보 값 → 관절값
    // 시리얼은 소수 넷째 자리(%.4f) — 한계값 명령(π = 3.14159)이 3.1416으로 오가며 한계를 0.0001쯤 넘는다.
    // 그대로 내보내면 MoveIt이 출발 자세가 한계 밖이라며 모든 계획을 거부했음 (START_STATE_INVALID) → 반올림만큼 넘은 건 한계로
    const auto lim = info_.limits.find(name);
    if (lim != info_.limits.end() && lim->second.has_position_limits) {
      const double lo = lim->second.min_position, hi = lim->second.max_position;
      if (p > hi && p - hi < 1e-3) p = hi;
      if (p < lo && lo - p < 1e-3) p = lo;
    }
    set_state(std::string(name) + "/position", p);
    set_state(std::string(name) + "/velocity", v[i++]);
    set_state(std::string(name) + "/effort", v[i++]);
  }
  for (const auto * field : IMU_FIELDS) set_state(imu_ + "/" + field, v[i++]);
  if (v.size() == STATE_VALUES)
    for (size_t k = 0; k < MOTORS; ++k) hw_temp_[k] = static_cast<int>(std::lround(v[i++]));  // °C, −1 = 모름
  return true;
}

void BeebotSystemHardware::publish_motor_status() {
  std::array<int, MOTORS> now{}, temp{};
  for (size_t i = 0; i < MOTORS; ++i) now[i] = hw_error_[i].load(), temp[i] = hw_temp_[i].load();
  if (motor_published_ && now == motor_sent_ && temp == temp_sent_) return;  // 바뀔 때만 (transient_local이라 늦게 붙어도 마지막 값)
  control::msg::MotorStatus m;
  m.header.stamp = get_node()->now();
  for (size_t i = 0; i < MOTORS; ++i) {
    m.joints.push_back(motor_name(i));
    m.error.push_back(now[i]);
    m.text.push_back(now[i] == 0 ? "정상" : error_text(now[i]));
    m.temperature.push_back(temp[i]);
  }
  motor_pub_->publish(m);
  motor_sent_ = now;
  temp_sent_ = temp;
  motor_published_ = true;
}

bool BeebotSystemHardware::apply_errors(const std::string & text) {
  std::stringstream ss(text);
  size_t i = 0;
  for (std::string field; std::getline(ss, field, ',') && i < MOTORS; ++i) {
    char * end = nullptr;
    const long e = std::strtol(field.c_str(), &end, 10);
    if (end == field.c_str()) return false;
    hw_error_[i] = static_cast<int>(e);
  }
  return i == MOTORS;
}

void BeebotSystemHardware::reboot_motors(const std_srvs::srv::Trigger::Request::SharedPtr,
                                         std_srvs::srv::Trigger::Response::SharedPtr res) {
  // 멀쩡한 모터까지 재부팅하면 그 관절 토크가 빠져 팔이 떨어진다 → 에러(>0) 난 모터만
  uint32_t mask = 0;
  std::string names;
  for (size_t i = 0; i < MOTORS; ++i) {
    if (hw_error_[i].load() > 0) {
      mask |= 1u << i;
      names += (names.empty() ? "" : ", ") + std::string(motor_name(i));
    }
  }
  if (mock_ || mask == 0) {
    res->success = false;
    res->message = mock_ ? "mock — 재부팅할 모터 없음" : "Hardware Error 난 모터 없음 (응답 없는 모터는 재부팅 불가 — 12V·배선 확인)";
    return;
  }
  reboot_mask_ = mask;  // write()가 다음 주기에 R 줄로 보냄 (시리얼 쓰기는 제어 주기 한 곳에서만)
  res->success = true;
  res->message = "재부팅 요청: " + names + " — 팔은 천천히 명령 자세로 돌아감";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

void BeebotSystemHardware::set_arm_torque(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                          std_srvs::srv::SetBool::Response::SharedPtr res) {
  if (mock_) {
    res->success = false;
    res->message = "mock — 토크 없음";
    return;
  }
  torque_req_ = req->data ? 1 : 0;  // write()가 다음 주기에 T 줄로 보냄 (시리얼 쓰기는 제어 주기 한 곳에서만)
  res->success = true;
  res->message = req->data ? "팔·그리퍼 토크 켬 (지금 자세를 목표로)" : "팔·그리퍼 토크 끔 — 손으로 자세를 잡을 수 있음 (팔이 처질 수 있으니 잡고 있을 것)";
  RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
}

return_type BeebotSystemHardware::write_serial() {
  if (const int t = torque_req_.exchange(-1); t >= 0) {  // T,<바퀴L>,<바퀴R>,<j1>…<j5>,<그리퍼> — 바퀴 빈칸 = 그대로
    std::string line = "T,,";
    for (size_t i = 0; i < ARM.size(); ++i) line += t ? ",1" : ",0";  // joint1~5 + 그리퍼
    line += '\n';
    if (::write(fd_, line.data(), line.size()) < 0 && errno != EAGAIN)
      RCLCPP_ERROR(get_logger(), "OpenCR T 쓰기 실패: %s", strerror(errno));
    else
      arm_torque_ = t != 0;
  }
  if (const uint32_t mask = reboot_mask_.exchange(0)) {  // R,<8필드> — 고른 모터만 1
    std::string r = "R";
    for (size_t i = 0; i < MOTORS; ++i) r += (mask & (1u << i)) ? ",1" : ",";
    r += '\n';
    if (::write(fd_, r.data(), r.size()) < 0 && errno != EAGAIN)
      RCLCPP_ERROR(get_logger(), "OpenCR R 쓰기 실패: %s", strerror(errno));
  }
  std::string out = "C";
  char num[32];
  for (const auto * name : WHEELS) {
    const double v = get_command<double>(std::string(name) + "/velocity");
    std::snprintf(num, sizeof(num), ",%.4f", std::isnan(v) ? 0.0 : v);
    out += num;
  }
  for (const auto * name : ARM) {
    const double p = get_command<double>(std::string(name) + "/position");
    // 명령 없으면 빈칸 = 펌웨어가 현재 자세 유지 (상태로 채우면 S 받기 전엔 0rad를 보내 팔이 튄다)
    if (std::isnan(p)) {
      out += ',';
      continue;
    }
    std::snprintf(num, sizeof(num), ",%.4f", p + offset_[name]);  // 관절값 → 서보 값
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
