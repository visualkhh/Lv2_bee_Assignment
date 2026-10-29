// signal_decoder 구현 — 노드 설명·입출력·멤버 선언은 include/perception/signal_decoder.hpp, main은 signal_decoder_main.cpp
#include "perception/signal_decoder.hpp"

#include <algorithm>
#include <cmath>

namespace perception {

namespace {
const std::map<std::string, std::string> LETTER{{"red", "R"}, {"yellow", "Y"}, {"green", "G"}};
const char * STATE_NAME[] = {"WAITING", "OBSERVING", "DONE", "ERROR"};
}  // namespace

SignalDecoder::SignalDecoder(SignalParams p) : p_(p) { out_.state = MissionSignal::WAITING; }

bool SignalDecoder::feed(double t, const std::string & state) {
  if (out_.state == MissionSignal::DONE || out_.state == MissionSignal::ERROR) return false;
  const std::string s = state.empty() ? "off" : state;
  // 지난 샘플의 상태가 지금까지 이어졌다고 보고 시간을 더함 (샘플이 끊기면 0.5초까지만)
  if (on_ && last_state_ != "off") dur_[last_state_] += std::clamp(t - last_t_, 0.0, 0.5);
  bool changed = false;
  if (s != "off") {
    if (!on_) {
      on_ = true;
      on_since_ = t;
      dur_.clear();
    }
  } else {
    if (last_state_ != "off") off_since_ = t;
    if (on_ && t - off_since_ >= p_.min_off_s) changed = close_blink();
    if (!on_ && static_cast<int>(out_.sequence.size()) == p_.total && out_.state == MissionSignal::OBSERVING &&
        t - off_since_ >= p_.end_off_s) {
      out_.state = MissionSignal::DONE;
      changed = true;
    }
  }
  last_state_ = s;
  last_t_ = t;
  return changed;
}

bool SignalDecoder::close_blink() {
  on_ = false;
  std::vector<std::pair<double, std::string>> d;
  for (const auto & [k, v] : dur_) d.push_back({v, k});
  std::sort(d.rbegin(), d.rend());
  if (d.empty() || d[0].first < p_.min_on_s) return false;  // 잡음
  if (!LETTER.count(d[0].second) || (d.size() > 1 && d[1].first >= p_.min_on_s)) {
    fail("unknown_color");  // all(여러 색) · 한 점멸 안에 두 색 — 추정하지 않음
    return true;
  }
  const std::string c = LETTER.at(d[0].second);
  out_.sequence.push_back(c);
  out_.onsets.push_back(rclcpp::Time(static_cast<int64_t>(on_since_ * 1e9)));
  (c == "R" ? out_.count_r : c == "Y" ? out_.count_y : out_.count_g)++;
  out_.state = MissionSignal::OBSERVING;
  if (static_cast<int>(out_.sequence.size()) > p_.total) fail("too_many");
  return true;
}

void SignalDecoder::fail(const std::string & why) {
  out_.state = MissionSignal::ERROR;
  out_.error = why;
}

std::string SignalDecoder::progress() const {
  std::string t = STATE_NAME[out_.state];
  t += " [";
  for (const auto & c : out_.sequence) t += c;
  t += "]";
  if (on_) t += " (켜짐)";
  if (!out_.error.empty()) t += " " + out_.error;
  return t;
}

SignalDecoderNode::SignalDecoderNode()
: rclcpp::Node("signal_decoder") {
  params_.min_on_s = declare_parameter<double>("min_on_s", params_.min_on_s);
  params_.min_off_s = declare_parameter<double>("min_off_s", params_.min_off_s);
  params_.end_off_s = declare_parameter<double>("end_off_s", params_.end_off_s);
  params_.total = static_cast<int>(declare_parameter<int64_t>("total", params_.total));
  dec_ = SignalDecoder(params_);
  pub_ = create_publisher<MissionSignal>("/mission/signal", rclcpp::QoS(1).reliable().transient_local());
  sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { on_objects(*m); });
  reset_sub_ = create_subscription<std_msgs::msg::Empty>("/mission/signal/reset", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Empty::ConstSharedPtr) {
      dec_ = SignalDecoder(params_);
      RCLCPP_WARN(get_logger(), "신호 처음부터 다시 셈");
      publish();
    });
  publish();  // WAITING — 늦게 붙은 구독자도 지금 상태를 알게 (transient_local)
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto & r = dec_.result();
    s.summary(r.state == MissionSignal::ERROR ? DiagnosticStatus::ERROR : DiagnosticStatus::OK, dec_.progress());
    s.add("신호등", state_);
    s.add("R·Y·G", std::to_string(r.count_r) + "·" + std::to_string(r.count_y) + "·" + std::to_string(r.count_g));
  });
}

void SignalDecoderNode::on_objects(const perception::msg::TrackedObjectArray & m) {
  // 가장 최근에 본 신호등 하나 — 0.6초 넘게 못 봤으면 '안 보임'(= 꺼짐으로 셈. 꺼진 신호등은 YOLO가 놓치기 쉬움)
  const rclcpp::Time now_t = now();
  const perception::msg::TrackedObject * best = nullptr;
  for (const auto & o : m.objects) {
    if (o.class_name != "traffic_light") continue;
    if (!best || rclcpp::Time(o.last_seen) > rclcpp::Time(best->last_seen)) best = &o;
  }
  state_ = best && (now_t - rclcpp::Time(best->last_seen, now_t.get_clock_type())).seconds() < 0.6 ? best->color : "";
  if (!dec_.feed(now_t.seconds(), state_)) return;
  publish();
  const auto & r = dec_.result();
  if (r.state == MissionSignal::DONE)
    RCLCPP_WARN(get_logger(), "신호 끝 %s — R%d Y%d G%d", dec_.progress().c_str(), r.count_r, r.count_y, r.count_g);
  else if (r.state == MissionSignal::ERROR)
    RCLCPP_ERROR(get_logger(), "신호 관측 실패: %s", dec_.progress().c_str());
  else
    RCLCPP_INFO(get_logger(), "점멸 %zu: %s", r.sequence.size(), r.sequence.back().c_str());
}

void SignalDecoderNode::publish() {
  MissionSignal out = dec_.result();
  out.header.stamp = now();
  pub_->publish(out);
}

}  // namespace perception
