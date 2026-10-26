// signal_decoder 구현 — 노드 설명·입출력·멤버 선언은 include/plan/signal_decoder.hpp, main은 signal_decoder_main.cpp
#include "plan/signal_decoder.hpp"

#include <algorithm>

namespace plan {

namespace {
bool is_color(const std::string & s) { return s == "red" || s == "yellow" || s == "green"; }
char letter(const std::string & c) { return c == "red" ? 'R' : c == "yellow" ? 'Y' : c == "green" ? 'G' : '?'; }
std::string text(const std::vector<std::string> & seg) {
  std::string t;
  for (const auto & c : seg) t += std::string(t.empty() ? "" : " ") + letter(c);
  return t;
}
}  // namespace

SignalDecoder::SignalDecoder(SignalParams p) : p_(std::move(p)) {}

bool SignalDecoder::feed(double t, const std::string & state, MissionSignal & out) {
  const std::string s = state.empty() ? "off" : state;
  if (s != cur_) {
    end_run(t);
    cur_ = s;
    since_ = t;
    marked_ = false;
  }
  if (marked_) return false;
  // 단계 끝: 세 색 동시(all)가 separator_s · 꺼짐이 gap_s 넘게 이어짐 (구간마다 한 번)
  if ((cur_ == "all" && t - since_ >= p_.separator_s) || (cur_ == "off" && t - since_ >= p_.gap_s)) {
    marked_ = true;
    return boundary(out);
  }
  return false;
}

void SignalDecoder::end_run(double t) {
  if (is_color(cur_) && t - since_ >= p_.min_on_s) seg_.push_back(cur_);  // 한 번 깜빡임
}

bool SignalDecoder::boundary(MissionSignal & out) {
  if (seg_.empty()) return false;
  segs_.push_back(seg_);
  seg_.clear();
  if (segs_.size() > 3) segs_.erase(segs_.begin());
  if (segs_.size() < 3 || !decode(out)) return false;
  same_ = out.raw == last_raw_ ? same_ + 1 : 1;
  last_raw_ = out.raw;
  return same_ >= p_.confirm;
}

bool SignalDecoder::decode(MissionSignal & out) const {
  const auto & s1 = segs_[0], & s2 = segs_[1], & s3 = segs_[2];
  // 1단계 한 색만 · 3단계 노랑만 1~3번 — 아니면 지금 셋은 한 바퀴가 아님 (반복 신호의 중간부터 봄)
  if (std::any_of(s1.begin(), s1.end(), [&](const std::string & c) { return c != s1[0]; })) return false;
  if (std::any_of(s3.begin(), s3.end(), [](const std::string & c) { return c != "yellow"; })) return false;
  if (s3.size() > p_.dest_tags.size()) return false;
  out.color = s1[0];
  out.count = static_cast<int32_t>(s1.size());
  out.sum = 0;
  for (const auto & c : s2) {
    const auto it = p_.scores.find(c);
    out.sum += it != p_.scores.end() ? it->second : 0;
  }
  out.dest_tag = p_.dest_tags[s3.size() - 1];
  out.raw = text(s1) + " | " + text(s2) + " | " + text(s3);
  return true;
}

std::string SignalDecoder::progress() const {
  std::string t;
  for (const auto & s : segs_) t += text(s) + " | ";
  return t + text(seg_) + (is_color(cur_) ? " (" + std::string(1, letter(cur_)) + ")" : "");
}

SignalDecoderNode::SignalDecoderNode()
: rclcpp::Node("signal_decoder") {
  SignalParams p;
  p.min_on_s = declare_parameter<double>("min_on_s", p.min_on_s);
  p.separator_s = declare_parameter<double>("separator_s", p.separator_s);
  p.gap_s = declare_parameter<double>("gap_s", p.gap_s);
  for (auto & [c, v] : p.scores) v = static_cast<int>(declare_parameter<int64_t>("score_" + c, v));
  const auto dest = declare_parameter<std::vector<int64_t>>("dest_tags", {12, 13, 14});
  p.dest_tags.assign(dest.begin(), dest.end());
  p.confirm = static_cast<int>(declare_parameter<int64_t>("confirm", p.confirm));
  dec_ = SignalDecoder(p);
  pub_ = create_publisher<MissionSignal>("/mission/signal", rclcpp::QoS(1).reliable().transient_local());
  sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { on_objects(*m); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, have_ ? "읽음: " + last_.raw : "신호 읽는 중");
    s.add("신호등", state_);
    s.add("지금까지", dec_.progress());
    if (have_) {
      s.add("목표", last_.color + " " + std::to_string(last_.count) + "개 · 합 " + std::to_string(last_.sum) +
                    " · 목적지 태그 " + std::to_string(last_.dest_tag));
    }
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
  MissionSignal out;
  if (!dec_.feed(now_t.seconds(), state_, out)) return;
  if (have_ && out.raw == last_.raw) return;  // 반복 신호 — 같은 답은 한 번만
  last_ = out;
  have_ = true;
  pub_->publish(out);
  RCLCPP_WARN(get_logger(), "신호 읽음 [%s] → %s 퍽 %d개 · 번호 합 %d · 목적지 태그 %d",
    out.raw.c_str(), out.color.c_str(), out.count, out.sum, out.dest_tag);
}

}  // namespace plan
