// signal_decoder 구현 — 노드 설명·입출력·멤버 선언은 include/perception/signal_decoder.hpp, main은 signal_decoder_main.cpp
#include "perception/signal_decoder.hpp"

#include <algorithm>

namespace perception {

namespace {
bool is_color(const std::string & s) { return s == "red" || s == "yellow" || s == "green"; }
bool is_off(const std::string & s) { return s == "off" || s.empty(); }
char letter(const std::string & c) { return c == "red" ? 'R' : c == "yellow" ? 'Y' : c == "green" ? 'G' : '?'; }
std::string text(const std::vector<std::string> & seg) {
  std::string t;
  for (const auto & c : seg) t += std::string(t.empty() ? "" : " ") + letter(c);
  return t;
}
}  // namespace

std::string light_state(const std::vector<std::pair<double, std::string>> & lights, double now_s, double max_age_s) {
  const std::pair<double, std::string> * lit = nullptr, * any = nullptr;
  for (const auto & l : lights) {
    if (now_s - l.first >= max_age_s) continue;  // 오래 못 본 것은 안 씀
    if (!any || l.first > any->first) any = &l;
    if ((is_color(l.second) || l.second == "all") && (!lit || l.first > lit->first)) lit = &l;
  }
  return lit ? lit->second : any ? any->second : "";
}

SignalDecoder::SignalDecoder(SignalParams p) : p_(std::move(p)) {}

bool SignalDecoder::feed(double t, const std::string & state, MissionSignal & out) {
  if (off_since_ < 0) off_since_ = t;  // 첫 샘플 — 노드가 켜진 순간부터 잼 (켜기 전 시간을 꺼짐으로 세지 않음)
  if (!is_off(state)) {  // 켜짐 — 구간 시작 또는 이어감 (색이 바뀌어도 같은 구간)
    if (!in_on_) {
      in_on_ = true;
      on_start_ = t;
      votes_.clear();
      off_since_saved_ = off_since_;
    }
    ++votes_[state];
    last_on_ = t;
    return false;
  }
  if (in_on_) {  // 꺼짐 — 짧으면 구간 안, 길어지면 구간을 닫음
    if (t - last_on_ < p_.min_off_s - 1e-6) return false;  // 1e-6: 샘플 시각 소수점 오차로 경계값이 갈리지 않게
    in_on_ = false;
    if (last_on_ - on_start_ >= p_.min_on_s - 1e-9) {
      seg_.push_back(blink_color());  // 점멸 1번
      off_since_ = last_on_;
      marked_ = false;
      if (seg_.size() == 1 && on_start_ - tail_end_ > p_.tail_gap_max_s) tail_.clear();  // 사이클 사이가 3초보다 길었음 — 자리 모름
      if (!tail_.empty() && seg_.size() + tail_.size() == static_cast<size_t>(p_.blinks)) {  // 앞부분 + 보관한 뒷부분 = 한 사이클
        std::vector<std::string> seq(seg_);
        seq.insert(seq.end(), tail_.begin(), tail_.end());
        tail_.clear();
        check(out, seq);
        seg_.clear();
        return true;
      }
    } else {
      off_since_ = off_since_saved_;  // 잡음 — 그 전부터 이어진 꺼짐으로 계속 잼
    }
  }
  // 사이클 끝: 마지막 켜짐 뒤 꺼짐이 gap_s 넘게 이어짐 (꺼짐 구간마다 한 번)
  if (!marked_ && t - off_since_ >= p_.gap_s - 1e-6) {
    marked_ = true;
    return boundary(out);
  }
  return false;
}

std::string SignalDecoder::blink_color() const {
  // 구간에서 가장 많이 나온 상태 — 빨·노·초가 아니거나(all 등) 비율이 min_share 미만이면 추정하지 않고 unknown
  int total = 0;
  const std::pair<const std::string, int> * best = nullptr;
  for (const auto & kv : votes_) {
    total += kv.second;
    if (!best || kv.second > best->second) best = &kv;
  }
  if (!best || !is_color(best->first) || best->second < p_.min_share * total) return "unknown";
  return best->first;
}

bool SignalDecoder::boundary(MissionSignal & out) {
  if (seg_.empty()) return false;  // 점멸 없이 꺼짐만 이어짐 (신호 시작 전 · 신호등 안 보임)
  if (seg_.size() < static_cast<size_t>(p_.blinks)) {  // 끊긴 조각 — 이 사이클 뒷부분으로 보관 (오류 아님 · 지난 보관은 교체)
    tail_.swap(seg_);
    seg_.clear();
    tail_end_ = off_since_;
    return false;
  }
  check(out, seg_);  // blinks개 (넘으면 count 오류 — 두 사이클이 합쳐짐)
  seg_.clear();
  tail_.clear();
  return true;
}

void SignalDecoder::check(MissionSignal & out, const std::vector<std::string> & seq) {
  out.sequence = seq;
  out.red = static_cast<int32_t>(std::count(seq.begin(), seq.end(), "red"));
  out.yellow = static_cast<int32_t>(std::count(seq.begin(), seq.end(), "yellow"));
  out.green = static_cast<int32_t>(std::count(seq.begin(), seq.end(), "green"));
  out.raw = text(seq);
  // 검사 순서: 모르는 색 → 개수 → 색별 범위 (앞에서 걸리면 그 이유 하나만)
  const auto out_of_range = [&](int32_t n) { return n < p_.min_count || n > p_.max_count; };
  if (std::count(seq.begin(), seq.end(), "unknown") > 0) out.error = "unknown";
  else if (static_cast<int>(seq.size()) != p_.blinks) out.error = "count";
  else if (out_of_range(out.red) || out_of_range(out.yellow) || out_of_range(out.green)) out.error = "range";
  else out.error = "";
  if (!out.error.empty()) {  // 정상 사이클이 아니면 연속 횟수를 끊음 — 다음 정상 사이클부터 다시 셈
    same_ = 0;
    last_raw_.clear();
    out.cycles = 0;
    out.ok = false;
    return;
  }
  same_ = out.raw == last_raw_ ? same_ + 1 : 1;
  last_raw_ = out.raw;
  out.cycles = same_;
  out.ok = same_ >= p_.confirm;
  if (!out.ok) out.error = "unconfirmed";
}

std::string SignalDecoder::progress() const {
  return (tail_.empty() ? "" : "(뒷부분 " + text(tail_) + " 보관) ") + text(seg_) +
         (in_on_ ? " (" + std::string(1, letter(blink_color())) + " 켜짐 중)" : "") +
         " · " + std::to_string(seg_.size()) + "/" + std::to_string(p_.blinks);
}

SignalDecoderNode::SignalDecoderNode()
: rclcpp::Node("signal_decoder") {
  SignalParams p;
  p.min_on_s = declare_parameter<double>("min_on_s", p.min_on_s);
  p.min_off_s = declare_parameter<double>("min_off_s", p.min_off_s);
  p.min_share = declare_parameter<double>("min_share", p.min_share);
  p.gap_s = declare_parameter<double>("gap_s", p.gap_s);
  p.blinks = static_cast<int>(declare_parameter<int64_t>("blinks", p.blinks));
  p.min_count = static_cast<int>(declare_parameter<int64_t>("min_count", p.min_count));
  p.max_count = static_cast<int>(declare_parameter<int64_t>("max_count", p.max_count));
  p.tail_gap_max_s = declare_parameter<double>("tail_gap_max_s", p.tail_gap_max_s);
  p.confirm = static_cast<int>(declare_parameter<int64_t>("confirm", p.confirm));
  dec_ = SignalDecoder(p);
  pub_ = create_publisher<MissionSignal>("/mission/signal", rclcpp::QoS(1).reliable().transient_local());
  status_pub_ = create_publisher<MissionSignal>("/mission/signal_status", rclcpp::QoS(10).reliable());
  sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
    [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { on_objects(*m); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    s.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
              !have_ ? "신호 읽는 중" : last_.ok ? "확정: " + last_.raw : "마지막 사이클 " + last_.error + ": " + last_.raw);
    s.add("신호등", state_);
    s.add("지금 사이클", dec_.progress());
    if (have_) {
      s.add("마지막 결과", "R" + std::to_string(last_.red) + " Y" + std::to_string(last_.yellow) + " G" +
                          std::to_string(last_.green) + " · " + std::to_string(last_.cycles) + "사이클 연속");
    }
  });
}

void SignalDecoderNode::on_objects(const perception::msg::TrackedObjectArray & m) {
  // 0.6초 안에 본 신호등 중 켜진 것 우선, 없으면 가장 최근 것 — 다 못 봤으면 '안 보임'(= 꺼짐으로 셈. 꺼진 신호등은 YOLO가 놓치기 쉬움)
  //   (신호등이 트랙 둘로 잡혀도 꺼진 쪽을 따라가지 않게 — light_state)
  const rclcpp::Time now_t = now();
  std::vector<std::pair<double, std::string>> lights;
  for (const auto & o : m.objects)
    if (o.class_name == "traffic_light") lights.emplace_back(rclcpp::Time(o.last_seen, now_t.get_clock_type()).seconds(), o.color);
  state_ = light_state(lights, now_t.seconds(), 0.6);
  MissionSignal out;
  if (!dec_.feed(now_t.seconds(), state_, out)) return;
  out.header.stamp = now_t;  // 사이클 끝을 확인한 시각 (로그용)
  last_ = out;
  have_ = true;
  status_pub_->publish(out);  // 사이클마다 — ok든 오류든 (통합이 오류 대응·로그용으로 구독)
  if (!out.ok) {  // 확정 전·오류 사이클 — /mission/signal에는 안 냄 (받을 때마다 mission_manager가 다시 계획하므로)
    RCLCPP_WARN(get_logger(), "신호 사이클 %s [%s] (%zu개)", out.error.c_str(), out.raw.c_str(), out.sequence.size());
    return;
  }
  if (out.raw == published_raw_) return;  // 반복 신호 — 같은 확정 답은 한 번만 발행 (사이에 오류 사이클이 껴도)
  published_raw_ = out.raw;
  pub_->publish(out);
  RCLCPP_WARN(get_logger(), "신호 확정 [%s] → R%d Y%d G%d (%d사이클 연속)",
    out.raw.c_str(), out.red, out.yellow, out.green, out.cycles);
}

}  // namespace perception
