// rl_arm_correction 구현 — 설명·선언은 include/ml/rl_arm_correction.hpp
#include "ml/rl_arm_correction.hpp"

#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace ml {

CallbackReturn RlArmCorrection::on_init() {
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  auto_declare<bool>("enabled", true);
  auto_declare<double>("max_delta", 0.05);          // Δ 상한 [rad] (안전)
  auto_declare<double>("residual_rad", 0.05);       // 행동 1 = 이만큼 [rad] (시뮬 residual_rad)
  auto_declare<bool>("sag_enabled", true);
  auto_declare<double>("sag_gain", 4.2);            // [1/s] 학습: 85 ms마다 오차 30 % 제거 = −ln(0.7)/0.085 (송정혁)
  auto_declare<double>("sag_max", 0.2);             // [rad]
  auto_declare<double>("still_rate", 0.005);        // [rad/s] reference가 이보다 느리면 '멈춤'
  // 발열 방지: 쉬는 자세(stow, plan basket.yaml pose_stow)에선 적분 안 함 · 상한에 windup_s 붙으면 풂. [] = 끔
  auto_declare<std::vector<double>>("rest_pose", {0.0, -1.9024, 1.344, 0.9023, 0.0});
  auto_declare<double>("rest_tol", 0.03);           // [rad] 모든 관절이 이 안이면 쉬는 자세
  auto_declare<double>("debug_hz", 20.0);          // ~/debug (ml/ArmCorrectionDebug) 주기 — 0 = 안 냄
  auto_declare<double>("windup_s", 2.0);            // [s] 0 = 막힘 감지 끔
  auto_declare<bool>("gravity_enabled", false);     // 중력 앞먹임 (헤더 설명)
  auto_declare<double>("gravity_compliance_j2", 0.0817);  // [rad/Nm] = 1/강성 12.2 Nm/rad
  auto_declare<double>("gravity_compliance_j3", 0.0532);  // [rad/Nm] = 1/강성 18.8 Nm/rad
  auto_declare<double>("gravity_ramp_s", 2.0);      // [s] 켜고 끌 때 서서히
  auto_declare<double>("gravity_max", 0.2);         // [rad] 한 관절 상한
  auto_declare<double>("policy_hz", 50.0);          // 시뮬 정책 주기
  auto_declare<double>("max_cam_age_s", 1.0);       // 손 카메라 값이 이보다 오래되면 Δ 그대로
  return CallbackReturn::SUCCESS;
}

CallbackReturn RlArmCorrection::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  enabled_ = node->get_parameter("enabled").as_bool();
  max_delta_ = node->get_parameter("max_delta").as_double();
  residual_rad_ = node->get_parameter("residual_rad").as_double();
  sag_enabled_ = node->get_parameter("sag_enabled").as_bool();
  sag_gain_ = node->get_parameter("sag_gain").as_double();
  sag_max_ = node->get_parameter("sag_max").as_double();
  still_rate_ = node->get_parameter("still_rate").as_double();
  rest_pose_ = node->get_parameter("rest_pose").as_double_array();
  rest_tol_ = node->get_parameter("rest_tol").as_double();
  windup_s_ = node->get_parameter("windup_s").as_double();
  if (!rest_pose_.empty() && rest_pose_.size() != 5) {
    RCLCPP_ERROR(node->get_logger(), "rest_pose는 관절 5개 (지금 %zu) — 쉬는 자세 판정 끔", rest_pose_.size());
    rest_pose_.clear();
  }
  gravity_enabled_ = node->get_parameter("gravity_enabled").as_bool();
  gravity_c2_ = node->get_parameter("gravity_compliance_j2").as_double();
  gravity_c3_ = node->get_parameter("gravity_compliance_j3").as_double();
  gravity_ramp_s_ = node->get_parameter("gravity_ramp_s").as_double();
  gravity_max_ = node->get_parameter("gravity_max").as_double();
  gravity_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
    for (const auto & p : params)
      if (p.get_name() == "gravity_enabled") gravity_enabled_ = p.as_bool();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  });
  // 스킬 시작마다 호출 (학습 에피소드도 corr = 0에서 시작). 실제 리셋은 다음 update에서 (송정혁)
  reset_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "~/reset_sag", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
      reset_sag_ = true;
      res->success = true;
    });
  const double debug_hz = node->get_parameter("debug_hz").as_double();
  if (debug_hz > 0.0) {  // 실기 디버깅: 보정이 무엇을 했는지 bag에 남김 (실시간 루프는 try_publish — 못 보내면 건너뜀)
    debug_dt_ = 1.0 / debug_hz;
    debug_pub_ = node->create_publisher<ml::msg::ArmCorrectionDebug>("~/debug", rclcpp::SystemDefaultsQoS());
    debug_rt_ = std::make_unique<realtime_tools::RealtimePublisher<ml::msg::ArmCorrectionDebug>>(debug_pub_);
  }
  policy_dt_ = 1.0 / node->get_parameter("policy_hz").as_double();
  max_cam_age_s_ = node->get_parameter("max_cam_age_s").as_double();
  obs_.assign(ARM_OBS_MAX, 0.0);
  act_.assign(5, 0.0);
  ctx_sub_ = node->create_subscription<ml::msg::SkillContext>(
    "/skill_context", rclcpp::QoS(10).reliable(), [this](ml::msg::SkillContext::ConstSharedPtr m) { ctx_box_.set(Ctx{*m, policy_for(*m)}); });
  hand_sub_ = node->create_subscription<perception::msg::HandRelative>(
    "/hand_relative", 10, [this](perception::msg::HandRelative::ConstSharedPtr m) { hand_box_.set(*m); });
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration RlArmCorrection::command_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  return cfg;
}

InterfaceConfiguration RlArmCorrection::state_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

CallbackReturn RlArmCorrection::on_activate(const rclcpp_lifecycle::State &) {
  // 시작할 때 reference를 현재 자세로 — 첫 주기에 0rad로 튀지 않게
  for (size_t i = 0; i < refs_.size(); ++i) {
    const auto pos = state_interfaces_[2 * i].get_optional<double>();
    if (pos) (void)refs_[i]->set_value(*pos);
  }
  ctx_ = ml::msg::SkillContext();
  reset_run();
  corr_.fill(0.0);
  gravity_scale_ = 0.0;
  have_prev_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RlArmCorrection::on_deactivate(const rclcpp_lifecycle::State &) { return CallbackReturn::SUCCESS; }

std::vector<hardware_interface::CommandInterface::SharedPtr> RlArmCorrection::on_export_reference_interfaces_list() {
  refs_.clear();
  for (const auto & j : joints_) {
    hardware_interface::InterfaceInfo info;
    info.name = j + "/" + hardware_interface::HW_IF_POSITION;
    info.data_type = "double";
    refs_.push_back(std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return refs_;
}

return_type RlArmCorrection::update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) {
  return return_type::OK;
}

std::shared_ptr<const RlArmCorrection::Skill> RlArmCorrection::policy_for(const ml::msg::SkillContext & m) {
  if (m.policy.empty()) return nullptr;
  if (const auto it = cache_.find(m.policy); it != cache_.end()) return it->second;
  auto s = std::make_shared<Skill>();
  std::string why;
  const auto log = get_node()->get_logger();
  if (!arm_layout_from(m.layout, s->layout)) {
    RCLCPP_WARN(log, "%s: 입력 모양 '%s' 모름 — 보정 없음", m.skill.c_str(), m.layout.c_str());
    s = nullptr;
  } else if (!s->policy.load(m.policy, why) || s->policy.obs_dim() != arm_obs_size(s->layout) || s->policy.act_dim() != 5) {
    RCLCPP_ERROR(log, "%s: 정책을 못 씀 %s — %s (입력 %zu, 필요 %zu)", m.skill.c_str(), m.policy.c_str(), why.c_str(),
                 s->policy.obs_dim(), arm_obs_size(s->layout));
    s = nullptr;
  } else {
    RCLCPP_INFO(log, "%s: 정책 %s (%s 모양)", m.skill.c_str(), m.policy.c_str(), m.layout.c_str());
  }
  cache_[m.policy] = s;  // 못 쓰는 파일도 기억 (매번 다시 읽지 않게)
  return s;
}

void RlArmCorrection::reset_run() {  // 보정 Δ만 (처짐 적분은 ~/reset_sag · 닿는 구간 시작에서)
  delta_.fill(0.0);
  prev_act_.fill(0.0);
  since_policy_ = 0.0;
}

return_type RlArmCorrection::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) {
  if (reset_sag_.exchange(false) || !sag_enabled_) sag_.fill(SagJoint{});
  if (auto c = ctx_box_.try_get()) {  // 못 잡으면 (콜백이 쓰는 중) 지난 값으로
    if (c->msg.run_id != ctx_.run_id || c->msg.skill != ctx_.skill) {
      // 새 닿는 구간: 시뮬 에피소드처럼 보정·처짐 적분 0부터 (skill이 꺼질 때는 처짐 적분 유지)
      if (!c->msg.skill.empty()) sag_.fill(SagJoint{});
      run_id_ = c->msg.run_id;
      reset_run();
    }
    ctx_ = c->msg;
    skill_ = c->skill;
  }
  if (auto h = hand_box_.try_get()) hand_ = *h;

  J5 q{}, qd{}, ref{};
  bool ok = joints_.size() == 5;
  for (size_t i = 0; i < joints_.size() && i < 5; ++i) {
    const auto r = refs_[i]->get_optional<double>();
    const auto p = state_interfaces_[2 * i].get_optional<double>();
    const auto v = state_interfaces_[2 * i + 1].get_optional<double>();
    if (!r || std::isnan(*r) || !p) { ok = false; break; }
    ref[i] = *r;
    q[i] = *p;
    qd[i] = v ? *v : 0.0;
  }

  const double dt = period.seconds();
  // 멈춤 판정 (송정혁): 모든 관절의 reference 변화율이 still_rate 미만 (첫 주기·reference 없음은 움직이는 것으로)
  bool still = have_prev_ && ok && dt > 0.0;
  for (int i = 0; ok && i < 5; ++i) {
    if (std::abs(ref[i] - prev_ref_[i]) > still_rate_ * dt) still = false;
    prev_ref_[i] = ref[i];
  }
  have_prev_ = ok;
  bool rest = ok && !rest_pose_.empty();
  for (int i = 0; rest && i < 5; ++i) rest = std::abs(ref[i] - rest_pose_[i]) <= rest_tol_;
  for (int i = 0; i < 5; ++i) corr_[i] = sag_[i].corr;  // 정책 관측용

  const Skill * it = enabled_ && ok ? skill_.get() : nullptr;
  bool cam_ok = false;
  double cam_age = -1.0;
  if (!it) {  // 보정 꺼짐: 닿는 구간이 아니거나 이 skill 정책이 없음 → Δ = 0 (처짐 적분은 계속)
    if (delta_ != J5{}) reset_run();
  } else {
    since_policy_ += dt;
    const bool cam_kind = it->layout == ArmLayout::Pick ? hand_.kind == "puck" : (hand_.kind == "cell" || hand_.kind == "slot");
    cam_age = (time - rclcpp::Time(hand_.header.stamp, time.get_clock_type())).seconds();
    cam_ok = hand_.valid && cam_kind && cam_age >= -0.1 && cam_age <= max_cam_age_s_;
    if (since_policy_ >= policy_dt_ - 1e-6 && ctx_.q_above.size() == 5 && ctx_.q_down.size() == 5 && cam_ok) {
      since_policy_ = 0.0;
      ArmObsIn in;
      in.q = q; in.qd = qd; in.q_ref = ref; in.prev_act = prev_act_; in.corr = corr_;
      std::copy(ctx_.q_above.begin(), ctx_.q_above.end(), in.q_above.begin());
      std::copy(ctx_.q_down.begin(), ctx_.q_down.end(), in.q_down.begin());
      in.phase = ctx_.phase;
      in.prog_s = (time - rclcpp::Time(ctx_.header.stamp, time.get_clock_type())).seconds();
      in.cam_x = hand_.rel.x;
      in.cam_y = hand_.rel.y;
      in.face_yaw = hand_.face_yaw;
      in.is_box = hand_.is_box;
      arm_obs(it->layout, in, obs_.data());
      it->policy.act_into(obs_.data(), act_.data());
      for (int i = 0; i < 5; ++i) {
        const double a = std::clamp(act_[i], -1.0, 1.0);
        if (ctx_.residual_live) delta_[i] = std::clamp(residual_rad_ * a, -max_delta_, max_delta_);
        prev_act_[i] = a;
      }
    }
  }
  // 중력 앞먹임 (헤더 설명)
  const double gravity_step = dt / std::max(gravity_ramp_s_, 1e-3);
  gravity_scale_ = gravity_enabled_ ? std::min(1.0, gravity_scale_ + gravity_step) : std::max(0.0, gravity_scale_ - gravity_step);
  J5 grav{};
  if (ok && gravity_scale_ > 0.0) {
    const auto tau = gravity::torques(q[1], q[2], q[3]);
    grav[1] = gravity_scale_ * std::clamp(-gravity_c2_ * tau[0], -gravity_max_, gravity_max_);
    grav[2] = gravity_scale_ * std::clamp(-gravity_c3_ * tau[1], -gravity_max_, gravity_max_);
  }
  for (size_t i = 0; i < joints_.size() && i < 5; ++i) {
    if (!ok) continue;  // reference가 아직 없음 — 아무것도 안 씀 (예전처럼)
    const double total = delta_[i] + grav[i];  // 둘 다 오차에 넣어 처짐 적분이 지우지 않는다
    if (sag_enabled_) {
      sag_joint_step(sag_[i], still, rest, ref[i], total, q[i], dt, sag_gain_, sag_max_, windup_s_);
      corr_[i] = sag_[i].corr;
    }
    (void)command_interfaces_[i].set_value(ref[i] + corr_[i] + total);
  }
  since_debug_ += dt;
  if (debug_rt_ && ok && since_debug_ >= debug_dt_ - 1e-6) {
    since_debug_ = 0.0;
    debug_.header.stamp = time;
    debug_.skill = ctx_.skill;
    debug_.run_id = run_id_;
    for (int i = 0; i < 5; ++i) {
      debug_.ref[i] = ref[i];
      debug_.q[i] = q[i];
      debug_.corr[i] = corr_[i];
      debug_.delta[i] = delta_[i];
      debug_.sag_blocked[i] = sag_[i].blocked;
    }
    debug_.still = still;
    debug_.rest = rest;
    debug_.policy_on = it != nullptr;
    debug_.cam_ok = cam_ok;
    debug_.cam_age = cam_age;
    (void)debug_rt_->try_publish(debug_);
  }
  return return_type::OK;
}

}  // namespace ml

PLUGINLIB_EXPORT_CLASS(ml::RlArmCorrection, controller_interface::ChainableControllerInterface)
