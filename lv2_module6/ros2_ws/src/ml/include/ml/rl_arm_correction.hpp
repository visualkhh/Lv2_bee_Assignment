#pragma once

// rl_arm_correction (담당: 송정혁) — 설계: documents/project/ros/05-arm.md, 08-ctrl.md · 배포 결정 2026-11-03 (bee skill-training 9장)
// ROS 노드가 아니라 controller_manager 안에서 arm_controller(JTC) 바로 뒤에 체인으로 붙는 컨트롤러 플러그인.
//   reference interface: rl_arm_correction/<joint>/position  ← JTC가 command_joints로 여기에 쓴다 (기준 궤적 q_ref)
//   command interface:   <joint>/position                     → hardware interface
//   state interface:     <joint>/position, velocity           (관측)
//   구독: /skill_context (ml/SkillContext, 팔 skill 노드 — skills 패키지) · /hand_relative (perception/HandRelative)
// update (update_rate, 100Hz): command = q_ref + 처짐 적분 corr + 학습 보정 Δ
//   - corr = 처짐 보정 적분 (송정혁, isaac/rl/bee_isaac/tasks place_env._sag_step과 같은 법칙, 시간 기준): reference가 멈춰 있을 때만
//     (모든 관절 |d ref/dt| < still_rate) corr += sag_gain·dt·(reference + Δ − 현재각), ±sag_max. 움직이는 동안은 유지.
//     학습은 85 ms마다 오차 30 % 제거 → sag_gain = −ln(0.7)/0.085 ≈ 4.2 /s. 스킬 시작마다 ~/reset_sag 로 0.
//     skill과 상관없이 늘 돈다 (sag_enabled). 발열 방지: stow(rest_pose)에선 0으로 풂 · 상한에 windup_s 붙으면 풀고 쉼
//   - Δ: skill_context에 정책 파일(policy · layout)이 있을 때만 (닿는 구간). 아니면 Δ = 0
//     정책은 skill 노드가 갖고 관리 (models/skills/<skill>/) — 처음 받은 경로는 구독 콜백(실시간 루프 밖)에서 정책을 읽고 검산해 둠 (경로별 한 번).
//     실시간 루프는 미리 잡은 버퍼로 실행만 (작은 MLP — 루프 안 추론 시간은 test_onnx_policy가 잼)
//   - 정책: policy_hz(50)로 arm_obs(…) → Policy(.onnx 또는 .policy) → a ∈ [−1, 1] → residual_live면 Δ = residual_rad·a (아니면 마지막 Δ 유지)
//   - run_id가 바뀌면 corr·Δ·이전 행동을 0부터 (시뮬 에피소드 시작)
//   - 손 카메라 값(/hand_relative)이 max_cam_age_s보다 오래되거나 valid가 아니면 정책을 안 돌림: Δ·행동 기록 그대로
//   - 입력 모양 place 39 · pick 41 · pick_box 67 · pick_far 68 · unload_slot 58 (arm_obs.hpp). pick_box·pick_far·unload_slot은 관절·그리퍼 effort도 읽음
// 중력 앞먹임 (gravity_enabled, 기본 켜짐 · 실행 중 ros2 param set 가능): 서보 엔코더가 못 보는 관절 비틀림 d = c·τ 를 미리 뺀다
//   command = q_ref + corr + Δ − d. c = 1/강성 (실측 feature/isaac-setup isaac/sim/R2S/sag_fit.py: j2 12.2 · j3 18.8 Nm/rad).
//   τ는 현재 엔코더 각의 팔 평면 점질량 모델(gravity::torques, URDF 값). 처짐 적분 오차에도 넣어 적분이 지우지 않는다.
//   gravity_z_offset_m: 자세와 무관한 고정 높이 오차를 올리는 양 [m] — 손가락 끝면 중심을 x·손 피치 그대로 위로 (gravity::lift).
// twist_state (기본 켜짐 · ros_system launch가 하드웨어 twist_state와 같은 값을 줌): 상태 = 엔코더 + d (실제 각, TF = 실제). 앞먹임은 그대로 빼고,
//   처짐 적분 오차·정책 관측에서는 앞먹임을 뺀다 — 상태가 이미 실제 각이라 넣으면 d가 두 번 빠진다.
#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ml/arm_obs.hpp"
#include "ml/policy.hpp"
#include "ml/msg/arm_correction_debug.hpp"
#include "ml/msg/skill_context.hpp"
#include "perception/msg/hand_relative.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ml {

// 처짐 보정 적분 한 스텝 (송정혁): corr + gain·dt·(ref + delta − q), ±max로 자름
inline double sag_step(double corr, double ref, double delta, double q, double dt, double gain, double max) {
  return std::clamp(corr + gain * dt * (ref + delta - q), -max, max);
}

// 발열 방지 (11-03): 처짐 적분이 서보를 계속 밀지 않게
//   rest    = reference가 쉬는 자세(stow) → 적분 안 하고 0으로 풂 (주행 내내 접힌 팔을 밀지 않음)
//   blocked = 적분이 상한(±max)에 windup_s 넘게 붙어 있음 → 처짐이 아니라 무언가에 닿아 못 감 → 0으로 풀고 다음 reset까지 쉼
//   moving  = reference가 움직이는 중 → 그대로 유지 (송정혁 방식)
struct SagJoint {
  double corr = 0.0, sat_s = 0.0;
  bool blocked = false;
};
inline void sag_joint_step(SagJoint & j, bool still, bool rest, double ref, double delta, double q, double dt,
                           double gain, double max, double windup_s) {
  if (rest || j.blocked) {
    j.corr -= j.corr * std::min(1.0, gain * dt);  // 한 번에 튀지 않게 같은 속도로 0까지
    j.sat_s = 0.0;
    return;
  }
  if (!still) return;
  j.corr = sag_step(j.corr, ref, delta, q, dt, gain, max);
  j.sat_s = std::abs(j.corr) >= max - 1e-9 ? j.sat_s + dt : 0.0;
  if (windup_s > 0.0 && j.sat_s >= windup_s) j.blocked = true;
}

// 팔 평면(joint2~4, 피치 축)의 점질량 모델 — 값은 URDF(beebot.urdf), URDF가 바뀌면 여기를 갱신 (isaac/scripts/calib/arm_model.py가 같은 값)
namespace gravity {
constexpr double G = 9.81;
// 질량 [kg]·무게중심 (x, z) [m] — 각 링크 프레임 안. 손 = gripper_link + 손가락 + 카메라를 하나로 (gripper_link 프레임)
constexpr double M_UA = 0.128865, UA_X = 0.008487, UA_Z = 0.104997;  // upper_arm (joint2 뒤)
constexpr double M_FA = 0.117300, FA_X = 0.093796;                   // forearm (joint3 뒤)
constexpr double M_WR = 0.090152, WR_X = 0.043623, WR_Z = 0.011449;  // wrist (joint4 뒤)
constexpr double M_HD = 0.371242, HD_X = 0.052686, HD_Z = 0.014472;  // 손 (joint5 뒤, 롤은 무게중심이 축 위라 무시)
// 관절 위치: upper_arm 안 joint3, forearm 안 joint4, wrist 안 gripper_link 원점, gripper_link 안 손가락 끝면 중심
constexpr double J3_X = 0.024, J3_Z = 0.128, J4_X = 0.124, HAND_X = 0.0629;
constexpr double MARK_X = 0.109055 + 0.0206;  // end_effector_link 0.109055 + 거기서 면까지 0.0206 (plan/stand_plan.hpp FINGER_TIP)

struct V2 {
  double x, z;
};
// 팔 평면 안 벡터를 피치 th [rad]만큼 돌림 (θ=0: 앞 +x, θ=90°: 아래를 가리킴)
inline V2 rot(double th, double x, double z) {
  const double c = std::cos(th), s = std::sin(th);
  return {x * c + z * s, -x * s + z * c};
}

// joint2, 3, 4 중력 토크 [Nm] — 각 관절 축(피치 y) 둘레, 앞으로 처지는 쪽이 +. τ_j = g · Σ m_k (x_k − x_j)
inline std::array<double, 3> torques(double q2, double q3, double q4) {
  const double t23 = q2 + q3, t234 = t23 + q4;
  const V2 ua = rot(q2, UA_X, UA_Z), p3 = rot(q2, J3_X, J3_Z);
  const V2 fa = rot(t23, FA_X, 0.0), p4v = rot(t23, J4_X, 0.0);
  const double p4x = p3.x + p4v.x;
  const double wr = p4x + rot(t234, WR_X, WR_Z).x, hd = p4x + rot(t234, HAND_X + HD_X, HD_Z).x;
  const double fax = p3.x + fa.x;
  return {G * (M_UA * ua.x + M_FA * fax + M_WR * wr + M_HD * hd),
          G * (M_FA * (fax - p3.x) + M_WR * (wr - p3.x) + M_HD * (hd - p3.x)),
          G * (M_WR * (wr - p4x) + M_HD * (hd - p4x))};
}

// 손가락 끝면 중심을 dz [m] 위로 올리는 joint2~4 각도 변화 [rad] (x와 손 피치 그대로): 평면 야코비안 J·Δq = [0, dz, 0]. 풀 수 없으면 false
inline bool lift(double q2, double q3, double q4, double dz, std::array<double, 3> & dq) {
  const double t23 = q2 + q3, t234 = t23 + q4;
  const V2 a = rot(q2, J3_X, J3_Z), b = rot(t23, J4_X, 0.0), c = rot(t234, HAND_X + MARK_X, 0.0);
  // d(rot v)/dθ = (v.z, −v.x) (돌린 뒤 값 기준)
  const double j4x = c.z, j4z = -c.x;
  const double j3x = j4x + b.z, j3z = j4z - b.x;
  const double j2x = j3x + a.z, j2z = j3z - a.x;
  // 행: x, z, 피치(=1,1,1)  열: q2, q3, q4
  const double m[3][3] = {{j2x, j3x, j4x}, {j2z, j3z, j4z}, {1.0, 1.0, 1.0}};
  const double rhs[3] = {0.0, dz, 0.0};
  const auto det3 = [](const double r[3][3]) {
    return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
           r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  };
  const double d = det3(m);
  if (std::abs(d) < 1e-6) return false;
  for (int k = 0; k < 3; ++k) {  // 크래머: k번째 열을 rhs로 바꾼 행렬식 / d
    double t[3][3];
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) t[i][j] = (j == k) ? rhs[i] : m[i][j];
    dq[k] = det3(t) / d;
  }
  return true;
}
}  // namespace gravity

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

class RlArmCorrection : public controller_interface::ChainableControllerInterface {
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  InterfaceConfiguration command_interface_configuration() const override;
  InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;
  // 토픽으로 reference를 받지 않음 — 항상 JTC 뒤 체인 모드
  return_type update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Skill {
    ArmLayout layout = ArmLayout::Place;
    Policy policy;  // .onnx 또는 .policy (meta runtime)
  };
  struct Ctx {  // 받은 skill_context + 그 정책 (없으면 null) — 구독 콜백이 만들고 실시간 루프가 가져감
    Ctx() = default;
    Ctx(const ml::msg::SkillContext & m, std::shared_ptr<const Skill> s) : msg(m), skill(std::move(s)) {}
    ml::msg::SkillContext msg;
    std::shared_ptr<const Skill> skill;
  };
  std::shared_ptr<const Skill> policy_for(const ml::msg::SkillContext & m);  // 실시간 루프 밖 (구독 콜백)
  void reset_run();

  std::vector<std::string> joints_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs_;
  std::map<std::string, std::shared_ptr<const Skill>> cache_;  // 정책 파일 경로 → 읽은 정책 (지우지 않음 — 실시간 루프가 메모리를 풀지 않게)
  std::shared_ptr<const Skill> skill_;                        // 지금 정책 (실시간 쪽)

  realtime_tools::RealtimeThreadSafeBox<Ctx> ctx_box_;
  realtime_tools::RealtimeThreadSafeBox<perception::msg::HandRelative> hand_box_;
  rclcpp::Subscription<ml::msg::SkillContext>::SharedPtr ctx_sub_;
  rclcpp::Subscription<perception::msg::HandRelative>::SharedPtr hand_sub_;

  ml::msg::SkillContext ctx_;            // 마지막으로 받은 것 (실시간 쪽 복사본)
  perception::msg::HandRelative hand_;
  uint32_t run_id_ = 0;
  J5 corr_{}, delta_{}, prev_act_{}, act1_{}, act2_{}, prev_ref_{};
  std::array<SagJoint, 5> sag_{};
  rclcpp::Publisher<ml::msg::ArmCorrectionDebug>::SharedPtr debug_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<ml::msg::ArmCorrectionDebug>> debug_rt_;
  ml::msg::ArmCorrectionDebug debug_;
  double debug_dt_ = 0.05, since_debug_ = 0.0;
  std::vector<double> rest_pose_;  // 비면 쉬는 자세 판정 안 함
  bool have_prev_ = false;
  std::atomic<bool> reset_sag_{false};
  std::atomic<bool> gravity_enabled_{true};  // 파라미터 콜백이 씀 — 실행 중 켜고 끔
  std::atomic<bool> twist_state_{true};      // 같은 콜백 — 상태 = 실제 각 (헤더 설명)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr gravity_cb_;
  double gravity_c2_ = 0.0, gravity_c3_ = 0.0, gravity_c4_ = 0.0, gravity_z_offset_ = 0.0, gravity_ramp_s_ = 2.0, gravity_max_ = 0.2;
  double gravity_scale_ = 0.0;  // 앞먹임 세기 0..1 (켜고 끌 때 서서히)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  double since_policy_ = 0.0;
  std::vector<double> obs_, act_;
  // 파라미터 (on_configure에서 읽음 — 실시간 루프에서 get_parameter 안 함)
  bool enabled_ = true, sag_enabled_ = true;
  double max_delta_ = 0.05, residual_rad_ = 0.05, sag_gain_ = 4.2, sag_max_ = 0.2, still_rate_ = 0.005;
  double rest_tol_ = 0.03, windup_s_ = 2.0;
  double policy_dt_ = 0.02, max_cam_age_s_ = 0.45;
  std::string gripper_joint_ = "gripper_left_joint";
  double arm_effort_limit_ = 2.3, gripper_effort_limit_ = 0.35;
};

}  // namespace ml
