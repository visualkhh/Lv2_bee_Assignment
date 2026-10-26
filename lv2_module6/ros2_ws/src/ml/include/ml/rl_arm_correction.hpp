#pragma once

// rl_arm_correction (🛠 직접 작성, 담당: 송정혁) — 설계: documents/project/ros/05-arm.md, 08-ctrl.md
// ROS 노드가 아니라 controller_manager 안에서 arm_controller(JTC) 바로 뒤에 체인으로 붙는 컨트롤러 플러그인.
//   reference interface: rl_arm_correction/<joint>/position  ← JTC가 command_joints로 여기에 쓴다
//   command interface:   <joint>/position                     → hardware interface
//   state interface:     <joint>/position, velocity           (관측)
// update: command = reference + Δ (Δ는 ±max_delta, 기본 0.05rad). enabled:=false 면 Δ=0.
// TODO(송정혁): 관측(목표·현재 관절각·속도·추종 오차·파지 여부·직전 Δ) → ONNX 정책 → Δ
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ml {

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

  // 토픽 입력 없음 — 항상 JTC 뒤 체인 모드로만 쓴다
  return_type update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override;

  return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joints_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs_;
};

}  // namespace ml
