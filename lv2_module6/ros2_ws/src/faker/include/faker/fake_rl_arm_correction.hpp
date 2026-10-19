#pragma once

// FakeRlArmCorrection (faker) — ml/RlArmCorrection 대역 컨트롤러 플러그인. 같은 자리(arm_controller 뒤 체인)에서
//   reference(rl_arm_correction/jointN/position)를 그대로 command(jointN/position)로 쓴다 (Δ=0, 학습 정책 없음).
//   faker.launch.py ml:=fake(기본)일 때 ros_system arm_correction_type:=faker/FakeRlArmCorrection 으로 끼운다.
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"

namespace faker {

class FakeRlArmCorrection : public controller_interface::ChainableControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joints_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> refs_;
};

}  // namespace faker
