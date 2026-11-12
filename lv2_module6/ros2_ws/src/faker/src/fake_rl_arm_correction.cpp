// fake_rl_arm_correction 구현 — 설명·선언은 include/faker/fake_rl_arm_correction.hpp
#include "faker/fake_rl_arm_correction.hpp"

#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace faker {

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

CallbackReturn FakeRlArmCorrection::on_init() {
  auto_declare<std::vector<std::string>>("joints", {"joint1", "joint2", "joint3", "joint4", "joint5"});
  return CallbackReturn::SUCCESS;
}

CallbackReturn FakeRlArmCorrection::on_configure(const rclcpp_lifecycle::State &) {
  joints_ = get_node()->get_parameter("joints").as_string_array();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FakeRlArmCorrection::on_activate(const rclcpp_lifecycle::State &) {
  for (size_t i = 0; i < refs_.size(); ++i) {  // 시작할 때 reference = 지금 자세
    const auto pos = state_interfaces_[2 * i].get_optional<double>();
    if (pos) (void)refs_[i]->set_value(*pos);
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration FakeRlArmCorrection::command_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) cfg.names.push_back(j + "/" + HW_IF_POSITION);
  return cfg;
}

InterfaceConfiguration FakeRlArmCorrection::state_interface_configuration() const {
  InterfaceConfiguration cfg{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/" + HW_IF_POSITION);
    cfg.names.push_back(j + "/" + HW_IF_VELOCITY);
  }
  return cfg;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> FakeRlArmCorrection::on_export_reference_interfaces_list() {
  refs_.clear();
  for (const auto & j : joints_) {  // 이름은 컨트롤러 이름 기준 → rl_arm_correction/jointN/position (JTC command_joints와 같음)
    hardware_interface::InterfaceInfo info;
    info.name = j + "/" + HW_IF_POSITION;
    info.data_type = "double";
    refs_.push_back(std::make_shared<hardware_interface::CommandInterface>(
      hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return refs_;
}

return_type FakeRlArmCorrection::update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) {
  return return_type::OK;
}

return_type FakeRlArmCorrection::update_and_write_commands(const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto ref = refs_[i]->get_optional<double>();
    if (ref && !std::isnan(*ref)) (void)command_interfaces_[i].set_value(*ref);
  }
  return return_type::OK;
}

}  // namespace faker

PLUGINLIB_EXPORT_CLASS(faker::FakeRlArmCorrection, controller_interface::ChainableControllerInterface)
