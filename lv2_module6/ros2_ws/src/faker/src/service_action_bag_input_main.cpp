// service_action_bag_input 실행 파일 — main만 (구현은 src/service_action_bag_input.cpp)
#include <memory>

#include "faker/service_action_bag_input.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::ServiceActionBagInput>());
  rclcpp::shutdown();
  return 0;
}
