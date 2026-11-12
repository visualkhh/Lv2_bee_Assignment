// fake_nvblox 실행 파일 — 노드 faker::FakeNvblox (include/faker/fake_nvblox.hpp)
#include "faker/fake_nvblox.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeNvblox>());
  rclcpp::shutdown();
  return 0;
}
