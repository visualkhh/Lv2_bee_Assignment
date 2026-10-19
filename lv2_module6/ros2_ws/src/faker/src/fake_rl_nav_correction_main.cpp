// fake_rl_nav_correction 실행 파일 — 노드 rl_nav_correction (include/faker/fake_rl_nav_correction.hpp)
#include "faker/fake_rl_nav_correction.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeRlNavCorrection>());
  rclcpp::shutdown();
  return 0;
}
