// fake_rl_nav_tuner 실행 파일 — 노드 rl_nav_tuner (include/faker/fake_rl_nav_tuner.hpp)
#include "faker/fake_rl_nav_tuner.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeRlNavTuner>());
  rclcpp::shutdown();
  return 0;
}
