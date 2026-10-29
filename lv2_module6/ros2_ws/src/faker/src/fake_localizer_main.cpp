// fake_localizer 실행 파일 — 노드 fake_localizer (include/faker/fake_localizer.hpp)
#include "faker/fake_localizer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeLocalizer>());
  rclcpp::shutdown();
  return 0;
}
