// fake_cuvslam 실행 파일 — 노드 fake_cuvslam (include/faker/fake_cuvslam.hpp)
#include "faker/fake_cuvslam.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeCuvslam>());
  rclcpp::shutdown();
  return 0;
}
