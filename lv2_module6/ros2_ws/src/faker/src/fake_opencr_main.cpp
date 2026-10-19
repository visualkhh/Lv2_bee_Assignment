// fake_opencr 실행 파일 — 노드는 faker::FakeOpenCR (include/faker/fake_opencr.hpp · src/fake_opencr.cpp)
#include "faker/fake_opencr.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeOpenCR>());
  rclcpp::shutdown();
  return 0;
}
