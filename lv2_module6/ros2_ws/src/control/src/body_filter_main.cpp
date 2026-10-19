// body_filter 실행 파일 — 노드는 control::BodyFilter (include/control/body_filter.hpp · src/body_filter.cpp)
#include "control/body_filter.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::BodyFilter>());
  rclcpp::shutdown();
  return 0;
}
