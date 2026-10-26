// body_filter 실행 파일 — 노드는 perception::BodyFilter (include/perception/body_filter.hpp · src/body_filter.cpp)
#include "perception/body_filter.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::BodyFilter>());
  rclcpp::shutdown();
  return 0;
}
