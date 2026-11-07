// explorer 실행 파일 — 노드는 plan::Explorer (include/plan/explorer.hpp · src/explorer.cpp)
#include "plan/explorer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::Explorer>());
  rclcpp::shutdown();
  return 0;
}
