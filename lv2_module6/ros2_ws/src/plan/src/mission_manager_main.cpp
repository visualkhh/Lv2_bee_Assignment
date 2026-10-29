// mission_manager 실행 파일 — 노드는 plan::MissionManager (include/plan/mission_manager.hpp · src/mission_manager.cpp)
#include "plan/mission_manager.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
