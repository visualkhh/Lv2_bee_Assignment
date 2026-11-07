// state_manager 실행 파일 — 노드는 manager::StateManager (include/manager/state_manager.hpp · src/state_manager.cpp)
#include "manager/state_manager.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<manager::StateManager>());
  rclcpp::shutdown();
  return 0;
}
