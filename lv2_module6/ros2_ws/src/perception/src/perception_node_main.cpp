// perception_node 실행 파일 (C++판) — 노드 설명은 include/perception/perception_node.hpp
#include "perception/perception_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
