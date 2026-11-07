// fake_grasp_scorer 실행 파일 — 노드 grasp_scorer (include/faker/fake_grasp_scorer.hpp)
#include "faker/fake_grasp_scorer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<faker::FakeGraspScorer>());
  rclcpp::shutdown();
  return 0;
}
