// static_esdf_slice 실행 파일 — 노드는 perception::StaticEsdfSlice (include/perception/static_esdf_slice.hpp)
#include "perception/static_esdf_slice.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::StaticEsdfSlice>());
  rclcpp::shutdown();
  return 0;
}
