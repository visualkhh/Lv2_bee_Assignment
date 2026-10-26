// signal_decoder 실행 파일 — 노드는 plan::SignalDecoderNode (include/plan/signal_decoder.hpp · src/signal_decoder.cpp)
#include "plan/signal_decoder.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan::SignalDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
