// signal_decoder 실행 파일 — 노드는 perception::SignalDecoderNode (include/perception/signal_decoder.hpp · src/signal_decoder.cpp)
#include "perception/signal_decoder.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception::SignalDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
