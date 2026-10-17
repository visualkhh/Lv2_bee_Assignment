#pragma once

// 입력 매니저: monitor_manager의 반대 방향 — 호스트(통제실·시뮬레이션)가 쓴 파일을 토픽으로 발행한다.
//   <debug_dir>/input/topic/<토픽명>/message 에 monitor_manager와 같은 형식 {"type": "...", "data": {...}} 을 쓰면
//   파일이 바뀔 때(mtime)마다 한 번 발행. 토픽·타입을 코드에 적지 않는다 (rosx_introspection이 .msg 정의로 직렬화).
//   예) 텍스트 명령 넣기 (호스트에서):
//     mkdir -p lv2_module6/debug/input/topic/cmd/user_command
//     echo '{"type": "std_msgs/msg/String", "data": {"data": "빨간 퍽을 목표 구역으로 옮겨"}}' > lv2_module6/debug/input/topic/cmd/user_command/message
//   monitor_manager가 쓴 debug/output/topic/<토픽>/message 를 복사해 값만 고쳐 넣어도 된다.
// 시작할 때 이미 있던 파일은 보내지 않는다 (이전 실행의 잔재) — 시작 뒤에 쓰거나 고친 파일만 발행.
// 발행자는 처음 본 순간 만들고, 구독자가 붙을 때까지(최대 2초) 기다렸다 보낸다 — 만들자마자 보내면 discovery 전이라 사라짐.
// 쓰는 쪽은 임시 파일에 쓰고 rename 할 것 (반쯤 쓰인 파일을 읽지 않게). 읽기·변환 실패는 경고만 하고 다음 변경을 기다린다.
// 파라미터: debug_dir(기본 /debug), period_s(폴더 훑는 간격, 기본 0.2)
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

namespace faker {

namespace fs = std::filesystem;

class InputManager : public rclcpp::Node {
public:
  InputManager();

private:
  struct Input {
    fs::file_time_type mtime{};
    std::string type;
    std::shared_ptr<RosMsgParser::Parser> parser;
    rclcpp::GenericPublisher::SharedPtr pub;
    std::shared_ptr<rclcpp::SerializedMessage> pending;
    rclcpp::Time since;
  };

  void scan(bool initial);

  void load(const std::string & topic, const fs::path & file, Input & in);

  void flush(const std::string & topic, Input & in);

  fs::path input_dir_;
  std::map<std::string, Input> inputs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace faker
