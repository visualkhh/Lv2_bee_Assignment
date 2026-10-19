// input_manager 구현 — 설명·선언은 include/faker/input_manager.hpp, main은 input_manager_main.cpp
#include "faker/input_manager.hpp"

namespace faker {

InputManager::InputManager()
: rclcpp::Node("input_manager") {
  input_dir_ = fs::path(declare_parameter<std::string>("debug_dir", "/debug")) / "input" / "topic";
  const double period = declare_parameter<double>("period_s", 0.2);
  std::error_code ec;
  fs::create_directories(input_dir_, ec);
  RCLCPP_INFO(get_logger(), "시작 | 입력 폴더: %s (<토픽>/message 에 {\"type\", \"data\"} JSON)", input_dir_.c_str());
  scan(true);
  timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() { scan(false); });
}

void InputManager::scan(bool initial) {
  std::error_code ec;
  std::vector<fs::path> done;
  for (auto it = fs::recursive_directory_iterator(input_dir_, ec); !ec && it != fs::recursive_directory_iterator();
    it.increment(ec))
  {
    if (it->path().filename() != "message" || !it->is_regular_file()) continue;
    const std::string topic = "/" + fs::relative(it->path().parent_path(), input_dir_).generic_string();
    auto & in = inputs_[topic];
    const auto mtime = fs::last_write_time(it->path(), ec);
    if (ec || mtime == in.mtime) continue;
    in.mtime = mtime;
    // 요청은 한 번만: 읽은 파일은 발행하기 전에 지운다 — 다시 떠도(재시작) 같은 요청을 또 발행하지 않게 (/cmd/restart가 되풀이되던 것)
    // 시작할 때 남아 있던 파일 = 지난 실행의 요청 → 발행하지 않고 지움
    if (!initial) load(topic, it->path(), in);
    else RCLCPP_INFO(get_logger(), "지난 요청 파일은 발행하지 않고 지움: %s", topic.c_str());
    done.push_back(it->path());
  }
  for (const auto & p : done) fs::remove(p, ec);  // 도는 중에 지우지 않고 다 돈 뒤에
  for (auto & [topic, in] : inputs_) flush(topic, in);
}

void InputManager::load(const std::string & topic, const fs::path & file, Input & in) {
  std::stringstream ss;
  ss << std::ifstream(file).rdbuf();
  rapidjson::Document doc;
  doc.Parse(ss.str().c_str());
  if (doc.HasParseError() || !doc.IsObject() || !doc.HasMember("type") || !doc["type"].IsString() ||
    !doc.HasMember("data"))
  {
    RCLCPP_WARN(get_logger(), "%s: JSON 형식 오류 — {\"type\": \"pkg/msg/Type\", \"data\": {...}}", file.c_str());
    return;
  }
  const std::string type = doc["type"].GetString();
  if (topic.rfind("/state/", 0) == 0) {  // 상태 토픽은 state_manager만 발행 — 바꾸려면 요청 토픽 /cmd/<이름>
    RCLCPP_WARN(get_logger(), "%s: /state/* 는 state_manager만 발행 — /cmd/<이름> 으로 보낼 것", topic.c_str());
    return;
  }
  try {
    if (type != in.type) {
      in.parser = std::make_shared<RosMsgParser::Parser>(
        topic, RosMsgParser::ROSType(type), RosMsgParser::GetMessageDefinition(type));
      in.pub = create_generic_publisher(topic, type, rclcpp::QoS(10).reliable());
      in.type = type;
      in.since = now();
      RCLCPP_INFO(get_logger(), "발행자 생성: %s (%s)", topic.c_str(), type.c_str());
    }
    rapidjson::StringBuffer data;
    rapidjson::Writer<rapidjson::StringBuffer> writer(data);
    doc["data"].Accept(writer);
    RosMsgParser::NanoCDR_Serializer serializer;
    if (!in.parser->serializeFromJson(data.GetString(), &serializer)) throw std::runtime_error("serializeFromJson 실패");
    auto msg = std::make_shared<rclcpp::SerializedMessage>(serializer.getBufferSize());
    auto & raw = msg->get_rcl_serialized_message();
    std::memcpy(raw.buffer, serializer.getBufferData(), serializer.getBufferSize());
    raw.buffer_length = serializer.getBufferSize();
    in.pending = msg;
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "%s (%s) 변환 실패: %s", topic.c_str(), type.c_str(), e.what());
  }
}

void InputManager::flush(const std::string & topic, Input & in) {
  if (!in.pending) return;
  if (in.pub->get_subscription_count() == 0 && (now() - in.since).seconds() < 2.0) return;
  in.pub->publish(*in.pending);
  in.pending.reset();
  RCLCPP_INFO(get_logger(), "발행: %s (구독자 %zu)", topic.c_str(), in.pub->get_subscription_count());
}

}  // namespace faker
