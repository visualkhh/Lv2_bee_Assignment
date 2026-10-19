// monitor_manager 구현 — 설명·선언은 include/faker/monitor_manager.hpp, main은 monitor_manager_main.cpp
#include "faker/monitor_manager.hpp"

#include <sys/stat.h>

#include <cmath>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace faker {

MonitorManager::MonitorManager()
: rclcpp::Node("monitor_manager") {
  // debug 폴더를 받는다. 토픽별 파일은 <debug_dir>/output/topic/<토픽>/ 에 저장 (통제실이 읽는 위치)
  debug_dir_ = declare_parameter<std::string>("debug_dir", "/debug");
  period_ = declare_parameter<double>("period_s", 0.2);
  raw_images_ = declare_parameter<bool>("raw_images", true);
  const auto excluded = declare_parameter<std::vector<std::string>>("exclude_topics", std::vector<std::string>{});
  excluded_ = std::set<std::string>(excluded.begin(), excluded.end());
  cloud_frame_ = declare_parameter<std::string>("cloud_frame", "base_footprint");
  cloud_max_points_ = declare_parameter<int>("cloud_max_points", 20000);
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  // 저장 폴더를 시작할 때 비우고 새로 만든다. 못 만들면 바로 알 수 있게 종료
  //   지난 실행의 마지막 메시지가 남으면 지금 값처럼 보임 — Nav2 /plan은 주행이 끝나도 빈 경로를 안 내서 몇 시간 전 경로가 지도에 그려졌음.
  //   바뀔 때만 오는 상태(/state/* · /map · managed_nodes_activated · /robot_description)는 transient_local이라 구독하면 다시 옴.
  //   input/(시뮬레이터 → ROS 요청 · test-faker 재시작 요청)과 output/hardware(가상 시리얼 기록)는 안 건드림
  const fs::path topic_dir = fs::absolute(fs::path(debug_dir_) / "output" / "topic");
  std::error_code ec;
  fs::remove_all(topic_dir, ec);
  fs::create_directories(topic_dir, ec);
  if (ec || !fs::is_directory(topic_dir)) {
    RCLCPP_FATAL(get_logger(), "저장 폴더를 만들 수 없음: %s (%s) — 경로·권한 확인 (root 소유면 chown)",
      topic_dir.c_str(), ec.message().c_str());
    throw std::runtime_error("debug_dir 생성 실패");
  }
  RCLCPP_INFO(get_logger(), "시작 | 저장: %s | ROS_DOMAIN_ID=%zu | raw_images=%s | period_s=%.2f | 제외 %zu개",
    topic_dir.c_str(), get_node_base_interface()->get_context()->get_domain_id(),
    raw_images_ ? "true" : "false", period_, excluded_.size());

  // 지난 실행의 그래프 기록도 지우고 새로 (topic/ · node/ · pkg/)
  for (const char * d : {"topic", "node", "pkg"}) fs::remove_all(fs::path(debug_dir_) / d, ec);
  scan_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() { scan(); });
  graph_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() { dump_graph(); });
  // message는 받을 때마다 쓰지 않고 토픽별 마지막 값만 들고 있다가 period_s마다 한 번 (100Hz 토픽을 다 쓰면
  // Docker Desktop 공유 폴더(gRPC FUSE)가 못 따라가 이 노드가 쓰기만 기다리다 멈춤). 마지막 값은 항상 써진다
  flush_timer_ = create_wall_timer(std::chrono::duration<double>(period_), [this]() { flush_messages(); });
  dump_packages();
  // 5초마다 상태 로그 — 실행 중인지, 실제로 저장되고 있는지 확인용
  status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() { report(); });
  scan();
}

void MonitorManager::report() {
  if (subs_.empty()) {
    RCLCPP_WARN(get_logger(), "구독할 토픽 없음 — 발행 노드가 떠 있는지, ROS_DOMAIN_ID(현재 %zu)·네트워크 확인",
      get_node_base_interface()->get_context()->get_domain_id());
  } else if (written_ == 0) {
    RCLCPP_WARN(get_logger(), "토픽 %zu개 구독 중, 최근 5초 저장 0건 — 발행이 멈췄거나 QoS 불일치",
      seen_.size() - excluded_count_);
  } else {
    RCLCPP_INFO(get_logger(), "토픽 %zu개 구독 중, 최근 5초 저장 %zu건%s",
      seen_.size() - excluded_count_, written_, write_failed_ ? " (쓰기 실패 있음)" : "");
  }
  written_ = 0;
  write_failed_ = 0;
}

void MonitorManager::scan() {
  for (const auto & [topic, types] : get_topic_names_and_types()) {
    // 숨김 토픽(이름 구간이 '_'로 시작, ros2 topic list에도 안 나옴) 제외.
    // Lyrical은 구독마다 <토픽>/_buf_cpu 를 만들어서 이걸 또 구독하면 끝없이 늘어난다.
    // /rosout·/parameter_events는 노드만 있으면 항상 있는 시스템 토픽 → 제외
    if (types.empty() || seen_.count(topic) || topic.find("/_") != std::string::npos ||
      topic == "/rosout" || topic == "/parameter_events")
    {
      continue;
    }
    // 발행자가 아직 없으면(구독자 때문에 이름만 먼저 보임) 다음 훑기로 미룸 — 지금 구독하면 QoS를 발행자에 맞출 수 없어
    // best effort·volatile로 굳어서, 마지막 값 보관(transient_local)으로 바뀔 때만 내는 /map 같은 토픽을 영영 못 받음 (실측)
    if (count_publishers(topic) == 0) continue;
    seen_.insert(topic);
    if (excluded_.count(topic) || (!raw_images_ && types.front() == "sensor_msgs/msg/Image")) {
      RCLCPP_INFO(get_logger(), "제외: %s (%s)", topic.c_str(), types.front().c_str());
      ++excluded_count_;
      continue;
    }
    types_[topic] = types.front();
    write_topic_list();
    subscribe_message(topic, types.front());
    subscribe_image(topic, types.front());
    subscribe_cloud(topic, types.front());
    subscribe_grid(topic, types.front());
  }
}

void MonitorManager::write_topic_list() {
  std::string list = "{\"topics\": [";
  bool first = true;
  for (const auto & [name, type] : types_) {
    if (!first) list += ", ";
    first = false;
    list += "{\"name\": \"" + name + "\", \"type\": \"" + type + "\"}";
  }
  list += "]}\n";
  write_file(fs::path(debug_dir_) / "output" / "topic" / "list.json", list);
}

rclcpp::QoS MonitorManager::qos() { return rclcpp::SensorDataQoS(); }

rclcpp::QoS MonitorManager::qos_for(const std::string & topic) {
  const auto pubs = get_publishers_info_by_topic(topic);
  const auto all = [&](auto pred) {
    return !pubs.empty() && std::all_of(pubs.begin(), pubs.end(), pred);
  };
  rclcpp::QoS q = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  if (all([](const auto & p) { return p.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable; })) {
    q.reliable();
  }
  if (all([](const auto & p) {
      return p.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
    }))
  {
    q.transient_local();
  }
  return q;
}

void MonitorManager::subscribe_message(const std::string & topic, const std::string & type) {
  std::shared_ptr<RosMsgParser::Parser> parser;
  try {
    parser = std::make_shared<RosMsgParser::Parser>(
      topic, RosMsgParser::ROSType(type), RosMsgParser::GetMessageDefinition(type));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "message 저장 불가 %s (%s): %s", topic.c_str(), type.c_str(), e.what());
    return;
  }
  const auto msg_qos = qos_for(topic);
  subs_.push_back(create_generic_subscription(topic, type, msg_qos,
    [this, topic, type, parser](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      const auto & raw = msg->get_rcl_serialized_message();
      RosMsgParser::NanoCDR_Deserializer deserializer;
      std::string data;
      try {
        parser->deserializeIntoJson({raw.buffer, raw.buffer_length}, &data, &deserializer, 0, true);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s JSON 변환 실패: %s",
          topic.c_str(), e.what());
        return;
      }
      pending_[topic] = "{\"type\": \"" + type + "\", \"data\": " + data + "}\n";  // flush_messages가 씀
    }));
  RCLCPP_INFO(get_logger(), "message 저장 시작: %s (%s, %s%s)", topic.c_str(), type.c_str(),
    msg_qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "reliable" : "best-effort",
    msg_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? ", transient_local" : "");
}

void MonitorManager::subscribe_image(const std::string & topic, const std::string & type) {
  if (type == "sensor_msgs/msg/CompressedImage") {
    subs_.push_back(create_subscription<sensor_msgs::msg::CompressedImage>(topic, qos(),
      [this, topic](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg) {
        if (due(topic + "#image")) {
          // NOTE (Lyrical): data가 std::vector가 아님(rosidl_buffer) → 복사 없이 Mat으로 감싸 디코드
          const cv::Mat buf(1, static_cast<int>(msg->data.size()), CV_8UC1,
            const_cast<uint8_t *>(msg->data.data()));
          save_image(topic, cv::imdecode(buf, cv::IMREAD_UNCHANGED));
        }
      }));
  } else if (type == "sensor_msgs/msg/Image") {
    subs_.push_back(create_subscription<sensor_msgs::msg::Image>(topic, qos(),
      [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        if (!due(topic + "#image")) {
          return;
        }
        try {
          if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            msg->encoding == sensor_msgs::image_encodings::MONO16)
          {
            save_depth(topic, cv_bridge::toCvShare(msg)->image);  // jpg로는 깊이 값이 깨진다
            return;
          }
          // mono8 마스크 등은 그대로, 컬러는 BGR로 맞춰 저장
          const bool mono = sensor_msgs::image_encodings::numChannels(msg->encoding) == 1;
          save_image(topic, cv_bridge::toCvShare(msg, mono ? "mono8" : "bgr8")->image);
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 변환 실패: %s",
            topic.c_str(), e.what());
        }
      }));
  } else {
    return;
  }
  RCLCPP_INFO(get_logger(), "image 저장 시작: %s", topic.c_str());
}

void MonitorManager::subscribe_cloud(const std::string & topic, const std::string & type) {
  if (type != "sensor_msgs/msg/PointCloud2") return;
  subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(topic, qos(),
    [this, topic](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      if (due(topic + "#cloud")) save_cloud(topic, *msg);
    }));
  RCLCPP_INFO(get_logger(), "points 저장 시작: %s (→ %s)", topic.c_str(), cloud_frame_.c_str());
}

void MonitorManager::save_cloud(const std::string & topic, const sensor_msgs::msg::PointCloud2 & cloud) {
  tf2::Transform T;
  try {  // 가장 최근 TF (점구름 시각과 몇십 ms 차이 — 지도 표시용이라 충분)
    tf2::fromMsg(tf_->lookupTransform(cloud_frame_, cloud.header.frame_id, tf2::TimePointZero).transform, T);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s → %s TF 없음: %s", topic.c_str(), cloud_frame_.c_str(), e.what());
    return;
  }
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  const size_t stride = std::max<size_t>(1, (n + cloud_max_points_ - 1) / std::max(1, cloud_max_points_));
  std::vector<float> xyz;
  // 빈 점구름(RTAB-Map 등이 냄)은 반복자가 빈 버퍼를 건드려 프로세스가 죽는다(실측 abort) → 빈 결과로 씀.
  // x 필드가 없는 점구름은 반복자가 예외 → 건너뜀
  if (n > 0 && !cloud.data.empty()) {
    try {
      xyz.reserve(n / stride * 3 + 3);
      sensor_msgs::PointCloud2ConstIterator<float> it(cloud, "x");
      for (size_t i = 0; i < n; ++i, ++it) {
        if (i % stride || !std::isfinite(it[0]) || !std::isfinite(it[1]) || !std::isfinite(it[2])) continue;
        const tf2::Vector3 p = T * tf2::Vector3(it[0], it[1], it[2]);
        xyz.insert(xyz.end(), {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s 점구름 읽기 실패: %s", topic.c_str(), e.what());
      return;
    }
  }
  char head[160];
  std::snprintf(head, sizeof(head), "{\"frame\": \"%s\", \"stamp\": %.3f, \"count\": %zu}\n", cloud_frame_.c_str(),
    rclcpp::Time(cloud.header.stamp).seconds(), xyz.size() / 3);
  write_file(dir_of(topic) / "data",
    std::string(head) + (xyz.empty() ? std::string() : std::string(reinterpret_cast<const char *>(xyz.data()), xyz.size() * sizeof(float))));
}

void MonitorManager::flush_messages() {
  for (auto & [topic, text] : pending_) write_file(dir_of(topic) / "message", text);
  pending_.clear();
}

void MonitorManager::subscribe_grid(const std::string & topic, const std::string & type) {
  if (type != "nav_msgs/msg/OccupancyGrid") return;
  subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(topic, qos_for(topic),
    [this, topic](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & m) {
      if (!due(topic + "#grid")) return;
      const auto & q = m->info.origin.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      char head[256];
      std::snprintf(head, sizeof(head),
        "{\"frame\": \"%s\", \"stamp\": %.3f, \"resolution\": %.5f, \"width\": %u, \"height\": %u, \"origin\": [%.4f, %.4f, %.4f]}\n",
        m->header.frame_id.c_str(), rclcpp::Time(m->header.stamp).seconds(), m->info.resolution, m->info.width, m->info.height,
        m->info.origin.position.x, m->info.origin.position.y, yaw);
      std::string out(head);
      if (!m->data.empty()) out.append(reinterpret_cast<const char *>(m->data.data()), m->data.size());
      write_file(dir_of(topic) / "data", out);
    }));
  RCLCPP_INFO(get_logger(), "grid 저장 시작: %s", topic.c_str());
}

bool MonitorManager::due(const std::string & key) {
  const auto t = now();
  auto it = last_.find(key);
  if (it != last_.end() && (t - it->second).seconds() < period_) {
    return false;
  }
  last_[key] = t;
  return true;
}

namespace {

std::string qos_text(const rclcpp::QoS & q) {
  const auto & p = q.get_rmw_qos_profile();
  return std::string(p.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "reliable" : "best_effort") + ", " +
         (p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "transient_local" : "volatile");
}

std::string endpoints(const std::vector<rclcpp::TopicEndpointInfo> & list) {
  std::string out;
  for (const auto & e : list) {
    const std::string ns = e.node_namespace() == "/" ? "/" : e.node_namespace() + "/";
    out += "  - " + ns + e.node_name() + " (" + qos_text(e.qos_profile()) + ")\n";
  }
  return out;
}

std::string names_types(const std::map<std::string, std::vector<std::string>> & m) {
  std::string out;
  for (const auto & [name, types] : m) out += "    " + name + ": " + (types.empty() ? "" : types.front()) + "\n";
  return out;
}

}  // namespace

void MonitorManager::write_if_changed(const fs::path & out, const std::string & text) {
  auto & last = written_text_[out.string()];
  if (last == text) return;
  last = text;
  write_file(out, text);
}

void MonitorManager::dump_graph() {
  const fs::path base(debug_dir_);
  // 토픽: 타입 · 발행/구독 노드 (ros2 topic info -v 와 같은 내용)
  std::string list;
  for (const auto & [topic, types] : get_topic_names_and_types()) {
    if (types.empty() || topic.find("/_") != std::string::npos) continue;
    list += topic + "\n";
    const fs::path d = base / "topic" / topic.substr(1);
    write_if_changed(d / "type", types.front() + "\n");
    const auto pubs = get_publishers_info_by_topic(topic), subs = get_subscriptions_info_by_topic(topic);
    write_if_changed(d / "info", "Type: " + types.front() + "\nPublisher count: " + std::to_string(pubs.size()) + "\n" +
      endpoints(pubs) + "Subscription count: " + std::to_string(subs.size()) + "\n" + endpoints(subs));
  }
  write_if_changed(base / "topic" / "list", list);

  // 노드: pub/sub/서비스 (ros2 node info) · 파라미터 (ros2 param dump, 노드마다 1회)
  std::string nodes;
  const auto g = get_node_graph_interface();
  for (const auto & [name, ns] : g->get_node_names_and_namespaces()) {
    if (name.empty() || name[0] == '_') continue;  // ros2 CLI 임시 노드
    const std::string full = (ns == "/" ? "" : ns) + "/" + name;
    nodes += full + "\n";
    const fs::path d = base / "node" / full.substr(1);
    try {
      write_if_changed(d / "info", full + "\n  Subscribers:\n" + names_types(g->get_subscriber_names_and_types_by_node(name, ns)) +
        "  Publishers:\n" + names_types(g->get_publisher_names_and_types_by_node(name, ns)) +
        "  Service Servers:\n" + names_types(g->get_service_names_and_types_by_node(name, ns)) +
        "  Service Clients:\n" + names_types(g->get_client_names_and_types_by_node(name, ns)));
    } catch (const std::exception &) {
      continue;  // 그 사이 노드가 사라짐
    }
    // 파라미터 (노드마다 1회): 이름 목록 → 다음 주기에 값. 값 요청을 목록 응답 콜백 안에서 보내면 응답이 안 온다(실측)
    if (full == "/" + std::string(get_name())) continue;  // 자기 자신에게 물으면 같은 실행기에서 응답 못 보냄
    auto & pq = params_[full];
    if (!pq.client) pq.client = std::make_shared<rclcpp::AsyncParametersClient>(this, full);
    if (pq.state == ParamQuery::NEW && pq.client->service_is_ready()) {
      pq.state = ParamQuery::LISTING;
      pq.client->list_parameters({}, 0, [&pq](std::shared_future<rcl_interfaces::msg::ListParametersResult> f) {
        pq.names = f.get().names;
        pq.state = ParamQuery::LISTED;
      });
    } else if (pq.state == ParamQuery::LISTED) {
      pq.state = ParamQuery::DONE;
      pq.client->get_parameters(pq.names, [this, d](std::shared_future<std::vector<rclcpp::Parameter>> g) {
        std::string text;
        for (const auto & p : g.get()) text += p.get_name() + ": " + p.value_to_string() + "\n";
        write_file(d / "params", text);
      });
    }
  }
  write_if_changed(base / "node" / "list", nodes);
}

void MonitorManager::dump_packages() {
  const fs::path base = fs::path(debug_dir_) / "pkg";
  std::string list;
  for (const auto & [pkg, prefix] : ament_index_cpp::get_packages_with_prefixes()) {
    list += pkg + "\n";
    write_file(base / pkg / "prefix", prefix + "\n");
    std::string exes;
    std::error_code ec;
    for (const auto & e : fs::directory_iterator(fs::path(prefix) / "lib" / pkg, ec)) {
      if (e.is_regular_file() && (fs::status(e.path()).permissions() & fs::perms::owner_exec) != fs::perms::none) {
        exes += pkg + " " + e.path().filename().string() + "\n";
      }
    }
    write_file(base / pkg / "executables", exes);
  }
  write_file(base / "list", list);
}

fs::path MonitorManager::dir_of(const std::string & topic) const {
  return fs::path(debug_dir_) / "output" / "topic" / topic.substr(1);
}

fs::path MonitorManager::prepare_tmp(const fs::path & out) {
  std::error_code ec;
  fs::create_directories(out.parent_path(), ec);
  ::chmod(out.parent_path().c_str(), 0777);  // 컨테이너(root)가 만들어도 호스트가 rename으로 교체 가능
  return out.parent_path() / (".tmp-" + out.filename().string());
}

void MonitorManager::write_file(const fs::path & out, const std::string & text) {
  const fs::path tmp = prepare_tmp(out);
  std::ofstream file(tmp);
  file << text;
  file.close();
  std::error_code ec;
  if (!file || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "쓰기 실패: %s — 권한·디스크 확인", out.c_str());
    return;
  }
  ++written_;
}

void MonitorManager::save_image(const std::string & topic, const cv::Mat & image) {
  if (image.empty()) {
    return;
  }
  const fs::path out = dir_of(topic) / "image.jpg";
  const fs::path tmp = prepare_tmp(out);  // 확장자 .jpg 유지 → imwrite가 형식 판별
  std::error_code ec;
  if (!cv::imwrite(tmp.string(), image) || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "이미지 쓰기 실패: %s", out.c_str());
    return;
  }
  ++written_;
}

void MonitorManager::save_depth(const std::string & topic, const cv::Mat & depth) {
  const fs::path out = dir_of(topic) / "depth.png";
  const fs::path tmp = prepare_tmp(out);
  std::error_code ec;
  if (!cv::imwrite(tmp.string(), depth) || (fs::rename(tmp, out, ec), ec)) {
    ++write_failed_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth 쓰기 실패: %s", out.c_str());
    return;
  }
  ++written_;
}

}  // namespace faker
