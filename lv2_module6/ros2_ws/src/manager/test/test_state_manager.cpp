// manager 테스트: state_manager는 요청을 받아 직접 바꾼 상태만 발행한다
//   /cmd/operation_mode → /state/operation_mode · /cmd/map_mode → RTAB-Map 서비스 응답 뒤 /state/map_mode · 서버 없으면 그대로
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <chrono>
#include <thread>

#include "manager/state_manager.hpp"

using namespace manager;

template <class Step, class Done>
bool spin_until(rclcpp::Executor & exec, Step step, Done done, double timeout_s = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    step();
    exec.spin_some(std::chrono::milliseconds(50));
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RosTest, OperationAndMapMode) {
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides({{"map_mode", "LOCALIZATION"}}));
  auto probe = std::make_shared<rclcpp::Node>("probe");
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  std::string op, map;
  auto op_sub = probe->create_subscription<std_msgs::msg::String>("/state/operation_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { op = m->data; });
  auto map_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { map = m->data; });
  auto op_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/operation_mode", rclcpp::QoS(1).reliable());
  auto req_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_mode", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std_msgs::msg::String req;
  req.data = "MAPPING";

  // 시작 값
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return op == "AUTO" && map == "LOCALIZATION"; }));
  // 조작 모드
  ASSERT_TRUE(spin_until(exec, [&] { std_msgs::msg::String o; o.data = "MANUAL"; op_pub->publish(o); },
    [&] { return op == "MANUAL"; }));
  // RTAB-Map 서비스가 없으면 지도 모드는 그대로
  spin_until(exec, [&] { req_pub->publish(req); }, [] { return false; }, 0.5);
  EXPECT_EQ(map, "LOCALIZATION");
  // RTAB-Map(대역 서버)이 응답하면 MAPPING
  int calls = 0;
  auto srv = probe->create_service<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_mapping",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) { ++calls; });
  ASSERT_TRUE(spin_until(exec, [&] { req_pub->publish(req); }, [&] { return map == "MAPPING"; }));
  EXPECT_GE(calls, 1);
}

TEST_F(RosTest, CourseCannotChange) {
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides({{"map_mode", "COURSE"}}));
  auto probe = std::make_shared<rclcpp::Node>("probe2");
  std::string map;
  auto map_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::SharedPtr m) { map = m->data; });
  auto req_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_mode", rclcpp::QoS(1).reliable());
  auto srv = probe->create_service<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_mapping",
    [](const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std_msgs::msg::String req;
  req.data = "MAPPING";
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return map == "COURSE"; }));
  spin_until(exec, [&] { req_pub->publish(req); }, [] { return false; }, 0.5);
  EXPECT_EQ(map, "COURSE");
}

TEST_F(RosTest, SwitchMapFolder) {
  // 지도 폴더 갈아끼우기: /cmd/map_name → (폴더에 rtabmap.db 있으면) load_database → set_mode_localization → /state/map_name · LOCALIZATION
  const auto dir = std::filesystem::temp_directory_path() / "state_manager_maps";
  std::filesystem::create_directories(dir / "a");
  std::ofstream(dir / "a" / "rtabmap.db") << "x";
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides(
    {{"map_mode", "MAPPING"}, {"map_name", "b"}, {"map_dir", dir.string()}}));
  auto probe = std::make_shared<rclcpp::Node>("probe3");
  std::string name, mode, loaded;
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto name_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_name", latched,
    [&](std_msgs::msg::String::SharedPtr m) { name = m->data; });
  auto mode_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { mode = m->data; });
  auto load = probe->create_service<rtabmap_msgs::srv::LoadDatabase>("/rtabmap/rtabmap/load_database",
    [&](const std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Request> r, std::shared_ptr<rtabmap_msgs::srv::LoadDatabase::Response>) {
      loaded = r->database_path; });
  auto loc = probe->create_service<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_localization",
    [](const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {});
  auto pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_name", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std::string list;
  auto list_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_list", latched,
    [&](std_msgs::msg::String::SharedPtr m) { list = m->data; });
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return name == "b" && mode == "MAPPING" && list == "a"; }));  // b는 rtabmap.db 없음
  std_msgs::msg::String req;
  req.data = "nope";  // 없는 폴더 → 그대로
  spin_until(exec, [&] { pub->publish(req); }, [] { return false; }, 0.5);
  EXPECT_EQ(name, "b");
  req.data = "a";
  ASSERT_TRUE(spin_until(exec, [&] { pub->publish(req); }, [&] { return name == "a" && mode == "LOCALIZATION"; }));
  EXPECT_EQ(loaded, (dir / "a" / "rtabmap.db").string());
}

TEST_F(RosTest, ExploreOnlyWhileMapping) {
  // EXPLORE는 지도 모드 MAPPING일 때만 · explorer가 /explore/result 를 내면 AUTO로
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides({{"map_mode", "LOCALIZATION"}}));
  auto probe = std::make_shared<rclcpp::Node>("probe4");
  std::string op;
  auto op_sub = probe->create_subscription<std_msgs::msg::String>("/state/operation_mode", rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::SharedPtr m) { op = m->data; });
  auto op_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/operation_mode", rclcpp::QoS(1).reliable());
  auto map_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_mode", rclcpp::QoS(1).reliable());
  auto done_pub = probe->create_publisher<std_msgs::msg::String>("/explore/result", rclcpp::QoS(1).reliable());
  auto srv = probe->create_service<std_srvs::srv::Empty>("/rtabmap/rtabmap/set_mode_mapping",
    [](const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>) {});
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std_msgs::msg::String m;
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return op == "AUTO"; }));
  m.data = "EXPLORE";  // 위치추정 중 → 거절
  spin_until(exec, [&] { op_pub->publish(m); }, [] { return false; }, 0.5);
  EXPECT_EQ(op, "AUTO");
  std_msgs::msg::String mapping;
  mapping.data = "MAPPING";
  spin_until(exec, [&] { map_pub->publish(mapping); }, [] { return false; }, 1.0);
  ASSERT_TRUE(spin_until(exec, [&] { op_pub->publish(m); }, [&] { return op == "EXPLORE"; }));
  std_msgs::msg::String done;
  done.data = "DONE 프론티어 없음";
  ASSERT_TRUE(spin_until(exec, [&] { done_pub->publish(done); }, [&] { return op == "AUTO"; }));
}

TEST_F(RosTest, MotorErrorInDeviceState) {
  // /motor_status (BeebotSystemHardware) joint2 과부하 → /state/devices/arm = error + 세부에 joint2:과부하 · 정상이면 active
  auto node = std::make_shared<StateManager>();
  auto probe = std::make_shared<rclcpp::Node>("probe_motor");
  std::string arm;
  auto arm_sub = probe->create_subscription<std_msgs::msg::String>("/state/devices/arm", rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::SharedPtr m) { arm = m->data; });
  auto js_pub = probe->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  auto motor_pub = probe->create_publisher<control::msg::MotorStatus>("/motor_status", rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  sensor_msgs::msg::JointState js;
  js.name = {"joint1", "joint2", "joint3", "joint4", "joint5"};
  js.position.assign(5, 0.0);
  const auto status = [](int joint2) {
    control::msg::MotorStatus m;
    m.joints = {"joint1", "joint2"};
    m.error = {0, joint2};
    m.text = {"정상", joint2 ? "과부하" : "정상"};
    return m;
  };
  ASSERT_TRUE(spin_until(exec, [&]() { js_pub->publish(js); motor_pub->publish(status(32)); },
    [&]() { return arm == "joint|error|joint1,joint2,joint3,joint4,joint5,joint2:과부하"; }, 8.0)) << arm;
  ASSERT_TRUE(spin_until(exec, [&]() { js_pub->publish(js); motor_pub->publish(status(0)); },
    [&]() { return arm == "joint|active|joint1,joint2,joint3,joint4,joint5"; }, 8.0)) << arm;
}
