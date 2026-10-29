// manager 테스트: state_manager는 요청을 받아 직접 바꾼 상태만 발행한다
//   /cmd/operation_mode → /state/operation_mode · /cmd/map_mode · /cmd/map_name → launch 다시 띄우기 요청 (/state/restart)
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
  std::string op, map, restart;
  auto op_sub = probe->create_subscription<std_msgs::msg::String>("/state/operation_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { op = m->data; });
  auto map_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { map = m->data; });
  auto restart_sub = probe->create_subscription<std_msgs::msg::String>("/state/restart", latched,
    [&](std_msgs::msg::String::SharedPtr m) { restart = m->data; });
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
  // 지도 모드 = launch 전체를 map:=slam으로 다시 띄우기 요청 — 테스트는 ros2 launch 아래가 아니라 재시작이 거절되고 모드는 그대로
  // (/state/restart는 시작할 때 "시작 (pid …)"을 냄 — 요청이 갔는지는 거절 이유의 "ros2 launch"로 봄)
  ASSERT_TRUE(spin_until(exec, [&] { req_pub->publish(req); }, [&] { return restart.find("ros2 launch") != std::string::npos; }))
    << restart;
  EXPECT_EQ(map, "LOCALIZATION");
}

TEST_F(RosTest, CourseCannotChange) {
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides({{"map_mode", "COURSE"}}));
  auto probe = std::make_shared<rclcpp::Node>("probe2");
  std::string map, restart;
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto map_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { map = m->data; });
  auto restart_sub = probe->create_subscription<std_msgs::msg::String>("/state/restart", latched,
    [&](std_msgs::msg::String::SharedPtr m) { restart = m->data; });
  auto req_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_mode", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  std_msgs::msg::String req;
  req.data = "MAPPING";
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return map == "COURSE"; }));
  spin_until(exec, [&] { req_pub->publish(req); }, [] { return false; }, 0.5);
  EXPECT_EQ(map, "COURSE");
  EXPECT_EQ(restart.find("ros2 launch"), std::string::npos) << restart;  // 다시 띄우기도 안 함 (시작 알림만)
}

TEST_F(RosTest, SwitchMapFolder) {
  // 지도 폴더 갈아끼우기: /cmd/map_name → (폴더에 nvblox.nvblx 있으면) map:=saved map_name:=<이름>으로 다시 띄우기 요청
  const auto dir = std::filesystem::temp_directory_path() / "state_manager_maps";
  std::filesystem::create_directories(dir / "a");
  std::filesystem::create_directories(dir / "c");
  std::ofstream(dir / "a" / "nvblox.nvblx") << "x";
  std::ofstream(dir / "c" / "map.pgm") << "x";  // 미리보기만 있는 폴더 — 목록에 안 나옴
  auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides(
    {{"map_mode", "MAPPING"}, {"map_name", "b"}, {"map_dir", dir.string()}}));
  auto probe = std::make_shared<rclcpp::Node>("probe3");
  std::string name, mode, list, restart;
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  auto name_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_name", latched,
    [&](std_msgs::msg::String::SharedPtr m) { name = m->data; });
  auto mode_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_mode", latched,
    [&](std_msgs::msg::String::SharedPtr m) { mode = m->data; });
  auto list_sub = probe->create_subscription<std_msgs::msg::String>("/state/map_list", latched,
    [&](std_msgs::msg::String::SharedPtr m) { list = m->data; });
  auto restart_sub = probe->create_subscription<std_msgs::msg::String>("/state/restart", latched,
    [&](std_msgs::msg::String::SharedPtr m) { restart = m->data; });
  auto pub = probe->create_publisher<std_msgs::msg::String>("/cmd/map_name", rclcpp::QoS(1).reliable());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);
  ASSERT_TRUE(spin_until(exec, [] {}, [&] { return name == "b" && mode == "MAPPING" && list == "a"; }));  // b는 지도 파일 없음
  std_msgs::msg::String req;
  req.data = "nope";  // 없는 폴더 → 그대로, 다시 띄우지도 않음
  spin_until(exec, [&] { pub->publish(req); }, [] { return false; }, 0.5);
  EXPECT_EQ(name, "b");
  EXPECT_EQ(restart.find("ros2 launch"), std::string::npos) << restart;
  req.data = "a";  // 있는 폴더 → 다시 띄우기 요청 (테스트는 launch 아래가 아니라 거절 — 요청까지 갔는지만)
  ASSERT_TRUE(spin_until(exec, [&] { pub->publish(req); }, [&] { return restart.find("ros2 launch") != std::string::npos; }))
    << restart;
}

TEST_F(RosTest, ExploreOnlyWhileMapping) {
  // EXPLORE는 지도 모드 MAPPING일 때만 · explorer가 /explore/result 를 내면 AUTO로
  for (const std::string start : {"LOCALIZATION", "MAPPING"}) {
    auto node = std::make_shared<StateManager>(rclcpp::NodeOptions().parameter_overrides({{"map_mode", start}}));
    auto probe = std::make_shared<rclcpp::Node>("probe4");
    std::string op;
    auto op_sub = probe->create_subscription<std_msgs::msg::String>("/state/operation_mode",
      rclcpp::QoS(1).reliable().transient_local(), [&](std_msgs::msg::String::SharedPtr m) { op = m->data; });
    auto op_pub = probe->create_publisher<std_msgs::msg::String>("/cmd/operation_mode", rclcpp::QoS(1).reliable());
    auto done_pub = probe->create_publisher<std_msgs::msg::String>("/explore/result", rclcpp::QoS(1).reliable());
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(probe);
    std_msgs::msg::String m;
    m.data = "EXPLORE";
    ASSERT_TRUE(spin_until(exec, [] {}, [&] { return op == "AUTO"; }));
    if (start == "LOCALIZATION") {  // 위치추정 중 → 거절
      spin_until(exec, [&] { op_pub->publish(m); }, [] { return false; }, 0.5);
      EXPECT_EQ(op, "AUTO");
      continue;
    }
    ASSERT_TRUE(spin_until(exec, [&] { op_pub->publish(m); }, [&] { return op == "EXPLORE"; }));
    std_msgs::msg::String done;
    done.data = "DONE 프론티어 없음";
    ASSERT_TRUE(spin_until(exec, [&] { done_pub->publish(done); }, [&] { return op == "AUTO"; }));
  }
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
