// 재시작 도우미 테스트: /cmd/restart 해석 · launch 인자 합치기 · 프로세스 고르기 · 도우미가 끄고 같은 명령으로 다시 띄우기
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "manager/restart.hpp"

using namespace manager::restart;

TEST(Restart, Parse) {
  EXPECT_EQ(parse("all").kind, Request::ALL);
  auto r = parse("all map:=slam urdf:=beebot_normal");
  ASSERT_EQ(r.kind, Request::ALL);
  EXPECT_EQ(r.args, (std::vector<std::string>{"map:=slam", "urdf:=beebot_normal"}));
  r = parse("node /perception");
  ASSERT_EQ(r.kind, Request::NODE);
  EXPECT_EQ(r.node, "perception");
  EXPECT_EQ(parse("").kind, Request::INVALID);
  EXPECT_EQ(parse("node").kind, Request::INVALID);
  EXPECT_EQ(parse("node a b").kind, Request::INVALID);
  EXPECT_EQ(parse("all slam").kind, Request::INVALID);       // 인자는 이름:=값만
  EXPECT_EQ(parse("map:=slam").kind, Request::INVALID);      // 예전 형식 (all 없이)은 거절
}

TEST(Restart, MergeLaunchArgs) {
  const std::vector<std::string> argv{"/usr/bin/python3", "/opt/ros/lyrical/bin/ros2", "launch", "faker", "faker.launch.py",
                                      "map:=saved", "map_name:=old", "urdf:=default"};
  EXPECT_EQ(merge_launch_args(argv, {}), argv);  // 그대로
  auto m = merge_launch_args(argv, {"urdf:=beebot_normal", "mock_hardware:=true"});
  EXPECT_EQ(m, (std::vector<std::string>{"/usr/bin/python3", "/opt/ros/lyrical/bin/ros2", "launch", "faker", "faker.launch.py",
                                         "map:=saved", "map_name:=old", "urdf:=beebot_normal", "mock_hardware:=true"}));
  m = merge_launch_args(argv, {"map:=slam"});  // map을 바꾸면 예전 map_name은 버림
  EXPECT_EQ(std::count(m.begin(), m.end(), "map_name:=old"), 0);
  EXPECT_EQ(m.back(), "map:=slam");
  m = merge_launch_args(argv, {"map:=saved", "map_name:=new"});
  EXPECT_EQ(std::count(m.begin(), m.end(), "map_name:=new"), 1);
  EXPECT_EQ(std::count(m.begin(), m.end(), "map_name:=old"), 0);
}

TEST(Restart, FindNode) {
  const std::vector<Proc> procs{
    {1, 0, {"/sbin/init"}},
    {10, 1, {"/bin/bash", "/usr/local/bin/test-faker", "0"}},
    {20, 10, {"/usr/bin/python3", "/opt/ros/lyrical/bin/ros2", "launch", "faker", "faker.launch.py"}},
    {21, 20, {"/ws/install/manager/lib/manager/state_manager", "--ros-args", "--params-file", "/tmp/p.yaml"}},
    {22, 20, {"/usr/bin/python3", "/ws/install/perception/lib/perception/perception_node", "--ros-args"}},
    {23, 20, {"/ws/install/faker/lib/faker/fake_grasp_scorer", "--ros-args", "-r", "__node:=grasp_scorer"}},
    {24, 20, {"/opt/ros/lyrical/lib/nav2_planner/planner_server", "--ros-args", "-r", "__node:=planner_server"}},
    {25, 20, {"/ws/install/faker/lib/faker/fake_camera", "--ros-args", "-r", "__node:=pole_cam"}},
    {26, 20, {"/ws/install/faker/lib/faker/fake_camera", "--ros-args", "-r", "__node:=gripper_cam"}},
    {30, 1, {"/ws/install/perception/lib/perception/perception_node", "--ros-args"}},  // launch 밖 — extra로 줄 때만
  };
  EXPECT_EQ(launch_root(procs, 21), 20);
  EXPECT_EQ(launch_root(procs, 30), 0);
  EXPECT_EQ(find_node(procs, 20, "perception"), std::vector<int>{22});         // 실행 파일 perception_node
  EXPECT_EQ(find_node(procs, 20, "state_manager"), std::vector<int>{21});
  EXPECT_EQ(find_node(procs, 20, "grasp_scorer"), std::vector<int>{23});       // 리매핑 이름
  EXPECT_TRUE(find_node(procs, 20, "fake_grasp_scorer").empty());              // 리매핑됐으면 실행 파일 이름으론 아님
  EXPECT_EQ(find_node(procs, 20, "/gripper_cam"), std::vector<int>{26});
  EXPECT_TRUE(find_node(procs, 20, "fake_camera").empty());
  EXPECT_TRUE(find_node(procs, 20, "nobody").empty());
  // node로 따로 다시 띄워 launch 밖(init 밑)에 있는 노드도 기록에 있으면 후보
  EXPECT_EQ(find_node(procs, 20, "perception", {30}).size(), 2u);
  EXPECT_EQ(node_name(procs[2]), "");  // launch
  const Proc rcn{40, 20, {"/opt/ros/lyrical/lib/controller_manager/ros2_control_node", "--ros-args", "--params-file", "x"}};
  EXPECT_EQ(node_name(rcn), "controller_manager");  // 실행 파일 기본 이름이 다른 것
  EXPECT_EQ(node_name({41, 20, {"/bin/sleep", "3"}}), "");  // ROS 노드가 아님
  auto r = restartable(procs, 20, {30});
  ASSERT_EQ(r.size(), 7u);  // state_manager · perception · grasp_scorer · planner_server · pole_cam · gripper_cam + 밖의 perception
  EXPECT_EQ(r.front().first, "grasp_scorer");
  auto d = descendants(procs, 20);
  EXPECT_EQ(d.size(), 6u);
  EXPECT_EQ(std::count(d.begin(), d.end(), 30), 0);
}

TEST(Restart, HelperStopsAndRelaunches) {
  // 오래 도는 대상 프로세스 → 도우미가 SIGINT로 끄고 → marker를 지우고 → 같은 자리에서 새 명령을 실행하는지
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / ("restart_test_" + std::to_string(getpid()));
  fs::create_directories(dir);
  const pid_t victim = fork();
  if (victim == 0) {
    signal(SIGINT, SIG_IGN);  // &로 띄운 launch처럼 SIGINT를 무시 → 도우미가 SIGTERM으로 끄는지
    execl("/bin/sleep", "sleep", "30", nullptr);
    _exit(127);
  }
  Plan plan;
  plan.stop = {victim};
  plan.kill = {victim};
  plan.wait_s = 5.0;
  plan.marker = (dir / "pending").string();
  plan.log = (dir / "log").string();
  plan.cwd = dir.string();
  plan.argv = {"/bin/sh", "-c", "echo \"again $FOO\" > relaunched; pwd >> relaunched"};
  plan.env = {"FOO=same env"};
  std::string err;
  int new_pid = 0;
  ASSERT_TRUE(spawn(plan, err, &new_pid)) << err;
  EXPECT_GT(new_pid, 0);
  int status = 0;
  ASSERT_EQ(waitpid(victim, &status, 0), victim);  // 도우미가 대상을 끔
  EXPECT_TRUE(WIFSIGNALED(status) && WTERMSIG(status) == SIGTERM);  // SIGINT 무시 → 3초 뒤 SIGTERM
  const auto out = dir / "relaunched";
  for (int i = 0; i < 50 && !fs::exists(out); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::ifstream in(out);
  std::string l1, l2;
  std::getline(in, l1);
  std::getline(in, l2);
  EXPECT_EQ(l1, "again same env");   // 같은 환경으로
  EXPECT_EQ(fs::path(l2), dir);      // 같은 작업 폴더에서
  EXPECT_FALSE(fs::exists(plan.marker));
  fs::remove_all(dir);
}

TEST(Restart, Registry) {
  const std::string file = std::filesystem::temp_directory_path() / ("restart_reg_" + std::to_string(getpid()));
  std::filesystem::remove(file);
  const std::vector<Proc> procs{{30, 1, {"/x/perception_node", "--ros-args"}}, {31, 1, {"/x/other"}}};
  record(file, 30, {"/x/perception_node", "--ros-args"});
  record(file, 31, {"/x/was_different"});   // pid는 있지만 명령줄이 다름 = 재사용된 pid → 아님
  record(file, 40, {"/x/gone"});            // 이미 없음
  EXPECT_EQ(registered(file, procs), std::vector<int>{30});
  std::filesystem::remove(file);
}
