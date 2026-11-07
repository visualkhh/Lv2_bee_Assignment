#pragma once

// state_manager 재시작 도우미 — /cmd/restart 처리 (설명은 state_manager.hpp 머리말)
//   "node <이름>"            그 노드 프로세스만: 명령줄·환경·작업 폴더를 /proc에서 그대로 읽어 끄고 → 같은 것으로 다시 띄움
//                            (파라미터 파일 · 리매핑 · 이름이 그대로 = 기존 파라미터 유지)
//   "all [이름:=값 …]"       이 노드를 띄운 ros2 launch 전체: 그 launch 명령줄 그대로 다시 (인자를 주면 그 인자만 바꿈)
// 실제 끄고 띄우는 일은 분리된(setsid) bash 도우미가 한다 — state_manager 자신이 꺼져도(all · node state_manager) 끝까지 감.
// 순수 함수(해석 · 인자 합치기 · 프로세스 고르기)는 /proc 없이 시험할 수 있게 프로세스 목록을 인자로 받는다.
#include <string>
#include <vector>

namespace manager::restart {

struct Request {
  enum Kind { ALL, NODE, INVALID } kind = INVALID;
  std::string node;               // NODE: 노드 이름 (앞 '/' 뺌)
  std::vector<std::string> args;  // ALL: 바꿀 launch 인자 [이름:=값 …]
  std::string error;              // INVALID: 이유
};

struct Proc {
  int pid = 0;
  int ppid = 0;
  std::vector<std::string> argv;
};

// 할 일 — 도우미가 차례로: marker 만들기 → stop(SIGINT) → 다 죽을 때까지 최대 wait_s → kill(남은 것 SIGKILL)
//   → (all) Fast DDS 공유메모리 잔재 삭제 → prepare(bash -lc, 예: 빌드) → marker 지우기 → cwd에서 env로 argv 실행 (출력 log에 이어 씀)
struct Plan {
  std::vector<int> stop;   // SIGINT 보낼 것 (launch 또는 그 노드)
  std::vector<int> kill;   // 시간 안에 안 죽으면 SIGKILL (stop + 그 자손)
  double wait_s = 20.0;
  bool clean_shm = false;
  std::string prepare;     // 다시 띄우기 전에 실행할 셸 명령 (비면 없음)
  std::string marker;      // 도는 동안 있는 파일 (테스트베드 test-faker 0이 이걸 보고 기다림, 비면 없음)
  std::string log;         // 다시 띄운 프로세스 출력
  std::string clear;       // 끈 뒤 지울 파일 (all: 따로 다시 띄운 노드 기록 — 다 껐으니 비움, 비면 없음)
  std::string cwd;
  std::vector<std::string> argv;
  std::vector<std::string> env;  // "이름=값"
};

Request parse(const std::string & text);

// launch 명령줄의 인자(이름:=값)를 바꾼 새 명령줄 — 같은 이름이면 새 값, 없던 이름이면 뒤에 붙임.
// map:=을 바꾸면 예전 map_name:=은 버림 (안 주면 slam은 새 이름 · saved는 기본 — 예전 이름이 남으면 slam이 '있는 이름'으로 거부)
std::vector<std::string> merge_launch_args(const std::vector<std::string> & argv, const std::vector<std::string> & overrides);

// 프로세스가 ros2 launch인가 (python3 …/ros2 launch … · ros2 launch …)
bool is_launch(const Proc & p);
// pid의 조상 중 가장 가까운 ros2 launch (없으면 0)
int launch_root(const std::vector<Proc> & procs, int pid);
// root 아래 자손 pid 전부 (root 제외)
std::vector<int> descendants(const std::vector<Proc> & procs, int root);
// 프로세스의 노드 이름 — __node:=<이름> 리매핑 · 아니면 실행 파일 기본 이름(DEFAULT_NAMES: ros2_control_node → controller_manager 등)
// · 아니면 실행 파일 이름에서 _node를 뗀 것 (perception_node → perception). --ros-args가 없으면(ROS 노드가 아님) ""
std::string node_name(const Proc & p);
// root 아래(+ extra)에서 노드 이름이 name인 프로세스 (node_name 기준 — 리매핑된 프로세스는 실행 파일 이름으로는 안 맞음)
//   extra: launch 밖에서 도는 것도 후보로 (node로 따로 다시 띄워 launch의 자식이 아닌 노드 — registered())
std::vector<int> find_node(const std::vector<Proc> & procs, int root, const std::string & name, const std::vector<int> & extra = {});

// node로 다시 띄운 프로세스 기록 — 그 노드는 launch의 자식이 아니라서 all이 launch 자손만 끄면 살아남아 둘이 됨(실측).
// 파일 한 줄 = pid + 명령줄. registered()는 아직 살아 있고 명령줄도 같은 것만 (pid 재사용에 안 속게)
void record(const std::string & file, int pid, const std::vector<std::string> & argv);
std::vector<int> registered(const std::string & file, const std::vector<Proc> & procs);
// 재시작할 수 있는 노드 [(이름, pid)] — root 아래 + extra 중 ROS 노드 프로세스 (launch 자신 제외, 이름순)
std::vector<std::pair<std::string, int>> restartable(const std::vector<Proc> & procs, int root, const std::vector<int> & extra);

// /proc 읽기
std::vector<Proc> list_procs();
bool read_proc(int pid, std::vector<std::string> & argv, std::vector<std::string> & env, std::string & cwd, std::string & exe);

// 도우미 bash 스크립트 (Plan → 문자열) — 시험용으로 따로
std::string helper_script(const Plan & plan);
// 분리된 도우미로 실행 (fork → setsid → fork → exec bash, 열린 파일은 닫음). 실패면 false + err
//   pid: 다시 띄운 프로세스 pid (도우미가 exec로 바뀌므로 같은 pid)
bool spawn(const Plan & plan, std::string & err, int * pid = nullptr);

}  // namespace manager::restart
