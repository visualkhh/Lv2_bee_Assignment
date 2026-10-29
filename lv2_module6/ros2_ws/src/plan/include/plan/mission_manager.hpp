#pragma once

// mission_manager (🛠 직접 작성) — 주사위 기억 미션 (documents/mission/README.md) 순서를 진행한다
//   설계 결정 D1~D13: documents/proposal/mission-perception-interface.md §11
//   입력: ✉️ /cmd/mission (std_msgs/String "start" | "abort") · /mission/signal · /mission/barcode (perception)
//         ✉️ /tracked_objects (퍽 tag·color·shape · 장소 태그 location · last_seen) · /task/status (coordinator 결과)
//         ✉️ photo_topic (사진 찍을 때만 잠깐 구독) · TF map → base_footprint
//   출력: ✉️ /task/sub_goals (SubGoalArray, plan_id = PLAN_ID_BASE + n — vlm_task_planner와 겹치지 않게) → coordinator
//         ✉️ /mission/signal/reset · /mission/state (plan/MissionState, latched — D10) · /mission/markers (RViz)
//         ✉️ 기록 log_dir/<시작 시각>/ signal · allocation · build · verify · return · error .json + photos/ (D13) · /diagnostics
//   단계 (순서와 실패 시 갈 곳은 mission_solver next_phase — D9):
//     GO_OBSERVE 팔 scan 자세 + 바코드 보는 자세로 → BARCODE 이번에 본 바코드 → 관측 자세(−X)로 → READ 12회 점멸
//     → SURVEY 시작 박스 6곳 전부: 퍽 두 자리 inspect(tag·색·모양) → 읽은 퍽을 적재함 빈 칸에 싣기 (칸 → tag 기억, D2·D3)
//              → 12개 한 번에 할당 (mission_solver allocate)
//     → BUILD 퍽마다: 장소 앞 → 장소 태그로 칸 위치 보정(D4) → [적재함 칸에서 꺼냄 | 못 실은 퍽은 시작 박스에서 직접 집어 옴]
//              → place 칸 "x,y,z" → 내려다보고 10 mm 밖·쓰러짐이면 다시 집어 놓기 (D5)
//     → VERIFY 장소마다 새로 본 퍽만으로 칸별 판정 + 12개 중복·누락 (D11) → RETURN 출발 구역 가운데 → 외곽이 구역 안인지
//              확인·보정 (D12) → home_hold_s 정지 → DONE
//   실패 (D1·D6): skill마다 retries번 다시 · 그래도 안 되면 그 퍽·장소만 실패로 results에 남기고 다음으로 (복귀까지).
//     관측(신호·바코드) 오류와 복귀 이동 실패만 ERROR로 멈추고 coordinator 정지
//   좌표: 설정은 경기장(arena, m) — start_pose_arena 하나로 map으로 바꾼다 (mission_solver arena_to_map).
//   판단(정렬·배정·할당·칸·판정)은 전부 mission_solver 순수 함수. 이 노드는 순서·기록·skill 요청만.
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/mission_signal.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/mission_solver.hpp"
#include "plan/msg/mission_state.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace plan {

using plan::msg::MissionState;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

struct Box { double x, y, half; };  // arena [m] — 정사각 박스 (윗면 = box_top_z)

class MissionManager : public rclcpp::Node {
public:
  MissionManager();

private:
  using Done = std::function<void(bool ok, const std::string & reason)>;
  using TrackedObject = perception::msg::TrackedObject;

  void on_command(const std::string & cmd);
  // 단계 — enter(p)가 그 단계를 시작하고, 단계가 끝나면 finish(ok)가 next_phase로 다음 단계를 정한다
  void enter(Phase p, const std::string & why = "");
  void finish(bool ok, const std::string & why = "");
  void note(const std::string & text);           // 지금 하는 일 → /mission/state
  void result(const std::string & text);         // 퍽·장소 결과 (실패해도 계속) → /mission/state · build.json
  void stop(const std::string & why);            // ERROR — coordinator 정지

  // skill 요청 — 계획 하나를 보내고 /task/status로 끝을 기다림. run_retry는 실패하면 retries번까지 다시 (D6)
  void run(std::vector<SubGoal> goals, Done done);
  void run_retry(std::vector<SubGoal> goals, Done done, int tries = 0);
  // 박스 위 점(arena)을 다룰 자리로 navigate + approach — 점에서 가까운 변부터, 실패하면 다음 변 (최대 approach_tries)
  void go_near(const Box & box, double px, double py, Done done, size_t k = 0, std::vector<Pose2> poses = {});

  void go_observe();
  void barcode_phase();
  void read_phase();
  void survey(size_t box);
  void load(size_t box, std::vector<std::pair<int, uint32_t>> found, size_t k);  // 찾은 퍽 (tag, 트랙 id)을 적재함에
  void solve_allocation();
  void build(size_t i);
  void bring(size_t i);                          // 그 퍽을 장소 앞에서 손에 쥘 때까지 (적재함 · 시작 박스)
  void place_check(size_t i, int tries);         // 놓기 → 내려다보고 확인 → 벗어났으면 다시 (D5)
  void verify(size_t loc);
  void go_home(bool corrected = false);

  // 장소 태그로 장소 li 칸 보정 (D4) — 지금 /tracked_objects에 그 태그가 있으면 corr_[li]를 갱신
  void correct(size_t li);
  std::pair<double, double> cell_target(size_t li, int cell) const;  // 보정을 더한 칸 중심 (arena)
  const TrackedObject * find_puck(int tag) const;  // tag를 읽은 퍽 트랙 (가장 최근에 본 것)

  Pose2 to_map(double x, double y, double yaw = 0.0) const;
  Pose2 to_arena(double mx, double my, double myaw = 0.0) const;
  std::string point(double ax, double ay, double z) const;  // arena 점 → "x,y,z" (map)
  bool robot_arena(Pose2 & p) const;
  void publish_state();
  void publish_markers();
  void write_log(const std::string & name, const std::string & json) const;
  void snap(const std::string & name);           // photo_topic 한 장 → photos/<번호>_<name>.jpg (D13)

  // 설정 (arena, m)
  Pose2 start_;
  std::vector<double> start_zone_, footprint_;
  std::vector<Box> start_boxes_, loc_boxes_;
  std::vector<std::pair<double, double>> locations_;
  std::vector<int> loc_tags_;
  std::vector<double> loc_normals_;
  std::vector<int> tray_slots_;                  // 쓰는 적재함 칸 (팔이 못 닿는 칸은 설정에서 뺌 — D3)
  std::vector<double> puck_dx_;
  double box_top_z_, puck_h_, grid_step_, standoff_, approach_gap_, verify_radius_, match_radius_;
  double place_tol_, place_search_, corr_max_;
  Pose2 barcode_pose_;
  double signal_timeout_s_, barcode_timeout_s_, home_hold_s_;
  int approach_tries_, retries_;
  std::string log_dir_, photo_topic_;

  // 진행 상태
  Phase phase_ = Phase::IDLE;
  std::string note_, error_;
  bool active_ = false;
  uint32_t plan_seq_ = 0, waiting_plan_ = 0;
  Done waiting_done_;
  perception::msg::MissionSignal signal_;
  std::string barcode_, barcode_msg_;
  rclcpp::Time barcode_rx_{0, 0, RCL_ROS_TIME}, phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time read_started_{0, 0, RCL_ROS_TIME};
  Counts counts_;
  std::string order_;                   // L1·L2·L3 그룹 ("YRG")
  std::vector<Puck> pucks_;             // 조사한 퍽 (tag·color·shape)
  std::map<int, std::pair<size_t, double>> puck_at_;  // tag → (시작 박스, x 오프셋) — 적재함에 못 실은 퍽을 집으러 갈 곳
  std::map<int, int> tray_;             // 적재함 칸 → tag (D3)
  std::map<int, char> alloc_;
  struct Job { int tag; size_t loc; int cell; };
  std::vector<Job> jobs_;
  std::vector<std::pair<double, double>> corr_;  // 장소마다 태그 보정 (arena, D4)
  std::vector<std::string> results_;
  std::string build_json_, verify_json_;
  std::vector<std::vector<int>> seen_tags_;      // 재검증 — 장소마다 본 tag (12개 중복·누락)
  int placed_ = 0, photo_n_ = 0;
  std::string back_;   // 박스에 붙어 있을 때 물러날 standoff 자세 "x,y,yaw" (map) — 다음 go_near가 먼저 씀
  std::string run_dir_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_, barcode_sub_;
  rclcpp::Subscription<perception::msg::MissionSignal>::SharedPtr signal_sub_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr photo_sub_;
  rclcpp::TimerBase::SharedPtr timer_, markers_timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
