#pragma once

// mission_manager (🛠 직접 작성) — 주사위 기억 미션 (documents/mission/README.md) 순서를 진행한다
//   입력: ✉️ /cmd/mission (std_msgs/String "start" | "abort") · /mission/signal · /mission/barcode (perception)
//         ✉️ /tracked_objects (퍽 tag·color·shape · last_seen) · /task/status (coordinator 결과) · TF map → base_footprint
//   출력: ✉️ /task/sub_goals (SubGoalArray, plan_id = PLAN_ID_BASE + n — vlm_task_planner와 겹치지 않게) → coordinator
//         ✉️ /mission/signal/reset · /mission/state (std_msgs/String, latched — "단계: 설명") · /mission/markers (RViz)
//         ✉️ 기록 log_dir/<시작 시각>/ signal.json · allocation.json · verify.json · /diagnostics
//   단계: GO_OBSERVE 팔 scan 자세(카메라 앞 비움) + 바코드 보는 자세로 → BARCODE 이번에 본 바코드 (barcode_timeout_s)
//         → 관측 자세(−X)로 → READ 12회 점멸 (signal_timeout_s) → 횟수 검사·L1~L3 배정 (mission_solver)
//         → SURVEY 시작 박스마다 다가가 퍽 두 자리를 inspect → tag·색·모양 수집 → 12개 할당
//         → BUILD 퍽마다 [박스 앞 → pick <트랙 id> → 장소 변 앞 → place 칸 "x,y,z"] → VERIFY 장소마다 inspect, 이번에 본 퍽만 기록
//         → RETURN 출발 자세로 → home_hold_s 정지 → DONE.  오류(신호·바코드·할당·skill 실패)면 ERROR로 멈추고 coordinator 정지
//   좌표: 설정은 경기장(arena, m) — start_pose_arena 하나로 map으로 바꾼다 (mission_solver arena_to_map).
//   판단(정렬·배정·할당·칸)은 전부 mission_solver 순수 함수. 이 노드는 순서·기록·skill 요청만.
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/mission_signal.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/mission_solver.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace plan {

using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

struct Box { double x, y, half; };  // arena [m] — 정사각 박스 (윗면 = box_top_z)

class MissionManager : public rclcpp::Node {
public:
  MissionManager();

private:
  using Done = std::function<void(bool ok, const std::string & reason)>;

  void on_command(const std::string & cmd);
  void phase(const std::string & name, const std::string & note = "");
  void fail(const std::string & why);

  // skill 요청 — 계획 하나를 보내고 /task/status로 끝을 기다림
  void run(std::vector<SubGoal> goals, Done done);
  // 박스 위 점(arena)을 다룰 자리로 navigate — 로봇에서 가까운 변부터, 실패하면 다음 변 (최대 approach_tries)
  void go_near(const Box & box, double px, double py, Done done, size_t k = 0, std::vector<Pose2> poses = {});

  void barcode_phase();
  void read_phase();
  void survey(size_t box);
  void solve_allocation();
  void build(size_t i);
  void verify(size_t loc);
  void go_home();

  Pose2 to_map(double x, double y, double yaw = 0.0) const;
  std::string point(double ax, double ay, double z) const;  // arena 점 → "x,y,z" (map)
  bool robot_arena(double & x, double & y) const;
  void publish_markers();
  void write_log(const std::string & name, const std::string & json) const;

  // 설정 (arena, m)
  Pose2 start_;
  std::vector<double> start_zone_;
  std::vector<Box> start_boxes_, loc_boxes_;
  std::vector<std::pair<double, double>> locations_;
  std::vector<double> puck_dx_;
  double box_top_z_, puck_h_, grid_step_, standoff_, approach_gap_, verify_radius_, match_radius_;
  Pose2 barcode_pose_;
  double signal_timeout_s_, barcode_timeout_s_, home_hold_s_;
  int approach_tries_;
  std::string log_dir_;

  // 진행 상태
  std::string phase_ = "IDLE";
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
  std::map<int, std::pair<size_t, double>> puck_at_;  // tag → (시작 박스, x 오프셋)
  std::map<int, char> alloc_;
  struct Job { int tag; size_t loc; int cell; };
  std::vector<Job> jobs_;
  std::string verify_json_;
  std::string back_;   // 박스에 붙어 있을 때 물러날 standoff 자세 "x,y,yaw" (map) — 다음 go_near가 먼저 씀
  std::string run_dir_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_, barcode_sub_;
  rclcpp::Subscription<perception::msg::MissionSignal>::SharedPtr signal_sub_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_, markers_timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
