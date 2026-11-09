#pragma once

// mission_manager (🛠 직접 작성) — 주사위 기억 미션 (documents/mission/README.md) 순서를 진행한다
//   흐름·skill: documents/project/ros/05-arm.md · 인지 인터페이스: documents/API.md
//   입력: ✉️ /cmd/mission (std_msgs/String "start" | "abort")
//         ✉️ /mission/signal (확정 답) · /mission/signal_status (사이클마다 — ok · error) · /mission/qr (바코드 원문) — perception
//         ✉️ /tracked_objects (퍽 tag·color·shape(box|cyl) · 장소 상자 location_box(tag · 윗면 가운데) · barcode · last_seen)
//         ✉️ /task/status (coordinator 결과)
//         ✉️ photo_topic (사진 찍을 때만 잠깐 구독) · TF map → base_footprint
//   출력: ✉️ /task/sub_goals (SubGoalArray, plan_id = PLAN_ID_BASE + n) → coordinator
//         ✉️ /mission/state (plan/MissionState, latched) · /mission/markers (RViz)
//         ✉️ 기록 log_dir/<시작 시각>/ signal · allocation · build · verify · return · error .json + photos/ · /diagnostics
//   단계 (순서와 실패 시 갈 곳은 mission_solver next_phase):
//     GO_OBSERVE 바코드 보는 자세로 (팔은 stow 그대로 — scan 자세는 없앰, 11-09) → BARCODE 이번에 본 바코드 → 관측 자세(−X)로
//     → READ 12회 점멸 — signal_status가 ok(두 사이클 같은 답)면 확정 · unconfirmed면 기다림 ·
//              count·unknown·range가 signal_error_cycles번 이어지면 ERROR (발제: 추정 금지 → 안전 정지) · signal_timeout_s 넘어도 ERROR
//     → SURVEY 시작 박스 6곳 전부: 퍽 두 자리 inspect(tag·색·모양) → pick → load next (적재함 12칸)
//              적재함 장부(칸 → tag)는 coordinator 하나만 씀 — 여기선 /coordinator/basket을 읽어 "그 tag가 실린 칸"을 찾음
//              → 12개 한 번에 할당 (mission_solver allocate)
//     → BUILD 퍽마다: 장소 앞 → 장소 태그로 칸 위치 보정 → [적재함 칸에서 꺼냄 | 못 실은 퍽은 시작 박스에서 직접 집어 옴]
//              (적재함이면 unload <칸>) → place 칸 "x,y,z" → 내려다보고 10 mm 밖·쓰러짐이면 다시 집어 놓기
//     → VERIFY 장소마다 새로 본 퍽만으로 칸별 판정 + 12개 중복·누락 → RETURN 출발 구역 가운데 → 외곽이 구역 안인지
//              확인·보정 → home_hold_s 정지 → DONE
//   실패: skill마다 retries번 다시 · 그래도 안 되면 그 퍽·장소만 실패로 results에 남기고 다음으로 (복귀까지).
//     관측(신호·바코드) 오류와 복귀 이동 실패만 ERROR로 멈추고 coordinator 정지
//   좌표: 설정은 경기장(arena, m) — start_pose_arena 하나로 map으로 바꾼다 (mission_solver arena_to_map).
//   판단(정렬·배정·할당·칸·판정)은 전부 mission_solver 순수 함수. 이 노드는 순서·기록·skill 요청만.
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/mission_signal.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/mission_solver.hpp"
#include "plan/plan_next.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "plan/msg/basket_state.hpp"
#include "plan/msg/mission_state.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

// 집을 퍽 트랙 고르기 — 인지는 안 보이는 물체를 2초 뒤 지우므로(long_memory: false) 퍽은 tag로 기억하고 집기 직전에 다시 찾는다.
//   ① tag를 읽은 퍽 중 가장 최근에 본 것 ② (spot이 있으면) 그 map 점에서 radius 안 가장 가까운 퍽 — tag를 못 읽었거나 같은 tag인 것만
//   (다른 tag를 읽은 퍽은 고르지 않음). 없으면 nullptr
const perception::msg::TrackedObject * puck_for(const perception::msg::TrackedObjectArray * objs, int tag,
                                                const double * spot_xy, double radius);

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

  // skill 요청 — 계획 하나를 보내고 /task/status로 끝을 기다림. run_retry는 실패하면 retries번까지 다시
  void run(std::vector<SubGoal> goals, Done done);
  void run_retry(std::vector<SubGoal> goals, Done done, int tries = 0);
  // 박스 위 점(arena)을 다룰 자리로 navigate + approach — 점에서 가까운 변부터, 실패하면 다음 변 (최대 approach_tries)
  void go_near(const Box & box, double px, double py, Done done, size_t k = 0, std::vector<Pose2> poses = {});

  void go_observe();
  void barcode_phase();
  void read_phase();
  void survey(size_t box);
  void load(size_t box, std::vector<int> found, size_t k);  // 찾은 퍽(tag)을 적재함에 — 트랙 id는 집기 직전에 다시 찾음
  void solve_allocation();
  void build(size_t i);
  void bring(size_t i);                          // 그 퍽을 장소 앞에서 손에 쥘 때까지 (적재함 · 시작 박스)
  void place_check(size_t i, int tries);         // 놓기 → 내려다보고 확인 → 벗어났으면 다시
  void verify(size_t loc);
  void go_home(bool corrected = false);

  // 장소 상자로 장소 li 칸 보정 — 지금 /tracked_objects에 그 태그의 location_box가 있으면 corr_[li]를 갱신
  void correct(size_t li);
  std::pair<double, double> cell_target(size_t li, int cell) const;  // 보정을 더한 칸 중심 (arena)
  const TrackedObject * puck_near(double mx, double my) const;  // map 점에서 match_radius 안 가장 가까운 퍽 (tag 상관없이)
  // 누운 퍽을 세워서 손에 쥠: stand → stand_check_s 뒤 인지로 섰나 → stand_pick. 아직 누웠으면 그 퍽으로 stand 다시 (retries번)
  void stand_up(uint32_t id, double mx, double my, int tries, Done done);
  // 그 퍽(tag)을 손에 — 집기 직전에 locate로 트랙을 다시 찾고, 누웠으면 stand_up · 아니면 pick. pick이 실패하면 다시 찾아서 (retries번)
  void grab(int tag, Done done, int tries = 0);
  // 퍽(tag) 트랙을 지금 찾음 — 목록에 없으면 조사한 자리(puck_at_)를 inspect로 다시 보고 한 번 더 찾음 (그래도 없으면 nullptr)
  //   memory: 목록에 없을 때 먼저 마지막으로 본 트랙(last_seen_)을 줌 — 첫 시도만 (그 pick이 실패하면 다시 봄)
  void locate(int tag, bool memory, std::function<void(const TrackedObject *)> cb);

  Pose2 to_map(double x, double y, double yaw = 0.0) const;
  Pose2 to_arena(double mx, double my, double myaw = 0.0) const;
  std::string point(double ax, double ay, double z) const;  // arena 점 → "x,y,z" (map)
  bool robot_arena(Pose2 & p) const;
  void publish_state();
  void publish_markers();
  void write_log(const std::string & name, const std::string & json) const;
  void snap(const std::string & name);           // photo_topic 한 장 → photos/<번호>_<name>.jpg

  // 설정 (arena, m)
  Pose2 start_;
  std::vector<double> start_zone_, footprint_;
  std::vector<Box> start_boxes_, loc_boxes_;
  std::vector<size_t> survey_order_, build_loc_order_;  // 시작 박스 · 장소 방문 순서 (0부터, plan.yaml survey_order · build_location_order)
  size_t survey_k_ = 0;                                  // 지금 몇 번째 박스를 도는지 (survey_order_ 인덱스)
  std::vector<double> walls_;    // 경기장 벽 선분 [x0, y0, x1, y1, …] (arena, m) — 다가설 자리 거르기 · /arena/walls
  double wall_clearance_ = 0.22; // [m] 다가설 자리가 벽에서 이보다 가까우면 그 변은 안 감 (로봇 반경 0.16 + 여유)
  std::vector<std::pair<double, double>> locations_;
  std::vector<int> loc_tags_;
  std::vector<double> puck_dx_;
  double box_top_z_, puck_h_, grid_step_, standoff_, approach_gap_, verify_radius_, match_radius_;
  double place_tol_, place_search_, corr_max_;
  Pose2 barcode_pose_;
  double signal_timeout_s_, barcode_timeout_s_, home_hold_s_;
  int signal_error_cycles_ = 2;
  int approach_tries_, retries_;
  std::string log_dir_, photo_topic_;

  // 진행 상태
  Phase phase_ = Phase::IDLE;
  std::string note_, error_;
  bool active_ = false;
  uint32_t plan_seq_ = 0, waiting_plan_ = 0;
  Done waiting_done_;
  perception::msg::MissionSignal signal_;
  bool signal_ready_ = false;           // 이번 READ에서 확정 답을 받음
  int signal_streak_ = 0;               // 이어진 오류 사이클 수 (mission_solver signal_action)
  std::string signal_error_;            // ERROR로 갈 신호 오류 (count | unknown | range)
  std::string barcode_, barcode_msg_;
  rclcpp::Time barcode_rx_{0, 0, RCL_ROS_TIME}, phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time read_started_{0, 0, RCL_ROS_TIME};
  Counts counts_;
  std::string order_;                   // L1·L2·L3 그룹 ("YRG")
  std::vector<Puck> pucks_;             // 조사한 퍽 (tag·color·shape)
  std::map<int, std::pair<size_t, double>> puck_at_;  // tag → (시작 박스, x 오프셋) — 적재함에 못 실은 퍽을 집으러 갈 곳
  plan::msg::BasketState::ConstSharedPtr basket_;  // 적재함 장부 — coordinator가 씀 (/coordinator/basket), 여기선 읽기만
  int basket_slot_of(int tag) const;   // 그 tag가 실린 칸 (없으면 −1)
  bool basket_has_room() const;        // 빈 칸이 있나 (장부를 아직 못 받았으면 있다고 봄)
  std::map<int, char> alloc_;
  struct Job { int tag; size_t loc; int cell; };
  std::vector<Job> jobs_;
  std::vector<std::pair<double, double>> corr_;  // 장소마다 태그 보정 (arena, )
  std::vector<std::string> results_;
  std::string build_json_, verify_json_;
  std::vector<std::vector<int>> seen_tags_;      // 재검증 — 장소마다 본 tag (12개 중복·누락)
  // planner: rule (jobs_ 순서대로) | plan_next (학습된 플래너가 다음 퍽·복귀 시점을 고름 — 퍽 하나 처리는 bring 그대로)
  std::string planner_;
  PlanNext plan_next_;
  bool plan_next_ok_ = false;
  PlanNext::State pn_;              // plan_next 상태 (퍽 i = 시작 박스 i/2, x 오프셋 순)
  std::vector<size_t> pn_job_;      // 퍽 i → jobs_ 번호
  int pn_action_ = -1;
  rclcpp::Time build_started_{0, 0, RCL_ROS_TIME};
  bool build_pn_start();             // 상태 만들기 (안 되면 false → rule로)
  void build_pn();                   // 다음 행동 고르고 실행
  void job_done(size_t i, bool ok);  // 퍽 하나 끝 → rule: 다음 job · plan_next: 상태 갱신 후 다시 고르기
  std::set<int> lying_;                          // 조사 때 누워 있던 퍽 tag — 집을 때 세워서 (stand → stand_pick)
  double stand_check_s_ = 1.0;                   // stand 뒤 인지가 선 퍽을 볼 때까지 기다림
  int placed_ = 0, photo_n_ = 0;
  std::string back_;   // 박스에 붙어 있을 때 물러날 standoff 자세 "x,y,yaw" (map) — 다음 go_near가 먼저 씀
  std::string run_dir_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Publisher<MissionState>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr walls_pub_;  // /arena/walls (map, latched) → nvblox_map이 /map에 겹침
  void publish_walls();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_, barcode_sub_;
  rclcpp::Subscription<perception::msg::MissionSignal>::SharedPtr signal_sub_, signal_status_sub_;
  std::map<int, perception::msg::TrackedObject> last_seen_;  // 태그 → 마지막으로 본 트랙 (locate memory)
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<plan::msg::BasketState>::SharedPtr basket_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr photo_sub_;
  rclcpp::TimerBase::SharedPtr timer_, markers_timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
