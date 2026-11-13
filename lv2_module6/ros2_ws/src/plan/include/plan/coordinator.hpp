#pragma once

// coordinator (🛠 직접 작성, 담당: 권형중) — 설계: documents/project/ros/03-plan.md "coordinator가 지키는 실행 규칙"
//   계획(/task/sub_goals)의 단계를 차례로 skill 노드(skills 패키지, 🎬 /skills/<skill> 액션)에 맡기는 순서 담당 (2026-11-04 skill 노드로 나눔)
//   입력: /task/sub_goals (mission_manager 계획), /tracked_objects (대상 id 검증 · 물체 자세), /state/operation_mode · /state/emergency
//   출력: 🎬 /skills/{navigate,approach,pose,inspect,pick,place,load,unload,stand,stand_pick}, ✉️ /task/status → mission_manager,
//         📢 /coordinator/basket (적재함 장부, latched) · 📞 /coordinator/basket_clear
//   이 노드에 남은 것: 계획 검증 · 단계 순서 · 물체 → 좌표 (다가갈 자리 · 놓을 면 · 볼 점) · 적재함 장부와 손에 든 퍽 · 모드·비상정지 취소 · 실패 횟수
//   팔·주행 동작, 학습 정책, 시작·스캔 자세는 skill 노드 (skills/launch/skills.launch.py · documents/project/ros/05-arm.md)
// 규칙:
//   1. 계획 검증 — 허용 동작(navigate·approach·pick·place·inspect·pose·load·unload·stand·stand_pick), 숫자 대상 id가 /tracked_objects에 있는지
//      (pose·load·unload·stand_pick의 target은 자세 이름·칸 번호라 물체 검증 안 함), pose 이름은 pose_names
//   2. 팔이 닿게 다가가기: pick · place · inspect · stand <물체 id> 차례에 대상이 로봇(base_footprint, TF)에서 reach_m 넘게 떨어져 있으면
//      먼저 그 물체를 바라보며 approach_m 떨어진 자리로 navigate한 뒤 그 단계를 실행 (단계마다 한 번 — 계획에 navigate가 빠져 있어도)
//   3. 실패는 FAILED + reason으로 바로 알리고 멈춤 — 다시 할지는 계획을 낸 쪽(mission_manager retries · approach_tries)이 정함
//   3-1. 단계 시간 제한: skill goal이 step_timeout_s.<skill> 안에 결과를 안 주면 취소하고 <skill>_timeout으로 실패 (0 = 없음).
//        skill 안의 시간 제한(approach · 그리퍼)이 먼저고, 이건 skill이 응답 없이 멈췄을 때의 마지막 안전망
//   3-2. skill 노드(/skills/<skill>)가 server_wait_s 안에 안 뜨면 skill_unavailable:<skill>
//   4. AUTO가 아닌 모드(MANUAL · TEACH) · 비상정지: 진행 중 skill goal을 취소하고 FAILED, 그동안 새 계획은 받지 않음
//   적재함 장부는 이 노드 하나만 씀: 칸마다 {모양 · 색 · tag · 확인 필요} + 실은 순서 + 손에 든 퍽 → 파일(basket_file)에 저장해 다시 떠도 이어 감
//     pick·stand_pick 성공이면 그 퍽을 손에 · load 성공이면 칸에 · unload 성공이면 손으로 · place 성공이면 손 비움.
//     unload가 빈손(grasp_empty)이면 칸을 비우지 않고 check(확인 필요)로. load next는 basket_use 칸 중 빈 칸
//   같은 물체(구역) 위에 여러 번 place하면 겹치지 않게 place_spacing 간격으로 비켜 놓음
//   상태: /diagnostics (모드 · 계획 · 단계 · 실패 횟수 · 적재함 · 손)
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/action/skill.hpp"
#include "plan/msg/basket_state.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

// 적재함 칸 고르기 — filled[i] = 실린 퍽 모양("" = 빔) · order = 실은 순서.
//   load: "next"·"tray" = 빈 칸 중 가장 낮은 번호 · 숫자 = 그 칸 (차 있으면 slot_full)
//   unload: "last"·"tray" = 마지막에 실은 칸 · 숫자 = 그 칸 (비었으면 slot_empty). 못 고르면 −1 + why(slot_invalid · slot_full · slot_empty)
//   usable[i] = false인 칸은 next가 고르지 않음 (비면 전부 씀 · 칸 번호를 직접 주면 그대로)
int basket_slot(const std::string & target, const std::vector<std::string> & filled, const std::vector<int> & order,
                bool load, std::string & why, const std::vector<bool> & usable = {});
// 상자 위 n번째(0부터)로 놓을 자리의 (dx, dy) [m, map] — spread_offset을 상자 안(반 변 − margin)으로 자르고 상자 yaw로 돌림
std::pair<double, double> box_spread(int n, double spacing, double half_x, double half_y, double margin, double yaw);
// "x,y[,z|yaw]" (map 좌표) 목표 파싱 — 쉼표로 나눈 숫자 2개 이상 (숫자가 아닌 게 섞이거나 1개면 빈 목록)
std::vector<double> parse_numbers(const std::string & target);
// 물체를 바라보며 standoff 떨어진 로봇 자세 (map) — 로봇이 있는 쪽에서. 로봇이 물체와 거의 같은 자리면 물체의 −x 쪽
geometry_msgs::msg::Pose approach_pose(double ox, double oy, double rx, double ry, double standoff);
// 같은 곳에 n번째(0부터)로 내려놓을 때 중심에서 비킬 (dx, dy) — 가운데 · 좌 · 우 · 앞 · 뒤 · 대각선 순, spacing 간격
std::pair<double, double> spread_offset(int n, double spacing);

using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;
using Skill = plan::action::Skill;

class Coordinator : public rclcpp::Node {
public:
  explicit Coordinator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct PuckInfo { std::string shape, color; int tag = -1; bool check = false; };  // shape "" = 빔
  using Ok = std::function<void(const Skill::Result &)>;
  using Fail = std::function<void(const std::string &)>;

  // ---------- 계획 받기 · 검증 · 모드
  void on_mode(const std::string & mode);
  void on_plan(const SubGoalArray & plan);
  bool validate(const SubGoalArray & plan, std::string & why) const;
  static bool is_id(const std::string & s);
  const perception::msg::TrackedObject * find(uint32_t id) const;
  std::string step_text() const;

  // ---------- 단계 실행 — 단계 하나 = skill goal 하나 (물체 → 좌표는 여기서)
  void run_step();
  void next();
  void fail(const std::string & reason);
  // /skills/<action>에 goal — 서버가 아직 없으면 1초마다 다시 (server_wait_s 넘으면 skill_unavailable:<action>).
  //   step_timeout_s.<action> 안에 결과가 없으면 취소하고 <action>_timeout. 성공이면 ok, 실패면 on_fail(이유)
  void send(const std::string & action, const Skill::Goal & goal, Ok ok, Fail on_fail = nullptr);
  void cancel_all();
  bool robot_xy(double & x, double & y) const;
  bool far_fallback(const std::string & why, const std::string & target) const;  // pick 실패 → pick_far로 다시 할지
  bool approach_if_far(const SubGoal & g);  // 대상이 reach_m 밖이면 다가가는 navigate를 맡기고 true (끝나면 같은 단계를 다시)
  void navigate(const SubGoal & g);
  void approach(const SubGoal & g);
  void pick(const SubGoal & g);
  void place(const SubGoal & g);
  void inspect(const SubGoal & g);
  void load(const SubGoal & g);
  void unload(const SubGoal & g);
  void stand(const SubGoal & g);
  void stand_pick(const SubGoal & g);
  std::string shape_of(const perception::msg::TrackedObject & o) const;  // details shape (모르면 box)
  int tag_of(const perception::msg::TrackedObject & o) const;            // details tag (없으면 −1)

  // ---------- 적재함 장부 — 발행 · 파일 저장/읽기
  void basket_changed();
  void basket_save() const;
  void basket_load();
  std::vector<std::string> basket_shapes() const;

  void status(uint32_t step, uint8_t state, const std::string & reason);

  double server_wait_s_, waited_s_ = 0, reach_m_, approach_m_, place_spacing_, box_margin_m_, pick_far_min_m_, pick_far_max_m_;
  std::set<std::string> pose_names_;
  std::map<std::string, rclcpp_action::Client<Skill>::SharedPtr> skills_;  // action 이름 → 클라이언트
  std::map<std::string, double> step_timeout_s_;                            // action 이름 → 단계 시간 제한 [s] (0 = 없음)
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp_action::ClientGoalHandle<Skill>::SharedPtr active_;
  std::string active_action_;
  rclcpp::TimerBase::SharedPtr retry_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  size_t approached_ = static_cast<size_t>(-1);  // 다가가기를 이미 한 단계 (idx_) — 단계마다 한 번만
  std::map<std::string, int> placed_;            // 계획 안에서 물체(구역) id별로 내려놓은 수 — 겹치지 않게 비킴
  std::vector<PuckInfo> basket_;                 // 칸마다 (장부 — 이 노드만 씀)
  std::vector<int> basket_order_;                // 실은 순서 (unload last)
  std::vector<bool> basket_use_;                 // load next가 쓰는 칸
  PuckInfo held_;                                // 손에 든 퍽 (load가 모양으로 표를 고름)
  PuckInfo stood_;                               // 마지막 stand로 세운 퍽 (stand_pick 성공이면 손에)
  std::string basket_file_;
  rclcpp::Publisher<plan::msg::BasketState>::SharedPtr basket_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr basket_clear_srv_;
  std::string mode_ = "AUTO";
  bool estop_ = false, running_ = false;
  SubGoalArray plan_;
  size_t idx_ = 0;
  uint64_t gen_ = 0;  // 새 계획·취소 때마다 증가 — 옛 goal 결과가 새 단계를 건드리지 않게
  TrackedObjectArray::ConstSharedPtr objects_;
  // 기억한 퍽 — perception은 안 보이면 2초 뒤 지우므로(long_memory false, 11/04 "기억은 통합 쪽") 마지막으로 본 모습을 remember_s 동안
  //   (적재 뒤 팔이 상자를 안 보는 자세여도 다음 퍽을 그 자리로 바로 pick). 집으면 지움 · 같은 자리에 새 트랙이 생기면 지움
  struct Seen { perception::msg::TrackedObject o; rclcpp::Time t; };
  std::map<uint32_t, Seen> remembered_;
  double remember_s_ = 120.0;
  void remember(const TrackedObjectArray & m);
  std::shared_ptr<SubGoalArray> waiting_plan_;
  rclcpp::Publisher<TaskStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<SubGoalArray>::SharedPtr goals_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
