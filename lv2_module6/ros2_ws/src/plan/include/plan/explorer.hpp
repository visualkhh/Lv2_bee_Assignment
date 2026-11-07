#pragma once

// explorer (🛠 직접 작성, 담당: 김현하) — 자동 탐사: 매핑 중인 지도의 프론티어(빈 곳과 모르는 곳의 경계)로 Nav2를 차례로 보냄.
//   바퀴는 직접 안 움직인다 — 목표만 고르고 주행·회피·충돌 멈춤은 Nav2(navigate_to_pose → 바퀴 명령 체인)가 한다.
//   조작 모드가 EXPLORE일 때만 일한다 (state_manager가 지도 모드 MAPPING일 때만 허락). 다른 모드가 되면 목표를 취소.
//   입력: ✉️ /state/operation_mode (std_msgs/String, transient_local) — "EXPLORE"면 탐사
//         ✉️ /map (nav_msgs/OccupancyGrid, transient_local — RTAB-Map이 매핑하며 키움) · TF map→base_footprint
//         ✉️ /global_costmap/costmap (nav_msgs/OccupancyGrid — Nav2 planner가 보는 그것) — 로봇에서 실제로 갈 수 있는지
//   출력: 🎯 /navigate_to_pose (nav2_msgs/action/NavigateToPose) — 프론티어 목표 (도착 방향 = 로봇에서 목표 쪽 → 카메라가 모르는 곳을 봄)
//         ✉️ /explore/result (std_msgs/String) — "DONE …"(갈 프론티어 없음) · "FAILED …"(남았는데 못 감) → state_manager가 AUTO로 되돌림
//         ✉️ /diagnostics
//   고르기: 크기 min_frontier_cells 이상인 프론티어 덩어리 중 — 코스트맵에서 로봇과 이어진 칸으로 가장 짧게 갈 수 있는 것.
//     · 목표 자리 = 프론티어에서 goal_search_radius 안의 '갈 수 있는 칸' 중 로봇에서 가장 가까운 칸 (프론티어 칸은 벽 옆이라 막혀 있기 쉬움)
//     · 벽 너머(코스트맵에서 이어진 길 없음)는 고르지 않음 — 직선거리로 고르면 길 없는 목표를 planner가 붙잡다 실패했음
//     · 로봇에서 min_goal_dist 안의 프론티어는 안 고름 — 카메라가 로봇 바로 앞·아래를 못 봐 그 자리는 계속 '모름'으로 남음
//     · 도착·실패·시간 초과한 프론티어 근처(blacklist_radius)는 다시 안 감 — 도착해도 안 지워지는 프론티어로 계속 다시 가서 제자리를 돌았음
//   끝: 고를 게 없음이 empty_checks번 이어지면 (지도가 막 갱신되는 중일 수 있어 한 번에 끝내지 않음)
//       남은 프론티어가 없거나 로봇 발밑 것뿐이면 DONE, 실패 목록·길 없음 때문에 못 가면 FAILED
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace plan {

// 프론티어 덩어리 — (x, y) = 덩어리 중심에 가장 가까운 프론티어 칸 (빈 칸이라 목표로 쓸 수 있음, map 좌표), size = 칸 수
struct Frontier {
  double x, y;
  size_t size;
};

// 지도에서 프론티어 찾기: 빈 칸(0 ≤ 값 < free_thresh) 중 4-이웃에 모르는 칸(−1)이 있는 것을 8-이웃으로 묶음
std::vector<Frontier> find_frontiers(const nav_msgs::msg::OccupancyGrid & map, size_t min_cells, int free_thresh = 25);

// 코스트맵에서 로봇이 갈 수 있는 칸까지의 거리 [칸, 8-이웃 BFS] — 못 가면 −1. 칸 순서 = data 순서
//   지나갈 수 있는 칸 = 값 < lethal (99 = 로봇 반경 안에 장애물 · 100 = 장애물) 또는 모름(−1 — planner allow_unknown과 같게)
//   로봇 칸이 막혀 있으면(벽에 붙음·추정이 벽 안) seed_r 안의 지나갈 수 있는 칸들에서 시작
std::vector<int> reach_cells(const nav_msgs::msg::OccupancyGrid & cm, double rx, double ry, double seed_r, int lethal = 99);

// 목표 자리 — (x, y) map 좌표, dist = 로봇에서 코스트맵 길로 [m]
struct Goal {
  double x, y, dist;
};
// 프론티어 (fx, fy)에서 radius 안의 갈 수 있는 칸(reach_cells ≥ 0) 중 로봇에서 가장 가까운 칸 · 없으면 nullopt (벽 너머)
std::optional<Goal> reachable_goal(const nav_msgs::msg::OccupancyGrid & cm, const std::vector<int> & dist,
                                   double fx, double fy, double radius);

class Explorer : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  explicit Explorer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();
  void send_goal(const Frontier & f, const Goal & g, double rx, double ry);
  void finish(const std::string & result);
  bool blacklisted(const Frontier & f) const;

  size_t min_cells_;
  double blacklist_radius_, goal_timeout_s_, min_goal_dist_, goal_search_radius_;
  int empty_checks_;
  std::string mode_ = "AUTO", note_ = "대기";
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_, costmap_;
  std::vector<std::pair<double, double>> blacklist_;
  std::optional<Frontier> target_;
  rclcpp::Time goal_start_;
  bool goal_active_ = false;
  int empty_count_ = 0, reached_ = 0, failed_ = 0;
  size_t last_frontiers_ = 0;
  std::string last_skipped_ = "-";  // 못 고른 프론티어 내역 (진단)
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, costmap_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
