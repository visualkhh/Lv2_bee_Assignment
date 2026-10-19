// explorer 구현 — 설명·선언은 include/plan/explorer.hpp, main은 explorer_main.cpp
#include "plan/explorer.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "tf2/LinearMath/Quaternion.hpp"

namespace plan {

std::vector<Frontier> find_frontiers(const nav_msgs::msg::OccupancyGrid & map, size_t min_cells, int free_thresh) {
  const int w = static_cast<int>(map.info.width), h = static_cast<int>(map.info.height);
  std::vector<Frontier> out;
  if (w == 0 || h == 0 || map.data.size() < static_cast<size_t>(w) * h) return out;
  auto at = [&](int c, int r) { return static_cast<int>(map.data[static_cast<size_t>(r) * w + c]); };
  auto is_free = [&](int c, int r) { const int v = at(c, r); return v >= 0 && v < free_thresh; };
  std::vector<uint8_t> frontier(static_cast<size_t>(w) * h, 0), seen(frontier.size(), 0);
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      if (!is_free(c, r)) continue;
      const int nb[4][2] = {{c + 1, r}, {c - 1, r}, {c, r + 1}, {c, r - 1}};
      for (const auto & n : nb) {
        if (n[0] >= 0 && n[0] < w && n[1] >= 0 && n[1] < h && at(n[0], n[1]) < 0) {
          frontier[static_cast<size_t>(r) * w + c] = 1;
          break;
        }
      }
    }
  }
  const double res = map.info.resolution, ox = map.info.origin.position.x, oy = map.info.origin.position.y;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      const size_t i0 = static_cast<size_t>(r) * w + c;
      if (!frontier[i0] || seen[i0]) continue;
      std::vector<std::pair<int, int>> cells;  // 8-이웃으로 이어진 프론티어 칸 묶기 (BFS)
      std::queue<std::pair<int, int>> q;
      q.push({c, r});
      seen[i0] = 1;
      while (!q.empty()) {
        const auto [cc, rr] = q.front();
        q.pop();
        cells.push_back({cc, rr});
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int nc = cc + dc, nr = rr + dr;
            if (nc < 0 || nc >= w || nr < 0 || nr >= h) continue;
            const size_t ni = static_cast<size_t>(nr) * w + nc;
            if (frontier[ni] && !seen[ni]) { seen[ni] = 1; q.push({nc, nr}); }
          }
        }
      }
      if (cells.size() < min_cells) continue;
      double mc = 0, mr = 0;
      for (const auto & [cc, rr] : cells) { mc += cc; mr += rr; }
      mc /= cells.size();
      mr /= cells.size();
      auto best = cells.front();  // 중심에 가장 가까운 실제 프론티어 칸 (중심 자체는 벽·모르는 곳일 수 있음)
      double bd = 1e18;
      for (const auto & cell : cells) {
        const double d = (cell.first - mc) * (cell.first - mc) + (cell.second - mr) * (cell.second - mr);
        if (d < bd) { bd = d; best = cell; }
      }
      out.push_back({ox + (best.first + 0.5) * res, oy + (best.second + 0.5) * res, cells.size()});
    }
  }
  return out;
}

std::vector<int> reach_cells(const nav_msgs::msg::OccupancyGrid & cm, double rx, double ry, double seed_r, int lethal) {
  const int w = static_cast<int>(cm.info.width), h = static_cast<int>(cm.info.height);
  std::vector<int> dist(static_cast<size_t>(w) * h, -1);
  if (w == 0 || h == 0 || cm.data.size() < dist.size()) return dist;
  const double res = cm.info.resolution, ox = cm.info.origin.position.x, oy = cm.info.origin.position.y;
  auto passable = [&](int c, int r) { const int v = cm.data[static_cast<size_t>(r) * w + c]; return v < lethal; };  // −1(모름)도 통과
  const int c0 = static_cast<int>(std::floor((rx - ox) / res)), r0 = static_cast<int>(std::floor((ry - oy) / res));
  std::queue<std::pair<int, int>> q;
  auto seed = [&](int c, int r) {
    if (c < 0 || c >= w || r < 0 || r >= h || !passable(c, r)) return;
    int & d = dist[static_cast<size_t>(r) * w + c];
    if (d < 0) { d = 0; q.push({c, r}); }
  };
  seed(c0, r0);
  if (q.empty()) {  // 로봇 칸이 막힘 → 둘레 seed_r 안의 지나갈 수 있는 칸에서
    const int k = static_cast<int>(std::ceil(seed_r / res));
    for (int dr = -k; dr <= k; ++dr)
      for (int dc = -k; dc <= k; ++dc)
        if (dc * dc + dr * dr <= k * k) seed(c0 + dc, r0 + dr);
  }
  while (!q.empty()) {
    const auto [c, r] = q.front();
    q.pop();
    const int d = dist[static_cast<size_t>(r) * w + c];
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nc = c + dc, nr = r + dr;
        if ((dc == 0 && dr == 0) || nc < 0 || nc >= w || nr < 0 || nr >= h || !passable(nc, nr)) continue;
        int & nd = dist[static_cast<size_t>(nr) * w + nc];
        if (nd < 0) { nd = d + 1; q.push({nc, nr}); }
      }
    }
  }
  return dist;
}

std::optional<Goal> reachable_goal(const nav_msgs::msg::OccupancyGrid & cm, const std::vector<int> & dist,
                                   double fx, double fy, double radius) {
  const int w = static_cast<int>(cm.info.width), h = static_cast<int>(cm.info.height);
  if (dist.size() < static_cast<size_t>(w) * h) return std::nullopt;
  const double res = cm.info.resolution, ox = cm.info.origin.position.x, oy = cm.info.origin.position.y;
  const int fc = static_cast<int>(std::floor((fx - ox) / res)), fr = static_cast<int>(std::floor((fy - oy) / res));
  const int k = static_cast<int>(std::ceil(radius / res));
  std::optional<Goal> best;
  for (int dr = -k; dr <= k; ++dr) {
    for (int dc = -k; dc <= k; ++dc) {
      const int c = fc + dc, r = fr + dr;
      if (dc * dc + dr * dr > k * k || c < 0 || c >= w || r < 0 || r >= h) continue;
      const int d = dist[static_cast<size_t>(r) * w + c];
      if (d < 0) continue;
      const double m = d * res;  // ponytail: 8-이웃 걸음 수 × 칸 크기 — 대각선을 1칸으로 쳐 실제보다 짧음, 고르는 순서엔 충분
      if (!best || m < best->dist) best = Goal{ox + (c + 0.5) * res, oy + (r + 0.5) * res, m};
    }
  }
  return best;
}

Explorer::Explorer(const rclcpp::NodeOptions & options)
: rclcpp::Node("explorer", options) {
  min_cells_ = static_cast<size_t>(declare_parameter<int>("min_frontier_cells", 8));
  blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.4);
  goal_timeout_s_ = declare_parameter<double>("goal_timeout_s", 60.0);
  empty_checks_ = declare_parameter<int>("empty_checks", 3);
  min_goal_dist_ = declare_parameter<double>("min_goal_dist", 0.4);
  goal_search_radius_ = declare_parameter<double>("goal_search_radius", 0.3);
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  mode_sub_ = create_subscription<std_msgs::msg::String>("/state/operation_mode", latched,
    [this](std_msgs::msg::String::ConstSharedPtr m) {
      if (m->data == "EXPLORE" && mode_ != "EXPLORE") {  // 새 탐사 — 지난번 실패 목록·횟수 초기화
        blacklist_.clear();
        empty_count_ = reached_ = failed_ = 0;
        note_ = "탐사 시작";
      }
      mode_ = m->data;
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", latched,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { map_ = m; });
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", latched,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { costmap_ = m; });
  result_pub_ = create_publisher<std_msgs::msg::String>("/explore/result", rclcpp::QoS(1).reliable());
  nav_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { tick(); });
  diag_.setHardwareID("beebot");
  diag_.add("상태", [this](diagnostic_updater::DiagnosticStatusWrapper & s) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    s.summary(DiagnosticStatus::OK, mode_ == "EXPLORE" ? "탐사 중 — " + note_ : "대기 (" + mode_ + ")");
    s.add("고를 수 있는 프론티어", last_frontiers_);
    s.add("못 고른 프론티어", last_skipped_);
    s.add("도착", reached_);
    s.add("실패", failed_);
    s.add("목표", target_ ? std::to_string(target_->x) + ", " + std::to_string(target_->y) : std::string("-"));
  });
}

bool Explorer::blacklisted(const Frontier & f) const {
  for (const auto & [x, y] : blacklist_) {
    if (std::hypot(f.x - x, f.y - y) < blacklist_radius_) return true;
  }
  return false;
}

void Explorer::finish(const std::string & result) {
  note_ = result;
  RCLCPP_WARN(get_logger(), "탐사 끝: %s", result.c_str());
  std_msgs::msg::String m;
  m.data = result;
  result_pub_->publish(m);
}

void Explorer::tick() {
  if (mode_ != "EXPLORE") {
    if (goal_active_ && goal_handle_) nav_->async_cancel_goal(goal_handle_);  // 수동·AUTO가 되면 바로 멈춤
    goal_active_ = false;
    target_.reset();
    return;
  }
  if (goal_active_) {
    if ((now() - goal_start_).seconds() > goal_timeout_s_) {  // 너무 오래 걸림 — 그 목표는 포기
      if (goal_handle_) nav_->async_cancel_goal(goal_handle_);
      blacklist_.push_back({target_->x, target_->y});
      ++failed_;
      goal_active_ = false;
      note_ = "목표 시간 초과";
    }
    return;
  }
  if (!map_) { note_ = "/map 대기"; return; }
  if (!costmap_) { note_ = "/global_costmap/costmap 대기 (Nav2 주행 계획)"; return; }
  if (!nav_->action_server_is_ready()) { note_ = "/navigate_to_pose 서버 대기 (Nav2 주행 계획)"; return; }
  double rx = 0, ry = 0;
  try {
    const auto t = tf_->lookupTransform(costmap_->header.frame_id, "base_footprint", tf2::TimePointZero);
    rx = t.transform.translation.x;
    ry = t.transform.translation.y;
  } catch (const tf2::TransformException & e) {
    note_ = std::string("TF 대기: ") + e.what();
    return;
  }
  const auto dist = reach_cells(*costmap_, rx, ry, goal_search_radius_);
  size_t all = 0, near = 0, black = 0, no_path = 0;
  std::optional<std::pair<Frontier, Goal>> best;
  for (const auto & f : find_frontiers(*map_, min_cells_)) {
    ++all;
    if (std::hypot(f.x - rx, f.y - ry) < min_goal_dist_) { ++near; continue; }  // 발밑 — 카메라가 못 보는 거리
    if (blacklisted(f)) { ++black; continue; }
    const auto g = reachable_goal(*costmap_, dist, f.x, f.y, goal_search_radius_);
    if (!g) { ++no_path; continue; }  // 벽 너머 — 지도가 커지면 길이 생길 수 있어 목록엔 안 넣음
    if (!best || g->dist < best->second.dist) best = std::make_pair(f, *g);
  }
  last_frontiers_ = all - near - black - no_path;
  last_skipped_ = "발밑 " + std::to_string(near) + " · 다녀옴·실패 " + std::to_string(black) + " · 길 없음 " + std::to_string(no_path);
  if (!best) {
    note_ = "고를 프론티어 없음 (" + last_skipped_ + ")";
    if (++empty_count_ >= empty_checks_) {
      const std::string tally = " — 도착 " + std::to_string(reached_) + " · 실패 " + std::to_string(failed_);
      if (black + no_path == 0) {
        finish("DONE 갈 프론티어 없음" + std::string(near ? " (남은 건 로봇 발밑뿐)" : "") + tally);
      } else {
        finish("FAILED 남은 프론티어 " + std::to_string(black + no_path) + "개 못 감 (다녀옴·실패 " + std::to_string(black) +
               " · 길 없음 " + std::to_string(no_path) + ")" + tally);
      }
      empty_count_ = 0;
    }
    return;
  }
  empty_count_ = 0;
  send_goal(best->first, best->second, rx, ry);
}

void Explorer::send_goal(const Frontier & f, const Goal & g, double rx, double ry) {
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = costmap_->header.frame_id;
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = g.x;
  goal.pose.pose.position.y = g.y;
  // 도착 방향 = 목표 자리에서 프론티어 쪽 (같은 자리면 로봇에서 목표 쪽) → 앞을 보는 카메라가 모르는 곳을 봄
  const bool on_it = std::hypot(f.x - g.x, f.y - g.y) < 0.05;
  tf2::Quaternion q;
  q.setRPY(0, 0, on_it ? std::atan2(g.y - ry, g.x - rx) : std::atan2(f.y - g.y, f.x - g.x));
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();
  target_ = f;
  goal_active_ = true;
  goal_start_ = now();
  note_ = "프론티어로 이동 (" + std::to_string(f.size) + "칸, 길 " + std::to_string(g.dist).substr(0, 4) + "m)";
  RCLCPP_INFO(get_logger(), "프론티어 (%.2f, %.2f) %zu칸 → 목표 (%.2f, %.2f) 길 %.2fm", f.x, f.y, f.size, g.x, g.y, g.dist);
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
  opt.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr h) {
    goal_handle_ = h;
    if (!h) {  // 거절 — 그 자리는 다시 안 감
      if (target_) blacklist_.push_back({target_->x, target_->y});
      ++failed_;
      goal_active_ = false;
      note_ = "목표 거절";
    }
  };
  opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
    if (!goal_active_) return;  // 이미 취소·시간 초과 처리됨
    goal_active_ = false;
    if (target_) blacklist_.push_back({target_->x, target_->y});  // 도착해도 — 안 지워지는 프론티어로 다시 가 제자리를 돌았음
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
      ++reached_;
      note_ = "도착";
    } else {
      ++failed_;
      note_ = "목표 실패";
    }
  };
  nav_->async_send_goal(goal, opt);
}

}  // namespace plan
