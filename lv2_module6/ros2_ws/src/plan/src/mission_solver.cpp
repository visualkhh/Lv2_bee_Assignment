// mission_solver 구현 — 설명은 include/plan/mission_solver.hpp
#include "plan/mission_solver.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace plan {

bool valid_counts(const Counts & c, std::string & why) {
  for (int n : {c.r, c.y, c.g}) {
    if (n < 1 || n > 6) { why = "count_out_of_range"; return false; }
  }
  if (c.r + c.y + c.g != 12) { why = "sum_not_12"; return false; }
  return true;
}

bool parse_barcode(const std::string & code, bool & ascending) {
  if (code == "P2-01" || code == "P2-03") { ascending = true; return true; }
  if (code == "P2-02" || code == "P2-04") { ascending = false; return true; }
  return false;
}

int group_count(const Counts & c, char group) { return group == 'R' ? c.r : group == 'Y' ? c.y : c.g; }

std::string assign_locations(const Counts & c, bool ascending) {
  std::string order = "RYG";  // 동률이면 이 순서 그대로 (stable)
  std::stable_sort(order.begin(), order.end(), [&](char a, char b) {
    return ascending ? group_count(c, a) < group_count(c, b) : group_count(c, a) > group_count(c, b);
  });
  return order;
}

bool allocate(const std::vector<Puck> & pucks, const Counts & c, std::map<int, char> & out, std::string & why) {
  out.clear();
  if (pucks.size() != 12) { why = "puck_count"; return false; }
  std::set<int> tags;
  for (const auto & p : pucks) {
    if (!tags.insert(p.tag).second) { why = "duplicate_tag"; return false; }
    if ((p.color != "red" && p.color != "green") || (p.shape != "cylinder" && p.shape != "cuboid")) {
      why = "unknown_puck"; return false;
    }
  }
  const auto n = [&](const std::string & color, const std::string & shape) {
    return static_cast<int>(std::count_if(pucks.begin(), pucks.end(), [&](const Puck & p) {
      return p.color == color && (shape.empty() || p.shape == shape);
    }));
  };
  const int y_red = n("red", "") - c.r, y_green = n("green", "") - c.g;  // Y로 가야 하는 빨강·초록 수
  if (y_red < 0 || y_green < 0 || y_red + y_green != c.y) { why = "no_allocation"; return false; }
  for (const std::string shape : {"cylinder", "cuboid"}) {
    if (n("red", shape) < y_red || n("green", shape) < y_green) continue;
    int left_red = y_red, left_green = y_green;
    for (const auto & p : pucks) {  // 모양 s인 퍽부터 Y를 채우고 나머지는 색대로
      int & left = p.color == "red" ? left_red : left_green;
      if (p.shape == shape && left > 0) { out[p.tag] = 'Y'; --left; }
      else out[p.tag] = p.color == "red" ? 'R' : 'G';
    }
    return true;
  }
  why = "no_allocation";
  return false;
}

std::vector<int> dice_cells(int n) {
  static const std::vector<int> table[7] = {{}, {4}, {0, 8}, {0, 4, 8}, {0, 2, 6, 8}, {0, 2, 4, 6, 8}, {0, 3, 6, 2, 5, 8}};
  return (n >= 1 && n <= 6) ? table[n] : std::vector<int>{};
}

std::pair<double, double> cell_offset(int cell, double step) {
  return {(cell % 3 - 1) * step, (1 - cell / 3) * step};
}

Pose2 arena_to_map(const Pose2 & p, const Pose2 & start) {
  const double dx = p.x - start.x, dy = p.y - start.y;
  const double c = std::cos(-start.yaw), s = std::sin(-start.yaw);
  return {c * dx - s * dy, s * dx + c * dy, std::remainder(p.yaw - start.yaw, 2 * M_PI)};
}

std::vector<Pose2> approach_poses(double cx, double cy, double half, double px, double py, double standoff) {
  const double lx = std::clamp(px, cx - half, cx + half), ly = std::clamp(py, cy - half, cy + half);
  // 변마다 (점에서 그 변까지 거리, 자세) — 서쪽 변이면 x = cx − half − standoff에 서서 +x(동)를 봄
  std::vector<std::pair<double, Pose2>> sides = {
    {px - (cx - half), {cx - half - standoff, ly, 0.0}},
    {(cx + half) - px, {cx + half + standoff, ly, M_PI}},
    {py - (cy - half), {lx, cy - half - standoff, M_PI / 2}},
    {(cy + half) - py, {lx, cy + half + standoff, -M_PI / 2}},
  };
  std::stable_sort(sides.begin(), sides.end(), [](const auto & a, const auto & b) { return a.first < b.first; });
  std::vector<Pose2> out;
  for (const auto & s : sides) out.push_back(s.second);
  return out;
}

}  // namespace plan
