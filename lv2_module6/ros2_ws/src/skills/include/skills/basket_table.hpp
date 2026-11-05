#pragma once

// 적재함 12칸 자세표 (skills/params/basket.yaml — isaac-lab slot_table, 송정혁) — load · unload 노드가 같은 표를 읽음
//   칸 0~11 = URDF basket_slot_0~11 · 퍽 모양별(box · cyl) · 칸마다 carry 방향(left | right)
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace skills {

struct BasketTable {
  std::map<std::string, std::vector<double>> poses;   // stow · carry_left · carry_right
  std::vector<std::string> carry;                     // 칸마다 left | right
  std::map<std::string, std::vector<double>> q;       // "box/q_over" … → 칸 0~11 × joint1..5

  void declare(rclcpp::Node & n) {
    for (const char * p : {"stow", "carry_left", "carry_right"})
      poses[p] = n.declare_parameter<std::vector<double>>(std::string("pose_") + p, std::vector<double>{});
    carry = n.declare_parameter<std::vector<std::string>>("basket_carry", std::vector<std::string>{});
    for (const char * shape : {"box", "cyl"})
      for (const char * key : {"q_over", "q_low", "q_lift", "q_grasp"})
        q[std::string(shape) + "/" + key] =
          n.declare_parameter<std::vector<double>>(std::string("basket_") + shape + "_" + key, std::vector<double>{});
  }
  // 칸 slot의 관절값 (없으면 빈 목록)
  std::vector<double> slot_q(const std::string & shape, const std::string & key, int slot) const {
    const auto it = q.find(shape + "/" + key);
    if (slot < 0 || it == q.end() || it->second.size() < static_cast<size_t>(slot + 1) * 5) return {};
    return {it->second.begin() + slot * 5, it->second.begin() + slot * 5 + 5};
  }
  std::vector<double> carry_q(int slot) const {
    if (slot < 0 || slot >= static_cast<int>(carry.size())) return {};
    const auto it = poses.find("carry_" + carry[slot]);
    return it == poses.end() ? std::vector<double>{} : it->second;
  }
};

}  // namespace skills
