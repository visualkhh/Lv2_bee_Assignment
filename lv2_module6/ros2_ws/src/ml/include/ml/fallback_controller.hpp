#pragma once
// RlNavController 의 바이패스(inner) controller 이름 해석 — ROS 없이 시험하는 순수 함수 (test/test_fallback_controller.cpp)
//   별칭 dwb | mppi | rpp | rotation_shim 또는 전체 pluginlib 클래스명. 각 inner 의 파라미터 공간 = <name>.inner_<별칭>
//   (전체 클래스명을 직접 준 경우만 <name>.inner). 파라미터 공간을 나눠 두어 실행 중에 inner 를 바꿔도 서로 설정이 섞이지 않는다.
#include <algorithm>
#include <cctype>
#include <string>

namespace ml {

struct Fallback {
  std::string key;   // dwb | mppi | rpp | rotation_shim | custom
  std::string cls;   // pluginlib 클래스명
  int kind = 0;      // 1=DWB 2=MPPI 3=RPP 4=RotationShim 0=기타 — status/CLI 표시용
};

inline std::string lower_ascii(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// 이름 → Fallback. 빈 문자열이면 false.
inline bool resolve_fallback(const std::string & name, Fallback & out) {
  if (name.empty()) return false;
  const std::string f = lower_ascii(name);
  if (f == "dwb" || f == "dwb_core::dwblocalplanner") {
    out = {"dwb", "dwb_core::DWBLocalPlanner", 1};
  } else if (f == "mppi" || f == "nav2_mppi_controller::mppicontroller") {
    out = {"mppi", "nav2_mppi_controller::MPPIController", 2};
  } else if (f == "rpp" || f == "regulated_pure_pursuit" ||
             f == "nav2_regulated_pure_pursuit_controller::regulatedpurepursuitcontroller") {
    out = {"rpp", "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController", 3};
  } else if (f == "rotation_shim" || f == "nav2_rotation_shim_controller::rotationshimcontroller") {
    out = {"rotation_shim", "nav2_rotation_shim_controller::RotationShimController", 4};
  } else {
    out = {"custom", name, 0};  // 전체 pluginlib 클래스명 그대로
  }
  return true;
}

inline std::string inner_namespace(const std::string & name, const Fallback & f) {
  return f.kind ? name + ".inner_" + f.key : name + ".inner";
}

}  // namespace ml
