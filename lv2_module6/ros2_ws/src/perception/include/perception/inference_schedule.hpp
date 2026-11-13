#pragma once

#include <cmath>
#include <stdexcept>

namespace perception {

// Only image inference is throttled. Call under the perception frame mutex.
class InferenceSchedule {
public:
  void configure(bool enabled, double navigation_hz) {
    if (!std::isfinite(navigation_hz) || navigation_hz <= 0.0 || navigation_hz > 4.0)
      throw std::invalid_argument("navigation_infer_hz must be finite and in (0, 4]");
    enabled_ = enabled;
    navigation_interval_ = 1.0 / navigation_hz;
  }
  // action_msgs/GoalStatus: ACCEPTED, EXECUTING and CANCELING remain active.
  static bool active_status(int status) { return status >= 1 && status <= 3; }
  void navigation_active(bool active) { active_ = active; }
  bool throttled() const { return enabled_ && active_; }
  bool due(double now, double previous) const {
    // Retain the existing normal-mode gate; the 250ms timer caps it at 4Hz.
    return throttled() ? now - previous >= navigation_interval_ : now - previous > 0.2;
  }

private:
  bool enabled_ = false;
  bool active_ = false;
  double navigation_interval_ = 1.0;
};

}  // namespace perception
