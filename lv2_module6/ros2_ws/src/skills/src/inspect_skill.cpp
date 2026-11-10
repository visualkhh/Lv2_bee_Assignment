// skill_inspect — /skills/inspect: gripper_cam으로 보기 (학습 정책 없음). 읽었는지(details tag)는 coordinator · mission_manager도 /tracked_objects로 봄
//   점 보기 (target 비움): goal.pose(map) 점 위 inspect_height_m에 손끝을 두고 inspect_wait_s 기다림 (camera_look_down).
//     퍽 윗면 16mm AprilTag는 gripper_cam이 위에서 봐야 읽힘. values[0] = 손 방향 yaw (로봇 → 점) — 없으면 자유
//   상자 보기 (target "box"): goal.pose = 상자 위 퍽 윗면 높이의 상자 가운데 (map), values[0] = 기대 퍽 수 (없으면 2)
//     ⓪ 지금 자세에서 이미 기대 수만큼 보이고 태그도 다 읽었으면 끝 (팔 안 움직임)
//     ① pose_look 관절 자세(arm.yaml — skill_pose "look"과 같은 값)로 → inspect_look_wait_s 기다림
//     ② 상자 가운데에서 inspect_box_radius_m 안 퍽 트랙이 기대 수 이상이고 모두 태그를 읽었으면 끝
//     ③ 아니면 본 퍽(모자라면 상자 가운데도)을 한 장에 담는 자세(look_plan)로 → 기다림 → 다시 셈. 모자라면 inspect_partial
//     11/07 Isaac 전체 스택 퍽 40개: 1차 자세만 85 % · 다시 담으면 92 % (isaac/scripts/demo/inspect_test.py center)
#include <cmath>
#include <limits>
#include <mutex>

#include "perception/msg/tracked_object_array.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "skills/arm_io.hpp"
#include "skills/look_plan.hpp"
#include "skills/skill_node.hpp"
#include "tf2/LinearMath/Transform.hpp"

namespace skills {

class InspectSkill : public SkillNode {
public:
  explicit InspectSkill(const rclcpp::NodeOptions & options)
  : SkillNode("skill_inspect", options), arm_(*this) {
    height_m_ = declare_parameter<double>("inspect_height_m", 0.15);  // [m] 손끝을 퍽 윗면 위 이만큼 (16mm 태그 ≈ 50px)
    wait_s_ = declare_parameter<double>("inspect_wait_s", 1.0);       // [s] perception 추론 4Hz × 몇 장
    look_q_ = declare_parameter<std::vector<double>>("pose_look", {0.0, -1.682497, 0.324631, 1.926843, 0.0});  // arm.yaml이 덮음
    look_wait_s_ = declare_parameter<double>("inspect_look_wait_s", 1.5);  // [s] 상자 보기 자세마다 (궤적 끝 · 처짐 보정 수렴 + 추론)
    box_radius_m_ = declare_parameter<double>("inspect_box_radius_m", 0.16);  // [m] 상자 가운데에서 이 안 퍽만 셈 (반 변 0.1025 + 여유)
    objects_sub_ = create_subscription<perception::msg::TrackedObjectArray>("/tracked_objects", rclcpp::QoS(5).reliable(),
      [this](perception::msg::TrackedObjectArray::ConstSharedPtr m) { std::lock_guard<std::mutex> lk(mu_); objects_ = m; });
    serve("inspect", [this](const SkillGoal & g, Done done) {
      when_ready([this]() { return arm_.moveit_ready(); }, "arm_unavailable", done, [this, g, done]() {
        if (g.target == "box") look_box(g, done);
        else look_point(g, done);
      });
    });
  }

protected:
  void stop() override { arm_.stop(); }

private:
  void look_point(const SkillGoal & g, Done done) {
    auto at = g.pose;
    if (at.header.frame_id.empty()) at.header.frame_id = "map";
    at.header.stamp = now();
    at.pose.position.z += height_m_;
    at.pose.orientation = geometry_msgs::msg::Quaternion();
    const double yaw = g.values.empty() ? std::numeric_limits<double>::quiet_NaN() : g.values[0];
    stage("위에서 내려다보기");
    const auto look = [this, done](bool ok) {
      if (!ok) { done(false, "inspect_failed"); return; }
      stage("기다림");
      arm_.wait_step(wait_s_)([done](bool) { done(true, ""); });
    };
    // 손을 아래로 향한 채 높이 못 가는 자리가 많음 — 해석 IK로 낮춰 가며 (시뮬 camera_look_down 0.08). 다 안 되면 MoveIt
    ArmIO::Q5 now, q;
    if (arm_.arm_now(now))
      for (const double drop : {0.0, height_m_ - 0.10, height_m_ - 0.08}) {
        auto p = at;
        p.pose.position.z -= drop;
        if (arm_.ik_ee(p, now, q)) { arm_.move_joints(std::vector<double>(q.begin(), q.end()), look); return; }
      }
    arm_.move_arm(at, look, yaw);
  }

  // c 좌표계(보통 map) → base_footprint
  bool to_base(const geometry_msgs::msg::PoseStamped & c, tf2::Transform & t) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = arm_.tf().lookupTransform("base_footprint", c.header.frame_id.empty() ? "map" : c.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception &) {
      return false;
    }
    const auto & r = tf.transform.rotation;
    const auto & tr = tf.transform.translation;
    t = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(tr.x, tr.y, tr.z));
    return true;
  }

  // 상자 가운데 c에서 box_radius_m_ 안 퍽 트랙 → base_footprint 윗면 점들 (z = c.z) · 태그 읽은 수
  //   트랙 중복은 perception이 합침 (merge_tracks · merge_overlap_dets)
  bool box_pucks(const geometry_msgs::msg::PoseStamped & c, std::vector<look::V3> & tops, int & tagged) {
    tops.clear();
    tagged = 0;
    tf2::Transform t;
    if (!to_base(c, t)) return false;
    std::lock_guard<std::mutex> lk(mu_);
    if (!objects_) return true;
    for (const auto & o : objects_->objects) {
      if (o.class_name.rfind("puck", 0) != 0) continue;
      const auto & p = o.pose.pose.position;
      if (std::hypot(p.x - c.pose.position.x, p.y - c.pose.position.y) > box_radius_m_) continue;
      const tf2::Vector3 b = t * tf2::Vector3(p.x, p.y, c.pose.position.z);
      tops.push_back({b.x(), b.y(), b.z()});
      for (const auto & kv : o.details)
        if (kv.key == "tag" && !kv.value.empty()) { ++tagged; break; }
    }
    return true;
  }

  void look_box(const SkillGoal & g, Done done) {
    const auto c = g.pose;
    const size_t want = g.values.empty() ? 2 : static_cast<size_t>(std::max(1.0, g.values[0]));
    if (look_q_.size() != 5) { done(false, "inspect_no_look_pose"); return; }
    {  // 지금 자세(보통 stow — gripper_cam이 상자 쪽)에서 이미 퍽이 다 보이고 태그도 읽었으면 팔을 안 움직임
      //   11/07 Isaac 6배치: stow에서 둘 다 태그까지 1/6 (태그 중심이라 위치 1 mm 안) · 태그 없이 본 퍽은 19~39 mm 틀려 이 조건으로만
      std::vector<look::V3> tops;
      int tagged = 0;
      if (box_pucks(c, tops, tagged) && tops.size() >= want && tagged >= static_cast<int>(tops.size())) {
        stage("이미 보임 — 자세 그대로");
        done(true, "");
        return;
      }
    }
    stage("look 자세");
    arm_.move_joints(look_q_, [this, c, want, done](bool ok) {
      if (!ok) { done(false, "inspect_failed"); return; }
      arm_.wait_step(look_wait_s_)([this, c, want, done](bool) {
        std::vector<look::V3> tops;
        int tagged = 0;
        if (!box_pucks(c, tops, tagged)) { done(false, "inspect_no_tf"); return; }
        if (tops.size() >= want && tagged >= static_cast<int>(tops.size())) { done(true, ""); return; }
        // 다시 담기: 본 퍽 + (모자라면) 상자 가운데 — 못 본 퍽이 상자 위 어딘가에 있음
        tf2::Transform t;
        if (tops.size() < want && to_base(c, t)) {
          const tf2::Vector3 b = t * tf2::Vector3(c.pose.position.x, c.pose.position.y, c.pose.position.z);
          tops.push_back({b.x(), b.y(), b.z()});
        }
        look::Q5 q;
        look::Score s;
        if (!look::search(tops, look::Params{}, q, s)) { done(false, "inspect_partial"); return; }
        stage("다시 담기 (점 " + std::to_string(tops.size()) + "개 · 덮음 " + std::to_string(static_cast<int>(100 * s.cover)) + "%)");
        arm_.move_joints(std::vector<double>(q.begin(), q.end()), [this, c, want, done](bool ok2) {
          if (!ok2) { done(false, "inspect_failed"); return; }
          arm_.wait_step(look_wait_s_)([this, c, want, done](bool) {
            std::vector<look::V3> tops2;
            int tagged2 = 0;
            box_pucks(c, tops2, tagged2);
            const bool all = tops2.size() >= want && tagged2 >= static_cast<int>(tops2.size());
            done(all, all ? "" : "inspect_partial");
          });
        });
      });
    });
  }

  ArmIO arm_;
  double height_m_, wait_s_, look_wait_s_, box_radius_m_;
  std::vector<double> look_q_;
  std::mutex mu_;
  perception::msg::TrackedObjectArray::ConstSharedPtr objects_;
  rclcpp::Subscription<perception::msg::TrackedObjectArray>::SharedPtr objects_sub_;
};

}  // namespace skills

RCLCPP_COMPONENTS_REGISTER_NODE(skills::InspectSkill)
