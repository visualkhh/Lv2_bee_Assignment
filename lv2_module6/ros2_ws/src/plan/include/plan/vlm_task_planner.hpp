#pragma once

// vlm_task_planner (🛠 직접 작성, 담당: 권형중) — 설계: documents/ros2-design/03-plan.md
//   입력: /cmd/user_command (명령), /tracked_objects (지금 보이는 물체), /task_status (FAILED면 재계획)
//   출력: /sub_goals (plan/SubGoalArray, reliable + transient_local) → coordinator
// VLM은 "무엇을, 어떤 순서로"만 정한다. 어떻게 움직일지·파지 후보 재시도는 coordinator 몫.
// VLM 경로: vlm_endpoint가 비어 있지 않으면 호스트 ai-service에 POST /plan (계약: lv2_module6/ai-service/README)
//   → SubGoalArray 발행. 실패·타임아웃·검증 실패면 템플릿으로 폴백.
//   컨테이너는 network_mode: host라 127.0.0.1이 호스트다. endpoint가 비면(기본·CI) 템플릿만.
// 재계획은 명령 하나당 max_replans(기본 1)회 — coordinator의 "같은 단계 2회 연속 실패면 정지"와 짝.
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "perception/msg/tracked_object_array.hpp"
#include "plan/msg/sub_goal_array.hpp"
#include "plan/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan {

using perception::msg::TrackedObjectArray;
using plan::msg::SubGoal;
using plan::msg::SubGoalArray;
using plan::msg::TaskStatus;

// 재계획 문맥 — 그대로 ai-service replan{plan_id, failed_step, reason}에 실린다
struct ReplanInfo {
  uint32_t plan_id = 0;
  uint32_t step = 0;
  std::string reason;
  bool active = false;
};

// 카메라 최신 프레임 1장 (ai-service images[]용 — pole_cam·gripper_cam)
struct ImageFrame {
  std::string camera;
  sensor_msgs::msg::Image::ConstSharedPtr msg;
};

// MISSION ③ MOVE_TO_B 템플릿 — 퍽(최대 capacity개)마다 navigate·pick·place tray → navigate zone → (pick tray · place zone)×개수
std::vector<SubGoal> move_to_b_goals(const TrackedObjectArray & objs, const std::string & zone, int capacity);

// 계획 요청에 실을 로봇 위치 (map) — ok=false면 뺌
struct RobotPose {
  bool ok = false;
  double x = 0, y = 0, yaw = 0;
};

class VlmTaskPlanner : public rclcpp::Node {
public:
  VlmTaskPlanner();

private:
  void on_status(const TaskStatus & s);

  static std::string find_in(const TrackedObjectArray & objs, const std::string & cls);

  void on_command(const char * source, const std::string & text);  // /cmd/user_command · /mission_qr 공통
  void make_plan();                            // 물체 확인 후 VLM 시도 (endpoint 없으면 템플릿 즉시)
  void make_template(const TrackedObjectArray & objs, const std::string & cmd);  // 고정 4단계 발행
  void try_vlm_async();                        // 스냅샷 → 별 스레드 POST → 성공 발행·실패 템플릿

  static SubGoal goal(uint32_t step, const std::string & skill, const std::string & target,
    const std::string & relation, float offset);

  // ai-service 계약 (POST /plan → {"ok","goals"}) — 의존성 없이 소켓+최소 파서
  //   objects[]: /tracked_objects 그대로의 뜻 — id · class_name · position(map) · size · confidence · color ·
  //              camera · px {카메라: [u, v]} (ai-service images.py가 그 카메라 영상에 번호를 찍는 형식)
  //   robot: 로봇 위치 {x, y, yaw} (map, TF map→base_footprint) — VLM이 물체까지 거리를 보고 navigate를 넣게. TF 없으면 뺌
  static std::string plan_request(
    const std::string & cmd, const TrackedObjectArray & objs, const ReplanInfo & rp,
    const std::vector<ImageFrame> & imgs, const RobotPose & robot = {});
  RobotPose robot_pose() const;
  static bool fetch_goals(const std::string & endpoint, const std::string & body,
    std::vector<SubGoal> & goals, double timeout_s, std::string * err = nullptr);
  // 320px JPEG base64 (ai-service images[]용) — OpenCV 의존
  static std::string encode_jpeg_base64(const sensor_msgs::msg::Image & msg);

  int tray_capacity_;  // 적재함 용량 (coordinator tray_slots 개수와 같게)
  int max_replans_, replans_ = 0;  // 메인 스레드만 (구독 콜백·진단)
  std::string vlm_endpoint_;       // 비면 VLM 안 씀 (기본·CI) — 예: http://127.0.0.1:8000
  double vlm_timeout_s_;
  std::atomic<bool> pending_{false};   // 계획할 명령이 남아 있음
  std::atomic<uint32_t> plan_id_{0};
  std::atomic<uint64_t> epoch_{0};     // make_plan마다 증가 — 낡은 워커 응답 버림
  std::atomic<bool> vlm_busy_{false};  // VLM 워커 비행 중
  std::string cmd_;
  ReplanInfo replan_;
  TrackedObjectArray::ConstSharedPtr objects_;
  std::string last_objs_sig_;  // 마지막으로 계획 시도한 물체 집합 — 같으면 재시도 안 함 (VLM 폭주 방지)
  std::map<std::string, sensor_msgs::msg::Image::ConstSharedPtr> images_;  // 카메라별 최신 1장
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex mutex_;  // cmd_·replan_·objects_·images_·발행 결정 (구독 콜백 ↔ 워커)
  rclcpp::Publisher<SubGoalArray>::SharedPtr goals_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<TaskStatus>::SharedPtr status_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  diagnostic_updater::Updater diag_{this};
};

}  // namespace plan
