#include <gtest/gtest.h>
#include <cstdlib>
#include <rcl/time.h>
#include "perception/perception_node.hpp"

namespace perception {
struct PerceptionNodeTestPeer {
  static void infer(PerceptionNode & n) { n.on_infer(); }
  static void publish(PerceptionNode & n) { n.on_publish(); }
  static auto & tracks(PerceptionNode & n) { return n.tracks_; }
  static auto map(PerceptionNode & n, const Track & t) -> std::optional<Track> { return n.to_map(t); }
  static auto tf(PerceptionNode & n, const builtin_interfaces::msg::Time & t) { return n.lookup_tf("camera", t); }
  static void transform(PerceptionNode & n, double sec, double x) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom"; t.child_frame_id = "camera";
    t.header.stamp = rclcpp::Time(static_cast<int64_t>(sec * 1e9));
    t.transform.translation.x = x; t.transform.rotation.w = 1;
    n.tf_->setTransform(t, "test");
  }
  static bool model(PerceptionNode & n) {
    const char * p = std::getenv("PERCEPTION_TEST_MODEL");
    if (!p) return false;
    n.model_ = std::make_unique<YoloOnnx>(p, "cpu", 1, 640);
    return n.model_->ok();
  }
  static void frame(PerceptionNode & n, double age = 0) {
    auto & f = n.frames_["gripper_cam"];
    f.bgr = cv::Mat::zeros(64, 64, CV_8UC3);
    f.depth_mm = cv::Mat(64, 64, CV_16UC1, cv::Scalar(1000));
    f.fx = f.fy = 100; f.cx = f.cy = 32;
    f.stamp = n.now() - rclcpp::Duration::from_seconds(age);
    f.frame_id = "camera";
  }
  static void elapsed(PerceptionNode & n) { n.frames_.at("gripper_cam").infer_t = 0; }
  static double inferences(PerceptionNode & n) { return n.prof_[{"gripper_cam", "YOLO"}][1]; }
  static auto take(PerceptionNode & n) { return n.take_frames(); }
  static void stamp(PerceptionNode & n, int64_t ns) { n.frames_.at("gripper_cam").stamp = rclcpp::Time(ns); }
  static void set_clock(PerceptionNode & n, int64_t ns) {
    ASSERT_EQ(rcl_enable_ros_time_override(n.get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(n.get_clock()->get_clock_handle(), ns), RCL_RET_OK);
  }
  static void static_camera(PerceptionNode & n) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom"; t.child_frame_id = "camera"; t.transform.rotation.w = 1;
    n.tf_->setTransform(t, "test", true);
  }
  static void map_tf(PerceptionNode & n) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map"; t.child_frame_id = "odom";
    t.header.stamp = n.now();
    t.transform.translation.x = 2; t.transform.rotation.w = 1;
    n.tf_->setTransform(t, "test");
  }
  static void clear_tf(PerceptionNode & n) { n.tf_->clear(); }
  static bool started(PerceptionNode & n) { return n.map_output_started_; }
  static size_t published(PerceptionNode & n) { return n.published_; }
  static void near_box(PerceptionNode & n, double yaw = 0.0, bool inside_body = false) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "odom"; t.child_frame_id = "base_footprint";
    t.transform.rotation.w = std::cos(yaw / 2); t.transform.rotation.z = std::sin(yaw / 2);
    n.tf_->setTransform(t, "test", true);
    t.child_frame_id = "camera"; t.transform.translation.z = 0.45;
    t.transform.rotation.w = 0; t.transform.rotation.z = 0;
    t.transform.rotation.x = std::cos(yaw / 2); t.transform.rotation.y = std::sin(yaw / 2);
    n.tf_->setTransform(t, "test", true);
    n.body_box_ = {-0.25, 0.08, 0.15};
    PerceptionNode::Frame f;
    f.bgr = cv::Mat::zeros(240, 320, CV_8UC3);
    f.depth_mm = cv::Mat(240, 320, CV_32FC1, cv::Scalar(0.45));
    f.fx = f.fy = 200; f.cx = 160; f.cy = 120;
    f.stamp = n.now(); f.frame_id = "camera";
    for (int v = 0; v < 240; ++v)
      for (int u = 0; u < 320; ++u) {
        const double x = (u - f.cx) * 0.38 / f.fx, y = -(v - f.cy) * 0.38 / f.fy;
        if (std::abs(x - (inside_body ? -0.08 : 0.195)) < 0.1025 && std::abs(y) < 0.1025)
          f.depth_mm.at<float>(v, u) = 0.38f;
      }
    n.update_tracks("gripper_cam", f, {});
  }
};
}  // namespace perception
using namespace perception;

class Runtime : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override { make(); }
  // strict = Codex 엄격 모드 (map TF 필요 · 끊기면 2초 뒤 보류) — 기본은 실기 동작 유지 (odom 그대로 · 마지막 TF 계속)
  void make(bool strict = false) {
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> ps{rclcpp::Parameter("model_path", "/nonexistent.onnx")};
    if (strict) { ps.emplace_back("map_tf_required", true); ps.emplace_back("map_tf_cache_s", 2.0); }
    opts.parameter_overrides(ps);
    node.reset();
    node = std::make_shared<PerceptionNode>(opts);
  }
  std::shared_ptr<PerceptionNode> node;
};

TEST_F(Runtime, CameraStopExpiresTrackWithoutInference) {
  Track t; t.class_name = "puck";
  t.t = node->now().seconds() - 3;
  t.p = {1, 0, 0}; t.P = cv::Matx33d::eye();
  PerceptionNodeTestPeer::tracks(*node)[1] = t;
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_TRUE(PerceptionNodeTestPeer::tracks(*node).empty());
}

TEST_F(Runtime, RecentTrackSurvivesCameraGap) {
  Track t; t.class_name = "puck"; t.t = node->now().seconds();
  t.p = {1, 0, 0}; t.P = cv::Matx33d::eye();
  PerceptionNodeTestPeer::tracks(*node)[1] = t;
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::tracks(*node).size(), 1u);
}

TEST_F(Runtime, MissingMapTransformDoesNotMislabelOdom) {
  make(true);
  Track t; t.p = {1, 2, 0}; t.P = cv::Matx33d::eye();
  EXPECT_FALSE(PerceptionNodeTestPeer::map(*node, t));
}

TEST_F(Runtime, DefaultWithoutMapPublishesOdomAsBefore) {
  // map:=none (bringup 기본) — map→odom을 내는 쪽(cuVSLAM)이 없음. map 원점 = 출발 자리라 odom 그대로가 맞음 (예전 동작)
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  Track t; t.p = {1, 2, 0}; t.P = cv::Matx33d::eye();
  const auto m = PerceptionNodeTestPeer::map(*node, t);
  ASSERT_TRUE(m); EXPECT_DOUBLE_EQ(m->p[0], 1); EXPECT_DOUBLE_EQ(m->p[1], 2);
  t.class_name = "puck"; t.t = 10;
  PerceptionNodeTestPeer::tracks(*node)[1] = t;
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::published(*node), 1u);
}

TEST_F(Runtime, DefaultKeepsLastMapTransformWhenSlamDrops) {
  // cuVSLAM이 잠깐 놓쳐 map→odom이 끊겨도 좌표가 사라지지 않음 (예전 동작) — 마지막 변환 유지
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  PerceptionNodeTestPeer::map_tf(*node);
  Track t; t.p = {1, 0, 0}; t.P = cv::Matx33d::eye();
  ASSERT_TRUE(PerceptionNodeTestPeer::map(*node, t));
  PerceptionNodeTestPeer::clear_tf(*node);
  PerceptionNodeTestPeer::set_clock(*node, 60000000000LL);
  const auto m = PerceptionNodeTestPeer::map(*node, t);
  ASSERT_TRUE(m); EXPECT_DOUBLE_EQ(m->p[0], 3);
}

TEST_F(Runtime, OldImageDoesNotUseUnboundedLatestTransform) {
  PerceptionNodeTestPeer::transform(*node, 10, 1);
  EXPECT_FALSE(PerceptionNodeTestPeer::tf(*node, rclcpp::Time(11, 0)));
}

TEST_F(Runtime, SmallTransformDelayAndExactTimeStillWork) {
  PerceptionNodeTestPeer::transform(*node, 10, 1);
  ASSERT_TRUE(PerceptionNodeTestPeer::tf(*node, rclcpp::Time(10, 0)));
  ASSERT_TRUE(PerceptionNodeTestPeer::tf(*node, rclcpp::Time(10020000000LL)));
}

TEST_F(Runtime, IdenticalImageIsInferredOnlyOnce) {
  if (!PerceptionNodeTestPeer::model(*node)) GTEST_SKIP() << "Set PERCEPTION_TEST_MODEL to the deployed ONNX";
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  PerceptionNodeTestPeer::frame(*node);
  PerceptionNodeTestPeer::infer(*node);
  ASSERT_EQ(PerceptionNodeTestPeer::inferences(*node), 1);
  PerceptionNodeTestPeer::elapsed(*node);
  PerceptionNodeTestPeer::infer(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::inferences(*node), 1);
  PerceptionNodeTestPeer::frame(*node);
  PerceptionNodeTestPeer::stamp(*node, 10000000001LL);
  PerceptionNodeTestPeer::elapsed(*node);
  PerceptionNodeTestPeer::infer(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::inferences(*node), 2);
}

TEST_F(Runtime, StaleImageIsNotInferred) {
  if (!PerceptionNodeTestPeer::model(*node)) GTEST_SKIP() << "Set PERCEPTION_TEST_MODEL to the deployed ONNX";
  PerceptionNodeTestPeer::frame(*node, 3);
  PerceptionNodeTestPeer::infer(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::inferences(*node), 0);
}

TEST_F(Runtime, SelectionWithoutModelRejectsDuplicateStaleAndFutureFrames) {
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  PerceptionNodeTestPeer::frame(*node);
  ASSERT_EQ(PerceptionNodeTestPeer::take(*node).size(), 1u);
  PerceptionNodeTestPeer::elapsed(*node);
  EXPECT_TRUE(PerceptionNodeTestPeer::take(*node).empty());
  PerceptionNodeTestPeer::stamp(*node, 10000000001LL);
  ASSERT_EQ(PerceptionNodeTestPeer::take(*node).size(), 1u);
  PerceptionNodeTestPeer::elapsed(*node);
  PerceptionNodeTestPeer::stamp(*node, 7000000000LL);
  EXPECT_TRUE(PerceptionNodeTestPeer::take(*node).empty());
  PerceptionNodeTestPeer::stamp(*node, 11000000000LL);
  EXPECT_TRUE(PerceptionNodeTestPeer::take(*node).empty());
  PerceptionNodeTestPeer::stamp(*node, 0);
  EXPECT_TRUE(PerceptionNodeTestPeer::take(*node).empty());
}

TEST_F(Runtime, StaticAndLatestTransformRequestsRemainValid) {
  PerceptionNodeTestPeer::static_camera(*node);
  EXPECT_TRUE(PerceptionNodeTestPeer::tf(*node, rclcpp::Time(100, 0)));
}

TEST_F(Runtime, LatestDynamicTransformRequestRemainsValid) {
  PerceptionNodeTestPeer::transform(*node, 10, 1);
  EXPECT_TRUE(PerceptionNodeTestPeer::tf(*node, rclcpp::Time(0, 0)));
}

TEST_F(Runtime, MapTransformPreservesPositionAndExpiresCacheOnlyAfterGrace) {
  make(true);
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  PerceptionNodeTestPeer::map_tf(*node);
  Track t; t.p = {1, 0, 0}; t.P = cv::Matx33d::eye();
  auto mapped = PerceptionNodeTestPeer::map(*node, t);
  ASSERT_TRUE(mapped); EXPECT_DOUBLE_EQ(mapped->p[0], 3);
  PerceptionNodeTestPeer::clear_tf(*node);
  PerceptionNodeTestPeer::set_clock(*node, 11000000000LL);
  EXPECT_TRUE(PerceptionNodeTestPeer::map(*node, t));
  PerceptionNodeTestPeer::set_clock(*node, 13000000000LL);
  EXPECT_FALSE(PerceptionNodeTestPeer::map(*node, t));
}

TEST_F(Runtime, StartupWaitsForMapAndClockRewindDropsOldTracks) {
  make(true);
  PerceptionNodeTestPeer::set_clock(*node, 10000000000LL);
  Track t; t.class_name = "puck"; t.t = 10; t.p = {1, 0, 0}; t.P = cv::Matx33d::eye();
  PerceptionNodeTestPeer::tracks(*node)[1] = t;
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_FALSE(PerceptionNodeTestPeer::started(*node));
  EXPECT_EQ(PerceptionNodeTestPeer::published(*node), 0u);
  PerceptionNodeTestPeer::map_tf(*node);
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_TRUE(PerceptionNodeTestPeer::started(*node));
  ASSERT_EQ(PerceptionNodeTestPeer::tracks(*node).size(), 1u);
  PerceptionNodeTestPeer::set_clock(*node, 5000000000LL);
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_TRUE(PerceptionNodeTestPeer::tracks(*node).empty());
  EXPECT_FALSE(PerceptionNodeTestPeer::map(*node, t));  // rewind 이전 동적 TF도 제거
}

TEST_F(Runtime, EmptyHeartbeatDoesNotRequireMapTransform) {
  PerceptionNodeTestPeer::publish(*node);
  EXPECT_EQ(PerceptionNodeTestPeer::published(*node), 1u);
  EXPECT_FALSE(PerceptionNodeTestPeer::started(*node));
}

TEST_F(Runtime, NearDockingBoxOutsideActualBodyIsNotErased) {
  PerceptionNodeTestPeer::near_box(*node);
  const auto & tracks = PerceptionNodeTestPeer::tracks(*node);
  ASSERT_EQ(tracks.size(), 1u);
  const auto & box = tracks.begin()->second;
  EXPECT_EQ(box.class_name, "start_box");
  EXPECT_NEAR(box.p[0], 0.195, 0.015);
  EXPECT_NEAR(box.size[0], 0.205, 0.03);
  ASSERT_TRUE(box.face_pt);
  // 로봇 쪽 면 0.0925 — box_face_dist는 맨 앞 잡음을 피하려 하위 5 % 점을 씀 (변 0.205 × 0.05 ≈ 1 cm 안쪽, develop과 같음)
  EXPECT_NEAR((*box.face_pt)[0], 0.0925 + 0.05 * 0.205, 0.01);
}

TEST_F(Runtime, BodyFootprintFollowsRobotRotation) {
  PerceptionNodeTestPeer::near_box(*node, 1.5707963267948966);
  const auto & tracks = PerceptionNodeTestPeer::tracks(*node);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks.begin()->second.p[0], 0.0, 0.015);
  EXPECT_NEAR(tracks.begin()->second.p[1], 0.195, 0.015);
}

TEST_F(Runtime, ActualBodySurfaceStillExcluded) {
  PerceptionNodeTestPeer::near_box(*node, 0, true);
  EXPECT_TRUE(PerceptionNodeTestPeer::tracks(*node).empty());
}
