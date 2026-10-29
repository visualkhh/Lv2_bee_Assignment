// fake_localizer 구현 — 설명은 include/faker/fake_localizer.hpp
#include "faker/fake_localizer.hpp"

#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace faker {

FakeLocalizer::FakeLocalizer()
: rclcpp::Node("fake_localizer") {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/fake_opencr/ground_truth", 10,
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { on_truth(*m); });
}

void FakeLocalizer::on_truth(const geometry_msgs::msg::PoseStamped & truth) {
  geometry_msgs::msg::TransformStamped odom_base;
  try {
    odom_base = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  } catch (const std::exception &) {
    return;  // EKF가 아직 안 뜸
  }
  tf2::Transform map_base, odom_base_t;
  tf2::fromMsg(truth.pose, map_base);
  tf2::fromMsg(odom_base.transform, odom_base_t);
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = now();
  out.header.frame_id = "map";
  out.child_frame_id = "odom";
  out.transform = tf2::toMsg(map_base * odom_base_t.inverse());
  tf_pub_->sendTransform(out);
}

}  // namespace faker
