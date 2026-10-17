// control 테스트: body_filter는 URDF 충돌 상자(+padding) 안의 점을 빼고 밖의 점만 내보낸다
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "control/body_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

using namespace control;  // 노드 클래스·메시지 별칭 (include/control/*.hpp)

template <class Step, class Done>
bool spin_until(rclcpp::Executor & exec, Step step, Done done, double timeout_s = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    step();
    exec.spin_some(std::chrono::milliseconds(50));
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

class RosTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// 팔 링크 하나: 0.2m 상자 + 원점에서 z로 0.5m 올라간 반경 0.05·길이 0.2 원기둥
const char * URDF = R"(<robot name="t">
  <link name="cam"/>
  <link name="arm">
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
    <collision><origin xyz="0 0 0.5"/><geometry><cylinder radius="0.05" length="0.2"/></geometry></collision>
  </link>
  <joint name="j" type="fixed"><parent link="cam"/><child link="arm"/></joint>
</robot>)";

TEST(BodyFilterParse, BoxAndCylinder) {
  const auto boxes = parse_collision_boxes(URDF);
  ASSERT_EQ(boxes.size(), 2u);
  EXPECT_DOUBLE_EQ(boxes[0].half.x(), 0.1);
  EXPECT_DOUBLE_EQ(boxes[1].half.z(), 0.1);
  EXPECT_DOUBLE_EQ(boxes[1].origin.getOrigin().z(), 0.5);
}

TEST_F(RosTest, DropsPointsInsideBody) {
  auto node = std::make_shared<BodyFilter>(rclcpp::NodeOptions().parameter_overrides(
    {{"input_topic", "/test/points"}, {"output_topic", "/test/points_filtered"}, {"padding", 0.03}}));
  auto probe = std::make_shared<rclcpp::Node>("probe");
  auto urdf_pub = probe->create_publisher<std_msgs::msg::String>("/robot_description", rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::String u;
  u.data = URDF;
  urdf_pub->publish(u);
  // cam → arm: arm이 카메라 앞 x=1m
  tf2_ros::StaticTransformBroadcaster tf_pub(*probe);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "cam";
  t.child_frame_id = "arm";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;
  tf_pub.sendTransform(t);

  // 점 4개: 상자 안 · padding 안(가장자리 0.12) · 원기둥 안 · 몸 밖
  PointCloud2 cloud;
  cloud.header.frame_id = "cam";
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(4);
  const float pts[4][3] = {{1.0f, 0, 0}, {1.12f, 0, 0}, {1.0f, 0, 0.55f}, {2.0f, 0, 0}};
  sensor_msgs::PointCloud2Iterator<float> it(cloud, "x");
  for (const auto & p : pts) { it[0] = p[0]; it[1] = p[1]; it[2] = p[2]; ++it; }

  auto cloud_pub = probe->create_publisher<PointCloud2>("/test/points", rclcpp::SensorDataQoS());
  PointCloud2::SharedPtr out;
  auto sub = probe->create_subscription<PointCloud2>("/test/points_filtered", rclcpp::SensorDataQoS(),
    [&](PointCloud2::SharedPtr m) { if (m->width < 4) out = m; });  // 거른 결과만 (URDF·TF 전 통과본 무시)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(probe);

  ASSERT_TRUE(spin_until(exec, [&]() { cloud_pub->publish(cloud); }, [&]() { return out != nullptr; }));
  ASSERT_EQ(out->width, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> o(*out, "x");
  EXPECT_FLOAT_EQ(o[0], 2.0f);
}
