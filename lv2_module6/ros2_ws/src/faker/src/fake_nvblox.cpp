// fake_nvblox 구현 — nvblox ES2DF 대신 free-space 점구름 발행 (headless 테스트용).
#include "faker/fake_nvblox.hpp"

#include <cmath>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace faker {

FakeNvblox::FakeNvblox()
: rclcpp::Node("nvblox_node") {
  frame_id_ = declare_parameter("frame_id", frame_id_);
  radius_m_ = declare_parameter("radius_m", radius_m_);
  beams_ = static_cast<int>(declare_parameter("beams", beams_));
  z_m_ = declare_parameter("z_m", z_m_);
  intensity_m_ = static_cast<float>(declare_parameter("intensity_m", intensity_m_));
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/combined_esdf_pointcloud", rclcpp::SensorDataQoS());
  timer_ = create_wall_timer(std::chrono::milliseconds(200),
    [this]() { on_timer(); });
}

void FakeNvblox::on_timer() {
  sensor_msgs::msg::PointCloud2 m;
  m.header.stamp = now();
  m.header.frame_id = frame_id_;
  m.height = 1;
  m.width = static_cast<uint32_t>(std::max(1, beams_));
  m.fields = {sensor_msgs::msg::PointField(), sensor_msgs::msg::PointField(),
              sensor_msgs::msg::PointField(), sensor_msgs::msg::PointField()};
  m.fields[0].name = "x"; m.fields[0].offset = 0;
  m.fields[1].name = "y"; m.fields[1].offset = 4;
  m.fields[2].name = "z"; m.fields[2].offset = 8;
  m.fields[3].name = "intensity"; m.fields[3].offset = 12;
  for (auto & f : m.fields) {
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
  }
  m.is_bigendian = false;
  m.point_step = 16;
  m.row_step = m.point_step * m.width;
  m.is_dense = true;
  m.data.resize(m.row_step * m.height);
  float * d = reinterpret_cast<float *>(m.data.data());
  for (uint32_t i = 0; i < m.width; ++i) {
    const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(m.width);
    d[4 * i + 0] = static_cast<float>(radius_m_ * std::cos(a));
    d[4 * i + 1] = static_cast<float>(radius_m_ * std::sin(a));
    d[4 * i + 2] = static_cast<float>(z_m_);
    d[4 * i + 3] = intensity_m_;
  }
  cloud_pub_->publish(m);
}

}  // namespace faker
